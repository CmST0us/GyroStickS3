#!/usr/bin/env python3
"""
gyrostick - host tool for the StickS3 Gyroflow logger.

The firmware stores IMU samples in a compact binary page format (see
components/gyl/include/gyl_format.h).  This tool

  * talks to the device over its USB serial port (list / pull / erase / settime),
  * decodes the pages and writes Gyroflow ``.gcsv`` files, one per recording,
  * can also convert a raw dump of the log partition (``convert``).

Only the Python standard library plus ``pyserial`` (for the commands that talk to
the device) is needed.
"""
from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

PAGE_SIZE = 4096
HDR_SIZE = 64
MAGIC = 0x4C505947  # "GYPL"
VERSION = 1
MAX_CH = 6
ESC_Q = 20
ESC_BITS = 17

FLAG_FIRST = 0x01
FLAG_LAST = 0x02
FLAG_GAP = 0x04
FLAG_LSLEEP = 0x08

ST_TICK_US = 39.0625          # BMI270 sensor time resolution
ST_TICKS_PER_SEC = 25600
ST_WRAP = 1 << 24

# Datasheet sensitivities (LSB per unit) for the BMI270.
GYRO_LSB_PER_DPS = {125: 262.4, 250: 131.2, 500: 65.6, 1000: 32.8, 2000: 16.4}
ACC_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}

DEFAULT_ID = "StickS3_Gyro_Logger"
DEFAULT_VENDOR = "M5Stack StickS3"


# --------------------------------------------------------------------------------------
# page format
# --------------------------------------------------------------------------------------
@dataclass
class PageInfo:
    version: int
    flags: int
    payload_len: int
    session_id: int
    page_seq: int
    first_idx: int
    n_samples: int
    odr_hz: int
    gyro_fs_dps: int
    acc_fs_g: int
    channels: int
    session_unix: int
    sync_idx: int
    sync_st24: int
    sync_us32: int
    batt_mv: int
    orient: str
    index: int = -1  # page number inside the log partition (filled by the scanner)


ERASED = "erased"
CORRUPT = "corrupt"


def parse_page(page: bytes):
    """Return PageInfo, ERASED or CORRUPT."""
    if len(page) != PAGE_SIZE:
        return CORRUPT
    if page[:16] == b"\xff" * 16:
        return ERASED
    magic, version = struct.unpack_from("<IB", page, 0)
    if magic != MAGIC or version != VERSION:
        return CORRUPT
    payload_len = struct.unpack_from("<H", page, 6)[0]
    if payload_len > PAGE_SIZE - HDR_SIZE:
        return CORRUPT
    crc = zlib.crc32(page[:60])
    crc = zlib.crc32(page[HDR_SIZE:HDR_SIZE + payload_len], crc)
    if crc != struct.unpack_from("<I", page, 60)[0]:
        return CORRUPT
    (flags, plen, session_id, page_seq, first_idx, n_samples, odr_hz, gyro_fs, acc_fs, channels,
     session_unix, sync_idx, sync_st, sync_us, batt_mv) = struct.unpack_from("<xxxxxBHIIIHHHBBIIIIH", page, 0)
    orient = page[46:54].split(b"\0", 1)[0].decode("ascii", "replace")
    return PageInfo(version, flags, plen, session_id, page_seq, first_idx, n_samples, odr_hz, gyro_fs, acc_fs,
                    channels, session_unix, sync_idx, sync_st, sync_us, batt_mv, orient)


# --------------------------------------------------------------------------------------
# sample codec (mirror of components/gyl/gyl_codec.c -- keep in lock step)
# --------------------------------------------------------------------------------------
def _rice_k(mean16: int) -> int:
    m = (mean16 >> 4) + 1
    return m.bit_length() - 1


def decode_payload(payload: bytes, channels: int, n_samples: int) -> List[Tuple[int, ...]]:
    """Decode the samples of one page.  Raises ValueError on a malformed stream."""
    if channels not in (3, 6):
        raise ValueError(f"bad channel count {channels}")
    if len(payload) < 2 * channels:
        raise ValueError("payload too short")
    prev = list(struct.unpack_from(f"<{channels}h", payload, 0))
    prev2 = prev[:]
    err1 = [0] * channels
    err2 = [0] * channels
    mean16 = [128] * channels
    out: List[Tuple[int, ...]] = [tuple(prev)]

    # bit reader over the rest of the payload (MSB first)
    data = payload
    pos = 2 * channels
    acc = 0
    nbits = 0
    n = len(data)

    def get_bit():
        nonlocal acc, nbits, pos
        if nbits == 0:
            if pos >= n:
                raise ValueError("bit stream exhausted")
            acc = data[pos]
            pos += 1
            nbits = 8
        nbits -= 1
        return (acc >> nbits) & 1

    def get_bits(count):
        v = 0
        for _ in range(count):
            v = (v << 1) | get_bit()
        return v

    for _ in range(n_samples - 1):
        sample = []
        for c in range(channels):
            k = _rice_k(mean16[c])
            q = 0
            while True:
                if not get_bit():
                    break
                q += 1
                if q == ESC_Q:
                    break
            if q == ESC_Q:
                u = get_bits(ESC_BITS)
            else:
                u = (q << k) | (get_bits(k) if k else 0)
            delta = (u >> 1) ^ -(u & 1)
            if err2[c] < err1[c]:
                pred = max(-32768, min(32767, 2 * prev[c] - prev2[c]))
            else:
                pred = prev[c]
            x = pred + delta
            if not -32768 <= x <= 32767:
                raise ValueError("decoded sample out of int16 range")
            mean16[c] = mean16[c] + u - (mean16[c] >> 4)
            err1[c] += abs(x - prev[c]) - (err1[c] >> 4)
            err2[c] += abs(x - (2 * prev[c] - prev2[c])) - (err2[c] >> 4)
            prev2[c] = prev[c]
            prev[c] = x
            sample.append(x)
        out.append(tuple(sample))
    return out


# --------------------------------------------------------------------------------------
# scanning the partition into recordings
# --------------------------------------------------------------------------------------
@dataclass
class Recording:
    session_id: int
    pages: List[Tuple[PageInfo, bytes]] = field(default_factory=list)

    @property
    def first(self) -> PageInfo:
        return self.pages[0][0]

    @property
    def odr_hz(self) -> int:
        return self.first.odr_hz

    @property
    def channels(self) -> int:
        return self.first.channels

    @property
    def n_samples(self) -> int:
        return sum(p.n_samples for p, _ in self.pages)

    @property
    def clean_stop(self) -> bool:
        return bool(self.pages[-1][0].flags & FLAG_LAST)

    @property
    def duration_s(self) -> float:
        if not self.pages:
            return 0.0
        last = self.pages[-1][0]
        return (last.first_idx + last.n_samples) / max(1, self.first.odr_hz)

    @property
    def stored_bytes(self) -> int:
        return len(self.pages) * PAGE_SIZE


def scan_pages(pages: Iterable[bytes]) -> Tuple[List[Recording], Dict[str, int]]:
    """Group raw pages into recordings.  Corrupt / erased pages are skipped and counted."""
    stats = {"ok": 0, "erased": 0, "corrupt": 0}
    by_session: Dict[int, Recording] = {}
    for index, raw in enumerate(pages):
        info = parse_page(raw)
        if info == ERASED:
            stats["erased"] += 1
            continue
        if info == CORRUPT:
            stats["corrupt"] += 1
            continue
        stats["ok"] += 1
        info.index = index
        rec = by_session.setdefault(info.session_id, Recording(info.session_id))
        rec.pages.append((info, raw))
    recs = sorted(by_session.values(), key=lambda r: r.session_id)
    for rec in recs:
        rec.pages.sort(key=lambda pr: (pr[0].page_seq, pr[0].index))
    return recs, stats


# --------------------------------------------------------------------------------------
# timeline reconstruction
# --------------------------------------------------------------------------------------
@dataclass
class Timeline:
    """Maps sample index -> BMI270 sensor time ticks, segment-wise."""
    tps: int                                   # ticks per sample
    segments: List[Tuple[int, int, int]]       # (first_idx, last_idx_exclusive, offset_ticks)
    t0: int                                    # tick value of the first sample (subtracted on output)

    def ticks(self, idx: int) -> int:
        for first, last, off in self.segments:
            if first <= idx < last:
                return off + idx * self.tps - self.t0
        # outside any segment (should not happen): extrapolate from the nearest one
        first, last, off = min(self.segments, key=lambda s: min(abs(idx - s[0]), abs(idx - s[1])))
        return off + idx * self.tps - self.t0


def build_timeline(rec: Recording) -> Timeline:
    odr = rec.odr_hz
    if odr <= 0 or ST_TICKS_PER_SEC % odr:
        raise ValueError(f"unsupported ODR {odr}")
    tps = ST_TICKS_PER_SEC // odr

    # Split into segments at pages flagged as "samples lost before this page".
    seg_pages: List[List[PageInfo]] = []
    for info, _ in rec.pages:
        if not seg_pages or (info.flags & FLAG_GAP):
            seg_pages.append([])
        seg_pages[-1].append(info)

    # Unwrap the 24 bit sensor time of every page anchor into a monotone tick count.
    unwrapped: List[Tuple[PageInfo, int]] = []
    prev: Optional[Tuple[PageInfo, int]] = None
    for seg in seg_pages:
        for i, info in enumerate(seg):
            if prev is None:
                val = info.sync_st24
            elif i == 0:
                # across a gap: smallest forward step
                val = prev[1] + ((info.sync_st24 - prev[1]) % ST_WRAP)
            else:
                expect = prev[1] + (info.sync_idx - prev[0].sync_idx) * tps
                wraps = round((expect - info.sync_st24) / ST_WRAP)
                val = info.sync_st24 + wraps * ST_WRAP
            unwrapped.append((info, val))
            prev = (info, val)

    segments: List[Tuple[int, int, int]] = []
    pos = 0
    first_off: Optional[int] = None
    for seg in seg_pages:
        pairs = unwrapped[pos:pos + len(seg)]
        pos += len(seg)
        offs = [val - info.sync_idx * tps for info, val in pairs]
        off = int(round(statistics.median(offs)))
        first_idx = seg[0].first_idx
        last_idx = seg[-1].first_idx + seg[-1].n_samples
        segments.append((first_idx, last_idx, off))
        if first_off is None:
            first_off = off + first_idx * tps
    return Timeline(tps=tps, segments=segments, t0=first_off or 0)


def mcu_clock_ratio(rec: Recording) -> Optional[float]:
    """Ratio (MCU clock / IMU clock) estimated from the per-page sync anchors, or None."""
    pts: List[Tuple[float, float]] = []
    tps = ST_TICKS_PER_SEC // max(1, rec.odr_hz)
    us_prev = None
    us_unw = 0
    for info, _ in rec.pages:
        if info.flags & FLAG_GAP:
            us_prev = None
            continue
        if us_prev is None:
            us_unw = info.sync_us32
        else:
            us_unw += (info.sync_us32 - us_prev) & 0xFFFFFFFF
        us_prev = info.sync_us32
        pts.append((info.sync_idx * tps * ST_TICK_US, float(us_unw)))
    if len(pts) < 8 or pts[-1][0] - pts[0][0] < 30e6:
        return None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx == 0:
        return None
    return sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx


# --------------------------------------------------------------------------------------
# .gcsv export
# --------------------------------------------------------------------------------------
def gcsv_lines(rec: Recording, orientation: Optional[str] = None, clock_scale: float = 1.0,
               note: str = "", fw: str = "GYROSTICK", log_id: str = DEFAULT_ID,
               vendor: str = DEFAULT_VENDOR, video: str = "") -> Iterator[str]:
    first = rec.first
    ch = first.channels
    tl = build_timeline(rec)
    orient = orientation or first.orient or "XYZ"
    gscale = (3.141592653589793 / 180.0) / GYRO_LSB_PER_DPS.get(first.gyro_fs_dps, 32768.0 / first.gyro_fs_dps)
    ascale = 1.0 / ACC_LSB_PER_G.get(first.acc_fs_g, 32768.0 / first.acc_fs_g)
    tscale = ST_TICK_US * 1e-6 * clock_scale

    yield "GYROFLOW IMU LOG"
    yield "version,1.3"
    yield f"id,{log_id}"
    yield f"orientation,{orient}"
    if note:
        yield f"note,{note}"
    yield f"fwversion,{fw}"
    if first.session_unix:
        yield f"timestamp,{first.session_unix}"
    yield f"vendor,{vendor}"
    if video:
        yield f"videofilename,{video}"
    yield f"tscale,{tscale:.10g}"
    yield f"gscale,{gscale:.10g}"
    if ch == 6:
        yield f"ascale,{ascale:.10g}"
        yield "t,gx,gy,gz,ax,ay,az"
    else:
        yield "t,gx,gy,gz"

    for info, raw in rec.pages:
        samples = decode_payload(raw[HDR_SIZE:HDR_SIZE + info.payload_len], ch, info.n_samples)
        for j, s in enumerate(samples):
            t = tl.ticks(info.first_idx + j)
            yield f"{t}," + ",".join(str(v) for v in s)


def default_name(rec: Recording) -> str:
    first = rec.first
    if first.session_unix:
        stamp = datetime.fromtimestamp(first.session_unix, tz=timezone.utc).astimezone().strftime("%Y%m%d_%H%M%S")
        return f"GYRO_{rec.session_id:04d}_{stamp}.gcsv"
    return f"GYRO_{rec.session_id:04d}.gcsv"


def export_recordings(recs: List[Recording], outdir: Path, orientation: Optional[str], use_mcu_clock: bool,
                      only: Optional[List[int]] = None, note: str = "") -> List[Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for rec in recs:
        if only and rec.session_id not in only:
            continue
        scale = 1.0
        ratio = mcu_clock_ratio(rec)
        if ratio is not None:
            msg = f"IMU clock runs {100.0 * (1.0 / ratio - 1.0):+.3f}% vs MCU clock"
            if rec.first.flags & FLAG_LSLEEP:
                msg += " (MCU time was kept by the RTC during light sleep -> not precise)"
            print(f"  session {rec.session_id}: {msg}")
            if use_mcu_clock:
                scale = ratio
        path = outdir / default_name(rec)
        with open(path, "w", newline="\n") as f:
            for line in gcsv_lines(rec, orientation=orientation, clock_scale=scale, note=note):
                f.write(line + "\n")
        written.append(path)
        print(f"  wrote {path}  ({rec.n_samples} samples, {rec.duration_s:.1f} s)")
    return written


# --------------------------------------------------------------------------------------
# device link
# --------------------------------------------------------------------------------------
class DeviceError(RuntimeError):
    pass


class Device:
    """USB serial protocol spoken by main/hostlink.c."""

    def __init__(self, port: Optional[str] = None, timeout: float = 5.0, ser=None):
        """`ser` lets tests inject any object with pyserial's read/readline/write/flush API."""
        if ser is None:
            try:
                import serial  # type: ignore
                from serial.tools import list_ports  # type: ignore
            except ImportError as exc:  # pragma: no cover
                raise DeviceError("pyserial is required: pip install pyserial") from exc
            if port is None:
                cands = [p.device for p in list_ports.comports() if p.vid == 0x303A]
                if not cands:
                    raise DeviceError("no Espressif USB serial port found (is the device awake? hold the main button)")
                port = cands[0]
            ser = serial.serial_for_url(port, 115200, timeout=timeout)
        self.port = port or "injected"
        self.ser = ser
        self.info: Dict[str, str] = {}
        self._hello()

    def close(self):
        self.ser.close()

    def _readline(self) -> str:
        line = self.ser.readline()
        return line.decode("ascii", "replace").strip()

    def _hello(self):
        deadline = time.time() + 15
        self.ser.reset_input_buffer()
        while time.time() < deadline:
            self.ser.write(b"\nHELLO\n")
            self.ser.flush()
            t_end = time.time() + 1.0
            while time.time() < t_end:
                line = self._readline()
                if line.startswith("GYLOG "):
                    parts = line.split()
                    self.info = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
                    self.info["proto"] = parts[1]
                    return
        raise DeviceError("device did not answer (wake it with a long press of the main button)")

    @property
    def total_pages(self) -> int:
        return int(self.info["pages"])

    @property
    def used_pages(self) -> int:
        return int(self.info["used"])

    def command(self, text: str) -> str:
        self.ser.write(text.encode("ascii") + b"\n")
        self.ser.flush()
        return self._readline()

    def settime(self, epoch: Optional[int] = None):
        epoch = int(time.time()) if epoch is None else epoch
        resp = self.command(f"TIME {epoch}")
        if not resp.startswith("OK"):
            raise DeviceError(f"TIME failed: {resp}")

    def read_pages(self, first: int, count: int) -> bytes:
        self.ser.write(f"READ {first} {count}\n".encode("ascii"))
        self.ser.flush()
        resp = self._readline()
        if not resp.startswith("OK"):
            raise DeviceError(f"READ failed: {resp}")
        want = count * PAGE_SIZE
        data = self.ser.read(want)
        if len(data) != want:
            raise DeviceError(f"short read: got {len(data)} of {want} bytes")
        return data

    def read_all(self, progress=True) -> List[bytes]:
        used = self.used_pages
        pages: List[bytes] = []
        chunk = 32
        t0 = time.time()
        for first in range(0, used, chunk):
            n = min(chunk, used - first)
            blob = self.read_pages(first, n)
            pages.extend(blob[i * PAGE_SIZE:(i + 1) * PAGE_SIZE] for i in range(n))
            if progress:
                done = first + n
                print(f"\r  reading {done}/{used} pages ({done * PAGE_SIZE / 1024 / max(0.01, time.time() - t0):.0f} KB/s)",
                      end="", file=sys.stderr, flush=True)
        if progress and used:
            print(file=sys.stderr)
        return pages

    def erase(self):
        self.ser.write(b"ERASE\n")
        self.ser.flush()
        old = self.ser.timeout
        self.ser.timeout = 120
        try:
            while True:
                line = self._readline()
                if line.startswith("OK"):
                    return
                if line.startswith("ERR") or not line:
                    raise DeviceError(f"ERASE failed: {line or 'timeout'}")
        finally:
            self.ser.timeout = old


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------
def fmt_duration(sec: float) -> str:
    sec = int(round(sec))
    return f"{sec // 3600}:{sec // 60 % 60:02d}:{sec % 60:02d}"


def print_recordings(recs: List[Recording], stats: Dict[str, int]):
    if not recs:
        print("no recordings")
    for rec in recs:
        first = rec.first
        when = ""
        if first.session_unix:
            when = datetime.fromtimestamp(first.session_unix).strftime("%Y-%m-%d %H:%M:%S")
        flags = "" if rec.clean_stop else "  (not stopped cleanly)"
        gaps = sum(1 for p, _ in rec.pages if p.flags & FLAG_GAP)
        if gaps:
            flags += f"  [{gaps} gap(s)]"
        print(f"  #{rec.session_id:<4} {when:19}  {fmt_duration(rec.duration_s)}  {first.odr_hz} Hz  "
              f"{first.channels} ch  {rec.n_samples} samples  {rec.stored_bytes / 1024:.0f} KiB{flags}")
    if stats.get("corrupt"):
        print(f"  warning: {stats['corrupt']} corrupt page(s) skipped")


def cmd_list(args):
    dev = Device(args.port)
    print(f"device {dev.port}: fw {dev.info.get('fw')}  battery {dev.info.get('batt')} mV  "
          f"storage {dev.used_pages}/{dev.total_pages} pages used")
    recs, stats = scan_pages(dev.read_all(progress=False))
    print_recordings(recs, stats)


def cmd_pull(args):
    dev = Device(args.port)
    dev.settime()
    print(f"device {dev.port}: fw {dev.info.get('fw')}  battery {dev.info.get('batt')} mV  "
          f"storage {dev.used_pages}/{dev.total_pages} pages used")
    pages = dev.read_all()
    recs, stats = scan_pages(pages)
    print_recordings(recs, stats)
    only = [int(x) for x in args.sessions.split(",")] if args.sessions else None
    written = export_recordings(recs, Path(args.out), args.orientation, args.mcu_clock, only, args.note)
    if args.erase:
        if not written:
            print("nothing exported, not erasing")
        else:
            print("erasing device log ...")
            dev.erase()
            print("done")
    dev.close()


def cmd_dump(args):
    dev = Device(args.port)
    pages = dev.read_all()
    Path(args.file).write_bytes(b"".join(pages))
    print(f"wrote {args.file} ({len(pages)} pages)")


def cmd_erase(args):
    dev = Device(args.port)
    if not args.yes:
        if input(f"erase all {dev.used_pages} used pages? [y/N] ").strip().lower() != "y":
            return
    dev.erase()
    print("erased")


def cmd_settime(args):
    dev = Device(args.port)
    dev.settime()
    print("device clock set")


def cmd_convert(args):
    blob = Path(args.file).read_bytes()
    if len(blob) % PAGE_SIZE:
        print(f"note: file size is not a multiple of {PAGE_SIZE}, ignoring the tail", file=sys.stderr)
    pages = [blob[i:i + PAGE_SIZE] for i in range(0, len(blob) - PAGE_SIZE + 1, PAGE_SIZE)]
    recs, stats = scan_pages(pages)
    print_recordings(recs, stats)
    only = [int(x) for x in args.sessions.split(",")] if args.sessions else None
    export_recordings(recs, Path(args.out), args.orientation, args.mcu_clock, only, args.note)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="StickS3 Gyroflow logger tool")
    ap.add_argument("-p", "--port", help="serial port (default: auto-detect Espressif USB device)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def export_opts(p):
        p.add_argument("-o", "--out", default=".", help="output directory for .gcsv files")
        p.add_argument("--orientation", help="override the Gyroflow IMU orientation string stored by the firmware")
        p.add_argument("--mcu-clock", action="store_true",
                       help="rescale time using the MCU crystal (only precise if recorded without light sleep)")
        p.add_argument("--sessions", help="comma separated session numbers to export (default: all)")
        p.add_argument("--note", default="", help="text for the gcsv note field")

    p = sub.add_parser("list", help="list recordings stored on the device")
    p.set_defaults(fn=cmd_list)
    p = sub.add_parser("pull", help="download recordings and write .gcsv files")
    export_opts(p)
    p.add_argument("--erase", action="store_true", help="erase the device log after a successful export")
    p.set_defaults(fn=cmd_pull)
    p = sub.add_parser("dump", help="save the raw log pages to a file")
    p.add_argument("file")
    p.set_defaults(fn=cmd_dump)
    p = sub.add_parser("convert", help="convert a raw page dump (from 'dump' or esptool read_flash) to .gcsv")
    p.add_argument("file")
    export_opts(p)
    p.set_defaults(fn=cmd_convert)
    p = sub.add_parser("erase", help="erase all recordings on the device")
    p.add_argument("-y", "--yes", action="store_true")
    p.set_defaults(fn=cmd_erase)
    p = sub.add_parser("settime", help="set the device clock from this computer")
    p.set_defaults(fn=cmd_settime)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        args.fn(args)
    except DeviceError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
