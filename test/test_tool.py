#!/usr/bin/env python3
"""Cross checks between the C codec (firmware) and tools/gyrostick.py (host)."""
import csv
import io
import os
import struct
import subprocess
import sys
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import gyrostick as gs  # noqa: E402

HOST = ROOT / "test" / "host"


def ensure_c_artifacts():
    if not (HOST / "test_pages.bin").exists():
        subprocess.run(["make", "-C", str(HOST), "check"], check=True, stdout=subprocess.DEVNULL)
    return (HOST / "test_pages.bin").read_bytes(), (HOST / "test_samples.csv").read_text()


# ---- python encoder, mirror of components/gyl/gyl_codec.c ---------------------------------------
class PyEnc:
    def __init__(self, channels, cap=gs.PAGE_SIZE - gs.HDR_SIZE):
        self.ch, self.cap = channels, cap
        self.buf = bytearray()
        self.acc, self.nbits, self.count = 0, 0, 0
        self.prev = [0] * channels
        self.prev2 = [0] * channels
        self.err1 = [0] * channels
        self.err2 = [0] * channels
        self.mean16 = [128] * channels

    def _bits(self, val, n):
        self.acc = ((self.acc << n) | (val & ((1 << n) - 1))) & 0xFFFFFFFF
        self.nbits += n
        while self.nbits >= 8:
            self.buf.append((self.acc >> (self.nbits - 8)) & 0xFF)
            self.nbits -= 8

    def put(self, v):
        if self.count == 0:
            if self.cap - len(self.buf) < self.ch * 2 + 30:
                return False
            for c in range(self.ch):
                self.buf += struct.pack("<h", v[c])
            self.prev = list(v[: self.ch])
            self.prev2 = list(v[: self.ch])
            self.count = 1
            return True
        if self.cap - len(self.buf) < 30:
            return False
        for c in range(self.ch):
            if self.err2[c] < self.err1[c]:
                pred = max(-32768, min(32767, 2 * self.prev[c] - self.prev2[c]))
            else:
                pred = self.prev[c]
            d = v[c] - pred
            u = ((d << 1) ^ (d >> 31)) & 0xFFFFFFFF
            k = ((self.mean16[c] >> 4) + 1).bit_length() - 1
            q = u >> k
            if q >= gs.ESC_Q:
                self._bits(0xFFFF, 16)
                self._bits(0xF, 4)
                self._bits(u, gs.ESC_BITS)
            else:
                while q >= 16:
                    self._bits(0xFFFF, 16)
                    q -= 16
                self._bits(((1 << q) - 1) << 1, q + 1)
                if k:
                    self._bits(u, k)
            self.mean16[c] += u - (self.mean16[c] >> 4)
            self.err1[c] += abs(v[c] - self.prev[c]) - (self.err1[c] >> 4)
            self.err2[c] += abs(v[c] - (2 * self.prev[c] - self.prev2[c])) - (self.err2[c] >> 4)
            self.prev2[c] = self.prev[c]
            self.prev[c] = v[c]
        self.count += 1
        return True

    def finish(self):
        if self.nbits:
            self.buf.append((self.acc << (8 - self.nbits)) & 0xFF)
            self.nbits, self.acc = 0, 0
        return bytes(self.buf)


def make_page(payload, n_samples, first_idx, seq, session=1, flags=0, odr=200, ch=6, sync_idx=None, sync_st=0,
              sync_us=0, unix=0, orient="Zxy", gyro_fs=1000, acc_fs=8):
    hdr = bytearray(gs.HDR_SIZE)
    struct.pack_into("<IBBHIIIHHHBBIIIIH", hdr, 0, gs.MAGIC, gs.VERSION, flags, len(payload), session, seq, first_idx,
                     n_samples, odr, gyro_fs, acc_fs, ch, unix,
                     first_idx + n_samples - 1 if sync_idx is None else sync_idx, sync_st & 0xFFFFFFFF,
                     sync_us & 0xFFFFFFFF, 4000)
    hdr[46:54] = orient.encode().ljust(8, b"\0")
    crc = zlib.crc32(bytes(hdr[:60]))
    crc = zlib.crc32(payload, crc)
    struct.pack_into("<I", hdr, 60, crc)
    return bytes(hdr) + payload + b"\xff" * (gs.PAGE_SIZE - gs.HDR_SIZE - len(payload))


class CodecCrossCheck(unittest.TestCase):
    def test_decode_c_pages(self):
        blob, csv_text = ensure_c_artifacts()
        rows = [tuple(int(x) for x in r) for r in csv.reader(io.StringIO(csv_text))]
        pages = [blob[i:i + gs.PAGE_SIZE] for i in range(0, len(blob), gs.PAGE_SIZE)]
        decoded = []
        for raw in pages:
            info = gs.parse_page(raw)
            self.assertIsInstance(info, gs.PageInfo)
            self.assertEqual(info.first_idx, len(decoded))
            self.assertEqual(info.orient, "Zxy")
            decoded += gs.decode_payload(raw[gs.HDR_SIZE:gs.HDR_SIZE + info.payload_len], info.channels, info.n_samples)
        self.assertEqual(decoded, rows)

    def test_python_encoder_matches_c_bytes(self):
        blob, csv_text = ensure_c_artifacts()
        rows = [tuple(int(x) for x in r) for r in csv.reader(io.StringIO(csv_text))]
        pages = [blob[i:i + gs.PAGE_SIZE] for i in range(0, len(blob), gs.PAGE_SIZE)]
        i = 0
        for raw in pages:
            info = gs.parse_page(raw)
            enc = PyEnc(6)
            while i < len(rows) and enc.put(rows[i]):
                i += 1
            payload = enc.finish()
            self.assertEqual(payload, raw[gs.HDR_SIZE:gs.HDR_SIZE + info.payload_len])
        self.assertEqual(i, len(rows))

    def test_corrupt_and_erased(self):
        blob, _ = ensure_c_artifacts()
        raw = bytearray(blob[:gs.PAGE_SIZE])
        raw[100] ^= 0x40
        self.assertEqual(gs.parse_page(bytes(raw)), gs.CORRUPT)
        self.assertEqual(gs.parse_page(b"\xff" * gs.PAGE_SIZE), gs.ERASED)
        self.assertEqual(gs.parse_page(b"\x00" * gs.PAGE_SIZE), gs.CORRUPT)


def synth_recording(n_pages=6, per_page=500, odr=200, ch=3, gap_before_page=None, st_start=0, jitter=0, gap_ticks=0):
    """Build raw pages whose sync anchors follow an ideal IMU clock (optionally with a gap / jitter)."""
    tps = gs.ST_TICKS_PER_SEC // odr
    pages, rows = [], []
    st_off = st_start
    for p in range(n_pages):
        flags = 0
        if p == 0:
            flags |= gs.FLAG_FIRST
        if gap_before_page == p:
            flags |= gs.FLAG_GAP
            st_off += gap_ticks
        enc = PyEnc(ch)
        first = len(rows)
        vals = [((first + j) % 200 - 100, (first + j) % 50, -(first + j) % 300) + (0, 0, 4096) for j in range(per_page)]
        for v in vals:
            assert enc.put(v[:ch])
        rows += [v[:ch] for v in vals]
        last = first + per_page - 1
        jit = ((p * 7919) % (2 * jitter + 1) - jitter) if jitter else 0
        pages.append(make_page(enc.finish(), per_page, first, p, odr=odr, ch=ch, flags=flags,
                               sync_idx=last, sync_st=(st_off + last * tps + jit) % gs.ST_WRAP,
                               sync_us=int((st_off + last * tps) * gs.ST_TICK_US)))
    return pages, rows, tps


class Timeline(unittest.TestCase):
    def test_uniform(self):
        pages, rows, tps = synth_recording()
        recs, stats = gs.scan_pages(pages)
        self.assertEqual(stats["ok"], len(pages))
        tl = gs.build_timeline(recs[0])
        for idx in (0, 1, 499, 500, 1500, len(rows) - 1):
            self.assertEqual(tl.ticks(idx), idx * tps)

    def test_wraparound_and_jitter(self):
        # start close to the 24 bit wrap; anchors jitter by +-1 sample worth of ticks
        pages, rows, tps = synth_recording(n_pages=12, st_start=gs.ST_WRAP - 100_000, jitter=100)
        recs, _ = gs.scan_pages(pages)
        tl = gs.build_timeline(recs[0])
        for idx in (0, 777, 3000, 5999):
            self.assertLessEqual(abs(tl.ticks(idx) - idx * tps), 100)

    def test_gap_segment(self):
        gap = 3 * gs.ST_TICKS_PER_SEC  # 3 s lost between page 2 and 3
        pages, rows, tps = synth_recording(n_pages=6, gap_before_page=3, gap_ticks=gap)
        recs, _ = gs.scan_pages(pages)
        tl = gs.build_timeline(recs[0])
        self.assertEqual(tl.ticks(1499), 1499 * tps)
        self.assertEqual(tl.ticks(1500), 1500 * tps + gap)

    def test_lost_page_keeps_time(self):
        pages, rows, tps = synth_recording(n_pages=6)
        del pages[2]  # corrupt page dropped by the scanner -> idx jump, no gap flag
        recs, _ = gs.scan_pages(pages)
        tl = gs.build_timeline(recs[0])
        self.assertEqual(tl.ticks(1000), 1000 * tps)
        self.assertEqual(tl.ticks(1500), 1500 * tps)


class GcsvExport(unittest.TestCase):
    def parse_like_gyroflow(self, text):
        """Re-implementation of telemetry-parser's gcsv reader (src/gyroflow/gcsv.rs)."""
        header, gyro, accl = {}, [], []
        time_scale, passed = 0.001, False
        for line in text.splitlines():
            row = [c.strip() for c in line.split(",")]
            if len(row) == 1:
                continue
            if len(row) == 2 and not passed:
                header[row[0]] = row[1]
                continue
            if row[0] in ("t", "time"):
                passed = True
                time_scale = float(header.get("tscale", "0.001"))
                continue
            t = float(row[0]) * time_scale
            if len(row) >= 4:
                gyro.append((t, *map(float, row[1:4])))
            if len(row) >= 7:
                accl.append((t, *map(float, row[4:7])))
        gscale = float(header.get("gscale", "1"))
        ascale = float(header.get("ascale", "1"))
        return header, gyro, accl, gscale, ascale

    def test_scales_and_times(self):
        blob, csv_text = ensure_c_artifacts()
        rows = [tuple(int(x) for x in r) for r in csv.reader(io.StringIO(csv_text))]
        pages = [blob[i:i + gs.PAGE_SIZE] for i in range(0, len(blob), gs.PAGE_SIZE)]
        recs, _ = gs.scan_pages(pages)
        text = "\n".join(gs.gcsv_lines(recs[0])) + "\n"
        header, gyro, accl, gscale, ascale = self.parse_like_gyroflow(text)
        self.assertEqual(text.splitlines()[0], "GYROFLOW IMU LOG")
        self.assertEqual(header["version"], "1.3")
        self.assertEqual(header["orientation"], "Zxy")
        self.assertEqual(header["timestamp"], "1700000000")
        self.assertEqual(len(gyro), len(rows))
        # 200 Hz -> 5 ms spacing
        self.assertAlmostEqual(gyro[1][0] - gyro[0][0], 0.005, places=9)
        self.assertAlmostEqual(gyro[-1][0], (len(rows) - 1) * 0.005, places=6)
        # +1000 dps full scale: raw 32.8 LSB per dps -> rad/s
        raw_gx = rows[0][0]
        self.assertAlmostEqual(gyro[0][1] * gscale, raw_gx / 32.8 * 3.141592653589793 / 180, places=6)
        self.assertAlmostEqual(accl[0][3] * ascale, rows[0][5] / 4096.0, places=6)

    def test_gyro_only_header(self):
        pages, rows, tps = synth_recording(n_pages=2, ch=3)
        recs, _ = gs.scan_pages(pages)
        lines = list(gs.gcsv_lines(recs[0]))
        self.assertIn("t,gx,gy,gz", lines)
        self.assertFalse(any(l.startswith("ascale") for l in lines))
        self.assertEqual(len(lines) - lines.index("t,gx,gy,gz") - 1, len(rows))


if __name__ == "__main__":
    unittest.main()
