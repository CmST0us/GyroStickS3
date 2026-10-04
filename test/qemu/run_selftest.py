#!/usr/bin/env python3
"""
End-to-end firmware test in QEMU (no hardware needed).

Builds the firmware with a simulated IMU and a scripted record/stop sequence, boots it in the
Espressif QEMU, then talks to the *real firmware host link* with tools/gyrostick.py over the
emulated UART: reads the pages back, converts them to .gcsv, checks timing / data / the injected
drop-out, and finally erases the log.

Needs an ESP-IDF environment (idf.py in PATH) and qemu-system-xtensa (idf_tools.py install qemu-xtensa).

    python3 test/qemu/run_selftest.py [--no-build]
"""
import argparse
import collections
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import gyrostick as gs  # noqa: E402

BUILD = ROOT / "build" / "qemu"
PART_OFFSET, PART_SIZE = 0x90000, 0x770000


class SockSerial:
    """Minimal pyserial look-alike around a connected socket."""

    def __init__(self, sock, timeout=5.0):
        self.sock = sock
        self.timeout = timeout
        self.buf = bytearray()
        self.log = bytearray()

    def _fill(self, deadline):
        self.sock.settimeout(max(0.01, deadline - time.time()))
        try:
            chunk = self.sock.recv(65536)
        except (socket.timeout, BlockingIOError):
            return False
        if not chunk:
            raise gs.DeviceError("connection closed")
        self.buf += chunk
        return True

    def read(self, n):
        deadline = time.time() + (self.timeout or 5)
        while len(self.buf) < n and time.time() < deadline:
            self._fill(deadline)
        out, self.buf = bytes(self.buf[:n]), self.buf[n:]
        return out

    def readline(self):
        deadline = time.time() + (self.timeout or 5)
        while b"\n" not in self.buf and time.time() < deadline:
            self._fill(deadline)
        i = self.buf.find(b"\n")
        if i < 0:
            out, self.buf = bytes(self.buf), bytearray()
            return out
        out, self.buf = bytes(self.buf[:i + 1]), self.buf[i + 1:]
        return out

    def write(self, data):
        self.sock.sendall(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.buf.clear()

    def close(self):
        self.sock.close()


def build():
    env = os.environ.copy()
    env["IDF_COMPONENT_MANAGER"] = "0"
    BUILD.mkdir(parents=True, exist_ok=True)
    sdk = BUILD / "sdkconfig"
    cmd = ["idf.py", "-B", str(BUILD), "-D", f"SDKCONFIG={sdk}",
           "-D", f"SDKCONFIG_DEFAULTS=sdkconfig.defaults;{ROOT / 'test/qemu/sdkconfig.qemu'}", "build"]
    subprocess.run(cmd, cwd=ROOT, env=env, check=True, stdout=subprocess.DEVNULL)
    flash = BUILD / "flash.bin"
    subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32s3", "merge_bin", "-o", str(flash), "--flash_mode", "dio",
                    "--flash_size", "8MB", "--fill-flash-size", "8MB", "@flash_args"], cwd=BUILD, check=True,
                   stdout=subprocess.DEVNULL)
    return flash


def find_qemu():
    q = shutil.which("qemu-system-xtensa")
    if q:
        return q
    tools = os.environ.get("IDF_TOOLS_PATH", str(Path.home() / ".espressif")) + "/tools/qemu-xtensa"
    for p in sorted(Path(tools).glob("*/qemu/bin/qemu-system-xtensa")):
        return str(p)
    raise SystemExit("qemu-system-xtensa not found")


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        check.failed += 1


check.failed = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--port", type=int, default=45123)
    args = ap.parse_args()

    flash = BUILD / "flash.bin" if args.no_build else build()
    qemu = find_qemu()
    work = Path(tempfile.mkdtemp(prefix="gyl_qemu_"))
    work_flash = work / "flash.bin"
    shutil.copy(flash, work_flash)
    proc = subprocess.Popen([qemu, "-nographic", "-machine", "esp32s3", "-drive", f"file={work_flash},if=mtd,format=raw",
                             "-serial", f"tcp:127.0.0.1:{args.port},server,nowait", "-monitor", "none"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        sock = None
        for _ in range(100):
            try:
                sock = socket.create_connection(("127.0.0.1", args.port), timeout=1)
                break
            except OSError:
                time.sleep(0.2)
        if sock is None:
            raise SystemExit("could not connect to QEMU")
        ser = SockSerial(sock)

        print("waiting for the scripted recordings ...")
        log = bytearray()
        deadline = time.time() + 180
        while b"SELFTEST: done" not in log and time.time() < deadline:
            ser._fill(time.time() + 1)
            log += ser.buf
            ser.buf.clear()
        end = time.time() + 1.0   # the rest of the line may still be in flight
        while time.time() < end:
            ser._fill(end)
            log += ser.buf
            ser.buf.clear()
        text = log.decode("ascii", "replace")
        check("SELFTEST: done, 8 pages used, 2 recordings" in text, "firmware reports 2 recordings / 8 pages")
        check("Guru Meditation" not in text and "abort()" not in text, "no crash in the firmware log")

        ser.timeout = 10
        dev = gs.Device(ser=ser)
        print("host link:", dev.info)
        check(dev.total_pages == 1904 and dev.used_pages == 8, "HELLO: 1904 pages, 8 used")
        check(dev.info.get("rec") == "0" and dev.info.get("orient") == "Zxy", "HELLO: not recording, orientation Zxy")
        dev.settime(1700000000)
        check(True, "TIME accepted")

        pages = dev.read_all(progress=False)
        recs, stats = gs.scan_pages(pages)
        check(len(recs) == 2 and stats["corrupt"] == 0, "2 recordings, no corrupt pages")
        out = work / "gcsv"
        paths = gs.export_recordings(recs, out, None, False)
        for p in paths:
            rows = [l.split(",") for l in p.read_text().splitlines() if l and l[0].isdigit()]
            t = [int(r[0]) for r in rows]
            hist = collections.Counter(b - a for a, b in zip(t, t[1:]))
            check(hist.get(128, 0) >= len(t) - 3 and len(hist) <= 2, f"{p.name}: 5 ms spacing, one gap, across the 24-bit time wrap")
            gap = [d for d in hist if d != 128]
            check(gap == [128 + 3840], f"{p.name}: injected 150 ms drop-out reconstructed ({gap})")
            gz = [int(r[3]) for r in rows]
            check(max(gz) > 2000 and min(gz) < -2000, f"{p.name}: gyro waveform survived compression")
        check(recs[0].n_samples > 3500 and recs[1].n_samples > 2000, "sample counts plausible for 19 s and 11 s")

        try:
            dev.read_pages(5000, 1)
            check(False, "READ out of range must be refused")
        except gs.DeviceError:
            check(True, "READ out of range refused")
        dev.erase()
        dev._hello()
        check(dev.used_pages == 0, "ERASE: log is empty afterwards")
        blank = dev.read_pages(0, 1)
        check(blank == b"\xff" * gs.PAGE_SIZE, "ERASE: first page reads back as 0xFF")
    finally:
        proc.kill()
        proc.wait()
        shutil.rmtree(work, ignore_errors=True)

    if check.failed:
        print(f"{check.failed} CHECK(S) FAILED")
        return 1
    print("QEMU SELFTEST OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
