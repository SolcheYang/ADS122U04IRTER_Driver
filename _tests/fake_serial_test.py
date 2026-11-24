# -*- coding: utf-8 -*-
"""Hardware-free test of the real SerialWorker path.

Replaces serial.Serial with a fake ADS122U04 that understands the commands
used by the application and streams frames using the datasheet frame layout.
"""
import csv
import math
import shutil
import sys
import time
from pathlib import Path

# Windows CI consoles may use cp1252; never let a Chinese log line crash tests.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import ads122u04_tool as m


class FakeSerial:
    def __init__(self, port=None, baudrate=115200, timeout=0.02, **kw):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.regs = [0, 0, 0, 0, 0]
        self.rx = bytearray()          # host-readable bytes
        self.inbuf = bytearray()       # bytes written by host
        self.mode = "idle"
        self.rate = 1000
        self.last_emit = time.monotonic()
        self.counter = 0
        self.sample_index = 0
        self.closed = False

    # -- config helpers ---------------------------------------------------
    @property
    def auto(self):
        return bool(self.regs[3] & 0x01)

    @property
    def continuous(self):
        return bool(self.regs[1] & 0x08)

    @property
    def dcnt(self):
        return bool(self.regs[2] & 0x40)

    @property
    def integrity(self):
        return (self.regs[2] >> 4) & 0x03

    def _cfg(self):
        return m.DeviceConfig(dcnt=self.dcnt, integrity=self.integrity,
                              continuous=self.continuous)

    def _frame(self):
        t = self.sample_index / float(self.rate)
        code = int(3_000_000 * math.sin(2 * math.pi * 4.0 * t)
                   + 120_000 * math.sin(2 * math.pi * 47.0 * t))
        code = max(-(1 << 23), min((1 << 23) - 1, code))
        frame = m.encode_sample(code, self.counter, self._cfg())
        self.counter = (self.counter + 1) & 0xFF
        self.sample_index += 1
        return frame

    def _rreg(self, addr):
        val = self.regs[addr]
        out = bytearray((val,))
        if self.integrity == 2:
            crc = m.crc16_ccitt(bytes((val,)))
            out += bytes((crc & 0xFF, crc >> 8))
        elif self.integrity == 1:
            out.append((~val) & 0xFF)
        return bytes(out)

    def _handle_cmd(self, cmd, data):
        if cmd == m.CMD_RESET:
            self.regs = [0, 0, 0, 0, 0]
            self.mode = "idle"
            self.rx.clear()
        elif cmd == m.CMD_START:
            if self.continuous and self.auto:
                self.mode = "running"
                self.last_emit = time.monotonic() - 1.0 / self.rate
            else:
                # single-shot: output exactly one frame
                self.rx += self._frame()
                self.mode = "idle"
        elif cmd == m.CMD_POWERDOWN:
            self.mode = "idle"
        elif cmd == m.CMD_RDATA:
            self.rx += self._frame()
        elif 0x20 <= cmd <= 0x2F:
            addr = (cmd >> 1) & 7
            if addr < 5:
                self.rx += self._rreg(addr)
        elif 0x40 <= cmd <= 0x4F:
            addr = (cmd >> 1) & 7
            if addr < 5 and data:
                self.regs[addr] = data[0]
                if addr == 1:
                    dr = (self.regs[1] >> 5) & 7
                    turbo = bool(self.regs[1] & 0x10)
                    table = m.DR_TABLE[turbo]
                    if dr < len(table):
                        self.rate = table[dr]
                self.last_emit = time.monotonic()

    def _parse(self):
        while len(self.inbuf) >= 2:
            idx = self.inbuf.find(b"\x55")
            if idx < 0:
                self.inbuf.clear()
                return
            if idx:
                del self.inbuf[:idx]
            if len(self.inbuf) < 2:
                return
            cmd = self.inbuf[1]
            need = 3 if 0x40 <= cmd <= 0x4F else 2
            if len(self.inbuf) < need:
                return
            data = bytes(self.inbuf[2:need])
            del self.inbuf[:need]
            self._handle_cmd(cmd, data)

    def _generate(self):
        if self.mode != "running" or not self.auto or not self.continuous:
            return
        now = time.monotonic()
        due = int((now - self.last_emit) * self.rate)
        if due <= 0:
            return
        due = min(due, 200)
        self.last_emit += due / float(self.rate)
        for _ in range(due):
            self.rx += self._frame()

    # -- serial API -------------------------------------------------------
    @property
    def in_waiting(self):
        self._generate()
        return len(self.rx)

    def read(self, size=1):
        self._generate()
        if not self.rx:
            time.sleep(min(self.timeout, 0.005))
            self._generate()
        if not self.rx:
            return b""
        n = min(size, len(self.rx))
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data):
        self.inbuf += data
        self._parse()
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._generate()
        self.rx.clear()
        self.last_emit = time.monotonic()

    def close(self):
        self.closed = True


def wait_until(pred, timeout=5.0, desc=""):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.01)
    print(f"TIMEOUT waiting for {desc}")
    return False


def run_case(name, cfg, record=True, run_secs=1.2, folder=None, sample_meta=None):
    print(f"\n=== {name} ===")
    fake = FakeSerial()
    m.serial.Serial = lambda *a, **k: fake
    w = m.SerialWorker()
    w.start()
    w.connect("COMFAKE", 115200)
    assert wait_until(lambda: w.get_state()["connected"], desc="connect")
    w.apply_config(cfg)
    ok = wait_until(lambda: "初始化完成" in w.get_state()["settings_status"] or
                    "回读" in w.get_state()["settings_status"], desc="apply")
    time.sleep(0.2)
    st = w.get_state()
    print("settings:", st["settings_status"])
    assert "5/5" in st["settings_status"], st
    if record:
        folder = Path(folder)
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True)
    meta = dict(sample_meta or {})
    meta.setdefault("lsb", 2.048 / 16 / (1 << 23))
    meta.setdefault("ain1", 1.65)
    w.start_acq(cfg, record, str(folder) if record else ".", meta)
    assert wait_until(lambda: w.get_state()["streaming"], desc="streaming")
    time.sleep(run_secs)
    q = w.stats.quick()
    print("samples:", q["total"], "errors:", q["errors"],
          "dropped:", q["dropped"], "rate:", round(q["rate"], 1))
    w.stop_acq()
    time.sleep(0.4)
    q = w.stats.quick()
    print("after stop samples:", q["total"], "state:", w.get_state()["message"])
    assert q["total"] > 10, q
    assert q["errors"] == 0, q
    assert q["dropped"] == 0, q
    if record:
        raws = list(folder.glob("*_raw.bin"))
        csvs = list(folder.glob("*_samples.csv"))
        jsons = list(folder.glob("*_config.json"))
        assert len(raws) == len(csvs) == len(jsons) == 1, (raws, csvs, jsons)
        raw = raws[0].read_bytes()
        with open(csvs[0], encoding="utf-8-sig") as f:
            rows = list(csv.reader(f))
        print("raw bytes:", len(raw), "csv rows:", len(rows) - 1,
              "frame len:", cfg.frame_len,
              "partial tail:", len(raw) % cfg.frame_len)
        # A stop can leave a partial frame in the raw stream; complete frames
        # must still match the decoded CSV row count.
        # Re-decode the saved raw stream independently.
        codes = []
        dec = m.FrameDecoder(cfg, lambda c, ctr: codes.append(c))
        dec.feed(raw)
        print("offline decoded:", len(codes), "offline errors:", dec.errors)
        assert len(codes) == len(rows) - 1, (len(codes), len(rows) - 1)
        assert dec.errors == 0
        # counter column should increment
        counters = [int(r[3]) for r in rows[1:] if r[3] != ""]
        if cfg.dcnt:
            gaps = [(b - a) & 0xFF for a, b in zip(counters, counters[1:])]
            assert all(g == 1 for g in gaps), set(gaps)
        # check code / volts consistency on the first row
        if len(rows) > 1:
            code = int(rows[1][4])
            expect_v = code * float(meta["lsb"])
            assert abs(float(rows[1][5]) - expect_v) < 1e-9
            assert abs(float(rows[1][6]) - (expect_v + meta["ain1"])) < 1e-9
            # nominal sample time = index / data rate
            assert abs(float(rows[1][1]) - 0.0) < 1e-9
            if len(rows) > 2:
                assert abs(float(rows[2][1]) - 1.0 / cfg.data_rate) < 1e-6
        print("files OK")
    w.shutdown()
    print("PASS", name)
    return fake


base = Path(__file__).resolve().parent / "_data"
for d in base.glob("*"):
    if d.is_dir():
        shutil.rmtree(d)

run_case("continuous / counter / no integrity",
         m.DeviceConfig(gain=16, dr_code=6, dcnt=True, integrity=0),
         folder=base / "case_a")
run_case("continuous / counter / CRC16",
         m.DeviceConfig(gain=16, dr_code=6, dcnt=True, integrity=2),
         folder=base / "case_b")
run_case("continuous / no counter / inverted",
         m.DeviceConfig(gain=16, dr_code=6, dcnt=False, integrity=1),
         folder=base / "case_c")
run_case("single-shot / counter",
         m.DeviceConfig(gain=16, dr_code=0, dcnt=True, integrity=0,
                        continuous=False),
         folder=base / "case_d", run_secs=2.0)

print("\nALL FAKE-SERIAL TESTS PASSED")
