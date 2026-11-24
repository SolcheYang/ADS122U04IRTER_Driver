#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADS122U04 24-bit delta-sigma ADC UART acquisition tool.

Features
  * serial port / baud selection (device auto-bauds on the 0x55 sync word)
  * full register configuration through a GUI + one-click apply / readback verify
  * automatic data read mode (AUTO=1), robust frame parsing (counter / inverted /
    CRC-16), raw byte stream + decoded CSV recording
  * real-time waveform, measurement range calculator for AIN0-AIN1 with AIN1 as
    the signal bias voltage
  * built-in hardware-free simulator

Register / protocol reference: TI ADS122U04 datasheet SBAS752B.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import random
import sys
import threading
import time
import traceback
from array import array
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


APP_NAME = "ADS122U04 串口采集工具"
APP_VERSION = "1.0.0"
FONT = "Microsoft YaHei UI"
MONO = "Consolas"

# ---------------------------------------------------------------------------
# ADS122U04 protocol constants
# ---------------------------------------------------------------------------
SYNC = 0x55
CMD_RESET = 0x06
CMD_START = 0x08
CMD_POWERDOWN = 0x02
CMD_RDATA = 0x10
REG_COUNT = 5

SIM_PORT = "SIM"
SIM_LABEL = "模拟器  —  无硬件演示"

BAUD_RATES = [9600, 19200, 38400, 57600, 115200]

MUX_CHOICES = [
    (0, "AIN0 − AIN1   （差分，默认）"),
    (1, "AIN0 − AIN2"),
    (2, "AIN0 − AIN3"),
    (3, "AIN1 − AIN0   （反向）"),
    (4, "AIN1 − AIN2"),
    (5, "AIN1 − AIN3"),
    (6, "AIN2 − AIN3"),
    (7, "AIN3 − AIN2"),
    (8, "AIN0 − AVSS   （单端）"),
    (9, "AIN1 − AVSS   （单端）"),
    (10, "AIN2 − AVSS   （单端）"),
    (11, "AIN3 − AVSS   （单端）"),
    (12, "参考电压监测 (REFP−REFN)/4"),
    (13, "电源监测 (AVDD−AVSS)/4"),
    (14, "输入短接至 (AVDD+AVSS)/2"),
]

GAIN_CHOICES = [(1, "× 1"), (2, "× 2"), (4, "× 4"), (8, "× 8"),
                (16, "× 16"), (32, "× 32"), (64, "× 64"), (128, "× 128")]
GAIN_CODES = {g: i for i, g in enumerate([1, 2, 4, 8, 16, 32, 64, 128])}

MODE_CHOICES = [(False, "正常模式 (Normal)"), (True, "Turbo 模式")]
CM_CHOICES = [(True, "连续转换"), (False, "单次转换")]
VREF_CHOICES = [(0, "内部 2.048 V"), (1, "外部 REFP / REFN"), (2, "模拟电源 AVDD")]
INTEG_CHOICES = [(0, "关闭"), (1, "反码校验"), (2, "CRC-16 校验")]

DR_TABLE = {
    False: [20, 45, 90, 175, 330, 600, 1000],
    True: [40, 90, 180, 350, 660, 1200, 2000],
}

COL_BG = "#17181d"
COL_PANEL = "#1f2128"
COL_PANEL2 = "#262933"
COL_BORDER = "#333742"
COL_FG = "#dfe3ea"
COL_MUTED = "#8b93a3"
COL_ACCENT = "#4c9aff"
COL_GREEN = "#22c55e"
COL_GREEN_DARK = "#15803d"
COL_WARN = "#f59e0b"
COL_ERR = "#ef4444"
COL_PLOT_BG = "#111318"
COL_GRID = "#242833"
COL_TRACE = "#38e1c6"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def label_for(choices, value, default=None):
    for v, text in choices:
        if v == value:
            return text
    return choices[0][1] if default is None else default


def value_for(choices, label, default):
    for v, text in choices:
        if text == label:
            return v
    return default


def fmt_eng(v, sig=4):
    """Format a voltage in engineering units."""
    if v is None:
        return "—"
    a = abs(v)
    if a < 1e-9:
        return "0 V"
    if a < 1e-6:
        return f"{v * 1e9:.{sig}g} nV"
    if a < 1e-3:
        return f"{v * 1e6:.{sig}g} µV"
    if a < 1:
        return f"{v * 1e3:.{sig}g} mV"
    return f"{v:.{sig}g} V"


def fmt_seconds(s):
    if s is None:
        return "—"
    if s >= 10:
        return f"{s:.1f} s"
    if s >= 1:
        return f"{s:.2f} s"
    return f"{s * 1000:.0f} ms"


def get_work_area():
    """Physical work-area rectangle (left, top, right, bottom) on Windows."""
    try:
        import ctypes

        class RECT(ctypes.Structure):
            _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                        ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

        r = RECT()
        if ctypes.windll.user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(r), 0):
            return r.left, r.top, r.right, r.bottom
    except Exception:
        pass
    return 0, 0, 1920, 1080


def app_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent.parent


def resource_path(name: str) -> Path:
    cands = []
    if getattr(sys, "_MEIPASS", None):
        cands.append(Path(sys._MEIPASS))
    cands += [Path(__file__).resolve().parent, app_dir()]
    for c in cands:
        p = c / name
        if p.exists():
            return p
    return Path(name)


def settings_path() -> Path:
    base = os.environ.get("APPDATA") or str(Path.home())
    return Path(base) / "ADS122U04_Tool" / "settings.json"


def default_data_dir() -> str:
    p = app_dir() / "data"
    try:
        p.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    return str(p)


# ---------------------------------------------------------------------------
# Device configuration / frame handling
# ---------------------------------------------------------------------------
@dataclass
class DeviceConfig:
    mux: int = 0
    gain: int = 16
    dr_code: int = 3          # 175 SPS in normal mode
    turbo: bool = False
    continuous: bool = True
    vref_code: int = 0
    pga_bypass: bool = False
    dcnt: bool = True
    integrity: int = 0        # 0 off, 1 inverted, 2 CRC16
    auto: bool = True

    def effective(self):
        """Return (gain, bypass) actually used by the chip."""
        mux, gain, bypass = self.mux, self.gain, self.pga_bypass
        if mux in (8, 9, 10, 11):
            return min(gain, 4), True
        if mux in (12, 13, 14):
            return 1, True
        if gain > 4:
            bypass = False
        return gain, bool(bypass)

    @property
    def data_rate(self) -> int:
        return DR_TABLE[self.turbo][self.dr_code]

    @property
    def frame_len(self) -> int:
        base = 3 + (1 if self.dcnt else 0)
        if self.integrity == 2:
            return base + 2
        if self.integrity == 1:
            return base + base
        return base

    def registers(self):
        gain, bypass = self.effective()
        reg0 = ((self.mux & 0x0F) << 4) | ((GAIN_CODES[gain] & 7) << 1) | (1 if bypass else 0)
        reg1 = (((self.dr_code & 7) << 5) | ((1 if self.turbo else 0) << 4)
                | ((1 if self.continuous else 0) << 3) | ((self.vref_code & 3) << 1))
        reg2 = ((1 if self.dcnt else 0) << 6) | ((self.integrity & 3) << 4)
        reg3 = 0x01 if self.auto else 0x00
        reg4 = 0x00
        return [reg0, reg1, reg2, reg3, reg4]

    def description(self) -> str:
        gain, bypass = self.effective()
        return (f"{label_for(MUX_CHOICES, self.mux)} · 增益×{gain} · {self.data_rate} SPS · "
                f"{'连续' if self.continuous else '单次'} · "
                f"{label_for(VREF_CHOICES, self.vref_code)} · "
                f"PGA {'旁路' if bypass else '使能'} · "
                f"{'计数开' if self.dcnt else '计数关'} · "
                f"{label_for(INTEG_CHOICES, self.integrity)} · AUTO")

    def frame_description(self) -> str:
        parts = []
        if self.dcnt:
            parts.append("计数器 1 B")
        parts.append("数据 3 B")
        if self.integrity == 1:
            parts.append("反码 3/4 B")
        elif self.integrity == 2:
            parts.append("CRC 2 B")
        return f"{self.frame_len} B/样本（{' + '.join(parts)}，数据字 LSB first）"


def encode_sample(code: int, counter: int, cfg: DeviceConfig) -> bytes:
    u = code & 0xFFFFFF
    body = bytearray()
    if cfg.dcnt:
        body.append(counter & 0xFF)
    body += bytes((u & 0xFF, (u >> 8) & 0xFF, (u >> 16) & 0xFF))
    if cfg.integrity == 1:
        body += bytes((~b) & 0xFF for b in body)
    elif cfg.integrity == 2:
        crc = crc16_ccitt(bytes(body))
        body += bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    return bytes(body)


class FrameDecoder:
    """Incremental parser for ADS122U04 automatic-data-read output.

    Automatic mode emits frames back-to-back:
        [counter] + data LSB, data, data MSB [ + inverted copy | + CRC0, CRC1 ]
    There is no header, so the counter/CRC are used to detect a slipped frame
    boundary and re-align the stream.
    """

    def __init__(self, cfg: DeviceConfig, callback):
        self.cfg = cfg
        self.callback = callback
        self.frame_len = cfg.frame_len
        self.buf = bytearray()
        self.last_counter = None
        self.dropped = 0
        self.errors = 0
        self.resyncs = 0
        self.duplicates = 0
        self.bad_streak = 0

    def reset(self):
        self.buf.clear()
        self.last_counter = None
        self.bad_streak = 0

    # -- frame primitives -------------------------------------------------
    def _decode_raw(self, frame: bytes):
        """Validate one full frame at offset 0.
        Returns (code, counter, reason) or (None, None, reason)."""
        cfg = self.cfg
        body_len = 3 + (1 if cfg.dcnt else 0)
        if cfg.integrity == 2:
            calc = crc16_ccitt(bytes(frame[:body_len]))
            b0, b1 = frame[body_len], frame[body_len + 1]
            if calc != (b0 | (b1 << 8)) and calc != ((b0 << 8) | b1):
                return None, None, "crc"
        elif cfg.integrity == 1:
            for i in range(body_len):
                if frame[body_len + i] != ((~frame[i]) & 0xFF):
                    return None, None, "inverted"
        off = 1 if cfg.dcnt else 0
        u = frame[off] | (frame[off + 1] << 8) | (frame[off + 2] << 16)
        if u & 0x800000:
            u -= 0x1000000
        ctr = frame[0] if cfg.dcnt else None
        return u, ctr, ""

    def _frame_ok(self, frame: bytes, prev_ctr):
        code, ctr, reason = self._decode_raw(frame)
        if code is None:
            return False, None, None
        if self.cfg.dcnt and prev_ctr is not None:
            if ((ctr - prev_ctr) & 0xFF) != 1:
                return False, None, None
        return True, code, ctr

    def _try_resync(self):
        """Find a byte offset at which 3 consecutive frames validate."""
        if not (self.cfg.dcnt or self.cfg.integrity):
            return False
        L = self.frame_len
        need = L * 4
        if len(self.buf) < need:
            return False
        limit = min(L * 2, len(self.buf) - L * 3 + 1)
        for off in range(1, max(1, limit)):
            prev = None
            good = 0
            for j in range(3):
                fr = bytes(self.buf[off + j * L: off + (j + 1) * L])
                ok, _, ctr = self._frame_ok(fr, prev)
                if not ok:
                    break
                prev = ctr
                good += 1
            if good == 3:
                del self.buf[:off]
                self.last_counter = None
                self.resyncs += 1
                self.bad_streak = 0
                return True
        return False

    # -- stream -----------------------------------------------------------
    def feed(self, data: bytes):
        self.buf += data
        L = self.frame_len
        guard = 0
        while len(self.buf) >= L:
            guard += 1
            if guard > 200000:
                break
            frame = bytes(self.buf[:L])
            code, ctr, reason = self._decode_raw(frame)

            if code is None:
                self.errors += 1
                self.bad_streak += 1
                # A bad frame can be either corruption or a slipped byte
                # boundary.  Try to re-align first; otherwise skip one frame
                # (corruption), then one byte at a time if the errors persist.
                if (self.cfg.dcnt or self.cfg.integrity) and self._try_resync():
                    continue
                if self.bad_streak == 1:
                    del self.buf[:L]
                else:
                    del self.buf[:1]
                continue

            self.bad_streak = 0
            if self.cfg.dcnt:
                if self.last_counter is not None:
                    delta = (ctr - self.last_counter) & 0xFF
                    if delta == 0:
                        self.duplicates += 1
                        del self.buf[:L]
                        continue
                    if delta == 1:
                        pass
                    elif 2 <= delta <= 64:
                        self.dropped += delta - 1
                    elif self.cfg.integrity == 2:
                        # CRC-valid frame: trust it, account for the gap.
                        self.dropped += delta - 1
                    else:
                        # Likely a slipped boundary; don't emit garbage.
                        self.errors += 1
                        self.bad_streak += 1
                        if self.bad_streak >= 3 and self._try_resync():
                            continue
                        del self.buf[:L]
                        continue
                self.last_counter = ctr

            del self.buf[:L]
            try:
                self.callback(code, ctr)
            except Exception:
                traceback.print_exc()


# ---------------------------------------------------------------------------
# Range calculation
# ---------------------------------------------------------------------------
@dataclass
class RangeInfo:
    vref: float = 2.048
    gain: int = 1
    bypass: bool = False
    fs: float = 2.048
    lsb: float = 2.4414e-7
    abs_lo: float = 0.0
    abs_hi: float = 3.3
    usable_lo: float = 0.0
    usable_hi: float = 3.3
    bias_ok: bool = True
    valid: bool = True
    message: str = ""


def compute_range(cfg: DeviceConfig, refp: float, ain1: float, avdd: float,
                  avss: float = 0.0) -> RangeInfo:
    gain, bypass = cfg.effective()
    if cfg.vref_code == 0:
        vref = 2.048
    elif cfg.vref_code == 1:
        vref = refp
    else:
        vref = avdd - avss
    info = RangeInfo(vref=vref, gain=gain, bypass=bypass)
    if vref <= 0:
        info.valid = False
        info.message = "参考电压必须大于 0"
        return info
    info.fs = vref / gain
    info.lsb = 2.0 * info.fs / (1 << 24)

    if cfg.mux in (12, 13, 14):
        info.valid = False
        info.message = "当前为监测/短接模式，量程计算不适用"
        info.abs_lo, info.abs_hi = avss, avdd
        info.usable_lo = info.usable_hi = ain1
        return info

    if bypass:
        # PGA disabled: absolute inputs may span AVSS-0.1 ... AVDD+0.1
        info.abs_lo = avss - 0.1
        info.abs_hi = avdd + 0.1
    else:
        # PGA enabled, Equation 6/7 of the datasheet.  Use the full-scale
        # differential voltage as VINMAX (worst case / conservative).
        k = max(0.0, (gain - 4) / 8.0)
        info.abs_lo = avss + 0.2 + k * info.fs
        info.abs_hi = avdd - 0.2 - k * info.fs

    info.usable_lo = max(ain1 - info.fs, info.abs_lo)
    info.usable_hi = min(ain1 + info.fs, info.abs_hi)
    info.bias_ok = info.abs_lo <= ain1 <= info.abs_hi
    if not info.bias_ok:
        info.valid = False
        info.message = (f"AIN1 偏置 {ain1:.4f} V 超出 PGA 绝对输入窗口 "
                        f"{info.abs_lo:.3f} ~ {info.abs_hi:.3f} V")
    elif info.usable_lo >= info.usable_hi:
        info.valid = False
        info.message = "在当前增益 / 偏置下没有可用输入范围"
    elif gain > 4 and not bypass:
        info.message = "PGA 使能，绝对输入窗口随增益收窄"
    else:
        info.message = ""
    return info


# ---------------------------------------------------------------------------
# Session statistics / recorder
# ---------------------------------------------------------------------------
class SessionStats:
    MAX_BUFFER = 300000

    def __init__(self):
        self.lock = threading.Lock()
        self.codes = array("i")
        self.total = 0
        self.min_code = None
        self.max_code = None
        self.sum_codes = 0
        self.last_code = 0
        self.last_counter = None
        self.dropped = 0
        self.errors = 0
        self.resyncs = 0
        self.rate = 0.0

    def reset(self):
        with self.lock:
            self.codes = array("i")
            self.total = 0
            self.min_code = None
            self.max_code = None
            self.sum_codes = 0
            self.last_code = 0
            self.last_counter = None
            self.dropped = 0
            self.errors = 0
            self.resyncs = 0
            self.rate = 0.0

    def add(self, code, counter=None):
        with self.lock:
            self.codes.append(code)
            if len(self.codes) > self.MAX_BUFFER:
                del self.codes[:50000]
            self.total += 1
            if self.min_code is None or code < self.min_code:
                self.min_code = code
            if self.max_code is None or code > self.max_code:
                self.max_code = code
            self.sum_codes += code
            self.last_code = code
            self.last_counter = counter

    def note(self, dropped=None, errors=None, resyncs=None, rate=None):
        with self.lock:
            if dropped is not None:
                self.dropped = dropped
            if errors is not None:
                self.errors = errors
            if resyncs is not None:
                self.resyncs = resyncs
            if rate is not None:
                self.rate = rate

    def quick(self):
        with self.lock:
            return {
                "total": self.total,
                "buffered": len(self.codes),
                "min": self.min_code,
                "max": self.max_code,
                "sum": self.sum_codes,
                "last": self.last_code,
                "counter": self.last_counter,
                "dropped": self.dropped,
                "errors": self.errors,
                "resyncs": self.resyncs,
                "rate": self.rate,
            }

    def snapshot(self, tail_count=0, max_points=6000):
        with self.lock:
            buffered = len(self.codes)
            step = 1
            window = 0
            arr = array("i")
            if tail_count and tail_count > 0:
                window = min(int(tail_count), buffered)
                if window > 0:
                    arr = self.codes[buffered - window:]
                    if max_points and window > max_points:
                        step = int(math.ceil(window / float(max_points)))
                        arr = arr[::step]
            return {
                "total": self.total,
                "buffered": buffered,
                "min": self.min_code,
                "max": self.max_code,
                "sum": self.sum_codes,
                "last": self.last_code,
                "counter": self.last_counter,
                "dropped": self.dropped,
                "errors": self.errors,
                "resyncs": self.resyncs,
                "rate": self.rate,
                "window": window,
                "step": step,
                "codes": arr,
            }


class Recorder:
    def __init__(self, folder: Path, cfg: DeviceConfig, meta: dict, rate: float):
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.rate = float(rate) if rate else 1.0
        base = f"ADS122U04_{stamp}"
        self.raw_path = folder / f"{base}_raw.bin"
        self.csv_path = folder / f"{base}_samples.csv"
        self.json_path = folder / f"{base}_config.json"
        self.raw = open(self.raw_path, "wb", buffering=1 << 16)
        self.csv = open(self.csv_path, "w", newline="", encoding="utf-8-sig",
                        buffering=1 << 16)
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["index", "t_s", "host_time_s", "counter",
                              "code", "vin_V", "ain0_V"])
        self.rows = 0
        with open(self.json_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        self.raw_bytes = 0

    def write_raw(self, data: bytes):
        self.raw.write(data)
        self.raw_bytes += len(data)

    def add(self, index, host_t, counter, code, vin, ain0):
        self.writer.writerow([index, f"{index / self.rate:.6f}",
                              f"{host_t:.6f}",
                              "" if counter is None else counter, code,
                              f"{vin:.9f}",
                              "" if ain0 is None else f"{ain0:.9f}"])
        self.rows += 1
        if self.rows % 512 == 0:
            self.csv.flush()

    def flush(self):
        try:
            self.raw.flush()
            self.csv.flush()
        except Exception:
            pass

    def close(self):
        paths = (self.raw_path, self.csv_path, self.json_path)
        try:
            self.raw.flush()
            self.raw.close()
        except Exception:
            pass
        try:
            self.csv.flush()
            self.csv.close()
        except Exception:
            pass
        return paths


# ---------------------------------------------------------------------------
# Serial / acquisition worker
# ---------------------------------------------------------------------------
class SerialWorker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.cmds = queue.Queue()
        self.events = queue.Queue()
        self.stop_evt = threading.Event()
        self.ser = None
        self.connected = False
        self.sim = False
        self.streaming = False
        self.port = ""
        self.baud = 115200
        self.cfg = DeviceConfig()
        self.decoder = None
        self.recorder = None
        self.stats = SessionStats()
        self.state_lock = threading.Lock()
        self.state = {
            "connected": False,
            "streaming": False,
            "port": "",
            "baud": 0,
            "sim": False,
            "message": "未连接",
            "settings_status": "尚未初始化",
            "settings_ok": False,
            "raw_path": "",
            "csv_path": "",
            "recording": False,
        }
        self._lsb = 2.4414e-7
        self._ain1 = 0.0
        self._acq_t0 = 0.0
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_ema = 0.0
        self._trigger_pending = False
        self._sim_last = 0.0
        self._sim_acc = 0.0
        self._sim_idx = 0
        self._sim_counter = 0
        self._flush_t = 0.0
        self._sample_index = 0

    # -- public API (thread safe) ----------------------------------------
    def connect(self, port, baud):
        self.cmds.put(("connect", (port, baud)))

    def disconnect(self):
        self.cmds.put(("disconnect", None))

    def apply_config(self, cfg):
        self.cmds.put(("apply", cfg))

    def start_acq(self, cfg, record, folder, meta):
        self.cmds.put(("start", (cfg, record, folder, meta)))

    def stop_acq(self):
        self.cmds.put(("stop", None))

    def read_config(self):
        self.cmds.put(("read", None))

    def clear_stats(self):
        self.cmds.put(("clear", None))

    def shutdown(self):
        self.cmds.put(("shutdown", None))
        self.join(2.0)
        self.stop_evt.set()

    def get_state(self):
        with self.state_lock:
            return dict(self.state)

    def _set_state(self, **kw):
        with self.state_lock:
            self.state.update(kw)
        self.events.put(("state", None))

    def log(self, level, text):
        self.events.put(("log", (level, text)))

    # -- thread ------------------------------------------------------------
    def run(self):
        while not self.stop_evt.is_set():
            try:
                while True:
                    name, arg = self.cmds.get_nowait()
                    self._dispatch(name, arg)
            except queue.Empty:
                pass

            if self.connected and self.streaming:
                try:
                    if self.sim:
                        self._sim_pump()
                    else:
                        waiting = self.ser.in_waiting
                        data = self.ser.read(waiting if waiting else 1)
                        if data:
                            self._on_rx(data)
                    if self._trigger_pending and self.streaming and not self.sim:
                        self._trigger_pending = False
                        if not self.cfg.continuous:
                            time.sleep(0.002)
                            self.ser.write(bytes((SYNC, CMD_START)))
                    self._update_rate()
                    now = time.monotonic()
                    if self.recorder and now - self._flush_t > 0.5:
                        self.recorder.flush()
                        self._flush_t = now
                except Exception as exc:
                    self.log("error", f"串口通信异常：{exc}")
                    try:
                        self._do_disconnect()
                    except Exception:
                        pass
            else:
                time.sleep(0.008)

    def _dispatch(self, name, arg):
        try:
            if name == "connect":
                self._do_connect(*arg)
            elif name == "disconnect":
                self._do_disconnect()
            elif name == "apply":
                self._do_apply(arg)
            elif name == "start":
                cfg, record, folder, meta = arg
                self._do_start(cfg, record, folder, meta)
            elif name == "stop":
                self._do_stop()
            elif name == "read":
                self._do_read_registers()
            elif name == "clear":
                self.stats.reset()
                self._sample_index = 0
                self.log("info", "显示缓冲已清空（数据记录未中断）")
            elif name == "shutdown":
                self._do_disconnect()
                self.stop_evt.set()
        except Exception as exc:
            self.log("error", f"操作失败：{exc}")
            self.log("error", traceback.format_exc().strip().splitlines()[-1])

    # -- connection --------------------------------------------------------
    def _do_connect(self, port, baud):
        self._do_disconnect(quiet=True)
        self.port, self.baud = port, int(baud)
        if port == SIM_PORT:
            self.sim = True
            self.ser = None
            self.connected = True
            self.log("ok", "已连接模拟数据源（无硬件演示模式）")
        else:
            if SERIAL_IMPORT_ERROR:
                raise RuntimeError(f"pyserial 不可用：{SERIAL_IMPORT_ERROR}")
            self.ser = serial.Serial(port=port, baudrate=int(baud), bytesize=8,
                                     parity=serial.PARITY_NONE, stopbits=1,
                                     timeout=0.02, write_timeout=0.5)
            self.sim = False
            self.connected = True
            self.ser.reset_input_buffer()
            self.log("ok", f"已连接 {port} @ {baud} 8-N-1（ADS122U04 自动波特率）")
        self.stats.reset()
        self._set_state(connected=True, sim=self.sim, port=port, baud=int(baud),
                        streaming=False, message=("模拟器已连接" if self.sim else f"{port} 已连接"),
                        settings_status="未初始化", settings_ok=False,
                        raw_path="", csv_path="", recording=False)

    def _do_disconnect(self, quiet=False):
        was_streaming = self.streaming
        self._do_stop(quiet=quiet or not self.connected)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self.sim = False
        self.decoder = None
        self.streaming = False
        if not quiet:
            self.log("info", "串口已断开")
        self._set_state(connected=False, streaming=False, sim=False, port="",
                        baud=0, message="未连接", recording=False,
                        raw_path="", csv_path="", settings_status="尚未初始化",
                        settings_ok=False)
        if was_streaming:
            self.log("info", "采集已停止")

    # -- commands ----------------------------------------------------------
    def _do_apply(self, cfg: DeviceConfig):
        self._do_stop(quiet=True)
        self.stats.reset()
        self.cfg = cfg
        self._set_state(settings_status="正在写入配置…", settings_ok=False)
        if self.sim:
            time.sleep(0.05)
            self.log("info", "[模拟器] " + cfg.description())
            self.log("info", f"[模拟器] 数据帧格式：{cfg.frame_description()}")
            self._set_state(settings_status="模拟器配置已生效", settings_ok=True)
            return
        ok, total, detail = self._configure(cfg)
        if ok == total:
            self._set_state(settings_status=f"初始化完成 · 回读 {ok}/{total} 一致",
                            settings_ok=True)
            self.log("ok", f"初始化完成，寄存器回读 {ok}/{total} 一致")
        else:
            self._set_state(settings_status=f"初始化完成但回读 {ok}/{total} 一致",
                            settings_ok=False)
            self.log("warn", f"寄存器回读不一致：{ok}/{total}（请检查波特率/接线）")
        for line in detail:
            self.log("info", line)

    def _do_start(self, cfg: DeviceConfig, record: bool, folder, meta: dict):
        self._do_stop(quiet=True)
        self.stats.reset()
        self._sample_index = 0
        self.cfg = cfg
        self._lsb = float(meta.get("lsb", 2.4414e-7)) or 2.4414e-7
        self._ain1 = float(meta.get("ain1", 0.0))
        self._mux = cfg.mux
        self._acq_t0 = time.monotonic()
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_ema = 0.0
        self._trigger_pending = False
        self.decoder = FrameDecoder(cfg, self._on_sample)
        self.recorder = None

        self._set_state(settings_status="正在初始化…", settings_ok=False)
        if self.sim:
            self.log("info", "[模拟器] " + cfg.description())
            self.log("info", f"[模拟器] 数据帧：{cfg.frame_description()}")
        else:
            ok, total, detail = self._configure(cfg)
            for line in detail:
                self.log("info", line)
            self.log("ok" if ok == total else "warn",
                     f"寄存器回读 {ok}/{total} 一致")
            self._set_state(settings_status=f"初始化完成 · 回读 {ok}/{total}",
                            settings_ok=(ok == total))

        if record:
            try:
                self.recorder = Recorder(Path(folder), cfg, meta, cfg.data_rate)
                self.log("info", f"原始码流文件：{self.recorder.raw_path}")
                self.log("info", f"解码 CSV 文件：{self.recorder.csv_path}")
            except Exception as exc:
                self.recorder = None
                self.log("error", f"无法创建记录文件：{exc}")

        if not self.sim:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(bytes((SYNC, CMD_START)))
                self.ser.flush()
            except Exception:
                if self.recorder:
                    self.recorder.close()
                    self.recorder = None
                raise

        self.streaming = True
        self._sim_last = 0.0
        self._sim_acc = 0.0
        self._sim_idx = 0
        self._sim_counter = 0
        self._flush_t = time.monotonic()
        self.stats.note(rate=0.0)
        self._set_state(streaming=True, recording=self.recorder is not None,
                        raw_path=str(self.recorder.raw_path) if self.recorder else "",
                        csv_path=str(self.recorder.csv_path) if self.recorder else "",
                        settings_status="采集中", settings_ok=True,
                        message=f"采集中 · {cfg.data_rate} SPS")
        self.log("ok", f"开始采集 · {cfg.description()}")
        self.log("info", f"帧格式：{cfg.frame_description()}")

    def _do_stop(self, quiet=False):
        was = self.streaming
        self.streaming = False
        self._trigger_pending = False
        if was and self.connected and not self.sim and self.ser is not None:
            try:
                for _ in range(3):
                    self.ser.write(bytes((SYNC, CMD_POWERDOWN)))
                    time.sleep(0.012)
                time.sleep(0.06)
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.log("info", "已发送 POWERDOWN，停止转换")
        if self.recorder is not None:
            try:
                raw_path, csv_path, _ = self.recorder.close()
                self.log("ok", f"原始码流已保存：{raw_path}（{self.recorder.raw_bytes} 字节）")
                self.log("info", f"解码数据：{csv_path}（{self.recorder.rows} 行）")
                self._set_state(raw_path=str(raw_path), csv_path=str(csv_path),
                                recording=False)
            except Exception as exc:
                self.log("warn", f"关闭记录文件时出错：{exc}")
            self.recorder = None
        if was:
            self.stats.note(rate=0.0)
            if self.connected:
                self._set_state(streaming=False, message="已停止", recording=False,
                                settings_status="已停止")

    def _do_read_registers(self):
        if not self.connected:
            return
        if self.streaming:
            self.log("warn", "采集进行中，请先停止采集再读取配置")
            return
        if self.sim:
            for i, v in enumerate(self.cfg.registers()):
                self.log("info", f"[模拟器] 寄存器 {i:02X}h = 0x{v:02X}")
            return
        self.log("info", "回读配置寄存器…")
        for addr in range(REG_COUNT):
            val = self._read_register(addr, self.cfg)
            if val is None:
                self.log("warn", f"  寄存器 {addr:02X}h：回读超时")
            else:
                self.log("info", f"  寄存器 {addr:02X}h = 0x{val:02X}")

    # -- low level UART ----------------------------------------------------
    def _write_cmd(self, cmd: int, data: bytes = b""):
        payload = bytes((SYNC, cmd)) + data
        self.ser.write(payload)
        self.log("info", "TX  " + " ".join(f"{b:02X}" for b in payload))

    def _read_exact(self, n, timeout=0.15):
        end = time.monotonic() + timeout
        out = bytearray()
        while len(out) < n and time.monotonic() < end:
            chunk = self.ser.read(n - len(out))
            if chunk:
                out += chunk
        return bytes(out) if len(out) == n else None

    def _read_register(self, addr, cfg):
        extra = 2 if cfg.integrity == 2 else (1 if cfg.integrity == 1 else 0)
        self.ser.reset_input_buffer()
        self._write_cmd(0x20 | ((addr & 7) << 1))
        raw = self._read_exact(1 + extra, 0.2)
        if raw is None:
            return None
        if extra == 2:
            calc = crc16_ccitt(raw[:1])
            b0, b1 = raw[1], raw[2]
            if calc not in (b0 | (b1 << 8), (b0 << 8) | b1):
                self.log("warn", f"寄存器 {addr:02X}h 回读 CRC 错误")
        elif extra == 1:
            if raw[1] != ((~raw[0]) & 0xFF):
                self.log("warn", f"寄存器 {addr:02X}h 回读反码错误")
        return raw[0]

    def _configure(self, cfg: DeviceConfig):
        """Reset, write all registers, verify by reading them back."""
        regs = cfg.registers()
        self.ser.reset_input_buffer()
        self._write_cmd(CMD_RESET)
        time.sleep(0.006)  # td(RSRX) >= 80 us
        # Drop any bytes that were still in flight from a previous stream.
        self.ser.reset_input_buffer()
        for addr, val in enumerate(regs):
            self._write_cmd(0x40 | ((addr & 7) << 1), bytes((val,)))
            time.sleep(0.003)
        self.log("info", "[配置] " + "  ".join(
            f"{i:02X}h={v:02X}" for i, v in enumerate(regs)))
        self.log("info", "[参数] " + cfg.description())

        self.ser.reset_input_buffer()
        results = []
        ok_count = 0
        detail = []
        for addr, val in enumerate(regs):
            got = self._read_register(addr, cfg)
            good = got == val
            if good:
                ok_count += 1
            results.append((addr, val, got))
            detail.append(f"  回读 {addr:02X}h = " +
                          ("超时" if got is None else f"0x{got:02X}") +
                          f" / 写入 0x{val:02X}  " + ("✓" if good else "✗"))
        self.ser.reset_input_buffer()
        return ok_count, len(regs), detail

    # -- receive path ------------------------------------------------------
    def _on_rx(self, data: bytes):
        if self.recorder is not None:
            try:
                self.recorder.write_raw(data)
            except Exception as exc:
                self.log("error", f"写入原始流失败：{exc}")
                try:
                    self.recorder.close()
                except Exception:
                    pass
                self.recorder = None
        if self.decoder is not None:
            self.decoder.feed(data)
        self.stats.note(dropped=self.decoder.dropped if self.decoder else 0,
                        errors=self.decoder.errors if self.decoder else 0,
                        resyncs=self.decoder.resyncs if self.decoder else 0)

    def _on_sample(self, code, counter):
        host_t = time.monotonic() - self._acq_t0
        vin = code * self._lsb
        if self._mux == 0:
            ain0 = vin + self._ain1       # AIN0 = (AIN0 - AIN1) + AIN1
        elif self._mux == 3:
            ain0 = self._ain1 - vin       # reversed channel AIN1 - AIN0
        else:
            ain0 = None                   # AIN0 absolute is not derivable
        self.stats.add(code, counter)
        if self.recorder is not None:
            try:
                self.recorder.add(self._sample_index, host_t, counter, code, vin, ain0)
            except Exception as exc:
                self.log("error", f"写入 CSV 失败：{exc}")
                try:
                    self.recorder.close()
                except Exception:
                    pass
                self.recorder = None
        self._sample_index += 1
        if not self.cfg.continuous:
            self._trigger_pending = True

    def _update_rate(self):
        now = time.monotonic()
        dt = now - self._rate_t
        if dt < 0.5:
            return
        total = self.stats.quick()["total"]
        inst = (total - self._rate_n) / dt
        self._rate_n = total
        self._rate_t = now
        self._rate_ema = inst if self._rate_ema <= 0 else (self._rate_ema * 0.6 + inst * 0.4)
        self.stats.note(rate=self._rate_ema,
                        dropped=self.decoder.dropped if self.decoder else 0,
                        errors=self.decoder.errors if self.decoder else 0,
                        resyncs=self.decoder.resyncs if self.decoder else 0)

    # -- simulator ---------------------------------------------------------
    def _sim_pump(self):
        now = time.monotonic()
        if self._sim_last == 0.0:
            self._sim_last = now
            return
        dt = min(now - self._sim_last, 0.25)
        self._sim_last = now
        rate = self.cfg.data_rate
        self._sim_acc += dt * rate
        n = int(self._sim_acc)
        self._sim_acc -= n
        n = min(n, max(1, int(rate * 0.1)))
        if n <= 0:
            return
        fs = max(self._lsb * (1 << 23), 1e-12)
        chunk = bytearray()
        for _ in range(n):
            t = self._sim_idx / float(rate)
            v = (0.55 * fs * math.sin(2 * math.pi * 2.0 * t)
                 + 0.12 * fs * math.sin(2 * math.pi * 7.0 * t + 0.7)
                 + 0.05 * fs * math.sin(2 * math.pi * 0.23 * t)
                 + random.gauss(0.0, 0.0035 * fs))
            code = int(round(v / self._lsb))
            code = max(-(1 << 23), min((1 << 23) - 1, code))
            chunk += encode_sample(code, self._sim_counter, self.cfg)
            self._sim_counter = (self._sim_counter + 1) & 0xFF
            self._sim_idx += 1
        self._on_rx(bytes(chunk))


# ---------------------------------------------------------------------------
# Scrollable sidebar
# ---------------------------------------------------------------------------
class ScrollFrame(tk.Frame):
    def __init__(self, master):
        super().__init__(master, bg=COL_PANEL)
        self.canvas = tk.Canvas(self, bg=COL_PANEL, highlightthickness=0, bd=0)
        self.vsb = ttk.Scrollbar(self, orient="vertical",
                                 command=self.canvas.yview,
                                 style="Vertical.TScrollbar")
        self.canvas.configure(yscrollcommand=self.vsb.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.vsb.pack(side="right", fill="y")
        self.inner = tk.Frame(self.canvas, bg=COL_PANEL)
        self.win = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>", self._inner_cfg)
        self.canvas.bind("<Configure>", self._canvas_cfg)
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._wheel))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _inner_cfg(self, _=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _canvas_cfg(self, event):
        self.canvas.itemconfigure(self.win, width=event.width)

    def _wheel(self, event):
        self.canvas.yview_scroll(int(-event.delta / 120) * 2, "units")


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, root: tk.Tk, demo=False):
        self.root = root
        self.demo = demo
        self.demo_pending = False
        self.worker = SerialWorker()
        self.worker.start()

        self.ui_scale = max(1.0, float(root.winfo_fpixels("1i")) / 96.0)
        self.settings = self._load_settings_file()
        self.dev_cfg = DeviceConfig()
        self.range_info = None
        self._range_ctx = None
        self._dr_code = 3
        self._snapshot_quick = None
        self._plot_dirty = True
        self.plot_paused = False
        self._last_draw = 0.0
        self._last_state = {}
        self._port_map = {}
        self._last_valid = None

        self._build_styles()
        self._build_ui()
        self._apply_settings_to_ui()
        self._refresh_ports()
        self._on_config_widget_change()
        self._update_range()
        self._refresh_state(self.worker.get_state())

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(60, self._tick)
        if demo:
            self.root.after(500, self._demo_connect)

    # -- persistence -------------------------------------------------------
    @staticmethod
    def _load_settings_file():
        try:
            with open(settings_path(), "r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def _save_settings_file(self):
        try:
            path = settings_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._gather_settings(), f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def _gather_settings(self):
        try:
            cfg = self._ui_config()
            refp, ain1, avdd = self._range_values()
        except Exception:
            return self.settings
        return {
            "port": self._port_map.get(self.port_var.get(), ""),
            "port_label": self.port_var.get(),
            "baud": int(self.baud_var.get() or 115200),
            "mux": cfg.mux, "gain": cfg.gain, "dr_code": self._dr_code,
            "turbo": cfg.turbo, "continuous": cfg.continuous,
            "vref": cfg.vref_code, "bypass": cfg.pga_bypass,
            "dcnt": cfg.dcnt, "integrity": cfg.integrity,
            "refp": refp, "ain1": ain1, "avdd": avdd,
            "timebase": self.timebase_var.get(),
            "autoscale": bool(self.autoscale_var.get()),
            "plot_mode": self.plot_mode_var.get(),
            "record": bool(self.record_var.get()),
            "data_dir": self.data_dir_var.get(),
        }

    def _apply_settings_to_ui(self):
        s = self.settings
        try:
            if s.get("baud") in BAUD_RATES:
                self.baud_var.set(str(s["baud"]))
            if isinstance(s.get("mux"), int):
                self.mux_var.set(label_for(MUX_CHOICES, s["mux"]))
            if isinstance(s.get("gain"), int):
                gain = s["gain"]
                if gain not in GAIN_CODES:
                    gain = 16
                self.gain_var.set(label_for(GAIN_CHOICES, gain))
            turbo = bool(s.get("turbo", False))
            self.mode_var.set(label_for(MODE_CHOICES, turbo))
            self._dr_code = int(s.get("dr_code", 3))
            if not 0 <= self._dr_code <= 6:
                self._dr_code = 3
            self._refresh_dr_values(self._dr_code)
            self.cm_var.set(label_for(CM_CHOICES, bool(s.get("continuous", True))))
            self.vref_var.set(label_for(VREF_CHOICES, int(s.get("vref", 0)), VREF_CHOICES[0][1]))
            self.bypass_var.set(bool(s.get("bypass", False)))
            self.dcnt_var.set(bool(s.get("dcnt", True)))
            self.integ_var.set(label_for(INTEG_CHOICES, int(s.get("integrity", 0))))
            self.refp_var.set(str(s.get("refp", 2.048)))
            self.ain1_var.set(str(s.get("ain1", 1.65)))
            self.avdd_var.set(str(s.get("avdd", 3.3)))
            tb = str(s.get("timebase", "2 s"))
            if tb not in list(self.timebase_combo.cget("values")):
                tb = "2 s"
            self.timebase_var.set(tb)
            self.autoscale_var.set(bool(s.get("autoscale", True)))
            mode = s.get("plot_mode", "abs")
            self.plot_mode_var.set(mode if mode in ("abs", "diff") else "abs")
            self.record_var.set(bool(s.get("record", True)))
            data_dir = s.get("data_dir") or default_data_dir()
            self.data_dir_var.set(data_dir)
        except Exception:
            pass

    # -- styles ------------------------------------------------------------
    def _build_styles(self):
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except Exception:
            pass
        self.root.option_add("*Font", (FONT, 9))
        self.root.option_add("*TCombobox*Listbox.background", COL_PANEL2)
        self.root.option_add("*TCombobox*Listbox.foreground", COL_FG)
        self.root.option_add("*TCombobox*Listbox.selectBackground", COL_ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#0b1220")

        st.configure(".", background=COL_BG, foreground=COL_FG, font=(FONT, 9))
        st.configure("TFrame", background=COL_BG)
        st.configure("TLabel", background=COL_BG, foreground=COL_FG)
        st.configure("Muted.TLabel", background=COL_BG, foreground=COL_MUTED,
                     font=(FONT, 8))
        st.configure("Panel.TFrame", background=COL_PANEL)
        st.configure("Panel.TLabelframe", background=COL_PANEL,
                     bordercolor=COL_BORDER, relief="solid", borderwidth=1)
        st.configure("Panel.TLabelframe.Label", background=COL_PANEL,
                     foreground=COL_ACCENT, font=(FONT, 10, "bold"))
        st.configure("P.TLabel", background=COL_PANEL, foreground=COL_FG)
        st.configure("PMuted.TLabel", background=COL_PANEL, foreground=COL_MUTED,
                     font=(FONT, 8))
        st.configure("PHead.TLabel", background=COL_PANEL, foreground=COL_ACCENT,
                     font=(FONT, 10, "bold"))
        st.configure("TButton", background=COL_PANEL2, foreground=COL_FG,
                     borderwidth=0, focusthickness=0, padding=(10, 6),
                     relief="flat")
        st.map("TButton",
               background=[("active", "#343846"), ("pressed", "#3b4050"),
                           ("disabled", "#22242b")],
               foreground=[("disabled", "#6b7280")])
        st.configure("Accent.TButton", background=COL_ACCENT, foreground="#07101f",
                     font=(FONT, 10, "bold"), padding=(10, 8), borderwidth=0)
        st.map("Accent.TButton",
               background=[("active", "#6cb0ff"), ("pressed", "#3b87e6"),
                           ("disabled", "#2b3a52")],
               foreground=[("disabled", "#7f8798")])
        st.configure("Go.TButton", background=COL_GREEN_DARK, foreground="#eafff2",
                     font=(FONT, 10, "bold"), padding=(10, 8), borderwidth=0)
        st.map("Go.TButton",
               background=[("active", "#1a9a4b"), ("pressed", "#126c38"),
                           ("disabled", "#22352a")],
               foreground=[("disabled", "#7c8a80")])
        st.configure("Small.TButton", background=COL_PANEL2, foreground=COL_FG,
                     padding=(6, 3), borderwidth=0, font=(FONT, 8))
        st.map("Small.TButton", background=[("active", "#343846")])

        st.configure("TCombobox", fieldbackground=COL_PANEL2,
                     background=COL_PANEL2, foreground=COL_FG,
                     arrowcolor=COL_FG, bordercolor=COL_BORDER,
                     lightcolor=COL_PANEL2, darkcolor=COL_PANEL2,
                     selectbackground=COL_PANEL2, selectforeground=COL_FG,
                     padding=3)
        st.map("TCombobox",
               fieldbackground=[("readonly", COL_PANEL2), ("disabled", "#23252c")],
               foreground=[("readonly", COL_FG), ("disabled", COL_MUTED)],
               arrowcolor=[("disabled", COL_MUTED)],
               selectbackground=[("readonly", COL_PANEL2)],
               selectforeground=[("readonly", COL_FG)])
        st.configure("TEntry", fieldbackground=COL_PANEL2, foreground=COL_FG,
                     bordercolor=COL_BORDER, lightcolor=COL_PANEL2,
                     darkcolor=COL_PANEL2, insertcolor=COL_FG, padding=3)
        st.map("TEntry", fieldbackground=[("disabled", "#23252c")],
               foreground=[("disabled", COL_MUTED)])
        st.configure("TCheckbutton", background=COL_PANEL, foreground=COL_FG,
                     indicatorcolor=COL_PANEL2, font=(FONT, 9))
        st.map("TCheckbutton",
               background=[("active", COL_PANEL)],
               foreground=[("disabled", COL_MUTED)],
               indicatorcolor=[("disabled", "#2a2c34"), ("selected", COL_ACCENT)])
        st.configure("TRadiobutton", background=COL_PANEL, foreground=COL_FG,
                     indicatorcolor=COL_PANEL2, font=(FONT, 9))
        st.map("TRadiobutton",
               background=[("active", COL_PANEL)],
               indicatorcolor=[("disabled", "#2a2c34"), ("selected", COL_ACCENT)])
        st.configure("Vertical.TScrollbar", background=COL_PANEL2,
                     troughcolor=COL_BG, bordercolor=COL_BG,
                     arrowcolor=COL_MUTED, lightcolor=COL_PANEL2,
                     darkcolor=COL_PANEL2)
        st.map("Vertical.TScrollbar", background=[("active", "#3c4152")])

    # -- UI construction ---------------------------------------------------
    def _build_ui(self):
        root = self.root
        root.title(f"{APP_NAME}  v{APP_VERSION}")
        root.configure(bg=COL_BG)
        try:
            icon = resource_path("icon.ico")
            if icon.exists():
                root.iconbitmap(str(icon))
        except Exception:
            pass

        s = self.ui_scale
        wl, wt, wr, wb = get_work_area()
        ww, wh = max(800, wr - wl), max(600, wb - wt)
        # Design size 1340x900 at 96 dpi, scaled to the actual DPI.
        # Reserve space for the title bar / frame, otherwise the window can
        # extend below the taskbar on high-DPI displays.
        w = min(int(1340 * s), ww - int(30 * s))
        h = min(int(900 * s), wh - int(80 * s))
        w = max(w, min(int(1120 * s), ww - int(30 * s)))
        h = max(h, min(int(740 * s), wh - int(80 * s)))
        x = wl + max(0, (ww - w) // 2)
        y = wt + max(0, (wh - h) // 2 - int(35 * s))
        root.geometry(f"{w}x{h}+{x}+{y}")
        root.minsize(min(int(1120 * s), ww - 20),
                     min(int(680 * s), wh - 20))

        self._build_header()

        content = tk.Frame(root, bg=COL_BG)
        content.pack(fill="both", expand=True,
                     padx=int(12 * s), pady=(0, int(10 * s)))
        content.grid_rowconfigure(0, weight=1)
        content.grid_columnconfigure(1, weight=1)

        left = tk.Frame(content, bg=COL_BG, width=int(438 * s))
        left.grid(row=0, column=0, sticky="ns", padx=(0, int(6 * s)))
        left.grid_propagate(False)
        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True)
        self.side = self.scroll.inner
        self._build_sidebar(self.side)
        self._bind_wheel_tree(self.side)

        right = tk.Frame(content, bg=COL_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(int(6 * s), 0))
        self._build_right(right)

        self._log_colors()
        self._log("info", f"{APP_NAME} v{APP_VERSION} 已启动")
        if SERIAL_IMPORT_ERROR:
            self._log("error", f"pyserial 初始化失败：{SERIAL_IMPORT_ERROR}")

    def _build_header(self):
        s = self.ui_scale
        header = tk.Frame(self.root, bg=COL_BG)
        header.pack(fill="x", padx=int(16 * s), pady=(int(10 * s), int(8 * s)))
        left = tk.Frame(header, bg=COL_BG)
        left.pack(side="left")
        tk.Label(left, text="ADS122U04  24 位 ΔΣ ADC 串口采集", bg=COL_BG,
                 fg=COL_FG, font=(FONT, 15, "bold")).pack(anchor="w")
        tk.Label(left, text="UART 2-wire · 自动波特率 · 图形化配置 · 原始码流 · 实时波形",
                 bg=COL_BG, fg=COL_MUTED, font=(FONT, 9)).pack(anchor="w", pady=(2, 0))

        right = tk.Frame(header, bg=COL_BG)
        right.pack(side="right")
        self.dot = tk.Canvas(right, width=14, height=14, bg=COL_BG,
                             highlightthickness=0)
        self.dot.pack(side="left", padx=(0, 8))
        self.dot_id = self.dot.create_oval(3, 3, 11, 11, fill="#555b6a", outline="")
        self.conn_var = tk.StringVar(value="未连接")
        tk.Label(right, textvariable=self.conn_var, bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 9)).pack(side="left")

    def _form_row(self, parent, row, label, widget, hint=None):
        tk.Label(parent, text=label, bg=COL_PANEL, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=row, column=0, sticky="w",
                                      padx=(2, 10), pady=4)
        widget.grid(row=row, column=1, sticky="ew", pady=4)

    def _build_sidebar(self, parent):
        s = self.ui_scale
        # ---- serial ----
        lf = ttk.Labelframe(parent, text=" 串口连接 ", style="Panel.TLabelframe")
        lf.pack(fill="x", padx=8, pady=(8, 5))
        body = tk.Frame(lf, bg=COL_PANEL)
        body.pack(fill="x", padx=10, pady=(6, 9))
        body.columnconfigure(1, weight=1)

        port_row = tk.Frame(body, bg=COL_PANEL)
        port_row.columnconfigure(0, weight=1)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port_var,
                                       state="readonly", width=25)
        self.port_combo.grid(row=0, column=0, sticky="ew")
        ttk.Button(port_row, text="刷新", style="Small.TButton",
                   command=self._refresh_ports).grid(row=0, column=1, padx=(6, 0))
        self._form_row(body, 0, "串口", port_row)

        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(body, textvariable=self.baud_var,
                                       values=[str(b) for b in BAUD_RATES],
                                       state="readonly")
        self._form_row(body, 1, "波特率", self.baud_combo)

        self.connect_btn = ttk.Button(body, text="连接串口",
                                      command=self._toggle_connect)
        self.connect_btn.grid(row=2, column=0, columnspan=2, sticky="ew",
                              pady=(6, 0))

        # ---- init settings ----
        lf2 = ttk.Labelframe(parent, text=" 初始化设置 ", style="Panel.TLabelframe")
        lf2.pack(fill="x", padx=8, pady=5)
        cfg = tk.Frame(lf2, bg=COL_PANEL)
        cfg.pack(fill="x", padx=10, pady=(6, 9))
        cfg.columnconfigure(1, weight=1)
        row = 0

        self.mux_var = tk.StringVar(value=MUX_CHOICES[0][1])
        self.mux_combo = ttk.Combobox(cfg, textvariable=self.mux_var,
                                      values=[x[1] for x in MUX_CHOICES],
                                      state="readonly")
        self._form_row(cfg, row, "输入通道", self.mux_combo); row += 1

        self.gain_var = tk.StringVar(value=label_for(GAIN_CHOICES, 16))
        self.gain_combo = ttk.Combobox(cfg, textvariable=self.gain_var,
                                       values=[x[1] for x in GAIN_CHOICES],
                                       state="readonly")
        self._form_row(cfg, row, "增益", self.gain_combo); row += 1

        self.mode_var = tk.StringVar(value=MODE_CHOICES[0][1])
        self.mode_combo = ttk.Combobox(cfg, textvariable=self.mode_var,
                                       values=[x[1] for x in MODE_CHOICES],
                                       state="readonly")
        self._form_row(cfg, row, "运行模式", self.mode_combo); row += 1

        self.dr_var = tk.StringVar()
        self.dr_combo = ttk.Combobox(cfg, textvariable=self.dr_var,
                                     state="readonly")
        self._form_row(cfg, row, "数据速率", self.dr_combo); row += 1
        self._refresh_dr_values(self._dr_code)

        self.cm_var = tk.StringVar(value=CM_CHOICES[0][1])
        self.cm_combo = ttk.Combobox(cfg, textvariable=self.cm_var,
                                     values=[x[1] for x in CM_CHOICES],
                                     state="readonly")
        self._form_row(cfg, row, "转换模式", self.cm_combo); row += 1

        self.vref_var = tk.StringVar(value=VREF_CHOICES[0][1])
        self.vref_combo = ttk.Combobox(cfg, textvariable=self.vref_var,
                                       values=[x[1] for x in VREF_CHOICES],
                                       state="readonly")
        self._form_row(cfg, row, "参考源", self.vref_combo); row += 1

        self.bypass_var = tk.BooleanVar(value=False)
        self.bypass_chk = ttk.Checkbutton(cfg, text="PGA 旁路（仅增益 ≤4）",
                                          variable=self.bypass_var)
        self.bypass_chk.grid(row=row, column=0, columnspan=2, sticky="w",
                             pady=(5, 2)); row += 1

        self.dcnt_var = tk.BooleanVar(value=True)
        self.dcnt_chk = ttk.Checkbutton(cfg, text="数据计数器（推荐，用于帧同步）",
                                        variable=self.dcnt_var)
        self.dcnt_chk.grid(row=row, column=0, columnspan=2, sticky="w",
                           pady=2); row += 1

        self.integ_var = tk.StringVar(value=INTEG_CHOICES[0][1])
        self.integ_combo = ttk.Combobox(cfg, textvariable=self.integ_var,
                                        values=[x[1] for x in INTEG_CHOICES],
                                        state="readonly")
        self._form_row(cfg, row, "数据校验", self.integ_combo); row += 1

        self.cfg_hint_var = tk.StringVar(value="")
        self.cfg_hint = tk.Label(cfg, textvariable=self.cfg_hint_var, bg=COL_PANEL,
                                 fg=COL_MUTED, font=(FONT, 8), justify="left",
                                 wraplength=int(360 * s))
        self.cfg_hint.grid(row=row, column=0, columnspan=2, sticky="w", pady=(4, 0))
        row += 1

        self.frame_hint_var = tk.StringVar(value="")
        tk.Label(cfg, textvariable=self.frame_hint_var, bg=COL_PANEL,
                 fg=COL_MUTED, font=(MONO, 8), justify="left",
                 wraplength=int(360 * s)).grid(row=row, column=0, columnspan=2,
                                      sticky="w", pady=(2, 0))

        # ---- actions ----
        lf3 = ttk.Labelframe(parent, text=" 操作 ", style="Panel.TLabelframe")
        lf3.pack(fill="x", padx=8, pady=5)
        act = tk.Frame(lf3, bg=COL_PANEL)
        act.pack(fill="x", padx=10, pady=(8, 10))
        act.columnconfigure(0, weight=1)
        act.columnconfigure(1, weight=1)

        self.apply_btn = ttk.Button(act, text="一键设置", style="Accent.TButton",
                                    command=self._apply_clicked)
        self.apply_btn.grid(row=0, column=0, columnspan=2, sticky="ew")

        self.start_btn = ttk.Button(act, text="开始采集", style="Go.TButton",
                                    command=self._start_clicked)
        self.start_btn.grid(row=1, column=0, sticky="ew", pady=(8, 0), padx=(0, 4))
        self.stop_btn = ttk.Button(act, text="停止采集", command=self._stop_clicked)
        self.stop_btn.grid(row=1, column=1, sticky="ew", pady=(8, 0), padx=(4, 0))

        self.read_btn = ttk.Button(act, text="读取当前寄存器", style="Small.TButton",
                                   command=self._read_clicked)
        self.read_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(7, 0))

        self.settings_var = tk.StringVar(value="尚未初始化")
        self.settings_lbl = tk.Label(act, textvariable=self.settings_var,
                                     bg=COL_PANEL, fg=COL_MUTED,
                                     font=(FONT, 8), anchor="w", justify="left",
                                     wraplength=int(360 * s))
        self.settings_lbl.grid(row=3, column=0, columnspan=2, sticky="w",
                               pady=(7, 0))

        # ---- recording ----
        lf4 = ttk.Labelframe(parent, text=" 数据记录 ", style="Panel.TLabelframe")
        lf4.pack(fill="x", padx=8, pady=(5, 8))
        rec = tk.Frame(lf4, bg=COL_PANEL)
        rec.pack(fill="x", padx=10, pady=(6, 9))
        rec.columnconfigure(0, weight=1)

        self.record_var = tk.BooleanVar(value=True)
        self.record_chk = ttk.Checkbutton(
            rec, text="保存原始码流 (.bin) + 解码数据 (.csv)",
            variable=self.record_var)
        self.record_chk.grid(row=0, column=0, columnspan=2, sticky="w")

        dirrow = tk.Frame(rec, bg=COL_PANEL)
        dirrow.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        dirrow.columnconfigure(0, weight=1)
        self.data_dir_var = tk.StringVar(value=default_data_dir())
        self.dir_entry = ttk.Entry(dirrow, textvariable=self.data_dir_var)
        self.dir_entry.grid(row=0, column=0, sticky="ew")
        self.dir_btn = ttk.Button(dirrow, text="选择", style="Small.TButton",
                                  command=self._choose_dir)
        self.dir_btn.grid(row=0, column=1, padx=(5, 0))

        self.open_dir_btn = ttk.Button(rec, text="打开数据文件夹",
                                       style="Small.TButton",
                                       command=self._open_data_dir)
        self.open_dir_btn.grid(row=2, column=0, sticky="w", pady=(7, 0))

        self.files_var = tk.StringVar(value="")
        tk.Label(rec, textvariable=self.files_var, bg=COL_PANEL,
                 fg=COL_MUTED, font=(MONO, 7), justify="left", anchor="w",
                 wraplength=int(370 * s)).grid(row=3, column=0, columnspan=2,
                                      sticky="w", pady=(6, 0))

        tk.Label(parent, text=f"v{APP_VERSION} · 参考 TI SBAS752B",
                 bg=COL_PANEL, fg="#5d6472", font=(FONT, 7)).pack(
                     side="bottom", pady=(2, 6))

        # live wiring
        for w in (self.mux_combo, self.gain_combo, self.mode_combo,
                  self.dr_combo, self.cm_combo, self.vref_combo,
                  self.integ_combo):
            w.bind("<<ComboboxSelected>>", self._on_config_widget_change)
        # A mouse-wheel event must not silently change a selection.
        for w in (self.port_combo, self.baud_combo, self.mux_combo,
                  self.gain_combo, self.mode_combo, self.dr_combo,
                  self.cm_combo, self.vref_combo, self.integ_combo):
            w.bind("<MouseWheel>", self._combo_wheel)

    def _build_right(self, parent):
        s = self.ui_scale
        # ---- range calculator ----
        self.calc_lf = ttk.Labelframe(
            parent, text=" 测量范围计算 · AIN0 − AIN1（AIN1 = 信号偏置） ",
            style="Panel.TLabelframe")
        self.calc_lf.pack(fill="x")
        body = tk.Frame(self.calc_lf, bg=COL_PANEL)
        body.pack(fill="x", padx=12, pady=(8, 10))

        inputs = tk.Frame(body, bg=COL_PANEL)
        inputs.pack(fill="x")
        self.refp_var = tk.StringVar(value="2.048")
        self.ain1_var = tk.StringVar(value="1.65")
        self.avdd_var = tk.StringVar(value="3.3")
        for col, (text, var, tip) in enumerate([
            ("REFP 参考电压 (V)", self.refp_var, "外部参考时生效"),
            ("AIN1 偏置电压 (V)", self.ain1_var, "信号输入端 AIN0 的偏置"),
            ("AVDD 模拟电源 (V)", self.avdd_var, "AVSS 视为 0 V"),
        ]):
            f = tk.Frame(inputs, bg=COL_PANEL)
            f.grid(row=0, column=col, sticky="w", padx=(0, 18))
            tk.Label(f, text=text, bg=COL_PANEL, fg=COL_MUTED,
                     font=(FONT, 8)).pack(anchor="w")
            e = ttk.Entry(f, textvariable=var, width=13)
            e.pack(anchor="w", pady=(2, 0))
            tk.Label(f, text=tip, bg=COL_PANEL, fg="#5d6472",
                     font=(FONT, 7)).pack(anchor="w", pady=(1, 0))

        for var in (self.refp_var, self.ain1_var, self.avdd_var):
            var.trace_add("write", lambda *a: self._update_range())

        self.range_line1 = tk.StringVar(value="")
        self.range_line2 = tk.StringVar(value="")
        self.range_line3 = tk.StringVar(value="")
        tk.Label(body, textvariable=self.range_line1, bg=COL_PANEL, fg=COL_FG,
                 font=(MONO, 9), anchor="w", justify="left",
                 wraplength=int(830 * s)).pack(fill="x", pady=(8, 3))
        self.range_lbl2 = tk.Label(body, textvariable=self.range_line2,
                                   bg=COL_PANEL, fg=COL_GREEN,
                                   font=(FONT, 9, "bold"), anchor="w",
                                   justify="left", wraplength=int(830 * s))
        self.range_lbl2.pack(fill="x")
        tk.Label(body, textvariable=self.range_line3, bg=COL_PANEL,
                 fg=COL_MUTED, font=(FONT, 8), anchor="w", justify="left",
                 wraplength=int(830 * s)).pack(fill="x", pady=(2, 0))

        self.range_bar = tk.Canvas(body, height=int(58 * s), bg="#1a1c23",
                                   highlightthickness=0, bd=0)
        self.range_bar.pack(fill="x", pady=(6, 0))
        self.range_bar.bind("<Configure>", lambda e: self._draw_range_bar())

        # ---- waveform ----
        self.plot_lf = ttk.Labelframe(parent, text=" 实时波形 ",
                                      style="Panel.TLabelframe")
        self.plot_lf.pack(fill="both", expand=True, pady=(8, 8))

        tb = tk.Frame(self.plot_lf, bg=COL_PANEL)
        tb.pack(fill="x", padx=10, pady=(6, 2))
        self.plot_mode_var = tk.StringVar(value="abs")
        ttk.Radiobutton(tb, text="AIN0 绝对电压", value="abs",
                        variable=self.plot_mode_var,
                        command=self._on_display_change).pack(side="left")
        ttk.Radiobutton(tb, text="差分 VIN", value="diff",
                        variable=self.plot_mode_var,
                        command=self._on_display_change).pack(side="left", padx=(8, 0))

        tk.Label(tb, text="时间窗", bg=COL_PANEL, fg=COL_MUTED,
                 font=(FONT, 8)).pack(side="left", padx=(18, 5))
        self.timebase_var = tk.StringVar(value="2 s")
        self.timebase_combo = ttk.Combobox(
            tb, textvariable=self.timebase_var, state="readonly", width=7,
            values=["0.5 s", "1 s", "2 s", "5 s", "10 s", "30 s", "全部"])
        self.timebase_combo.pack(side="left")
        self.timebase_combo.bind("<<ComboboxSelected>>",
                                 lambda e: self._on_display_change())
        self.timebase_combo.bind("<MouseWheel>", self._block_wheel)

        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(tb, text="自动 Y", variable=self.autoscale_var,
                        command=self._on_display_change).pack(side="left", padx=(14, 0))

        self.pause_btn = ttk.Button(tb, text="暂停", style="Small.TButton",
                                    command=self._toggle_pause)
        self.pause_btn.pack(side="right")
        ttk.Button(tb, text="清空", style="Small.TButton",
                   command=self._clear_clicked).pack(side="right", padx=(0, 5))

        self.stats_var = tk.StringVar(value="样本 0 · 速率 — · 最新 —")
        tk.Label(self.plot_lf, textvariable=self.stats_var, bg=COL_PANEL,
                 fg=COL_MUTED, font=(MONO, 8), anchor="w").pack(
                     fill="x", padx=12, pady=(2, 4))

        self.canvas = tk.Canvas(self.plot_lf, bg=COL_PLOT_BG,
                                highlightthickness=0, bd=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0, 9))
        self.canvas.bind("<Configure>", self._on_canvas_cfg)

        # ---- log ----
        self.log_lf = ttk.Labelframe(parent, text=" 事件日志 ",
                                     style="Panel.TLabelframe")
        self.log_lf.pack(fill="x")
        logbody = tk.Frame(self.log_lf, bg=COL_PANEL)
        logbody.pack(fill="both", expand=True, padx=8, pady=(5, 7))
        self.log_text = tk.Text(logbody, height=5, bg="#14161b", fg="#c7ccd6",
                                insertbackground=COL_FG, relief="flat",
                                bd=0, wrap="word", font=(MONO, 8),
                                padx=8, pady=4, state="disabled")
        lsb = ttk.Scrollbar(logbody, orient="vertical",
                            command=self.log_text.yview,
                            style="Vertical.TScrollbar")
        self.log_text.configure(yscrollcommand=lsb.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        lsb.pack(side="right", fill="y")

    def _log_colors(self):
        self.log_text.tag_configure("info", foreground="#9aa3b2")
        self.log_text.tag_configure("ok", foreground="#5ee6a0")
        self.log_text.tag_configure("warn", foreground=COL_WARN)
        self.log_text.tag_configure("error", foreground="#ff7b7b")
        self.log_text.tag_configure("time", foreground="#5d6472")

    # -- small UI callbacks -------------------------------------------------
    def _on_canvas_cfg(self, _=None):
        self._plot_dirty = True

    def _on_display_change(self):
        self._plot_dirty = True

    def _combo_wheel(self, event):
        """Scroll the sidebar instead of changing the combobox value."""
        try:
            self.scroll.canvas.yview_scroll(int(-event.delta / 120) * 2, "units")
        except Exception:
            pass
        return "break"

    def _block_wheel(self, event):
        return "break"

    def _bind_wheel_tree(self, widget):
        widget.bind("<MouseWheel>", self._combo_wheel)
        for child in widget.winfo_children():
            self._bind_wheel_tree(child)

    def _toggle_pause(self):
        self.plot_paused = not self.plot_paused
        self.pause_btn.configure(text="继续" if self.plot_paused else "暂停")
        self._plot_dirty = True

    def _clear_clicked(self):
        self.worker.clear_stats()
        self._plot_dirty = True

    def _choose_dir(self):
        d = filedialog.askdirectory(initialdir=self.data_dir_var.get() or default_data_dir(),
                                    title="选择数据保存文件夹")
        if d:
            self.data_dir_var.set(d)

    def _open_data_dir(self):
        d = self.data_dir_var.get() or default_data_dir()
        try:
            Path(d).mkdir(parents=True, exist_ok=True)
            os.startfile(d)  # type: ignore[attr-defined]
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"无法打开文件夹：\n{exc}")

    def _refresh_ports(self):
        current = self._port_map.get(self.port_var.get(), "")
        self._port_map = {}
        labels = []
        if list_ports is not None:
            try:
                for p in sorted(list_ports.comports(), key=lambda x: x.device):
                    label = f"{p.device}  —  {p.description}"
                    self._port_map[label] = p.device
                    labels.append(label)
            except Exception:
                pass
        self._port_map[SIM_LABEL] = SIM_PORT
        labels.append(SIM_LABEL)
        self.port_combo.configure(values=labels)

        saved_label = self.settings.get("port_label", "")
        saved_port = self.settings.get("port", "")
        target = ""
        if current:
            for label, dev in self._port_map.items():
                if dev == current:
                    target = label
                    break
        if not target:
            for label, dev in self._port_map.items():
                if dev == saved_port or (saved_port and label.startswith(saved_port)):
                    target = label
                    break
        if not target and saved_label in self._port_map:
            target = saved_label
        if not target:
            real = [l for l in labels if self._port_map.get(l) != SIM_PORT]
            target = real[0] if real else SIM_LABEL
        self.port_var.set(target)
        self._log("info", f"发现 {len(labels) - 1} 个串口设备")
        self._update_conn_text()

    def _update_conn_text(self):
        st = self.worker.get_state()
        if not st.get("connected"):
            self.conn_var.set("未连接")

    # -- log ---------------------------------------------------------------
    def _log(self, level, text):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] ", ("time",))
        self.log_text.insert("end", text + "\n", (level,))
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 700:
            self.log_text.delete("1.0", "81.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # -- UI -> config ------------------------------------------------------
    def _refresh_dr_values(self, code=3):
        turbo = value_for(MODE_CHOICES, self.mode_var.get(), False)
        rates = DR_TABLE[bool(turbo)]
        code = max(0, min(6, int(code)))
        self._dr_code = code
        self.dr_combo.configure(values=[f"{r} SPS" for r in rates])
        self.dr_var.set(f"{rates[code]} SPS")

    def _ui_config(self) -> DeviceConfig:
        mux = value_for(MUX_CHOICES, self.mux_var.get(), 0)
        gain = value_for(GAIN_CHOICES, self.gain_var.get(), 16)
        turbo = bool(value_for(MODE_CHOICES, self.mode_var.get(), False))
        continuous = bool(value_for(CM_CHOICES, self.cm_var.get(), True))
        vref = int(value_for(VREF_CHOICES, self.vref_var.get(), 0))
        dcnt = bool(self.dcnt_var.get())
        integ = int(value_for(INTEG_CHOICES, self.integ_var.get(), 0))
        bypass = bool(self.bypass_var.get())
        if mux in (8, 9, 10, 11):
            gain = min(gain, 4)
            bypass = True
        elif mux in (12, 13, 14):
            bypass = True
        if gain > 4:
            bypass = False
        # DR selection follows the combo box
        try:
            rates = DR_TABLE[turbo]
            idx = [f"{r} SPS" for r in rates].index(self.dr_var.get())
            dr_code = idx
        except Exception:
            dr_code = self._dr_code
        return DeviceConfig(mux=mux, gain=gain, dr_code=dr_code, turbo=turbo,
                            continuous=continuous, vref_code=vref,
                            pga_bypass=bypass, dcnt=dcnt, integrity=integ,
                            auto=True)

    def _on_config_widget_change(self, _=None):
        turbo = bool(value_for(MODE_CHOICES, self.mode_var.get(), False))
        rates = DR_TABLE[turbo]
        labels = [f"{r} SPS" for r in rates]
        if self.dr_var.get() in labels:
            self._dr_code = labels.index(self.dr_var.get())
        self._refresh_dr_values(max(0, min(6, self._dr_code)))
        mux = value_for(MUX_CHOICES, self.mux_var.get(), 0)
        gain = value_for(GAIN_CHOICES, self.gain_var.get(), 16)

        if mux in (8, 9, 10, 11):
            self.gain_combo.configure(values=[l for v, l in GAIN_CHOICES if v <= 4])
            if gain > 4:
                gain = 4
                self.gain_var.set(label_for(GAIN_CHOICES, 4))
            self.bypass_var.set(True)
            self.bypass_chk.state(["disabled"])
            self.cfg_hint_var.set("单端输入（AINN = AVSS）：PGA 自动旁路，增益限制为 1/2/4。")
        elif mux in (12, 13, 14):
            self.bypass_var.set(True)
            self.bypass_chk.state(["disabled"])
            self.cfg_hint_var.set("监测 / 偏移校准模式：PGA 自动旁路，增益设置无效。")
        else:
            self.gain_combo.configure(values=[l for v, l in GAIN_CHOICES])
            if gain > 4:
                self.bypass_var.set(False)
                self.bypass_chk.state(["disabled"])
                self.cfg_hint_var.set("增益 > ×4 时 PGA 自动使能，无法旁路。")
            else:
                self.bypass_chk.state(["!disabled"])
                self.cfg_hint_var.set("增益 ≤ ×4 时可旁路 PGA；旁路后绝对输入范围扩展到 AVSS−0.1 V ~ AVDD+0.1 V。")

        cfg = self._ui_config()
        need_bps = cfg.frame_len * 10 * cfg.data_rate
        try:
            baud = int(self.baud_var.get())
        except Exception:
            baud = 115200
        warn = "   ⚠ 超过当前波特率" if need_bps > baud * 0.85 else ""
        self.frame_hint_var.set(
            "数据帧：" + cfg.frame_description() +
            f" · 需 ≥ {need_bps / 1000:.1f} kbit/s{warn}")
        self._update_range()

    def _range_values(self):
        refp = float(self.refp_var.get().strip())
        ain1 = float(self.ain1_var.get().strip())
        avdd = float(self.avdd_var.get().strip())
        if not all(math.isfinite(v) for v in (refp, ain1, avdd)):
            raise ValueError("输入必须是有限数值")
        if not (refp > 0 and avdd > 0):
            raise ValueError("REFP / AVDD 必须大于 0")
        return refp, ain1, avdd

    def _update_range(self):
        cfg = self._ui_config()
        try:
            refp, ain1, avdd = self._range_values()
        except Exception:
            self.range_line1.set("请输入有效的 REFP / AIN1 / AVDD（单位 V）")
            self.range_line2.set("输入无效")
            self.range_lbl2.configure(fg=COL_ERR)
            self.range_line3.set("")
            self.range_info = None
            self._draw_range_bar()
            return
        info = compute_range(cfg, refp, ain1, avdd)
        self.range_info = info
        self._last_valid = (cfg, refp, ain1, avdd, info)
        self.range_line1.set(
            f"VREF {info.vref:.5g} V   ·   增益 ×{info.gain}   ·   "
            f"满量程 ±{fmt_eng(info.fs)}   ·   1 LSB = {fmt_eng(info.lsb)}   ·   "
            f"{'PGA 旁路' if info.bypass else 'PGA 使能'}")
        if info.valid:
            down = ain1 - info.usable_lo
            up = info.usable_hi - ain1
            self.range_line2.set(
                f"AIN0 可用范围  {info.usable_lo:.4f} V ~ {info.usable_hi:.4f} V"
                f"      （相对 AIN1 偏置：−{fmt_eng(down)} / +{fmt_eng(up)}）")
            self.range_lbl2.configure(fg=COL_GREEN)
        else:
            self.range_line2.set("范围不可用：" + info.message)
            self.range_lbl2.configure(fg=COL_ERR)
        note = ""
        if cfg.mux == 3:
            note = ("当前通道为 AIN1−AIN0（反向），波形按 AIN0 = AIN1 − VIN 显示。 ")
        elif cfg.mux != 0:
            note = (f"当前通道为 {label_for(MUX_CHOICES, cfg.mux)}，"
                    "量程按 AIN0−AIN1 估算；波形电压换算仅供参考。 ")
        self.range_line3.set(
            note + f"PGA 绝对输入窗口 {info.abs_lo:.3f} V ~ {info.abs_hi:.3f} V"
            f"（AIN0 与 AIN1 均需在此窗口内）；LSB = 2·VREF/(增益·2^24)。")
        self._draw_range_bar()
        self._plot_dirty = True

    # -- range bar ---------------------------------------------------------
    def _draw_range_bar(self):
        c = self.range_bar
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 60 or h < 20:
            return
        info = self.range_info
        try:
            _, ain1, avdd = self._range_values()
        except Exception:
            c.create_text(w // 2, h // 2, text="等待有效输入…", fill=COL_MUTED,
                          font=(FONT, 8))
            return
        if info is None:
            c.create_text(w // 2, h // 2, text="等待有效配置…", fill=COL_MUTED,
                          font=(FONT, 8))
            return

        s = self.ui_scale
        pad = int(12 * s)
        axis_lo = min(0.0, info.abs_lo, info.usable_lo, ain1) - 0.03 * max(avdd, 0.1)
        axis_hi = max(avdd, info.abs_hi, info.usable_hi, ain1) + 0.03 * max(avdd, 0.1)
        span = max(1e-9, axis_hi - axis_lo)

        def X(v):
            return pad + (v - axis_lo) / span * (w - 2 * pad)

        y0, y1 = int(8 * s), h - int(12 * s)
        c.create_rectangle(pad, y0, w - pad, y1, outline="", fill="#20232b")
        # absolute window
        lo, hi = X(info.abs_lo), X(info.abs_hi)
        c.create_rectangle(lo, y0 + int(6 * s), hi, y0 + int(14 * s),
                           outline="", fill="#3c4354")
        # usable window
        if info.usable_hi > info.usable_lo:
            lo, hi = X(info.usable_lo), X(info.usable_hi)
            color = "#1f7a5a" if info.valid else "#7a3b3b"
            c.create_rectangle(lo, y0 + int(20 * s), hi, y0 + int(29 * s),
                               outline="", fill=color)
            c.create_text((lo + hi) / 2, y0 + int(32.5 * s),
                          text="AIN0 可用", anchor="n",
                          fill="#8fe3c4" if info.valid else "#f3a3a3",
                          font=(FONT, 7))

        # AIN1 marker
        x = X(ain1)
        c.create_line(x, int(3 * s), x, y1 - 1, fill=COL_WARN, dash=(3, 2))
        c.create_polygon(x - int(5 * s), int(2 * s), x + int(5 * s),
                         int(2 * s), x, int(9 * s), fill=COL_WARN, outline="")
        c.create_text(x, y0 + int(2 * s), text="AIN1", anchor="s",
                      fill=COL_WARN, font=(FONT, 7, "bold"))
        c.create_text(pad, h - int(2 * s), text="0 V", anchor="sw",
                      fill="#5d6472", font=(FONT, 7))
        c.create_text(w - pad, h - int(2 * s), text=f"{avdd:.2f} V",
                      anchor="se", fill="#5d6472", font=(FONT, 7))

    # -- buttons -----------------------------------------------------------
    def _toggle_connect(self):
        st = self.worker.get_state()
        if st.get("connected"):
            self.worker.disconnect()
            return
        label = self.port_var.get()
        dev = self._port_map.get(label)
        if not dev:
            messagebox.showwarning(APP_NAME, "请选择串口")
            return
        try:
            baud = int(self.baud_var.get())
        except Exception:
            messagebox.showwarning(APP_NAME, "波特率无效")
            return
        self.worker.connect(dev, baud)
        self.settings["port"] = dev
        self.settings["port_label"] = label
        self.settings["baud"] = baud

    def _apply_clicked(self):
        if not self.worker.get_state().get("connected"):
            messagebox.showwarning(APP_NAME, "请先连接串口")
            return
        try:
            cfg = self._ui_config()
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"配置无效：{exc}")
            return
        self.worker.apply_config(cfg)
        self._log("info", "正在发送初始化设置…")

    def _start_clicked(self):
        st = self.worker.get_state()
        if not st.get("connected"):
            messagebox.showwarning(APP_NAME, "请先连接串口")
            return
        if st.get("streaming"):
            return
        try:
            cfg = self._ui_config()
            refp, ain1, avdd = self._range_values()
            info = compute_range(cfg, refp, ain1, avdd)
            if refp <= 0:
                raise ValueError("REFP 必须大于 0")
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"输入参数无效：\n{exc}")
            return

        if not st.get("sim"):
            try:
                baud_now = int(st.get("baud") or self.baud_var.get())
            except Exception:
                baud_now = 115200
            need_bps = cfg.frame_len * 10 * cfg.data_rate
            if need_bps > baud_now * 0.85:
                self._log("warn", f"带宽警告：{cfg.frame_len} B/帧 × {cfg.data_rate} SPS "
                                   f"约需 {need_bps/1000:.1f} kbit/s，接近/超过 "
                                   f"{baud_now} baud，可能出现丢帧。")
        folder = self.data_dir_var.get().strip() or default_data_dir()
        record = bool(self.record_var.get())
        if record:
            try:
                Path(folder).mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                messagebox.showerror(APP_NAME, f"无法创建数据文件夹：\n{exc}")
                return
        if not info.valid:
            self._log("warn", "测量范围警告：" + info.message)
            if not messagebox.askyesno(APP_NAME,
                                       "当前参数下测量范围不可用：\n\n" +
                                       (info.message or "未知原因") +
                                       "\n\n仍要开始采集吗？"):
                return
        meta = {
            "tool": APP_NAME, "version": APP_VERSION,
            "created": datetime.now().isoformat(timespec="seconds"),
            "port": st.get("port", ""), "baud": st.get("baud", 0),
            "device_config": {
                "mux": cfg.mux, "gain": cfg.gain, "data_rate": cfg.data_rate,
                "turbo": cfg.turbo, "continuous": cfg.continuous,
                "vref_code": cfg.vref_code, "pga_bypass": cfg.pga_bypass,
                "dcnt": cfg.dcnt, "integrity": cfg.integrity, "auto": cfg.auto,
                "registers_hex": [f"0x{v:02X}" for v in cfg.registers()],
                "description": cfg.description(),
            },
            "frame": {"length": cfg.frame_len,
                      "description": cfg.frame_description()},
            "range": {
                "refp_V": refp, "ain1_V": ain1, "avdd_V": avdd,
                "vref_V": info.vref, "gain": info.gain,
                "full_scale_V": info.fs, "lsb_V": info.lsb,
                "abs_input_V": [info.abs_lo, info.abs_hi],
                "usable_ain0_V": [info.usable_lo, info.usable_hi],
                "valid": info.valid, "message": info.message,
            },
            "note": ("CSV 中的 vin = 原始码 × LSB；AIN0−AIN1 通道下 "
                     "ain0 = vin + AIN1；反向通道 AIN1−AIN0 下 ain0 = AIN1 − vin；"
                     "其他通道 ain0 留空。"),
        }
        self.worker.start_acq(cfg, record, folder, meta)
        self._log("info", f"开始采集请求 · 记录={'开启' if record else '关闭'}")

    def _stop_clicked(self):
        self.worker.stop_acq()

    def _read_clicked(self):
        self.worker.read_config()

    # -- worker state ------------------------------------------------------
    def _refresh_state(self, st):
        connected = st.get("connected", False)
        streaming = st.get("streaming", False)
        self.connect_btn.configure(text="断开连接" if connected else "连接串口")
        self.start_btn.configure(text="采集中…" if streaming else "开始采集")
        if connected:
            self.port_combo.configure(state="disabled")
            self.baud_combo.configure(state="disabled")
        else:
            self.port_combo.configure(state="readonly")
            self.baud_combo.configure(state="readonly")

        for btn, enabled in ((self.apply_btn, connected and not streaming),
                             (self.start_btn, connected and not streaming),
                             (self.stop_btn, streaming),
                             (self.read_btn, connected and not streaming)):
            if enabled:
                btn.state(["!disabled"])
            else:
                btn.state(["disabled"])

        # Lock the device configuration while a stream is running so the
        # displayed waveform always matches the registers in the ADC.
        for w in (self.mux_combo, self.gain_combo, self.mode_combo,
                  self.dr_combo, self.cm_combo, self.vref_combo,
                  self.integ_combo):
            w.configure(state="disabled" if streaming else "readonly")
        for w in (self.bypass_chk, self.dcnt_chk):
            w.state(["disabled"] if streaming else ["!disabled"])
        if not streaming:
            self._on_config_widget_change()

        if streaming:
            self.record_chk.state(["disabled"])
            self.dir_entry.configure(state="disabled")
            self.dir_btn.state(["disabled"])
        else:
            self.record_chk.state(["!disabled"])
            self.dir_entry.configure(state="normal")
            self.dir_btn.state(["!disabled"])

        if st.get("sim"):
            self.conn_var.set("模拟器 · 运行中" if streaming else "模拟器 · 已连接")
        elif connected:
            self.conn_var.set(f"{st.get('port')} · {st.get('baud')} 8-N-1" +
                              (" · 采集中" if streaming else ""))
        else:
            self.conn_var.set("未连接")

        color = COL_GREEN if streaming else (COL_ACCENT if connected else "#555b6a")
        self.dot.itemconfigure(self.dot_id, fill=color)

        self.settings_var.set(st.get("settings_status", "尚未初始化"))
        self.settings_lbl.configure(fg=(COL_GREEN if st.get("settings_ok")
                                        else COL_MUTED))
        raw = st.get("raw_path") or ""
        csvp = st.get("csv_path") or ""
        if raw:
            self.files_var.set("RAW " + Path(raw).name + "\nCSV " + Path(csvp).name)
        elif not streaming:
            self.files_var.set("")
        if st.get("message"):
            self._update_conn_text()

    # -- tick --------------------------------------------------------------
    def _tick(self):
        try:
            while True:
                kind, payload = self.worker.events.get_nowait()
                if kind == "log":
                    level, text = payload
                    self._log(level, text)
        except queue.Empty:
            pass

        st = self.worker.get_state()
        if st != self._last_state:
            self._last_state = st
            self._refresh_state(st)
            if self.demo_pending and st.get("connected") and not st.get("streaming"):
                self.demo_pending = False
                self.record_var.set(False)
                self.root.after(200, self._start_clicked)

        now = time.monotonic()
        if now - self._last_draw >= 0.09:
            self._last_draw = now
            self._update_view()
        self.root.after(60, self._tick)

    def _update_view(self):
        quick = self.worker.stats.quick()
        self._update_stats(quick)
        streaming = self.worker.get_state().get("streaming", False)
        if self.plot_paused:
            if self._plot_dirty:
                self._draw_plot(quick)
                self._plot_dirty = False
            return
        if streaming or self._plot_dirty:
            self._draw_plot(quick)
            self._plot_dirty = False

    def _current_offset(self):
        mode = self.plot_mode_var.get()
        if mode == "abs":
            try:
                return float(self.ain1_var.get())
            except Exception:
                return 0.0
        return 0.0

    def _current_lsb(self):
        info = self.range_info
        if info and info.lsb > 0:
            return info.lsb
        if self._last_valid:
            try:
                return self._last_valid[4].lsb
            except Exception:
                pass
        return 2.4414e-7

    def _display_params(self):
        """Return (lsb, sign, offset, show_ain0, axis_label, formula, ref_value).

        AIN0 absolute display is only meaningful for the AIN0-AIN1 channel
        (and the reversed AIN1-AIN0 channel).  Other multiplexer settings fall
        back to the selected differential voltage.
        """
        lsb = self._current_lsb()
        cfg = self._ui_config()
        try:
            ain1 = float(self.ain1_var.get())
        except Exception:
            ain1 = 0.0
        want_abs = self.plot_mode_var.get() == "abs"
        if want_abs and cfg.mux == 0:
            return lsb, 1.0, ain1, True, "AIN0", "AIN0 = VIN + AIN1", ain1
        if want_abs and cfg.mux == 3:
            return lsb, -1.0, ain1, True, "AIN0", "AIN0 = AIN1 − VIN", ain1
        if want_abs:
            return (lsb, 1.0, 0.0, False, "VIN",
                    "所选通道差分（非 AIN0−AIN1）", 0.0)
        return lsb, 1.0, 0.0, False, "VIN", "VIN = AINP − AINN", 0.0

    def _update_stats(self, q):
        lsb, sign, off, _mode_abs, _lbl, _f, _ref = self._display_params()
        n = q["total"]
        rate = q["rate"]
        parts = []
        parts.append(f"样本 {n:,}")
        parts.append(f"速率 {rate:.1f} SPS" if rate > 0 else "速率 —")
        if q["max"] is not None:
            last = sign * q["last"] * lsb + off
            v_a = sign * q["min"] * lsb + off
            v_b = sign * q["max"] * lsb + off
            lo, hi = min(v_a, v_b), max(v_a, v_b)
            pp = (q["max"] - q["min"]) * lsb
            mean = sign * (q["sum"] / n) * lsb + off if n else 0.0
            parts.append(f"最新 {fmt_eng(last, 5)}")
            parts.append(f"最小 {fmt_eng(lo, 5)}")
            parts.append(f"最大 {fmt_eng(hi, 5)}")
            parts.append(f"峰峰 {fmt_eng(pp, 4)}")
            parts.append(f"均值 {fmt_eng(mean, 5)}")
        else:
            parts.append("最新 —")
        parts.append(f"丢帧 {q['dropped']}")
        if q["errors"]:
            parts.append(f"错误 {q['errors']}")
        if q["resyncs"]:
            parts.append(f"重同步 {q['resyncs']}")
        self.stats_var.set("  ·  ".join(parts))

    # -- plotting ----------------------------------------------------------
    def _nice_step(self, span, target=5):
        if span <= 0:
            return 1.0
        raw = span / target
        exp = math.floor(math.log10(raw))
        base = 10 ** exp
        for m in (1, 2, 2.5, 5, 10):
            if raw <= m * base:
                return m * base
        return 10 * base

    def _pick_unit(self, ymin, ymax):
        span = max(1e-15, ymax - ymin)
        center = (ymax + ymin) / 2.0
        if abs(center) >= 1.0 or span >= 1.0:
            return 1.0, "V"
        if span >= 5e-3:
            return 1e3, "mV"
        if span >= 5e-6:
            return 1e6, "µV"
        return 1e9, "nV"

    def _draw_plot(self, quick=None):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 120 or h < 100:
            return
        if quick is None:
            quick = self.worker.stats.quick()

        rate = quick.get("rate") or 0.0
        if rate <= 0:
            try:
                rate = float(self._ui_config().data_rate)
            except Exception:
                rate = 1.0
        tb = self.timebase_var.get()
        buffered = quick.get("buffered", 0)
        if tb == "全部":
            window = buffered
        else:
            try:
                secs = float(tb.split()[0])
            except Exception:
                secs = 2.0
            window = int(rate * secs)
        window = max(0, min(window, buffered))

        s = self.ui_scale
        ml, mr = int(66 * s), int(16 * s)
        mt, mb = int(26 * s), int(34 * s)
        pw, ph = w - ml - mr, h - mt - mb
        if pw < 20 or ph < 20:
            return
        c.create_rectangle(ml, mt, ml + pw, mt + ph, outline=COL_BORDER)

        if window < 2:
            c.create_text(ml + pw / 2, mt + ph / 2,
                          text="等待数据…\n连接串口后点击「开始采集」",
                          fill="#5d6472", font=(FONT, 10), justify="center")
            return

        snap = self.worker.stats.snapshot(window, max_points=5000)
        codes = snap["codes"]
        if len(codes) < 2:
            c.create_text(ml + pw / 2, mt + ph / 2, text="等待数据…",
                          fill="#5d6472", font=(FONT, 10))
            return

        lsb, sign, off, mode_abs, axis_label, formula, ref =             self._display_params()
        vals = [sign * code * lsb + off for code in codes]
        vmin, vmax = min(vals), max(vals)

        if self.autoscale_var.get():
            span = vmax - vmin
            if span < 30 * lsb:
                center = (vmax + vmin) / 2
                vmin, vmax = center - 15 * lsb, center + 15 * lsb
                span = vmax - vmin
            pad = span * 0.08 if span > 0 else lsb
            ymin, ymax = vmin - pad, vmax + pad
        else:
            info = self.range_info
            fs = info.fs if info else 1.0
            if mode_abs:
                ymin, ymax = off - fs, off + fs
            else:
                ymin, ymax = -fs, fs

        if ymax - ymin <= 0:
            ymax = ymin + 1e-9
        scale, unit = self._pick_unit(ymin, ymax)

        def Y(v):
            return mt + (ymax - v) / (ymax - ymin) * ph

        # grid + y labels
        step = self._nice_step(ymax - ymin, 5)
        start = math.ceil(ymin / step) * step
        v = start
        tick = 0
        while v <= ymax + step * 0.01 and tick < 20:
            y = Y(v)
            c.create_line(ml, y, ml + pw, y, fill=COL_GRID)
            sv = v * scale
            if abs(step * scale) >= 100:
                txt = f"{sv:.0f}"
            elif abs(step * scale) >= 10:
                txt = f"{sv:.1f}"
            elif abs(step * scale) >= 1:
                txt = f"{sv:.2f}"
            elif abs(step * scale) >= 0.1:
                txt = f"{sv:.3f}"
            else:
                txt = f"{sv:.4f}"
            c.create_text(ml - int(8 * s), y, text=txt, anchor="e",
                          fill="#6d7585", font=(MONO, 8))
            v += step
            tick += 1

        # x grid / time labels
        nx = 6
        t_span = (window - 1) / rate if rate > 0 else 1.0
        for i in range(nx + 1):
            x = ml + pw * i / nx
            c.create_line(x, mt, x, mt + ph, fill=COL_GRID)
            t = -t_span + t_span * i / nx
            if t_span >= 10:
                lbl = f"{t:.0f}s"
            elif t_span >= 1:
                lbl = f"{t:.1f}s"
            else:
                lbl = f"{t:.3f}s"
            c.create_text(x, mt + ph + int(10 * s), text=lbl, anchor="n",
                          fill="#6d7585", font=(MONO, 8))

        # axis titles
        c.create_text(ml, mt - int(13 * s),
                      text=f"{axis_label} ({unit})",
                      anchor="w", fill="#7d8698", font=(FONT, 8, "bold"))
        c.create_text(ml + pw, mt - int(13 * s),
                      text=formula,
                      anchor="e", fill="#5d6472", font=(FONT, 8))

        # reference line
        if ymin <= ref <= ymax:
            y = Y(ref)
            c.create_line(ml, y, ml + pw, y, fill="#59616f", dash=(4, 4))
            label = (f"AIN1 = {fmt_eng(ref, 5)}" if mode_abs else "0 V")
            c.create_text(ml + int(6 * s), y - int(8 * s), text=label,
                          anchor="w", fill="#8891a1", font=(FONT, 7))

        # trace
        n = len(vals)
        xs = [ml + pw * i / (n - 1) for i in range(n)]
        ys = [Y(v) for v in vals]
        pts = []
        for x, y in zip(xs, ys):
            pts.extend((x, y))
        c.create_line(*pts, fill=COL_TRACE,
                      width=max(1.0, 1.4 * s), joinstyle="round")

        # latest point
        lx, ly = xs[-1], ys[-1]
        r = max(2, int(3 * s))
        c.create_oval(lx - r, ly - r, lx + r, ly + r, fill=COL_WARN, outline="")

        # meta
        c.create_text(ml + int(6 * s), mt + int(8 * s),
                      text=f"{window} 样本 · {rate:.1f} SPS · 增益 ×{self._ui_config().gain}",
                      anchor="nw", fill="#5d6472", font=(MONO, 8))
        if self.plot_paused:
            c.create_text(ml + pw - int(6 * s), mt + int(8 * s),
                          text="显示已暂停", anchor="ne",
                          fill=COL_WARN, font=(FONT, 8, "bold"))

    # -- demo --------------------------------------------------------------
    def _demo_connect(self):
        self._log("info", "演示模式：使用模拟数据源")
        labels = list(self.port_combo.cget("values"))
        if SIM_LABEL in labels:
            self.port_var.set(SIM_LABEL)
        self.demo_pending = True
        self._toggle_connect()

    # -- shutdown ----------------------------------------------------------
    def _on_close(self):
        try:
            self._save_settings_file()
        except Exception:
            pass
        try:
            self.worker.shutdown()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------
def run_selftest():
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        print(("PASS  " if cond else "FAIL  ") + name + (("  " + extra) if extra else ""))
        if not cond:
            ok = False

    cfg = DeviceConfig()
    check("default registers", cfg.registers() == [0x08, 0x68, 0x40, 0x01, 0x00],
          str([hex(x) for x in cfg.registers()]))
    check("frame len dcnt", cfg.frame_len == 4, str(cfg.frame_len))
    cfg2 = DeviceConfig(dcnt=False, integrity=2)
    check("frame len crc", cfg2.frame_len == 5, str(cfg2.frame_len))

    # decoder roundtrip with CRC + counter
    for integrity in (0, 1, 2):
        for dcnt in (False, True):
            c = DeviceConfig(dcnt=dcnt, integrity=integrity)
            got = []
            dec = FrameDecoder(c, lambda code, ctr: got.append(code))
            codes = [0, 1, -1, 123456, -(1 << 23), (1 << 23) - 1] * 3
            data = bytearray()
            for i, code in enumerate(codes):
                data += encode_sample(code, i & 0xFF, c)
            dec.feed(bytes(data))
            check(f"roundtrip dcnt={dcnt} integ={integrity}",
                  got == codes, f"{len(got)}/{len(codes)}")
            check(f"no errors dcnt={dcnt} integ={integrity}", dec.errors == 0)

    # resync after a junk prefix
    c = DeviceConfig(dcnt=True, integrity=2)
    got = []
    dec = FrameDecoder(c, lambda code, ctr: got.append(code))
    codes = list(range(1000, 1010))
    data = b"\x13\x37" + b"".join(encode_sample(v, i & 0xFF, c) for i, v in enumerate(codes))
    dec.feed(data)
    check("resync after junk prefix", len(got) >= 9 and got[:5] == codes[:5],
          f"got {len(got)} codes, errors={dec.errors}, resync={dec.resyncs}")

    # counter gap -> dropped count
    c = DeviceConfig(dcnt=True, integrity=2)
    got = []
    dec = FrameDecoder(c, lambda code, ctr: got.append(code))
    frames = [encode_sample(v, i & 0xFF, c) for i, v in enumerate(range(100, 110))]
    data = b"".join(frames[:5] + frames[7:])  # skip counter 5 and 6
    dec.feed(data)
    check("counter gap drop count", dec.dropped == 2, str(dec.dropped))
    check("counter gap samples", len(got) == 8, str(len(got)))

    # range calculation
    r = compute_range(DeviceConfig(gain=16, vref_code=0), 2.048, 1.65, 3.3)
    check("range fs", abs(r.fs - 0.128) < 1e-12, str(r.fs))
    check("range lsb", abs(r.lsb - 0.128 / 2**23) < 1e-18, str(r.lsb))
    check("range usable", abs(r.usable_lo - (1.65 - 0.128)) < 1e-12 and
          abs(r.usable_hi - (1.65 + 0.128)) < 1e-12,
          f"{r.usable_lo:.6f}..{r.usable_hi:.6f}")
    check("range abs window", abs(r.abs_lo - 0.392) < 1e-12 and
          abs(r.abs_hi - 2.908) < 1e-12, f"{r.abs_lo:.4f}..{r.abs_hi:.4f}")
    check("range valid", r.valid)

    r2 = compute_range(DeviceConfig(gain=1, vref_code=1, pga_bypass=True),
                       1.0, 0.5, 3.3)
    check("external ref range", abs(r2.vref - 1.0) < 1e-12 and r2.valid)
    check("bypass abs window", abs(r2.abs_lo - (-0.1)) < 1e-12 and
          abs(r2.abs_hi - 3.4) < 1e-12)

    print("\nRESULT:", "ALL PASS" if ok else "FAILURES PRESENT")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def enable_dpi_awareness():
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            import ctypes
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=APP_NAME)
    parser.add_argument("--demo", action="store_true", help="启动模拟数据演示")
    parser.add_argument("--selftest", action="store_true", help="运行自检")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(run_selftest())

    enable_dpi_awareness()
    root = tk.Tk()
    app = App(root, demo=args.demo)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app._on_close()


if __name__ == "__main__":
    main()
