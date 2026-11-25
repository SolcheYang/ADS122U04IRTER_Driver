#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADS122U04 24-bit delta-sigma ADC UART acquisition tool.

Features
  * serial port / baud selection (device auto-bauds on the 0x55 sync word)
  * full register configuration through a GUI + one-click apply / readback verify
  * automatic data read mode (AUTO=1), robust frame parsing (counter / inverted /
    CRC-16), raw byte stream + decoded CSV recording
  * real-time waveform for the selected input channel, measurement range
    calculator with a configurable input bias voltage
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
import re
import sys
import threading
import time
import traceback
import webbrowser
from array import array
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog

import formula as formula_engine

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

try:
    from PIL import ImageGrab
except Exception:  # pragma: no cover
    ImageGrab = None

try:
    import updater as update_client
except Exception as exc:  # pragma: no cover
    update_client = None
    UPDATE_IMPORT_ERROR = exc
else:
    UPDATE_IMPORT_ERROR = None


APP_NAME = "MagSensor"
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

# (positive, negative) analog input pin selected by each MUX code.
MUX_PINS = {
    0: ("AIN0", "AIN1"), 1: ("AIN0", "AIN2"), 2: ("AIN0", "AIN3"),
    3: ("AIN1", "AIN0"), 4: ("AIN1", "AIN2"), 5: ("AIN1", "AIN3"),
    6: ("AIN2", "AIN3"), 7: ("AIN3", "AIN2"),
    8: ("AIN0", "AVSS"), 9: ("AIN1", "AVSS"), 10: ("AIN2", "AVSS"),
    11: ("AIN3", "AVSS"),
}


def is_differential_mux(mux: int) -> bool:
    """True for the eight differential pairs (MUX 0..7)."""
    return 0 <= mux <= 7


CHANNEL_MODES = ["差分", "单端", "监测"]
DIFF_MUXES = list(range(0, 8))
SE_MUXES = list(range(8, 12))
MON_MUXES = [12, 13, 14]
MUX_SHORT = {}
for _v, _t in MUX_CHOICES:
    MUX_SHORT[_v] = _t.split("（")[0].strip()


def muxes_for_mode(mode: str):
    if mode == "差分":
        return DIFF_MUXES
    if mode == "单端":
        return SE_MUXES
    return MON_MUXES


def mode_of_mux(mux: int) -> str:
    if 0 <= mux <= 7:
        return "差分"
    if 8 <= mux <= 11:
        return "单端"
    return "监测"


# Waveform configuration (one entry per user-added waveform window).
@dataclass
class SeriesSpec:
    key: str
    mux: int
    mode: str              # "raw" or "formula"
    label: str = ""
    formula: str = ""
    x_formula: str = "t"
    x_unit: str = "s"


@dataclass
class WaveSpec:
    wid: int
    title: str
    x_unit: str = "s"
    y_unit: str = "V"
    show_legend: bool = True
    timebase: str = "2 s"
    auto_y: bool = True
    trigger_mode: str = "关闭"
    trigger_value: float = 0.0
    trigger_source: str = ""
    series: list = field(default_factory=list)
    hidden: list = field(default_factory=list)


WAVE_COLORS = ["#22d3ee", "#f59e0b", "#a78bfa", "#22c55e", "#f472b6",
               "#93c5fd", "#facc15", "#fb7185", "#34d399", "#c084fc"]
TRIGGER_MODES = ["关闭", "上升沿", "下降沿", "电平"]
SCAN_DWELL_SECONDS = 0.5

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

FORMULA_VARIABLE_TYPES = {
    "i": "int", "t": "float", "code": "int", "vin": "float",
    "ain": "float", "ain0": "float", "rate": "float", "lsb": "float",
    "vref": "float", "gain": "int", "fs": "float",
    "ain1": "float", "avdd": "float", "refp": "float",
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
COL_TRACE2 = "#f5a97f"


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


def fmt_bytes(n):
    try:
        n = int(n)
    except Exception:
        return "—"
    if n >= 1024 * 1024:
        return f"{n / 1048576:.1f} MB"
    if n >= 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n} B"


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


def _load_build_info():
    """Version metadata generated by tools/build.py from the git tag."""
    try:
        p = resource_path("build_info.json")
        if p.exists():
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data
    except Exception:
        pass
    return {}


BUILD_INFO = _load_build_info()
APP_VERSION = str(BUILD_INFO.get("version") or APP_VERSION)
APP_TAG = str(BUILD_INFO.get("tag") or f"v{APP_VERSION}")
APP_COMMIT = str(BUILD_INFO.get("commit") or "")
APP_COMMIT_SHORT = str(BUILD_INFO.get("commit_short") or APP_COMMIT[:7])
IS_RELEASE_BUILD = bool(BUILD_INFO.get("release"))
UPDATE_REPO = str(BUILD_INFO.get("repo") or "SolcheYang/MagSensor")


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


def compute_range(cfg: DeviceConfig, refp: float, bias: float, avdd: float,
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
        info.usable_lo = info.usable_hi = bias
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

    info.usable_lo = max(bias - info.fs, info.abs_lo)
    info.usable_hi = min(bias + info.fs, info.abs_hi)
    if cfg.mux in (8, 9, 10, 11):
        # Single-ended: AINN = AVSS, the input cannot swing below ground.
        info.usable_lo = max(info.usable_lo, avss)
    info.bias_ok = info.abs_lo <= bias <= info.abs_hi
    if not info.bias_ok:
        info.valid = False
        info.message = (f"输入偏置 {bias:.4f} V 超出 PGA 绝对输入窗口 "
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
        self.reset()

    def reset(self):
        with self.lock:
            self.codes = array("i")
            self.channels = array("i")
            self.series_x = {}
            self.series_y = {}
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
            self.formula_errors = 0

    def set_series(self, keys):
        """Declare the formula series evaluated by the worker."""
        with self.lock:
            self.series_x = {k: array("d") for k in keys}
            self.series_y = {k: array("d") for k in keys}

    def add(self, code, counter=None, channel=0, series_values=None):
        vals = series_values or {}
        with self.lock:
            self.codes.append(int(code))
            self.channels.append(int(channel))
            for key in self.series_y:
                try:
                    xv, yv = vals.get(key, (float("nan"), float("nan")))
                    xv = float(xv)
                    yv = float(yv)
                except Exception:
                    xv = yv = float("nan")
                self.series_x[key].append(xv)
                self.series_y[key].append(yv)
            if len(self.codes) > self.MAX_BUFFER:
                del self.codes[:50000]
                del self.channels[:50000]
                for arr in list(self.series_x.values()) + list(self.series_y.values()):
                    if len(arr) >= 50000:
                        del arr[:50000]
            self.total += 1
            if self.min_code is None or code < self.min_code:
                self.min_code = code
            if self.max_code is None or code > self.max_code:
                self.max_code = code
            self.sum_codes += code
            self.last_code = code
            self.last_counter = counter

    def note(self, dropped=None, errors=None, resyncs=None, rate=None,
             formula_errors=None):
        with self.lock:
            if dropped is not None:
                self.dropped = dropped
            if errors is not None:
                self.errors = errors
            if resyncs is not None:
                self.resyncs = resyncs
            if rate is not None:
                self.rate = rate
            if formula_errors is not None:
                self.formula_errors = formula_errors

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
                "formula_errors": self.formula_errors,
            }

    def _stats_locked(self):
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
            "formula_errors": self.formula_errors,
        }

    def _slice_locked(self, start_abs, end_abs, max_points):
        """Slice by absolute sample index. Caller holds self.lock."""
        buffered = len(self.codes)
        first_abs = self.total - buffered
        s = 0
        e = -1
        window = 0
        step = 1
        codes = array("i")
        channels = array("i")
        series = {}
        if buffered > 0:
            try:
                s = max(first_abs, int(start_abs))
                e = min(self.total - 1, int(end_abs))
            except Exception:
                s, e = self.total, self.total - 1
            if e >= s:
                off_s = s - first_abs
                off_e = e - first_abs + 1
                window = off_e - off_s
                step = (int(math.ceil(window / float(max_points)))
                        if max_points and window > max_points else 1)
                codes = self.codes[off_s:off_e:step]
                channels = self.channels[off_s:off_e:step]
                for key, ys in self.series_y.items():
                    xs = self.series_x.get(key)
                    series[key] = (
                        xs[off_s:off_e:step] if xs is not None else array("d"),
                        ys[off_s:off_e:step])
        data = self._stats_locked()
        data.update({
            "start_index": s,
            "end_index": e,
            "window": window,
            "step": step,
            "codes": codes,
            "channels": channels,
            "series": series,
        })
        return data

    def snapshot(self, tail_count=0, max_points=6000):
        with self.lock:
            buffered = len(self.codes)
            if tail_count and tail_count > 0 and buffered > 0:
                window = min(int(tail_count), buffered)
                start = self.total - window
                return self._slice_locked(start, self.total - 1, max_points)
            return self._slice_locked(self.total, self.total - 1, max_points)

    def snapshot_range(self, start_index, end_index, max_points=6000):
        with self.lock:
            return self._slice_locked(start_index, end_index, max_points)


class Recorder:
    """Writes the enabled output files: raw stream, decoded CSV, analog CSV."""

    def __init__(self, outputs: dict, cfg: DeviceConfig, meta: dict,
                 rate: float, series_specs: list):
        self.rate = float(rate) if rate else 1.0
        self.paths = {}
        for key in ("raw", "decoded", "analog"):
            value = outputs.get(key)
            self.paths[key] = Path(value) if value else None
        for path in self.paths.values():
            if path is not None:
                path.parent.mkdir(parents=True, exist_ok=True)
        self.raw = None
        self.csv = None
        self.writer = None
        self.analog = None
        self.analog_writer = None
        self.rows = 0
        self.analog_rows = 0
        self.raw_bytes = 0
        self.series_specs = list(series_specs)
        self.analog_keys = [s.key for s in self.series_specs
                            if s.mode == "formula"]
        if self.paths.get("raw"):
            self.raw = open(self.paths["raw"], "wb", buffering=1 << 16)
        if self.paths.get("decoded"):
            self.csv = open(self.paths["decoded"], "w", newline="",
                            encoding="utf-8-sig", buffering=1 << 16)
            self.writer = csv.writer(self.csv)
            self.writer.writerow(["index", "t_s", "host_time_s", "counter",
                                  "channel", "code", "vin_V", "ain_V"])
        if self.paths.get("analog"):
            self.analog = open(self.paths["analog"], "w", newline="",
                               encoding="utf-8-sig", buffering=1 << 16)
            self.analog_writer = csv.writer(self.analog)
            labels = {s.key: (s.label or s.key) for s in self.series_specs}
            self.analog_writer.writerow(
                ["index", "t_s", "host_time_s", "channel"] +
                [labels.get(k, k) for k in self.analog_keys])
        self.json_path = None
        base = self.paths.get("decoded") or self.paths.get("raw") or \
            self.paths.get("analog")
        if base is not None:
            stem = re.sub(r"_(samples|raw|analog)$", "", base.stem)
            self.json_path = base.with_name(stem + "_config.json")
            try:
                doc = dict(meta)
                doc["series"] = [
                    {"key": s.key, "mux": s.mux, "mode": s.mode,
                     "label": s.label, "formula": s.formula,
                     "x_formula": s.x_formula, "x_unit": s.x_unit}
                    for s in self.series_specs]
                with open(self.json_path, "w", encoding="utf-8") as f:
                    json.dump(doc, f, ensure_ascii=False, indent=2)
            except Exception:
                self.json_path = None

    def write_raw(self, data: bytes):
        if self.raw is not None:
            self.raw.write(data)
            self.raw_bytes += len(data)

    @staticmethod
    def _num(v):
        try:
            v = float(v)
        except Exception:
            return ""
        if not math.isfinite(v):
            return ""
        return f"{v:.9g}"

    def add(self, index, host_t, counter, channel, code, vin, ain,
            series_values=None):
        vals = series_values or {}
        if self.writer is not None:
            self.writer.writerow([
                index, f"{index / self.rate:.6f}", f"{host_t:.6f}",
                "" if counter is None else counter, channel, code,
                f"{vin:.9f}", "" if ain is None else f"{ain:.9f}"])
            self.rows += 1
            if self.rows % 512 == 0:
                self.csv.flush()
        if self.analog_writer is not None:
            row = [index, f"{index / self.rate:.6f}", f"{host_t:.6f}",
                   channel]
            for key in self.analog_keys:
                _, yv = vals.get(key, (float("nan"), float("nan")))
                row.append(self._num(yv))
            self.analog_writer.writerow(row)
            self.analog_rows += 1
            if self.analog_rows % 512 == 0:
                self.analog.flush()

    def flush(self):
        for f in (self.raw, self.csv, self.analog):
            try:
                if f is not None:
                    f.flush()
            except Exception:
                pass

    def close(self):
        for f in (self.raw, self.csv, self.analog):
            try:
                if f is not None:
                    f.flush()
                    f.close()
            except Exception:
                pass
        self.raw = self.csv = self.analog = None
        return dict(self.paths)


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
            "output_paths": {},
            "stop_seq": 0,
        }
        self._lsb = 2.4414e-7
        self._ain1 = 0.0
        self._mux = 0
        self._formula_rate = 1.0
        self._f_expr = None
        self._x_expr = None
        self._formula_ctx = {}
        self._formula_errors = 0
        self._formula_text = ""
        self._x_formula_text = ""
        self._x_unit = "s"
        self._series_specs = []
        self._scan_channels = [0]
        self._scan_pos = 0
        self._scan_count = 0
        self._scan_dwell = 1
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
        scan = meta.get("scan_channels") or [cfg.mux]
        try:
            self._scan_channels = [int(v) for v in scan] or [int(cfg.mux)]
        except Exception:
            self._scan_channels = [int(cfg.mux)]
        self._scan_pos = 0
        self._scan_count = 0
        cfg = replace(cfg, mux=self._scan_channels[0])
        self.cfg = cfg
        self._mux = cfg.mux
        self._lsb = float(meta.get("lsb", 2.4414e-7)) or 2.4414e-7
        self._ain1 = float(meta.get("ain1", 0.0))
        self._formula_rate = float(cfg.data_rate) or 1.0
        self._scan_dwell = max(1, int(self._formula_rate * SCAN_DWELL_SECONDS))
        cc = meta.get("formula_context") or {}
        self._formula_ctx = {
            "rate": float(cc.get("rate", cfg.data_rate)),
            "lsb": float(cc.get("lsb", self._lsb)),
            "vref": float(cc.get("vref", 2.048)),
            "gain": int(cc.get("gain", cfg.effective()[0])),
            "fs": float(cc.get("fs", 0.128)),
            "ain1": float(cc.get("ain1", self._ain1)),
            "avdd": float(cc.get("avdd", 3.3)),
            "refp": float(cc.get("refp", 2.048)),
        }
        # ---- animation / formula series ----
        if "series" in meta:
            specs = list(meta.get("series") or [])
        else:
            specs = [{"key": "default", "mux": cfg.mux, "mode": "formula",
                      "label": "f",
                      "formula": meta.get("formula") or "ain",
                      "x_formula": meta.get("x_formula") or "t",
                      "x_unit": meta.get("x_unit") or "s"}]
        self._series_specs = []
        seen_keys = set()
        for idx, raw in enumerate(specs):
            try:
                mode = str(raw.get("mode") or "formula")
                key = str(raw.get("key") or f"s{idx}")
                while key in seen_keys:
                    key += "_"
                seen_keys.add(key)
                label = str(raw.get("label") or key)
                formula = str(raw.get("formula") or "ain")
                x_formula = str(raw.get("x_formula") or "t")
                f_expr = x_expr = None
                if mode == "formula":
                    f_expr = formula_engine.compile_expr(
                        formula, FORMULA_VARIABLE_TYPES)
                    x_expr = formula_engine.compile_expr(
                        x_formula, FORMULA_VARIABLE_TYPES)
                self._series_specs.append({
                    "key": key,
                    "mux": int(raw.get("mux", cfg.mux)),
                    "mode": mode,
                    "label": label,
                    "formula": formula,
                    "x_formula": x_formula,
                    "x_unit": str(raw.get("x_unit") or "s"),
                    "f": f_expr,
                    "x": x_expr,
                })
            except Exception as exc:
                self.log("error", f"公式无效（{raw.get('label', '')}）：{exc}")
                self._set_state(settings_status="公式错误", settings_ok=False,
                                message="公式错误，未开始采集")
                return
        self._formula_errors = 0
        self.stats.set_series([s["key"] for s in self._series_specs
                               if s["mode"] == "formula"])

        self._acq_t0 = time.monotonic()
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_ema = 0.0
        self._trigger_pending = False
        self.decoder = FrameDecoder(cfg, self._on_sample)
        self.recorder = None
        self._sim_idx = 0
        self._sim_counter = 0

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

        # ---- output files ----
        outputs = meta.get("outputs")
        if not isinstance(outputs, dict):
            outputs = {}
            if record:
                base = f"MagSensor_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
                folder = Path(folder)
                outputs = {"raw": str(folder / f"{base}_raw.bin"),
                           "decoded": str(folder / f"{base}_samples.csv"),
                           "analog": None}
        rec_series = [SeriesSpec(key=s["key"], mux=s["mux"], mode=s["mode"],
                                 label=s["label"], formula=s["formula"],
                                 x_formula=s["x_formula"],
                                 x_unit=s["x_unit"])
                      for s in self._series_specs]
        if any(outputs.get(k) for k in ("raw", "decoded", "analog")):
            try:
                self.recorder = Recorder(outputs, cfg, meta, cfg.data_rate,
                                         rec_series)
                for key, label in (("raw", "原始码流"), ("decoded", "解码数据"),
                                   ("analog", "模拟值")):
                    if self.recorder.paths.get(key):
                        self.log("info", f"{label}文件：{self.recorder.paths[key]}")
            except Exception as exc:
                self.recorder = None
                self.log("error", f"无法创建输出文件：{exc}")

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
        self._flush_t = time.monotonic()
        self.stats.note(rate=0.0)
        out_paths = {k: (str(v) if v else "")
                     for k, v in (self.recorder.paths.items()
                                  if self.recorder else [])}
        self._set_state(streaming=True,
                        recording=self.recorder is not None,
                        output_paths=out_paths,
                        raw_path=out_paths.get("raw", ""),
                        csv_path=out_paths.get("decoded", ""),
                        settings_status="采集中", settings_ok=True,
                        message=f"采集中 · {cfg.data_rate} SPS")
        self.log("ok", f"开始采集 · {cfg.description()}")
        self.log("info", f"帧格式：{cfg.frame_description()}")
        scan_txt = ", ".join(MUX_SHORT.get(m, str(m)) for m in self._scan_channels)
        self.log("info", f"扫描通道（{len(self._scan_channels)}）：{scan_txt}")
        f_count = sum(1 for s in self._series_specs if s["mode"] == "formula")
        self.log("info", f"公式序列：{f_count} 条；每通道停留 "
                          f"{SCAN_DWELL_SECONDS:g} s")

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
            paths = {}
            try:
                paths = self.recorder.close()
                if paths.get("raw"):
                    self.log("ok", f"原始码流已保存：{paths['raw']}"
                                   f"（{self.recorder.raw_bytes} 字节）")
                if paths.get("decoded"):
                    self.log("info", f"解码数据：{paths['decoded']}"
                                     f"（{self.recorder.rows} 行）")
                if paths.get("analog"):
                    self.log("info", f"模拟值数据：{paths['analog']}"
                                     f"（{self.recorder.analog_rows} 行）")
            except Exception as exc:
                self.log("warn", f"关闭输出文件时出错：{exc}")
            self.recorder = None
            if was:
                self.stats.note(rate=0.0)
                seq = int(self.state.get("stop_seq", 0)) + 1
                self._set_state(output_paths={k: (str(v) if v else "")
                                              for k, v in paths.items()},
                                stop_seq=seq, recording=False,
                                raw_path=str(paths.get("raw") or ""),
                                csv_path=str(paths.get("decoded") or ""))
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

    def _switch_channel_to(self, mux: int):
        self.cfg.mux = int(mux)
        self._mux = int(mux)
        if self.sim or self.ser is None:
            return
        try:
            gain, bypass = self.cfg.effective()
            reg0 = (((int(mux) & 0x0F) << 4) |
                    ((GAIN_CODES[gain] & 7) << 1) |
                    (1 if bypass else 0))
            # Only MUX/gain/PGA change; the frame length and counter stay.
            self.ser.write(bytes((SYNC, 0x40, reg0)))
            self.ser.flush()
        except Exception as exc:
            self.log("warn", f"切换输入通道失败：{exc}")

    def _on_sample(self, code, counter):
        host_t = time.monotonic() - self._acq_t0
        channel = self._mux
        vin = code * self._lsb
        ain = vin + self._ain1 if 0 <= channel <= 11 else None
        idx = self._sample_index
        ain_val = ain if ain is not None else float("nan")
        ctx = self._formula_ctx
        ctx["i"] = idx
        ctx["t"] = idx / self._formula_rate if self._formula_rate else 0.0
        ctx["code"] = code
        ctx["vin"] = vin
        ctx["ain"] = ain_val
        ctx["ain0"] = ain_val   # legacy alias
        series_values = {}
        sample_formula_error = False
        for spec in self._series_specs:
            if spec["mode"] != "formula":
                continue
            key = spec["key"]
            if spec["mux"] != channel or spec["f"] is None:
                series_values[key] = (float("nan"), float("nan"))
                continue
            try:
                yv = spec["f"].eval_float(ctx)
                xv = spec["x"].eval_float(ctx) if spec["x"] is not None \
                    else ctx["t"]
            except Exception:
                yv = xv = float("nan")
            if not (math.isfinite(yv) and math.isfinite(xv)):
                sample_formula_error = True
            series_values[key] = (xv, yv)
        if sample_formula_error:
            self._formula_errors += 1
            self.stats.note(formula_errors=self._formula_errors)
        self.stats.add(code, counter, channel, series_values)
        if self.recorder is not None:
            try:
                self.recorder.add(self._sample_index, host_t, counter, channel,
                                  code, vin, ain, series_values)
            except Exception as exc:
                self.log("error", f"写入输出文件失败：{exc}")
                try:
                    self.recorder.close()
                except Exception:
                    pass
                self.recorder = None
        self._sample_index += 1
        if len(self._scan_channels) > 1:
            self._scan_count += 1
            if self._scan_count >= self._scan_dwell:
                self._scan_count = 0
                self._scan_pos = (self._scan_pos + 1) % len(self._scan_channels)
                self._switch_channel_to(self._scan_channels[self._scan_pos])
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
                        resyncs=self.decoder.resyncs if self.decoder else 0,
                        formula_errors=self._formula_errors)

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
        ch = self._mux
        offset = ((ch % 4) - 1.5) * 0.10 * fs
        amp = 0.48 * fs
        chunk = bytearray()
        for _ in range(n):
            t = self._sim_idx / float(rate)
            v = (offset
                 + amp * math.sin(2 * math.pi * (1.5 + 0.4 * (ch % 4)) * t)
                 + 0.10 * fs * math.sin(2 * math.pi * 7.0 * t + 0.7)
                 + 0.04 * fs * math.sin(2 * math.pi * 0.23 * t)
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
# Waveform view state
# ---------------------------------------------------------------------------
class PlotState:
    def __init__(self, spec):
        self.canvas = None
        self.spec = spec
        self.y_view = None        # (ymin, ymax)
        self.index_view = None    # (start_abs_index, end_abs_index)
        self.last = None          # mapping from the last draw
        self.selecting = False
        self.box = None
        self.box_rect = None
        self.frozen = False
        self.frozen_snapshot = None
        self.trigger_scan_index = 0
        self.legend_hits = []


class WaveWindow:
    """One user-added waveform window (inline or detached)."""

    TIMEBASES = ["0.5 s", "1 s", "2 s", "5 s", "10 s", "30 s", "全部"]

    def __init__(self, app, spec: WaveSpec):
        self.app = app
        self.spec = spec
        self.state = PlotState(spec)
        self.detached = None
        self.detached_canvas = None
        self.frame = tk.Frame(app.wave_container, bg=COL_PANEL)
        self.frame.pack(fill="x", padx=4, pady=(2, 8))
        self._build()
        self.state.canvas = self.canvas
        self._bind_canvas(self.canvas)
        self._sync_controls()

    # -- construction ------------------------------------------------------
    def _build(self):
        s = self.app.ui_scale
        head = tk.Frame(self.frame, bg=COL_PANEL)
        head.pack(fill="x", padx=8, pady=(6, 2))
        tk.Label(head, text=self.spec.title, bg=COL_PANEL, fg=COL_ACCENT,
                 font=(FONT, 10, "bold")).pack(side="left")
        ttk.Button(head, text="删除", style="Small.TButton",
                   command=lambda: self.app._delete_wave_window(self)
                   ).pack(side="right", padx=(4, 0))
        self.detach_btn = ttk.Button(head, text="独立显示",
                                     style="Small.TButton",
                                     command=self.toggle_detach)
        self.detach_btn.pack(side="right", padx=(4, 0))
        ttk.Button(head, text="保存图像", style="Small.TButton",
                   command=self.save_image).pack(side="right", padx=(4, 0))

        tb = tk.Frame(self.frame, bg=COL_PANEL)
        tb.pack(fill="x", padx=8, pady=(0, 2))
        tk.Label(tb, text="时间窗", bg=COL_PANEL, fg=COL_MUTED,
                 font=(FONT, 8)).pack(side="left")
        self.timebase_var = tk.StringVar(value=self.spec.timebase)
        self.timebase_combo = ttk.Combobox(
            tb, textvariable=self.timebase_var, state="readonly", width=6,
            values=self.TIMEBASES)
        self.timebase_combo.pack(side="left", padx=(4, 8))
        self.timebase_combo.bind("<<ComboboxSelected>>", self._on_timebase)
        self.timebase_combo.bind("<MouseWheel>",
                                 lambda e: self.app._block_wheel(e))
        self.auto_var = tk.BooleanVar(value=self.spec.auto_y)
        ttk.Checkbutton(tb, text="自动 Y", variable=self.auto_var,
                        command=self._on_auto).pack(side="left")

        tk.Label(tb, text="触发", bg=COL_PANEL, fg=COL_MUTED,
                 font=(FONT, 8)).pack(side="left", padx=(12, 3))
        self.trig_mode_var = tk.StringVar(value=self.spec.trigger_mode)
        self.trig_mode_combo = ttk.Combobox(
            tb, textvariable=self.trig_mode_var, values=TRIGGER_MODES,
            state="readonly", width=6)
        self.trig_mode_combo.pack(side="left")
        self.trig_mode_combo.bind("<<ComboboxSelected>>", self._on_trigger_mode)
        self.trig_mode_combo.bind("<MouseWheel>",
                                  lambda e: self.app._block_wheel(e))
        self.trig_val_var = tk.StringVar(
            value="" if not self.spec.trigger_value else
            f"{self.spec.trigger_value:.6g}")
        self.trig_val_entry = ttk.Entry(tb, textvariable=self.trig_val_var,
                                        width=9)
        self.trig_val_entry.pack(side="left", padx=(4, 4))
        self.trig_val_entry.bind("<Return>", self._on_trigger_value)
        self.trig_val_entry.bind("<FocusOut>", self._on_trigger_value)
        self.trig_src_var = tk.StringVar(value=self._source_label())
        self.trig_src_combo = ttk.Combobox(
            tb, textvariable=self.trig_src_var, state="readonly", width=14)
        self.trig_src_combo.pack(side="left")
        self.trig_src_combo.bind("<<ComboboxSelected>>", self._on_trigger_source)
        self.trig_src_combo.bind("<MouseWheel>",
                                 lambda e: self.app._block_wheel(e))

        self.pause_btn = ttk.Button(tb, text="暂停", style="Small.TButton",
                                    command=self.toggle_pause)
        self.pause_btn.pack(side="right")
        ttk.Button(tb, text="复位", style="Small.TButton",
                   command=self.reset).pack(side="right", padx=(0, 4))

        self.canvas = tk.Canvas(self.frame, height=int(210 * s),
                                bg=COL_PLOT_BG, highlightthickness=0, bd=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(2, 6))
        self.placeholder = tk.Label(
            self.frame, text="波形已在独立窗口中显示 — 关闭独立窗口即可回归",
            bg=COL_PANEL, fg=COL_MUTED, font=(FONT, 9), height=8)
        self._sync_controls()

    def _bind_canvas(self, canvas):
        canvas.bind("<Configure>", self.app._on_canvas_cfg)
        canvas.bind("<MouseWheel>",
                    lambda e: self.app._on_plot_wheel(self, e))
        canvas.bind("<ButtonPress-1>",
                    lambda e: self.app._on_plot_press(self, e))
        canvas.bind("<B1-Motion>",
                    lambda e: self.app._on_plot_drag(self, e))
        canvas.bind("<ButtonRelease-1>",
                    lambda e: self.app._on_plot_release(self, e))

    def active_canvas(self):
        return self.detached_canvas or self.canvas

    def _source_label(self):
        for sr in self.spec.series:
            if sr.key == self.spec.trigger_source:
                return sr.label or sr.key
        return (self.spec.series[0].label if self.spec.series else "")

    def _sync_controls(self):
        labels = [sr.label or sr.key for sr in self.spec.series]
        try:
            self.trig_src_combo.configure(values=labels)
        except Exception:
            pass
        if not self.spec.trigger_source and self.spec.series:
            self.spec.trigger_source = self.spec.series[0].key
        self.trig_src_var.set(self._source_label())
        self.trig_mode_var.set(self.spec.trigger_mode)
        self.trig_val_var.set("" if not self.spec.trigger_value else
                              f"{self.spec.trigger_value:.6g}")
        self.timebase_var.set(self.spec.timebase)
        self.auto_var.set(self.spec.auto_y)
        self.pause_btn.configure(text="继续" if self.state.frozen else "暂停")

    # -- controls ----------------------------------------------------------
    def _on_timebase(self, _=None):
        self.spec.timebase = self.timebase_var.get()
        self.state.index_view = None
        self.app._plot_dirty = True

    def _on_auto(self):
        if self.auto_var.get():
            self.state.y_view = None
        elif self.state.last is not None:
            self.state.y_view = (self.state.last["ymin"],
                                 self.state.last["ymax"])
        self.spec.auto_y = bool(self.auto_var.get())
        self.app._plot_dirty = True

    def _on_trigger_mode(self, _=None):
        self.spec.trigger_mode = self.trig_mode_var.get()
        try:
            self.state.trigger_scan_index = self.app.worker.stats.quick()["total"]
        except Exception:
            self.state.trigger_scan_index = 0
        if self.spec.trigger_mode == "关闭":
            self.state.frozen = False
            self.state.frozen_snapshot = None
            self.pause_btn.configure(text="暂停")
        self.app._plot_dirty = True

    def _on_trigger_value(self, _=None):
        try:
            self.spec.trigger_value = float(self.trig_val_var.get().strip())
        except Exception:
            pass
        self.app._plot_dirty = True

    def _on_trigger_source(self, _=None):
        label = self.trig_src_var.get()
        for sr in self.spec.series:
            if (sr.label or sr.key) == label:
                self.spec.trigger_source = sr.key
                break
        self.state.trigger_scan_index = 0

    def reset(self):
        self.state.y_view = None
        self.state.index_view = None
        self.state.frozen = False
        self.state.frozen_snapshot = None
        try:
            self.state.trigger_scan_index = self.app.worker.stats.quick()["total"]
        except Exception:
            self.state.trigger_scan_index = 0
        self.auto_var.set(True)
        self.spec.auto_y = True
        self.pause_btn.configure(text="暂停")
        self.app._plot_dirty = True

    def toggle_pause(self):
        self.state.frozen = not self.state.frozen
        if self.state.frozen:
            try:
                quick = self.app.worker.stats.quick()
                count = self.app._window_count(
                    quick, self.app._plot_rate_value(), self.spec.timebase)
                self.state.frozen_snapshot = self.app.worker.stats.snapshot(
                    count, max_points=4000)
            except Exception:
                self.state.frozen_snapshot = None
        else:
            self.state.frozen_snapshot = None
            try:
                self.state.trigger_scan_index = self.app.worker.stats.quick()["total"]
            except Exception:
                pass
        self.pause_btn.configure(text="继续" if self.state.frozen else "暂停")
        self.app._plot_dirty = True

    def set_frozen(self, frozen, snapshot=None):
        self.state.frozen = bool(frozen)
        self.state.frozen_snapshot = snapshot
        self.pause_btn.configure(text="继续" if frozen else "暂停")
        self.app._plot_dirty = True

    # -- detach ------------------------------------------------------------
    def toggle_detach(self):
        if self.detached is not None:
            self._reattach()
            return
        s = self.app.ui_scale
        top = tk.Toplevel(self.app.root)
        top.title(self.spec.title)
        top.configure(bg=COL_BG)
        top.geometry(f"{int(900*s)}x{int(460*s)}")
        bar = tk.Frame(top, bg=COL_PANEL)
        bar.pack(fill="x")
        tk.Label(bar, text=self.spec.title, bg=COL_PANEL, fg=COL_ACCENT,
                 font=(FONT, 10, "bold")).pack(side="left", padx=8, pady=4)
        ttk.Button(bar, text="回归主窗口", style="Small.TButton",
                   command=self._reattach).pack(side="right", padx=4)
        ttk.Button(bar, text="保存图像", style="Small.TButton",
                   command=self.save_image).pack(side="right", padx=4)
        canvas = tk.Canvas(top, bg=COL_PLOT_BG, highlightthickness=0, bd=0)
        canvas.pack(fill="both", expand=True, padx=6, pady=6)
        self._bind_canvas(canvas)
        self.detached = top
        self.detached_canvas = canvas
        self.canvas.pack_forget()
        self.placeholder.pack(fill="both", expand=True, padx=8, pady=8)
        self.state.canvas = canvas
        self.state.last = None
        self.detach_btn.configure(text="回归主窗口")
        top.protocol("WM_DELETE_WINDOW", self._reattach)
        self.app._plot_dirty = True

    def _reattach(self):
        if self.detached is None:
            return
        try:
            self.detached.destroy()
        except Exception:
            pass
        self.detached = None
        self.detached_canvas = None
        self.placeholder.pack_forget()
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(2, 6))
        self.state.canvas = self.canvas
        self.state.last = None
        self.detach_btn.configure(text="独立显示")
        self.app._plot_dirty = True

    def save_image(self):
        self.app._save_window_image(self)

    def destroy(self):
        if self.detached is not None:
            try:
                self.detached.destroy()
            except Exception:
                pass
        try:
            self.frame.destroy()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, root: tk.Tk, demo=False, no_update_check=False,
                 updated=False):
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
        self._plot_dirty = True
        self._last_draw = 0.0
        self._last_state = {}
        self._port_map = {}
        self._last_valid = None
        self._acq_bias = None
        self._acq_lsb = None
        self._acq_rate = None
        self.wave_windows = []
        self.wave_specs = []
        self._next_wid = 1
        self.channel_mode = "差分"
        self.channel_sets = {"差分": {0}, "单端": {8}, "监测": {12}}
        self.chan_vars = {}
        self.outputs = self._default_outputs()
        self.log_visible_var = tk.BooleanVar(value=False)
        self._stop_dialog_pending = False
        self._stop_dialog_open = False
        self._last_stop_seq = int(self.worker.get_state().get("stop_seq", 0))
        self._danmaku_queue = []
        self._danmaku_active = None
        self._danmaku_after = None
        self.no_update_check = bool(no_update_check or demo)
        self.updated = bool(updated)
        self.update_queue = queue.Queue()
        self._update_checking = False
        self._update_info = None
        self._update_dialog = None
        self._update_progress = None
        self._update_download_thread = None

        self._build_styles()
        self._build_ui()
        self._apply_settings_to_ui()
        self._refresh_ports()
        self._on_config_widget_change()
        self._update_range()
        self._refresh_state(self.worker.get_state())

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick_after = self.root.after(60, self._tick)
        if demo:
            self.root.after(500, self._demo_connect)
        if self.updated:
            self._log("ok", f"已通过自动更新启动：v{APP_VERSION} "
                            f"({APP_COMMIT_SHORT or 'no-commit'})")
        if not self.no_update_check and IS_RELEASE_BUILD and update_client is not None:
            self.root.after(700, lambda: self._begin_update_check(manual=False))
            self.root.after(30 * 60 * 1000, self._periodic_update_check)

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

    def _default_outputs(self):
        d = default_data_dir()
        return {
            "raw": {"enabled": True, "dir": d},
            "decoded": {"enabled": True, "dir": d},
            "analog": {"enabled": False, "dir": d},
            "image": {"enabled": True, "dir": d},
        }

    def _gather_settings(self):
        try:
            cfg = self._ui_config()
            refp, bias, avdd = self._range_values()
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
            "refp": refp, "ain1": bias, "avdd": avdd,
            "channel_mode": self.channel_mode,
            "channel_sets": {k: sorted(v) for k, v in self.channel_sets.items()},
            "outputs": self.outputs,
            "log_visible": bool(self.log_visible_var.get()),
            "wave_windows": [self._spec_to_dict(s) for s in self.wave_specs],
        }

    def _apply_settings_to_ui(self):
        s = self.settings
        try:
            if s.get("baud") in BAUD_RATES:
                self.baud_var.set(str(s["baud"]))
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
            # ---- channels ----
            mode = s.get("channel_mode")
            if mode not in CHANNEL_MODES:
                mux = s.get("mux")
                mode = mode_of_mux(mux) if isinstance(mux, int) else "差分"
            sets = s.get("channel_sets") or {}
            for m in CHANNEL_MODES:
                vals = sets.get(m)
                keep = set()
                if isinstance(vals, list):
                    for v in vals:
                        try:
                            iv = int(v)
                        except Exception:
                            continue
                        if iv in muxes_for_mode(m):
                            keep.add(iv)
                if not keep:
                    old_mux = s.get("mux")
                    if m == mode and isinstance(old_mux, int) and \
                            old_mux in muxes_for_mode(m):
                        keep.add(old_mux)
                    else:
                        keep.add(muxes_for_mode(m)[0])
                self.channel_sets[m] = keep
            self.channel_mode = mode
            self.chan_mode_var.set(mode)
            self._rebuild_chan_menu()
            # ---- outputs ----
            outs = s.get("outputs")
            if isinstance(outs, dict):
                for key in ("raw", "decoded", "analog", "image"):
                    item = outs.get(key)
                    if isinstance(item, dict):
                        self.outputs[key]["enabled"] = bool(
                            item.get("enabled", self.outputs[key]["enabled"]))
                        d = item.get("dir")
                        if d:
                            self.outputs[key]["dir"] = str(d)
            self.log_visible_var.set(bool(s.get("log_visible", False)))
            self._apply_log_visibility()
            self._restore_wave_windows(s.get("wave_windows") or [])
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
        w = min(int(1500 * s), ww - int(30 * s))
        h = min(int(900 * s), wh - int(48 * s))
        w = max(w, min(int(1320 * s), ww - int(30 * s)))
        h = max(h, min(int(740 * s), wh - int(48 * s)))
        x = wl + max(0, (ww - w) // 2)
        y = wt + max(0, (wh - h) // 2 - int(20 * s))
        root.geometry(f"{w}x{h}+{x}+{y}")
        root.minsize(min(int(1320 * s), ww - 20),
                     min(int(680 * s), wh - 20))

        self._build_header()

        content = tk.Frame(root, bg=COL_BG)
        content.pack(fill="both", expand=True,
                     padx=int(12 * s), pady=(0, int(10 * s)))
        content.grid_rowconfigure(0, weight=1)
        content.grid_columnconfigure(1, weight=1)

        left = tk.Frame(content, bg=COL_BG, width=int(400 * s))
        left.grid(row=0, column=0, sticky="ns", padx=(0, int(6 * s)))
        left.grid_propagate(False)
        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True)
        self.side = self.scroll.inner
        self._build_sidebar(self.side)
        self._bind_wheel_tree(self.side)

        main = tk.Frame(content, bg=COL_BG)
        main.grid(row=0, column=1, sticky="nsew", padx=(int(6 * s), 0))
        self.main_frame = main
        self._build_main(main)

        self._build_log_col(content)
        self._log_col.grid(row=0, column=2, sticky="ns", padx=(int(8 * s), 0))
        self._log_col.grid_remove()

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
        tk.Label(left, text="MagSensor · 24 位 ADC 采集", bg=COL_BG,
                 fg=COL_FG, font=(FONT, 15, "bold")).pack(anchor="w")
        tk.Label(left, text="MagSensor · UART 2-wire · 多通道扫描 · 自定义波形 · 触发",
                 bg=COL_BG, fg=COL_MUTED, font=(FONT, 9)).pack(anchor="w", pady=(2, 0))

        right = tk.Frame(header, bg=COL_BG)
        right.pack(side="right")
        self.settings_btn = tk.Frame(right, bg=COL_PANEL2, cursor="hand2",
                                     padx=10, pady=4)
        self.settings_btn.pack(side="right", padx=(14, 0))
        gear = tk.Label(self.settings_btn, text="⚙", bg=COL_PANEL2,
                        fg=COL_FG, font=("Segoe UI Symbol", 12))
        gear.pack(side="left")
        gear_txt = tk.Label(self.settings_btn, text="设置", bg=COL_PANEL2,
                            fg=COL_FG, font=(FONT, 9))
        gear_txt.pack(side="left", padx=(4, 0))
        for widget in (self.settings_btn, gear, gear_txt):
            widget.bind("<Button-1>", lambda e: self._post_settings_menu())
        self._build_settings_menu()
        self.dot = tk.Canvas(right, width=14, height=14, bg=COL_BG,
                             highlightthickness=0)
        self.dot.pack(side="right", padx=(0, 8))
        self.dot_id = self.dot.create_oval(3, 3, 11, 11, fill="#555b6a", outline="")
        self.conn_var = tk.StringVar(value="未连接")
        tk.Label(right, textvariable=self.conn_var, bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 9)).pack(side="right")

    def _build_settings_menu(self):
        menu = tk.Menu(self.root, tearoff=0, bg=COL_PANEL2, fg=COL_FG,
                       activebackground=COL_ACCENT, activeforeground="#07101f",
                       bd=0)
        file_menu = tk.Menu(menu, tearoff=0, bg=COL_PANEL2, fg=COL_FG,
                            activebackground=COL_ACCENT,
                            activeforeground="#07101f")
        output_menu = tk.Menu(file_menu, tearoff=0, bg=COL_PANEL2, fg=COL_FG,
                              activebackground=COL_ACCENT,
                              activeforeground="#07101f")
        for key, label in (("raw", "原始码流输出"),
                           ("decoded", "解码输出"),
                           ("analog", "模拟值输出")):
            output_menu.add_command(
                label=label + "…",
                command=lambda k=key: self._open_output_dialog(k))
        file_menu.add_cascade(label="文件输出", menu=output_menu)
        file_menu.add_command(label="图像保存…",
                              command=lambda: self._open_output_dialog("image"))
        menu.add_cascade(label="文件输出包", menu=file_menu)
        menu.add_checkbutton(label="显示事件日志", variable=self.log_visible_var,
                             command=self._apply_log_visibility)
        menu.add_separator()
        menu.add_command(label="添加波形图…", command=self._open_add_wave_dialog)
        self.settings_menu = menu

    def _post_settings_menu(self):
        try:
            x = self.settings_btn.winfo_rootx()
            y = self.settings_btn.winfo_rooty() + self.settings_btn.winfo_height()
            self.settings_menu.tk_popup(x, y)
        except Exception:
            pass
        finally:
            try:
                self.settings_menu.grab_release()
            except Exception:
                pass

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

        self.chan_mode_var = tk.StringVar(value=self.channel_mode)
        self.chan_mode_combo = ttk.Combobox(
            cfg, textvariable=self.chan_mode_var, values=CHANNEL_MODES,
            state="readonly")
        self.chan_mode_combo.bind("<<ComboboxSelected>>",
                                  self._on_channel_mode_change)
        self.chan_mode_combo.bind("<MouseWheel>", self._block_wheel)
        self._form_row(cfg, row, "输入类型", self.chan_mode_combo); row += 1

        self.chan_menu = tk.Menu(cfg, tearoff=0, bg=COL_PANEL2, fg=COL_FG,
                                 activebackground=COL_ACCENT,
                                 activeforeground="#07101f")
        self.chan_menu_btn = tk.Menubutton(
            cfg, text="选择通道", bg=COL_PANEL2, fg=COL_FG,
            activebackground="#343846", activeforeground=COL_FG,
            relief="flat", bd=0, padx=8, pady=3, anchor="w",
            font=(FONT, 9), menu=self.chan_menu)
        self._form_row(cfg, row, "通道使能", self.chan_menu_btn); row += 1
        self.chan_hint_var = tk.StringVar(value="")
        tk.Label(cfg, textvariable=self.chan_hint_var, bg=COL_PANEL,
                 fg="#5d6472", font=(FONT, 7), justify="left", anchor="w",
                 wraplength=int(350 * s)).grid(row=row, column=0, columnspan=2,
                                               sticky="w", pady=(0, 2))
        row += 1
        self._rebuild_chan_menu()

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

        tk.Label(cfg, text="模拟参数", bg=COL_PANEL, fg=COL_ACCENT,
                 font=(FONT, 8, "bold")).grid(row=row, column=0, columnspan=2,
                                              sticky="w", pady=(7, 0))
        row += 1

        self.refp_var = tk.StringVar(value="2.048")
        self.refp_entry = ttk.Entry(cfg, textvariable=self.refp_var, width=11)
        self._form_row(cfg, row, "REFP 参考 (V)", self.refp_entry); row += 1

        self.ain1_var = tk.StringVar(value="1.65")
        self.bias_entry = ttk.Entry(cfg, textvariable=self.ain1_var, width=11)
        self._form_row(cfg, row, "输入偏置 (V)", self.bias_entry); row += 1

        self.bias_hint_var = tk.StringVar(value="")
        tk.Label(cfg, textvariable=self.bias_hint_var, bg=COL_PANEL,
                 fg="#5d6472", font=(FONT, 7), justify="left", anchor="w",
                 wraplength=int(350 * s)).grid(row=row, column=0, columnspan=2,
                                               sticky="w", pady=(0, 1))
        row += 1

        self.avdd_var = tk.StringVar(value="3.3")
        self.avdd_entry = ttk.Entry(cfg, textvariable=self.avdd_var, width=11)
        self._form_row(cfg, row, "AVDD 电源 (V)", self.avdd_entry); row += 1

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
        self.cfg_hint = tk.Label(cfg, textvariable=self.cfg_hint_var,
                                 bg=COL_PANEL, fg=COL_MUTED,
                                 font=(FONT, 8), justify="left",
                                 wraplength=int(340 * s))
        self.cfg_hint.grid(row=row, column=0, columnspan=2, sticky="w",
                           pady=(4, 0))
        row += 1

        self.frame_hint_var = tk.StringVar(value="")
        tk.Label(cfg, textvariable=self.frame_hint_var, bg=COL_PANEL,
                 fg=COL_MUTED, font=(MONO, 8), justify="left",
                 wraplength=int(340 * s)).grid(row=row, column=0,
                                               columnspan=2, sticky="w",
                                               pady=(2, 0))

        # ---- actions ----
        lf3 = ttk.Labelframe(parent, text=" 操作 ", style="Panel.TLabelframe")
        lf3.pack(fill="x", padx=8, pady=5)
        act = tk.Frame(lf3, bg=COL_PANEL)
        act.pack(fill="x", padx=10, pady=(8, 10))
        act.columnconfigure(0, weight=1)
        act.columnconfigure(1, weight=1)

        self.apply_btn = ttk.Button(act, text="一键设置",
                                    style="Accent.TButton",
                                    command=self._apply_clicked)
        self.apply_btn.grid(row=0, column=0, columnspan=2, sticky="ew")

        self.start_btn = ttk.Button(act, text="开始采集", style="Go.TButton",
                                    command=self._start_clicked)
        self.start_btn.grid(row=1, column=0, sticky="ew", pady=(8, 0),
                            padx=(0, 4))
        self.stop_btn = ttk.Button(act, text="停止采集",
                                   command=self._stop_clicked)
        self.stop_btn.grid(row=1, column=1, sticky="ew", pady=(8, 0),
                           padx=(4, 0))

        self.read_btn = ttk.Button(act, text="读取当前寄存器",
                                   style="Small.TButton",
                                   command=self._read_clicked)
        self.read_btn.grid(row=2, column=0, columnspan=2, sticky="ew",
                           pady=(7, 0))

        self.settings_var = tk.StringVar(value="尚未初始化")
        self.settings_lbl = tk.Label(act, textvariable=self.settings_var,
                                     bg=COL_PANEL, fg=COL_MUTED,
                                     font=(FONT, 8), anchor="w",
                                     justify="left",
                                     wraplength=int(340 * s))
        self.settings_lbl.grid(row=3, column=0, columnspan=2, sticky="w",
                               pady=(7, 0))

        footer = tk.Frame(parent, bg=COL_PANEL)
        footer.pack(side="bottom", fill="x", padx=8, pady=(2, 6))
        ver_txt = f"v{APP_VERSION}"
        if APP_COMMIT_SHORT:
            ver_txt += f" · {APP_COMMIT_SHORT}"
        tk.Label(footer, text=ver_txt + " · 参考 TI SBAS752B",
                 bg=COL_PANEL, fg="#5d6472", font=(FONT, 7)).pack(side="left")
        self.check_update_btn = ttk.Button(
            footer, text="检查更新", style="Small.TButton",
            command=self._manual_update_check)
        self.check_update_btn.pack(side="right")

        for w in (self.gain_combo, self.mode_combo, self.dr_combo,
                  self.cm_combo, self.vref_combo, self.integ_combo):
            w.bind("<<ComboboxSelected>>", self._on_config_widget_change)
        for w in (self.port_combo, self.baud_combo, self.gain_combo,
                  self.mode_combo, self.dr_combo, self.cm_combo,
                  self.vref_combo, self.integ_combo):
            w.bind("<MouseWheel>", self._combo_wheel)

    def _build_main(self, parent):
        s = self.ui_scale
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=1)
        self._build_range_strip(parent)

        wave_body = tk.Frame(parent, bg=COL_BG)
        wave_body.grid(row=1, column=0, sticky="nsew", pady=(6 * s, 0))
        wave_body.grid_columnconfigure(0, weight=1)
        wave_body.grid_rowconfigure(1, weight=1)

        top = tk.Frame(wave_body, bg=COL_BG)
        top.grid(row=0, column=0, sticky="ew")
        self.add_wave_btn = ttk.Button(top, text="＋ 添加波形图",
                                       style="Small.TButton",
                                       command=self._open_add_wave_dialog)
        self.add_wave_btn.grid(row=0, column=0, sticky="w")
        self.global_stats_var = tk.StringVar(value="样本 0 · 速率 —")
        tk.Label(top, textvariable=self.global_stats_var, bg=COL_BG,
                 fg=COL_MUTED, font=(MONO, 8)).grid(row=0, column=1,
                                                   sticky="w", padx=(12, 0))
        tk.Label(top, text="滚轮缩 Y（关闭自动 Y）· 拖动框选 · 点击图例显示/隐藏",
                 bg=COL_BG, fg="#5d6472", font=(FONT, 7)).grid(
                     row=0, column=2, sticky="e")
        top.grid_columnconfigure(2, weight=1)

        self._wave_scroll = ScrollFrame(wave_body)
        self._wave_scroll.grid(row=1, column=0, sticky="nsew", pady=(4 * s, 0))
        self.wave_container = self._wave_scroll.inner

        self.wave_empty = tk.Frame(self.wave_container, bg=COL_PANEL)
        self.wave_empty.pack(fill="both", expand=True, pady=int(50 * s))
        tk.Label(self.wave_empty, text="还没有波形窗口", bg=COL_PANEL,
                 fg=COL_MUTED, font=(FONT, 12)).pack(pady=(10, 12))
        ttk.Button(self.wave_empty, text="＋ 添加波形图",
                   style="Accent.TButton",
                   command=self._open_add_wave_dialog).pack()

        self.danmaku_canvas = tk.Canvas(wave_body, height=int(26 * s),
                                        bg=COL_PANEL2, highlightthickness=0,
                                        bd=0)
        self.danmaku_canvas.grid(row=2, column=0, sticky="ew", pady=(4 * s, 0))
        self.danmaku_canvas.grid_remove()
        self._refresh_wave_area()

    def _build_range_strip(self, parent):
        s = self.ui_scale
        self.calc_lf = ttk.Labelframe(parent, text=" 测量范围 ",
                                      style="Panel.TLabelframe")
        self.calc_lf.grid(row=0, column=0, sticky="ew")
        body = tk.Frame(self.calc_lf, bg=COL_PANEL)
        body.pack(fill="x", padx=10, pady=(6, 8))
        body.grid_columnconfigure(0, weight=1)

        self.range_line1 = tk.StringVar(value="")
        self.range_line_fs = tk.StringVar(value="")
        self.range_line2 = tk.StringVar(value="")
        self.range_line3 = tk.StringVar(value="")
        stats = tk.Frame(body, bg=COL_PANEL)
        stats.grid(row=0, column=0, sticky="w")
        tk.Label(stats, textvariable=self.range_line1, bg=COL_PANEL,
                 fg=COL_FG, font=(MONO, 9), anchor="w").pack(side="left")
        tk.Label(stats, textvariable=self.range_line_fs, bg=COL_PANEL,
                 fg=COL_FG, font=(MONO, 9), anchor="w").pack(
                     side="left", padx=(18, 0))
        self.range_lbl2 = tk.Label(body, textvariable=self.range_line2,
                                   bg=COL_PANEL, fg=COL_GREEN,
                                   font=(FONT, 9, "bold"), anchor="w",
                                   justify="left",
                                   wraplength=int(900 * s))
        self.range_lbl2.grid(row=1, column=0, sticky="w", pady=(2, 0))
        self.range_bar = tk.Canvas(body, height=int(44 * s), width=1,
                                   bg="#1a1c23", highlightthickness=0, bd=0)
        self.range_bar.grid(row=2, column=0, sticky="ew", pady=(4, 2))
        self.range_bar.bind("<Configure>", lambda e: self._draw_range_bar())
        tk.Label(body, textvariable=self.range_line3, bg=COL_PANEL,
                 fg=COL_MUTED, font=(FONT, 7), anchor="w",
                 justify="left",
                 wraplength=int(900 * s)).grid(row=3, column=0,
                                               sticky="w")

        for var in (self.refp_var, self.ain1_var, self.avdd_var):
            var.trace_add("write", lambda *a: self._update_range())

    def _build_log_col(self, parent):
        s = self.ui_scale
        self._log_col = ttk.Labelframe(parent, text=" 事件日志 ",
                                       style="Panel.TLabelframe",
                                       width=int(320 * s))
        self._log_col.pack_propagate(False)
        logbody = tk.Frame(self._log_col, bg=COL_PANEL)
        logbody.pack(fill="both", expand=True, padx=8, pady=(5, 7))
        self.log_text = tk.Text(logbody, height=3, bg="#14161b", fg="#c7ccd6",
                                insertbackground=COL_FG, relief="flat",
                                bd=0, wrap="word", font=(MONO, 8),
                                padx=8, pady=4, state="disabled")
        lsb = ttk.Scrollbar(logbody, orient="vertical",
                            command=self.log_text.yview,
                            style="Vertical.TScrollbar")
        self.log_text.configure(yscrollcommand=lsb.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        lsb.pack(side="right", fill="y")

    def _apply_log_visibility(self):
        try:
            if self.log_visible_var.get():
                self._log_col.grid()
            else:
                self._log_col.grid_remove()
        except Exception:
            pass

    def _refresh_wave_area(self):
        try:
            if self.wave_windows:
                self.wave_empty.pack_forget()
                self.add_wave_btn.grid()
            else:
                self.wave_empty.pack(fill="both", expand=True, pady=40)
                self.add_wave_btn.grid_remove()
        except Exception:
            pass

    # -- channel selection -------------------------------------------------
    def _enabled_muxes(self):
        allowed = set(muxes_for_mode(self.channel_mode))
        return sorted(self.channel_sets.get(self.channel_mode, set()) & allowed)

    def _chan_button_text(self):
        muxes = self._enabled_muxes()
        if not muxes:
            return "未选择通道"
        if len(muxes) == 1:
            return MUX_SHORT.get(muxes[0], str(muxes[0]))
        head = MUX_SHORT.get(muxes[0], str(muxes[0]))
        return f"{head} 等 {len(muxes)} 个通道"

    def _rebuild_chan_menu(self):
        try:
            self.chan_menu.delete(0, "end")
        except Exception:
            return
        self.chan_vars = {}
        for mux in muxes_for_mode(self.channel_mode):
            var = tk.BooleanVar(
                value=mux in self.channel_sets.get(self.channel_mode, set()))
            self.chan_vars[mux] = var
            self.chan_menu.add_checkbutton(
                label=MUX_SHORT.get(mux, str(mux)), variable=var,
                command=self._on_channel_menu_change)
        self.chan_menu_btn.configure(text=self._chan_button_text())
        self._update_chan_hint()

    def _on_channel_mode_change(self, _=None):
        mode = self.chan_mode_var.get()
        if mode not in CHANNEL_MODES:
            mode = "差分"
        self.channel_mode = mode
        if not self.channel_sets.get(mode):
            self.channel_sets[mode] = {muxes_for_mode(mode)[0]}
        self._rebuild_chan_menu()
        self._on_config_widget_change()

    def _on_channel_menu_change(self):
        chosen = {mux for mux, var in self.chan_vars.items() if var.get()}
        self.channel_sets[self.channel_mode] = chosen
        self.chan_menu_btn.configure(text=self._chan_button_text())
        self._update_chan_hint()
        self._on_config_widget_change()

    def _update_chan_hint(self):
        muxes = self._enabled_muxes()
        if not muxes:
            text = "未选择通道，无法开始采集。"
        else:
            names = "、".join(MUX_SHORT.get(m, str(m)) for m in muxes)
            if self.channel_mode == "差分":
                text = f"差分扫描：{names}；输入偏置为参考端电压。"
            elif self.channel_mode == "单端":
                text = f"单端扫描：{names}；AINN = AVSS，偏置固定 0 V。"
            else:
                text = f"监测模式：{names}；量程计算不适用。"
        try:
            self.chan_hint_var.set(text)
        except Exception:
            pass

    def _log_colors(self):
        self.log_text.tag_configure("info", foreground="#9aa3b2")
        self.log_text.tag_configure("ok", foreground="#5ee6a0")
        self.log_text.tag_configure("warn", foreground=COL_WARN)
        self.log_text.tag_configure("error", foreground="#ff7b7b")
        self.log_text.tag_configure("time", foreground="#5d6472")

    # -- small UI callbacks / waveform views ------------------------------
    def _on_canvas_cfg(self, _=None):
        self._plot_dirty = True

    def _plot_rate_value(self):
        if self._acq_rate:
            return float(self._acq_rate)
        try:
            return max(1.0, float(self._ui_config().data_rate))
        except Exception:
            return 1.0

    def _plot_lsb(self):
        if self._acq_lsb:
            return float(self._acq_lsb)
        return self._current_lsb()

    def _plot_bias(self):
        if self._acq_bias is not None:
            return float(self._acq_bias)
        try:
            return self._effective_bias(self._ui_config(),
                                        float(self.ain1_var.get()))
        except Exception:
            return 0.0

    @staticmethod
    def _safe_name(name: str) -> str:
        name = re.sub(r'[\\/:*?"<>|]+', "_", str(name)).strip()
        return name or "wave"

    def _on_plot_press(self, window, event):
        state = window.state
        for x0, y0, x1, y1, key in state.legend_hits:
            if x0 <= event.x <= x1 and y0 <= event.y <= y1:
                hidden = set(window.spec.hidden)
                if key in hidden:
                    hidden.discard(key)
                else:
                    hidden.add(key)
                window.spec.hidden = sorted(hidden)
                self._plot_dirty = True
                return "break"
        if state.last is None:
            return "break"
        l = state.last
        if not (l["ml"] <= event.x <= l["ml"] + l["pw"] and
                l["mt"] <= event.y <= l["mt"] + l["ph"]):
            return "break"
        state.selecting = True
        state.box = (event.x, event.y)
        state.box_rect = state.canvas.create_rectangle(
            event.x, event.y, event.x, event.y, outline=COL_ACCENT,
            dash=(3, 2), width=1)
        return "break"

    def _on_plot_drag(self, window, event):
        state = window.state
        if state is None or not state.selecting or state.box is None:
            return "break"
        x0, y0 = state.box
        if state.box_rect is not None:
            state.canvas.coords(state.box_rect, x0, y0, event.x, event.y)
        return "break"

    def _on_plot_release(self, window, event):
        state = window.state
        if state is None or not state.selecting:
            return "break"
        state.selecting = False
        x0, y0 = state.box or (event.x, event.y)
        state.box = None
        if state.box_rect is not None:
            try:
                state.canvas.delete(state.box_rect)
            except Exception:
                pass
            state.box_rect = None
        l = state.last
        x1, y1 = event.x, event.y
        if l is None or abs(x1 - x0) < 6 or abs(y1 - y0) < 6:
            self._plot_dirty = True
            return "break"
        ml, mt, pw, ph = l["ml"], l["mt"], l["pw"], l["ph"]
        x0c = min(max(min(x0, x1), ml), ml + pw)
        x1c = min(max(max(x0, x1), ml), ml + pw)
        y0c = min(max(min(y0, y1), mt), mt + ph)
        y1c = min(max(max(y0, y1), mt), mt + ph)
        if x1c - x0c < 3 or y1c - y0c < 3:
            self._plot_dirty = True
            return "break"
        xa = l["xmin"] + (x0c - ml) / max(1, pw) * (l["xmax"] - l["xmin"])
        xb = l["xmin"] + (x1c - ml) / max(1, pw) * (l["xmax"] - l["xmin"])
        ya = l["ymax"] - (y0c - mt) / max(1, ph) * (l["ymax"] - l["ymin"])
        yb = l["ymax"] - (y1c - mt) / max(1, ph) * (l["ymax"] - l["ymin"])
        ymin_sel, ymax_sel = min(ya, yb), max(ya, yb)
        if ymax_sel <= ymin_sel:
            self._plot_dirty = True
            return "break"
        state.y_view = (ymin_sel, ymax_sel)
        window.auto_var.set(False)
        window.spec.auto_y = False
        ids = [pt[0] for pt in l.get("pts", [])
               if min(xa, xb) <= pt[1] <= max(xa, xb)]
        if ids:
            state.index_view = (min(ids), max(ids))
        self._plot_dirty = True
        return "break"

    def _on_plot_wheel(self, window, event):
        state = window.state
        if state is None or state.last is None:
            return "break"
        if window.auto_var.get():
            return "break"
        l = state.last
        if not (l["ml"] <= event.x <= l["ml"] + l["pw"] and
                l["mt"] <= event.y <= l["mt"] + l["ph"]):
            return "break"
        if state.y_view is not None:
            ymin, ymax = state.y_view
        else:
            ymin, ymax = l["ymin"], l["ymax"]
        span = ymax - ymin
        if span <= 0:
            return "break"
        cursor = ymax - (event.y - l["mt"]) / max(1, l["ph"]) * span
        factor = 0.82 if getattr(event, "delta", 0) > 0 else 1.0 / 0.82
        new_span = span * factor
        new_span = max(min(new_span, 1e15), max(1e-15, abs(cursor) * 1e-12))
        ratio = (cursor - ymin) / span
        new_min = cursor - ratio * new_span
        state.y_view = (new_min, new_min + new_span)
        self._plot_dirty = True
        return "break"

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

    def _clear_clicked(self):
        self.worker.clear_stats()
        for window in self.wave_windows:
            window.reset()
        self._plot_dirty = True

    # -- danmaku -----------------------------------------------------------
    def _queue_danmaku(self, level, text):
        try:
            if not hasattr(self, "danmaku_canvas"):
                return
            text = str(text).strip()
            if not text or text.startswith("TX "):
                return
            if text.startswith(("[模拟器]", "[配置]", "[参数]", "帧格式",
                                "扫描通道", "公式序列", "回读 ")):
                return
            if level == "info":
                keywords = ("串口", "发现", "连接", "断开", "开始采集",
                            "已停止", "触发", "保存", "文件", "输出")
                if not any(k in text for k in keywords):
                    return
            self._danmaku_queue.append((level, text[:140]))
            if self._danmaku_active is None and self._danmaku_after is None:
                self._play_danmaku()
        except Exception:
            pass

    def _play_danmaku(self):
        self._danmaku_after = None
        if self._danmaku_active is not None or not self._danmaku_queue:
            if self._danmaku_active is None:
                try:
                    self.danmaku_canvas.grid_remove()
                except Exception:
                    pass
            return
        level, text = self._danmaku_queue.pop(0)
        colors = {"info": "#9aa3b2", "ok": "#5ee6a0",
                  "warn": COL_WARN, "error": "#ff7b7b"}
        c = self.danmaku_canvas
        try:
            c.grid()
            c.update_idletasks()
            w = max(1, c.winfo_width())
            h = max(1, c.winfo_height())
            ts = datetime.now().strftime("%H:%M:%S")
            item = c.create_text(w + 10, h // 2, anchor="w",
                                 text=f"[{ts}] {text}",
                                 fill=colors.get(level, COL_FG),
                                 font=(FONT, 9))
            self._danmaku_active = item
            self._danmaku_x = w + 10
            self._danmaku_tick()
        except Exception:
            self._danmaku_active = None

    def _danmaku_tick(self):
        if self._danmaku_active is None:
            return
        c = self.danmaku_canvas
        try:
            if not c.winfo_exists():
                return
            self._danmaku_x -= max(2, int(4 * self.ui_scale))
            c.coords(self._danmaku_active, self._danmaku_x,
                     max(1, c.winfo_height()) // 2)
            bbox = c.bbox(self._danmaku_active)
            if bbox and bbox[2] < 0:
                c.delete(self._danmaku_active)
                self._danmaku_active = None
                self._danmaku_after = self.root.after(100, self._play_danmaku)
            else:
                self._danmaku_after = self.root.after(30, self._danmaku_tick)
        except Exception:
            self._danmaku_active = None

    # -- output settings dialogs ------------------------------------------
    OUTPUT_LABELS = {
        "raw": "原始码流输出 (.bin)",
        "decoded": "解码输出 (.csv)",
        "analog": "模拟值输出 (.csv)",
        "image": "图像保存 (PNG)",
    }

    def _open_output_dialog(self, kind):
        item = self.outputs.get(kind) or {"enabled": True,
                                          "dir": default_data_dir()}
        top = tk.Toplevel(self.root)
        top.title("文件输出包 · " + self.OUTPUT_LABELS.get(kind, kind))
        top.configure(bg=COL_BG)
        top.transient(self.root)
        top.geometry(f"+{self.root.winfo_rootx() + 180}"
                     f"+{self.root.winfo_rooty() + 140}")
        body = tk.Frame(top, bg=COL_BG)
        body.pack(fill="both", expand=True, padx=14, pady=12)
        enabled_var = tk.BooleanVar(value=bool(item.get("enabled", True)))
        ttk.Checkbutton(body, text="使能", variable=enabled_var).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))
        tk.Label(body, text="输出地址", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=1, column=0, sticky="w", pady=4)
        dir_var = tk.StringVar(value=item.get("dir") or default_data_dir())
        entry = ttk.Entry(body, textvariable=dir_var, width=52)
        entry.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=4)
        body.grid_columnconfigure(1, weight=1)

        def pick_dir():
            d = filedialog.askdirectory(
                initialdir=dir_var.get() or default_data_dir(),
                title="选择输出地址", parent=top)
            if d:
                dir_var.set(d)

        def open_dir():
            d = dir_var.get().strip() or default_data_dir()
            try:
                Path(d).mkdir(parents=True, exist_ok=True)
                os.startfile(d)  # type: ignore[attr-defined]
            except Exception as exc:
                messagebox.showerror(APP_NAME, f"无法打开文件夹：\n{exc}",
                                     parent=top)

        btns = tk.Frame(body, bg=COL_BG)
        btns.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        ttk.Button(btns, text="选择", style="Small.TButton",
                   command=pick_dir).pack(side="left")
        ttk.Button(btns, text="打开", style="Small.TButton",
                   command=open_dir).pack(side="left", padx=(6, 0))

        def ok():
            d = dir_var.get().strip() or default_data_dir()
            self.outputs[kind] = {"enabled": bool(enabled_var.get()),
                                  "dir": d}
            top.destroy()
            self._save_settings_file()

        ttk.Button(btns, text="取消", command=top.destroy).pack(side="right")
        ttk.Button(btns, text="确定", style="Accent.TButton",
                   command=ok).pack(side="right", padx=(0, 6))
        top.grab_set()

    # -- add waveform dialog ----------------------------------------------
    def _open_add_wave_dialog(self):
        muxes = self._enabled_muxes()
        if not muxes:
            messagebox.showwarning(APP_NAME, "请先在“初始化设置 → 通道使能”中选择至少一个通道。")
            return
        s = self.ui_scale
        top = tk.Toplevel(self.root)
        top.title("添加波形图")
        top.configure(bg=COL_BG)
        top.transient(self.root)
        top.geometry(f"{int(560 * s)}x{int(520 * s)}"
                     f"+{self.root.winfo_rootx() + 220}"
                     f"+{self.root.winfo_rooty() + 90}")
        body = tk.Frame(top, bg=COL_BG)
        body.pack(fill="both", expand=True, padx=16, pady=12)
        body.grid_columnconfigure(1, weight=1)

        tk.Label(body, text="勾选通道（可多选）", bg=COL_BG, fg=COL_ACCENT,
                 font=(FONT, 9, "bold")).grid(row=0, column=0, columnspan=2,
                                              sticky="w", pady=(0, 4))
        ch_frame = tk.Frame(body, bg=COL_PANEL)
        ch_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ch_vars = {}
        for i, mux in enumerate(muxes):
            var = tk.BooleanVar(value=True)
            ch_vars[mux] = var
            ttk.Checkbutton(ch_frame, text=MUX_SHORT.get(mux, str(mux)),
                            variable=var).grid(row=i // 2, column=i % 2,
                                               sticky="w", padx=8, pady=2)

        custom_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(body, text="自定义公式（f(x)）", variable=custom_var,
                        command=lambda: _toggle_custom()).grid(
                            row=2, column=0, columnspan=2, sticky="w",
                            pady=(4, 2))
        tk.Label(body, text="f(x) =", bg=COL_BG, fg=COL_MUTED,
                 font=(MONO, 9)).grid(row=3, column=0, sticky="w")
        formula_var = tk.StringVar(value="ain")
        formula_entry = ttk.Entry(body, textvariable=formula_var)
        formula_entry.grid(row=3, column=1, sticky="ew", pady=2)
        tk.Label(body, text="X =", bg=COL_BG, fg=COL_MUTED,
                 font=(MONO, 9)).grid(row=4, column=0, sticky="w")
        xf_var = tk.StringVar(value="t")
        xf_entry = ttk.Entry(body, textvariable=xf_var)
        xf_entry.grid(row=4, column=1, sticky="ew", pady=2)

        tk.Label(body, text="X 单位", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=5, column=0, sticky="w", pady=(6, 2))
        xu_var = tk.StringVar(value="s")
        ttk.Entry(body, textvariable=xu_var, width=10).grid(
            row=5, column=1, sticky="w", pady=(6, 2))
        tk.Label(body, text="Y 单位", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=6, column=0, sticky="w", pady=2)
        yu_var = tk.StringVar(value="V")
        ttk.Entry(body, textvariable=yu_var, width=10).grid(
            row=6, column=1, sticky="w", pady=2)
        tk.Label(body, text="表头", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=7, column=0, sticky="w", pady=2)
        title_var = tk.StringVar(value=f"波形 {self._next_wid}")
        ttk.Entry(body, textvariable=title_var).grid(
            row=7, column=1, sticky="ew", pady=2)
        show_legend_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(body, text="显示图例", variable=show_legend_var).grid(
            row=8, column=0, sticky="w", pady=(6, 2))
        tk.Label(body, text="图例名称", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=9, column=0, sticky="w", pady=2)
        legend_var = tk.StringVar(value="")
        ttk.Entry(body, textvariable=legend_var).grid(
            row=9, column=1, sticky="ew", pady=2)
        tk.Label(body, text="按勾选顺序用逗号分隔，留空则自动命名",
                 bg=COL_BG, fg="#5d6472", font=(FONT, 7)).grid(
                     row=10, column=1, sticky="w")

        def _toggle_custom():
            state = "normal" if custom_var.get() else "disabled"
            formula_entry.configure(state=state)
            xf_entry.configure(state=state)

        _toggle_custom()

        def add():
            chosen = [m for m in muxes if ch_vars[m].get()]
            if not chosen:
                messagebox.showwarning(APP_NAME, "请至少勾选一个通道。",
                                       parent=top)
                return
            use_formula = bool(custom_var.get())
            formula = formula_var.get().strip() or "ain"
            xf = xf_var.get().strip() or "t"
            if use_formula:
                try:
                    formula_engine.compile_expr(formula,
                                                FORMULA_VARIABLE_TYPES)
                    formula_engine.compile_expr(xf, FORMULA_VARIABLE_TYPES)
                except Exception as exc:
                    messagebox.showerror(APP_NAME, f"公式错误：{exc}",
                                         parent=top)
                    return
            names = [x.strip() for x in legend_var.get().split(",")
                     if x.strip()]
            x_unit = xu_var.get().strip() or "s"
            y_unit = yu_var.get().strip() or "V"
            title = title_var.get().strip() or f"波形 {self._next_wid}"
            width = max(1, len(str(self._next_wid)) + 1)
            wid = self._next_wid
            series = []
            for i, mux in enumerate(chosen):
                label = names[i] if i < len(names) else MUX_SHORT.get(
                    mux, str(mux))
                if use_formula:
                    label = (names[i] if i < len(names)
                             else f"{label} · {formula}")
                series.append(SeriesSpec(
                    key=f"w{wid}_s{i}", mux=mux,
                    mode="formula" if use_formula else "raw",
                    label=label, formula=formula if use_formula else "",
                    x_formula=xf if use_formula else "t",
                    x_unit=x_unit))
            spec = WaveSpec(wid=0, title=title, x_unit=x_unit,
                            y_unit=y_unit,
                            show_legend=bool(show_legend_var.get()),
                            series=series)
            self._add_wave_window(spec)
            top.destroy()

        btns = tk.Frame(body, bg=COL_BG)
        btns.grid(row=11, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Button(btns, text="取消", command=top.destroy).pack(side="right")
        ttk.Button(btns, text="添加", style="Accent.TButton",
                   command=add).pack(side="right", padx=(0, 6))
        top.grab_set()

    # -- waveform window management ---------------------------------------
    def _add_wave_window(self, spec: WaveSpec, save=True):
        if spec.wid <= 0:
            spec.wid = self._next_wid
        self._next_wid = max(self._next_wid, spec.wid + 1)
        if not spec.series:
            return None
        if not spec.trigger_source:
            spec.trigger_source = spec.series[0].key
        self.wave_specs.append(spec)
        window = WaveWindow(self, spec)
        self.wave_windows.append(window)
        self._refresh_wave_area()
        if save:
            self._save_settings_file()
        return window

    def _delete_wave_window(self, window):
        try:
            self.wave_windows.remove(window)
        except ValueError:
            pass
        try:
            self.wave_specs.remove(window.spec)
        except ValueError:
            pass
        window.destroy()
        self._refresh_wave_area()
        self._save_settings_file()
        self._plot_dirty = True

    @staticmethod
    def _spec_to_dict(spec: WaveSpec):
        return {
            "wid": spec.wid, "title": spec.title,
            "x_unit": spec.x_unit, "y_unit": spec.y_unit,
            "show_legend": bool(spec.show_legend),
            "timebase": spec.timebase, "auto_y": bool(spec.auto_y),
            "trigger_mode": spec.trigger_mode,
            "trigger_value": spec.trigger_value,
            "trigger_source": spec.trigger_source,
            "hidden": list(spec.hidden),
            "series": [{"key": sr.key, "mux": sr.mux, "mode": sr.mode,
                        "label": sr.label, "formula": sr.formula,
                        "x_formula": sr.x_formula, "x_unit": sr.x_unit}
                       for sr in spec.series],
        }

    @staticmethod
    def _dict_to_spec(data: dict):
        series = []
        for i, sr in enumerate(data.get("series") or []):
            try:
                series.append(SeriesSpec(
                    key=str(sr.get("key") or f"s{i}"),
                    mux=int(sr.get("mux", 0)),
                    mode=str(sr.get("mode") or "raw"),
                    label=str(sr.get("label") or ""),
                    formula=str(sr.get("formula") or ""),
                    x_formula=str(sr.get("x_formula") or "t"),
                    x_unit=str(sr.get("x_unit") or "s")))
            except Exception:
                continue
        return WaveSpec(
            wid=int(data.get("wid") or 0),
            title=str(data.get("title") or "波形"),
            x_unit=str(data.get("x_unit") or "s"),
            y_unit=str(data.get("y_unit") or "V"),
            show_legend=bool(data.get("show_legend", True)),
            timebase=str(data.get("timebase") or "2 s"),
            auto_y=bool(data.get("auto_y", True)),
            trigger_mode=str(data.get("trigger_mode") or "关闭"),
            trigger_value=float(data.get("trigger_value") or 0.0),
            trigger_source=str(data.get("trigger_source") or ""),
            series=series,
            hidden=list(data.get("hidden") or []))

    def _restore_wave_windows(self, items):
        restored = False
        for item in items:
            if not isinstance(item, dict):
                continue
            spec = self._dict_to_spec(item)
            if spec.series:
                self._add_wave_window(spec, save=False)
                restored = True
        if not restored:
            old_formula = self.settings.get("formula")
            muxes = self._enabled_muxes()
            if old_formula and muxes:
                mux = muxes[0]
                spec = WaveSpec(
                    wid=0, title="自定义波形",
                    x_unit=str(self.settings.get("x_unit") or "s"),
                    series=[SeriesSpec(
                        key="legacy", mux=mux, mode="formula",
                        label=f"{MUX_SHORT.get(mux, '')} · {old_formula}",
                        formula=str(old_formula),
                        x_formula=str(self.settings.get("x_formula") or "t"),
                        x_unit=str(self.settings.get("x_unit") or "s"))])
                self._add_wave_window(spec, save=False)
        self._refresh_wave_area()

    # -- save image --------------------------------------------------------
    def _save_window_image(self, window):
        if ImageGrab is None:
            messagebox.showerror(
                APP_NAME, "图像保存需要 Pillow 组件（pip install pillow）。")
            return
        image_cfg = self.outputs.get("image") or {}
        if not image_cfg.get("enabled", True):
            if not messagebox.askyesno(
                    APP_NAME,
                    "图像保存未启用，仍要保存吗？\n"
                    "可在 设置 → 文件输出包 → 图像保存 中启用并设置地址。"):
                return
        folder = Path(image_cfg.get("dir") or default_data_dir())
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"无法创建图像目录：\n{exc}")
            return
        default = (self._safe_name(window.spec.title) + "_" +
                   datetime.now().strftime("%Y%m%d_%H%M%S") + ".png")
        name = simpledialog.askstring("保存图像", "文件名：",
                                      initialvalue=default,
                                      parent=self.root)
        if not name:
            return
        name = name.strip()
        stem, ext = os.path.splitext(name)
        name = self._safe_name(stem) + (ext if ext else ".png")
        path = folder / name
        try:
            canvas = window.active_canvas()
            self.root.update_idletasks()
            x = canvas.winfo_rootx()
            y = canvas.winfo_rooty()
            cw = canvas.winfo_width()
            ch = canvas.winfo_height()
            img = ImageGrab.grab(bbox=(x, y, x + cw, y + ch))
            img.save(path)
            self._log("ok", f"波形图像已保存：{path}")
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"保存图像失败：\n{exc}")

    # -- stop output dialog -----------------------------------------------
    def _show_stop_dialog(self, paths: dict):
        if self._stop_dialog_open:
            return
        if not any(paths.get(k) for k in ("raw", "decoded", "analog")):
            return
        items = [(key, paths.get(key) or "")
                 for key in ("raw", "decoded", "analog")]
        self._stop_dialog_open = True
        top = tk.Toplevel(self.root)
        top.title("输出文件命名")
        top.configure(bg=COL_BG)
        top.transient(self.root)
        top.geometry(f"+{self.root.winfo_rootx() + 240}"
                     f"+{self.root.winfo_rooty() + 180}")
        body = tk.Frame(top, bg=COL_BG)
        body.pack(fill="both", expand=True, padx=16, pady=14)
        body.grid_columnconfigure(1, weight=1)
        tk.Label(body, text="停止采集 — 可修改已使能输出的文件名",
                 bg=COL_BG, fg=COL_ACCENT, font=(FONT, 10, "bold")).grid(
                     row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        names = {"raw": "原始码流", "decoded": "解码输出", "analog": "模拟值"}
        entries = {}
        for i, (key, path) in enumerate(items, start=1):
            p = Path(path) if path else None
            tk.Label(body, text=names.get(key, key), bg=COL_BG,
                     fg=COL_MUTED, font=(FONT, 9)).grid(
                         row=i, column=0, sticky="w", pady=4)
            var = tk.StringVar(value=p.name if p else "未启用")
            ent = ttk.Entry(body, textvariable=var, width=44)
            ent.grid(row=i, column=1, sticky="ew", padx=(8, 0), pady=4)
            if p is None:
                ent.configure(state="disabled")
                directory = ""
            else:
                directory = str(p.parent)
            tk.Label(body, text=directory, bg=COL_BG, fg="#5d6472",
                     font=(FONT, 7)).grid(row=i, column=2, sticky="w",
                                          padx=(8, 0))
            entries[key] = (p, var)

        def ok():
            errors = []
            for key, (old, var) in entries.items():
                if old is None:
                    continue
                name = var.get().strip()
                if not name or name == old.name:
                    continue
                new = old.parent / name
                try:
                    if new.exists() and new != old:
                        if not messagebox.askyesno(
                                APP_NAME, f"{new.name} 已存在，覆盖吗？",
                                parent=top):
                            continue
                    os.replace(old, new)
                except Exception as exc:
                    errors.append(f"{old.name} → {name}: {exc}")
            if errors:
                messagebox.showerror(APP_NAME,
                                     "部分文件重命名失败：\n" + "\n".join(errors),
                                     parent=top)
            top.destroy()
            self._stop_dialog_open = False
            self._save_settings_file()

        def cancel():
            top.destroy()
            self._stop_dialog_open = False

        btns = tk.Frame(body, bg=COL_BG)
        btns.grid(row=len(items) + 1, column=0, columnspan=3, sticky="ew",
                  pady=(12, 0))
        ttk.Button(btns, text="保留原名", command=cancel).pack(side="right")
        ttk.Button(btns, text="确定", style="Accent.TButton",
                   command=ok).pack(side="right", padx=(0, 6))
        top.protocol("WM_DELETE_WINDOW", cancel)
        top.grab_set()


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
        try:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", f"[{ts}] ", ("time",))
            self.log_text.insert("end", text + "\n", (level,))
            lines = int(self.log_text.index("end-1c").split(".")[0])
            if lines > 700:
                self.log_text.delete("1.0", "81.0")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        except Exception:
            pass
        self._queue_danmaku(level, text)

    # -- UI -> config ------------------------------------------------------
    def _refresh_dr_values(self, code=3):
        turbo = value_for(MODE_CHOICES, self.mode_var.get(), False)
        rates = DR_TABLE[bool(turbo)]
        code = max(0, min(6, int(code)))
        self._dr_code = code
        self.dr_combo.configure(values=[f"{r} SPS" for r in rates])
        self.dr_var.set(f"{rates[code]} SPS")

    def _ui_config(self) -> DeviceConfig:
        muxes = self._enabled_muxes()
        mux = muxes[0] if muxes else 0
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
        muxes = self._enabled_muxes()
        mux = muxes[0] if muxes else 0
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
        self._sync_input_ui(cfg)
        self._update_range()

    def _sync_input_ui(self, cfg=None):
        """The bias entry is editable only for differential channels."""
        cfg = cfg or self._ui_config()
        differential = is_differential_mux(cfg.mux)
        if differential:
            self.bias_entry.state(["!disabled"])
            self.bias_hint_var.set("差分输入：填入参考端（AINN）的直流电压。")
        else:
            self.bias_entry.state(["disabled"])
            if cfg.mux < 12:
                self.bias_hint_var.set("单端输入：AINN = AVSS，输入偏置固定为 0 V。")
            else:
                self.bias_hint_var.set("监测 / 短接模式不使用输入偏置。")
        self._plot_dirty = True

    def _range_values(self):
        refp = float(self.refp_var.get().strip())
        bias = float(self.ain1_var.get().strip())
        avdd = float(self.avdd_var.get().strip())
        if not all(math.isfinite(v) for v in (refp, bias, avdd)):
            raise ValueError("输入必须是有限数值")
        if not (refp > 0 and avdd > 0):
            raise ValueError("REFP / AVDD 必须大于 0")
        return refp, bias, avdd

    def _effective_bias(self, cfg, bias: float) -> float:
        """Single-ended and monitor modes reference AVSS (0 V)."""
        return float(bias) if is_differential_mux(cfg.mux) else 0.0

    def _update_range(self):
        cfg = self._ui_config()
        differential = is_differential_mux(cfg.mux)
        try:
            refp, bias_raw, avdd = self._range_values()
        except Exception:
            self.range_line1.set("请输入有效的 REFP / 输入偏置 / AVDD（单位 V）")
            self.range_line_fs.set("")
            self.range_line2.set("输入无效")
            self.range_lbl2.configure(fg=COL_ERR)
            self.range_line3.set("")
            self.range_info = None
            self._draw_range_bar()
            return
        bias = self._effective_bias(cfg, bias_raw)
        info = compute_range(cfg, refp, bias, avdd)
        self.range_info = info
        self._last_valid = (cfg, refp, bias, avdd, info)
        self.range_line1.set(
            f"VREF {info.vref:.5g} V · 增益 ×{info.gain} · "
            f"{'PGA 旁路' if info.bypass else 'PGA 使能'}")
        self.range_line_fs.set(
            f"满量程 ±{fmt_eng(info.fs)} · 1 LSB = {fmt_eng(info.lsb)}")
        if info.valid:
            down = bias - info.usable_lo
            up = info.usable_hi - bias
            if differential:
                detail = (f"相对输入偏置 −{fmt_eng(down)} / "
                          f"+{fmt_eng(up)}")
            else:
                detail = "单端，参考端 AVSS"
            self.range_line2.set(
                f"AIN 可用范围 {info.usable_lo:.4f} V ~ "
                f"{info.usable_hi:.4f} V（{detail}）")
            self.range_lbl2.configure(fg=COL_GREEN)
        else:
            self.range_line2.set("范围不可用：" + info.message)
            self.range_lbl2.configure(fg=COL_ERR)
        if cfg.mux >= 12:
            self.range_line3.set("监测 / 短接模式，量程计算不适用。")
        else:
            if differential:
                note = "差分：绝对电压 = VIN + 输入偏置"
            else:
                note = "单端：参考端 AVSS，偏置固定 0 V"
            self.range_line3.set(
                note + f" · PGA 绝对输入窗口 {info.abs_lo:.3f} ~ "
                f"{info.abs_hi:.3f} V · LSB = 2·VREF/(增益·2^24)")
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
        cfg = self._ui_config()
        try:
            _, bias_raw, avdd = self._range_values()
        except Exception:
            c.create_text(w // 2, h // 2, text="等待有效输入…", fill=COL_MUTED,
                          font=(FONT, 8))
            return
        if info is None:
            c.create_text(w // 2, h // 2, text="等待有效配置…", fill=COL_MUTED,
                          font=(FONT, 8))
            return
        if cfg.mux >= 12:
            c.create_text(w // 2, h // 2, text="监测 / 短接模式 · 量程不适用",
                          fill=COL_MUTED, font=(FONT, 8))
            return
        differential = is_differential_mux(cfg.mux)
        bias = self._effective_bias(cfg, bias_raw)

        s = self.ui_scale
        pad = int(14 * s)
        axis_lo = min(0.0, info.abs_lo, info.usable_lo, bias) - 0.03 * max(avdd, 0.1)
        axis_hi = max(avdd, info.abs_hi, info.usable_hi, bias) + 0.03 * max(avdd, 0.1)
        span = max(1e-9, axis_hi - axis_lo)

        def X(v):
            return pad + (v - axis_lo) / span * (w - 2 * pad)

        top = int(14 * s)
        bar_h = max(5, int(7 * s))
        gap = max(2, int(3 * s))
        bottom = top + bar_h * 2 + gap
        c.create_rectangle(pad, top, w - pad, bottom,
                           outline="", fill="#20232b")
        # PGA absolute window
        c.create_rectangle(X(info.abs_lo), top, X(info.abs_hi), top + bar_h,
                           outline="", fill="#3c4354")
        # usable input window
        if info.usable_hi > info.usable_lo:
            color = "#1f7a5a" if info.valid else "#7a3b3b"
            c.create_rectangle(X(info.usable_lo), top + bar_h + gap,
                               X(info.usable_hi), top + bar_h * 2 + gap,
                               outline="", fill=color)
        if differential:
            x = X(bias)
            c.create_line(x, int(5 * s), x, bottom, fill=COL_WARN,
                          dash=(3, 2))
            c.create_polygon(x - int(5 * s), int(1 * s),
                             x + int(5 * s), int(1 * s),
                             x, int(9 * s), fill=COL_WARN, outline="")
            to_right = x < w / 2
            c.create_text(x + int(7 * s) if to_right else x - int(7 * s),
                          int(0 * s), text="输入偏置",
                          anchor="nw" if to_right else "ne",
                          fill=COL_WARN, font=(FONT, 7, "bold"))
        c.create_text(X(0.0), h - int(1 * s), text="0 V", anchor="sw",
                      fill="#5d6472", font=(FONT, 7))
        c.create_text(w - pad, h - int(1 * s), text=f"{avdd:.2f} V",
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
        muxes = self._enabled_muxes()
        if not muxes:
            messagebox.showwarning(APP_NAME, "请至少使能一个输入通道")
            return
        try:
            cfg = self._ui_config()
            refp, bias_raw, avdd = self._range_values()
            bias = self._effective_bias(cfg, bias_raw)
            info = compute_range(cfg, refp, bias, avdd)
            if refp <= 0:
                raise ValueError("REFP 必须大于 0")
        except Exception as exc:
            messagebox.showerror(APP_NAME, f"输入参数无效：\n{exc}")
            return

        # Validate all formula series before starting.
        series_meta = []
        for spec in self.wave_specs:
            for sr in spec.series:
                if sr.mode == "formula":
                    try:
                        formula_engine.compile_expr(
                            sr.formula or "ain", FORMULA_VARIABLE_TYPES)
                        formula_engine.compile_expr(
                            sr.x_formula or "t", FORMULA_VARIABLE_TYPES)
                    except Exception as exc:
                        messagebox.showerror(
                            APP_NAME,
                            f"波形“{spec.title}”的公式错误：\n{exc}")
                        return
                series_meta.append({
                    "key": sr.key, "mux": sr.mux, "mode": sr.mode,
                    "label": sr.label, "formula": sr.formula,
                    "x_formula": sr.x_formula, "x_unit": sr.x_unit,
                })

        if not st.get("sim"):
            try:
                baud_now = int(st.get("baud") or self.baud_var.get())
            except Exception:
                baud_now = 115200
            need_bps = cfg.frame_len * 10 * cfg.data_rate
            if need_bps > baud_now * 0.85:
                self._log("warn", f"带宽警告：{cfg.frame_len} B/帧 × "
                                   f"{cfg.data_rate} SPS 约需 "
                                   f"{need_bps/1000:.1f} kbit/s，接近/超过 "
                                   f"{baud_now} baud，可能出现丢帧。")
        if not info.valid:
            self._log("warn", "测量范围警告：" + info.message)
            if not messagebox.askyesno(APP_NAME,
                                       "当前参数下测量范围不可用：\n\n" +
                                       (info.message or "未知原因") +
                                       "\n\n仍要开始采集吗？"):
                return

        base = (f"MagSensor_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}")
        suffixes = {"raw": "_raw.bin", "decoded": "_samples.csv",
                    "analog": "_analog.csv"}
        output_paths = {}
        for key in ("raw", "decoded", "analog"):
            item = self.outputs.get(key) or {}
            if self.demo or not item.get("enabled"):
                output_paths[key] = None
                continue
            folder = Path(item.get("dir") or default_data_dir())
            try:
                folder.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                messagebox.showerror(APP_NAME,
                                     f"无法创建输出目录：\n{folder}\n{exc}")
                return
            output_paths[key] = str(folder / (base + suffixes[key]))
        record = any(output_paths.values())

        meta = {
            "tool": APP_NAME, "version": APP_VERSION,
            "created": datetime.now().isoformat(timespec="seconds"),
            "port": st.get("port", ""), "baud": st.get("baud", 0),
            "lsb": info.lsb, "ain1": bias,
            "vref": info.vref, "gain": info.gain, "fs": info.fs,
            "avdd": avdd, "refp": refp,
            "scan_channels": muxes,
            "channel_mode": self.channel_mode,
            "outputs": output_paths,
            "series": series_meta,
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
            "formula_context": {
                "rate": cfg.data_rate, "lsb": info.lsb, "vref": info.vref,
                "gain": info.gain, "fs": info.fs, "ain1": bias,
                "avdd": avdd, "refp": refp,
            },
            "range": {
                "refp_V": refp, "bias_V": bias, "ain1_V": bias,
                "avdd_V": avdd,
                "vref_V": info.vref, "gain": info.gain,
                "full_scale_V": info.fs, "lsb_V": info.lsb,
                "abs_input_V": [info.abs_lo, info.abs_hi],
                "usable_ain_V": [info.usable_lo, info.usable_hi],
                "usable_ain0_V": [info.usable_lo, info.usable_hi],
                "valid": info.valid, "message": info.message,
            },
            "note": ("解码 CSV 的 channel 为 MUX 编码，vin_V = code × LSB，"
                     "ain_V = vin + 输入偏置；模拟值 CSV 为各公式序列结果。"),
        }
        self._acq_bias = bias
        self._acq_lsb = info.lsb
        self._acq_rate = float(cfg.data_rate)
        self._reset_wave_states_start()
        self.worker.start_acq(cfg, record, default_data_dir(), meta)
        out_txt = ", ".join(k for k, v in output_paths.items() if v) or "无"
        self._log("info", f"开始采集请求 · 输出：{out_txt}")

    def _reset_wave_states_start(self):
        for window in self.wave_windows:
            window.state.frozen = False
            window.state.frozen_snapshot = None
            window.state.trigger_scan_index = 0
            window.state.index_view = None
            window.state.y_view = None
            window.state.last = None
            try:
                window.pause_btn.configure(text="暂停")
            except Exception:
                pass

    def _stop_clicked(self):
        self._stop_dialog_pending = True
        self.worker.stop_acq()

    def _read_clicked(self):
        self.worker.read_config()

    # -- update check / forced update --------------------------------------
    def _manifest_url(self):
        env = os.environ.get("ADS122U04_UPDATE_URL", "").strip()
        if env:
            return env
        if UPDATE_REPO:
            return (f"https://github.com/{UPDATE_REPO}/releases/latest/"
                    "download/latest.json")
        return ""

    def _manual_update_check(self):
        self._begin_update_check(manual=True)

    def _periodic_update_check(self):
        if (not self.no_update_check and IS_RELEASE_BUILD and
                update_client is not None and self._update_dialog is None):
            self._begin_update_check(manual=False)
        try:
            self.root.after(30 * 60 * 1000, self._periodic_update_check)
        except Exception:
            pass

    def _begin_update_check(self, manual=False):
        if update_client is None:
            msg = f"更新模块不可用：{UPDATE_IMPORT_ERROR}"
            self._log("error", msg)
            if manual:
                messagebox.showerror(APP_NAME, msg)
            return
        if self._update_checking:
            if manual:
                messagebox.showinfo(APP_NAME, "正在检查更新，请稍候…")
            return
        if not IS_RELEASE_BUILD and not os.environ.get("ADS122U04_UPDATE_URL"):
            msg = "当前是开发构建（无 git tag 元数据），跳过更新检查。"
            self._log("info", msg)
            if manual:
                messagebox.showinfo(APP_NAME, msg)
            return
        url = self._manifest_url()
        if not url:
            if manual:
                messagebox.showerror(APP_NAME, "未配置更新地址")
            return
        self._update_checking = True
        try:
            self.check_update_btn.configure(text="检查中…")
            self.check_update_btn.state(["disabled"])
        except Exception:
            pass
        if manual:
            self._log("info", "正在检查更新…")
        threading.Thread(target=self._update_check_worker,
                         args=(url, manual), daemon=True).start()

    def _update_check_worker(self, url, manual):
        try:
            info = update_client.fetch_manifest(url, timeout=6.0)
            if update_client.is_newer(info.version, APP_VERSION):
                self.update_queue.put(("update", (info, manual)))
            else:
                self.update_queue.put(("current", (info.version, manual)))
        except update_client.NoReleaseFound as exc:
            self.update_queue.put(("no_release", (str(exc), manual)))
        except Exception as exc:
            self.update_queue.put(("check_error", (str(exc), manual)))

    def _handle_update_event(self, kind, payload):
        if kind == "update":
            info, _manual = payload
            self._update_checking = False
            try:
                self.check_update_btn.configure(text="检查更新")
                self.check_update_btn.state(["!disabled"])
            except Exception:
                pass
            self._show_update_dialog(info)
        elif kind == "current":
            version, manual = payload
            self._update_checking = False
            try:
                self.check_update_btn.configure(text="检查更新")
                self.check_update_btn.state(["!disabled"])
            except Exception:
                pass
            self._log("info", f"更新检查：当前已是最新版本 v{version}")
            if manual:
                messagebox.showinfo(APP_NAME, f"已是最新版本：v{version}")
        elif kind == "no_release":
            msg, manual = payload
            self._update_checking = False
            try:
                self.check_update_btn.configure(text="检查更新")
                self.check_update_btn.state(["!disabled"])
            except Exception:
                pass
            self._log("info" if not manual else "warn", f"更新检查：{msg}")
            if manual:
                messagebox.showinfo(APP_NAME,
                                    "尚未发布任何版本。\n"
                                    "首次发布后，后续 tag 才会提示更新。")
        elif kind == "check_error":
            msg, manual = payload
            self._update_checking = False
            try:
                self.check_update_btn.configure(text="检查更新")
                self.check_update_btn.state(["!disabled"])
            except Exception:
                pass
            self._log("warn" if manual else "info", f"更新检查失败：{msg}")
            if manual:
                messagebox.showwarning(APP_NAME,
                                       "检查更新失败：\n" + msg +
                                       "\n\n已继续使用当前版本。")
        elif kind == "progress":
            done, total = payload
            self._update_dialog_progress(done, total)
        elif kind == "downloaded":
            (path,) = payload
            self._apply_downloaded_update(Path(path))
        elif kind == "download_error":
            (msg,) = payload
            self._update_dialog_error(msg)

    def _show_update_dialog(self, info):
        if self._update_dialog is not None:
            try:
                self._update_dialog.lift()
                self._update_dialog.focus_force()
            except Exception:
                pass
            return
        self._update_info = info
        if self.worker.get_state().get("streaming"):
            self.worker.stop_acq()
            self._log("warn", "检测到新版本，采集已停止；必须更新后才能继续使用。")
        self._log("warn", f"发现新版本 {info.tag}（commit {info.commit_short or '—'}），"
                          "必须更新。")

        s = self.ui_scale
        dlg = tk.Toplevel(self.root)
        dlg.title("必须更新 — MagSensor")
        dlg.configure(bg=COL_BG)
        dlg.transient(self.root)
        dlg.resizable(False, False)
        dlg.protocol("WM_DELETE_WINDOW", self._on_close)
        try:
            icon = resource_path("icon.ico")
            if icon.exists():
                dlg.iconbitmap(str(icon))
        except Exception:
            pass
        w, h = int(760 * s), int(620 * s)
        x = self.root.winfo_rootx() + max(0, (self.root.winfo_width() - w) // 2)
        y = self.root.winfo_rooty() + max(0, (self.root.winfo_height() - h) // 2)
        dlg.geometry(f"{w}x{h}+{x}+{y}")
        dlg.grab_set()
        try:
            dlg.lift()
            dlg.focus_force()
        except Exception:
            pass
        self._update_dialog = dlg

        head = tk.Frame(dlg, bg=COL_PANEL)
        head.pack(fill="x")
        tk.Label(head, text="发现新版本 · 必须更新", bg=COL_PANEL,
                 fg=COL_WARN, font=(FONT, 16, "bold")).pack(
                     anchor="w", padx=int(18 * s), pady=(int(14 * s), 2))
        tk.Label(head, text="更新完成后程序会自动重启；也可以退出程序。",
                 bg=COL_PANEL, fg=COL_MUTED, font=(FONT, 9)).pack(
                     anchor="w", padx=int(18 * s), pady=(0, int(12 * s)))

        body = tk.Frame(dlg, bg=COL_BG)
        body.pack(fill="both", expand=True, padx=int(18 * s),
                  pady=(int(10 * s), 0))

        ver = tk.Frame(body, bg=COL_BG)
        ver.pack(fill="x")
        tk.Label(ver, text="当前版本", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=0, column=0, sticky="w")
        cur_commit = f" · {APP_COMMIT_SHORT}" if APP_COMMIT_SHORT else ""
        tk.Label(ver, text=f"v{APP_VERSION}{cur_commit}", bg=COL_BG,
                 fg=COL_FG, font=(FONT, 10)).grid(row=1, column=0, sticky="w")
        tk.Label(ver, text="最新版本", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).grid(row=0, column=1, sticky="w",
                                      padx=(int(40 * s), 0))
        tk.Label(ver, text=f"{info.tag}  (v{info.version})", bg=COL_BG,
                 fg=COL_GREEN, font=(FONT, 10, "bold")).grid(
                     row=1, column=1, sticky="w", padx=(int(40 * s), 0))

        commit_box = tk.Frame(body, bg=COL_PANEL)
        commit_box.pack(fill="x", pady=(int(10 * s), int(8 * s)))
        commit_short = info.commit_short or "—"
        commit_msg = info.commit_message or "（无提交说明）"
        tk.Label(commit_box, text=f"Tag commit  {commit_short}",
                 bg=COL_PANEL, fg=COL_WARN, font=(MONO, 10, "bold")).pack(
                     anchor="w", padx=int(10 * s), pady=(int(7 * s), 1))
        tk.Label(commit_box, text=commit_msg, bg=COL_PANEL, fg=COL_FG,
                 font=(FONT, 9), justify="left",
                 wraplength=int(700 * s)).pack(anchor="w",
                                                padx=int(10 * s))
        if info.commit:
            tk.Label(commit_box, text=f"full commit: {info.commit}",
                     bg=COL_PANEL, fg=COL_MUTED, font=(MONO, 7)).pack(
                         anchor="w", padx=int(10 * s), pady=(1, 0))
        if info.commit_date:
            tk.Label(commit_box, text=f"提交时间: {info.commit_date}",
                     bg=COL_PANEL, fg=COL_MUTED, font=(FONT, 8)).pack(
                         anchor="w", padx=int(10 * s),
                         pady=(0, int(6 * s)))

        notes_hdr = tk.Frame(body, bg=COL_BG)
        notes_hdr.pack(fill="x")
        tk.Label(notes_hdr, text="发布说明", bg=COL_BG, fg=COL_MUTED,
                 font=(FONT, 8)).pack(side="left")
        tk.Label(notes_hdr, text=f"下载大小 {fmt_bytes(info.size)}",
                 bg=COL_BG, fg=COL_MUTED, font=(FONT, 8)).pack(side="right")
        notes_frame = tk.Frame(body, bg=COL_BG)
        notes_frame.pack(fill="both", expand=True, pady=(3, int(6 * s)))
        notes = tk.Text(notes_frame, height=8, bg="#14161b", fg="#c7ccd6",
                        relief="flat", bd=0, wrap="word", font=(FONT, 9),
                        padx=8, pady=6)
        nsb = ttk.Scrollbar(notes_frame, orient="vertical",
                            command=notes.yview, style="Vertical.TScrollbar")
        notes.configure(yscrollcommand=nsb.set)
        notes.pack(side="left", fill="both", expand=True)
        nsb.pack(side="right", fill="y")
        notes.insert("1.0", info.notes or "（无发布说明）")
        notes.configure(state="disabled")

        self._update_status_var = tk.StringVar(value="尚未开始下载")
        tk.Label(body, textvariable=self._update_status_var, bg=COL_BG,
                 fg=COL_MUTED, font=(FONT, 8), anchor="w").pack(fill="x")
        self._update_pct = 0.0
        self._update_progress = tk.Canvas(
            body, height=max(6, int(10 * s)), bg=COL_PANEL2,
            highlightthickness=0, bd=0)
        self._update_progress.pack(fill="x", pady=(3, int(4 * s)))
        self._update_progress.bind(
            "<Configure>", lambda e: self._draw_update_progress(self._update_pct))

        footer = tk.Frame(dlg, bg=COL_PANEL)
        footer.pack(fill="x", side="bottom")
        self.update_go_btn = ttk.Button(footer, text="立即更新",
                                        style="Accent.TButton",
                                        command=self._start_update_download)
        self.update_go_btn.pack(side="right", padx=(0, int(10 * s)),
                                pady=int(12 * s))
        self.update_exit_btn = ttk.Button(footer, text="退出程序",
                                          command=self._on_close)
        self.update_exit_btn.pack(side="right", padx=(0, int(6 * s)),
                                  pady=int(12 * s))
        self.update_page_btn = ttk.Button(
            footer, text="手动下载", style="Small.TButton",
            command=lambda: webbrowser.open(info.url) if info.url else None)
        self.update_page_btn.pack(side="left", padx=int(12 * s),
                                  pady=int(12 * s))
        self._log("info", f"新版本 {info.tag} · commit {info.commit_short} · "
                          f"{info.commit_date}")

    def _start_update_download(self):
        info = self._update_info
        if info is None or update_client is None:
            return
        self.update_go_btn.state(["disabled"])
        self.update_go_btn.configure(text="正在下载…")
        self.update_exit_btn.state(["!disabled"])
        self._update_status_var.set("正在下载更新包…")
        self._draw_update_progress(0)
        threading.Thread(target=self._download_update_worker,
                         args=(info,), daemon=True).start()

    def _download_update_worker(self, info):
        try:
            dest = update_client.make_update_dir()

            def progress(done, total):
                self.update_queue.put(("progress", (done, total)))

            path = update_client.download_update(info, dest, progress=progress)
            self.update_queue.put(("downloaded", (str(path),)))
        except Exception as exc:
            self.update_queue.put(("download_error", (str(exc),)))

    def _draw_update_progress(self, pct):
        self._update_pct = float(pct)
        c = self._update_progress
        if c is None:
            return
        try:
            c.delete("all")
            w, h = c.winfo_width(), c.winfo_height()
            if w < 2 or h < 2:
                return
            c.create_rectangle(0, 0, w, h, fill=COL_PANEL2, outline="")
            if pct > 0:
                c.create_rectangle(0, 0, max(1, int(w * min(100.0, pct) / 100.0)),
                                   h, fill=COL_ACCENT, outline="")
        except Exception:
            pass

    def _update_dialog_progress(self, done, total):
        if self._update_dialog is None or self._update_progress is None:
            return
        if total and total > 0:
            self._draw_update_progress(min(100.0, done * 100.0 / total))
            self._update_status_var.set(
                f"正在下载… {fmt_bytes(done)} / {fmt_bytes(total)}")
        else:
            self._update_status_var.set(f"正在下载… {fmt_bytes(done)}")

    def _update_dialog_error(self, msg):
        if self._update_dialog is None:
            return
        self._draw_update_progress(0)
        self._update_status_var.set("更新失败：" + msg)
        self._log("error", "自动更新失败：" + msg)
        try:
            self.update_go_btn.configure(text="重试更新")
            self.update_go_btn.state(["!disabled"])
        except Exception:
            pass
        messagebox.showerror(APP_NAME,
                             "自动更新失败：\n" + msg +
                             "\n\n可以点击“重试更新”，或点击“打开发布页”手动下载。")

    def _apply_downloaded_update(self, path: Path):
        if self._update_dialog is None:
            return
        if not getattr(sys, "frozen", False):
            self._update_dialog_error("当前是源码运行，不能自动替换 exe。")
            return
        target = Path(sys.executable).resolve()
        if not update_client.check_writable(target.parent):
            self._update_dialog_error(
                f"程序目录不可写：{target.parent}\n请以有写权限的方式运行，或手动替换 exe。")
            return
        try:
            self._update_status_var.set("校验完成，正在启动更新助手…")
            self._draw_update_progress(100)
            update_client.launch_updater(path, target, os.getpid(),
                                         self._update_info.version,
                                         parent_pid=os.getppid())
        except Exception as exc:
            self._update_dialog_error(f"启动更新助手失败：{exc}")
            return
        self._log("ok", "更新包已下载并通过 SHA-256 校验，程序即将重启完成更新。")
        try:
            self._save_settings_file()
        except Exception:
            pass
        try:
            self.update_go_btn.state(["disabled"])
            self.update_exit_btn.state(["disabled"])
        except Exception:
            pass
        try:
            self.worker.shutdown()
        except Exception:
            pass
        self.root.after(300, self.root.destroy)

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

        for w in (self.gain_combo, self.mode_combo, self.dr_combo,
                  self.cm_combo, self.vref_combo, self.integ_combo):
            w.configure(state="disabled" if streaming else "readonly")
        for w in (self.bypass_chk, self.dcnt_chk):
            w.state(["disabled"] if streaming else ["!disabled"])
        try:
            self.chan_mode_combo.configure(
                state="disabled" if streaming else "readonly")
            self.chan_menu_btn.configure(
                state="disabled" if streaming else "normal")
        except Exception:
            pass
        if not streaming:
            self._on_config_widget_change()

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

        seq = int(st.get("stop_seq", 0))
        if seq != self._last_stop_seq:
            self._last_stop_seq = seq
            paths = st.get("output_paths") or {}
            if self._stop_dialog_pending:
                self._stop_dialog_pending = False
                if paths:
                    self.root.after(160,
                                    lambda p=dict(paths):
                                    self._show_stop_dialog(p))

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
        try:
            while True:
                kind, payload = self.update_queue.get_nowait()
                self._handle_update_event(kind, payload)
        except queue.Empty:
            pass

        st = self.worker.get_state()
        if st != self._last_state:
            self._last_state = st
            self._refresh_state(st)
            if (self.demo_pending and st.get("connected") and
                    not st.get("streaming")):
                self.demo_pending = False
                self.root.after(200, self._start_clicked)

        now = time.monotonic()
        if now - self._last_draw >= 0.09:
            self._last_draw = now
            self._update_view()
        self._tick_after = self.root.after(60, self._tick)

    def _update_view(self):
        quick = self.worker.stats.quick()
        self._update_global_stats(quick)
        if not self.wave_windows:
            self._plot_dirty = False
            return
        streaming = self.worker.get_state().get("streaming", False)
        rate = self._plot_rate_value()
        for window in list(self.wave_windows):
            state = window.state
            try:
                if state.frozen:
                    if self._plot_dirty and state.frozen_snapshot is not None:
                        self._render_wave(window, state.frozen_snapshot)
                    continue
                if state.index_view is not None:
                    snap = self.worker.stats.snapshot_range(
                        state.index_view[0], state.index_view[1],
                        max_points=4000)
                else:
                    count = self._window_count(quick, rate,
                                               window.spec.timebase)
                    snap = self.worker.stats.snapshot(count, max_points=4000)
                self._check_trigger(window, snap)
                if state.frozen:
                    snap = state.frozen_snapshot
                if streaming or self._plot_dirty:
                    self._render_wave(window, snap)
            except Exception:
                pass
        self._plot_dirty = False

    def _update_global_stats(self, q):
        parts = [f"样本 {q['total']:,}",
                 f"速率 {q['rate']:.1f} SPS" if q["rate"] > 0 else "速率 —",
                 f"丢帧 {q['dropped']}"]
        if q["errors"]:
            parts.append(f"错误 {q['errors']}")
        if q.get("formula_errors"):
            parts.append(f"公式错误 {q['formula_errors']}")
        if q["resyncs"]:
            parts.append(f"重同步 {q['resyncs']}")
        try:
            self.global_stats_var.set("  ·  ".join(parts))
        except Exception:
            pass

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

    def _window_count(self, quick, rate, timebase):
        buffered = quick.get("buffered", 0)
        if timebase == "全部":
            n = buffered
        else:
            try:
                secs = float(str(timebase).split()[0])
            except Exception:
                secs = 2.0
            n = int(rate * secs)
        return max(0, min(n, buffered))

    def _series_points(self, sr: SeriesSpec, snap, rate, lsb, bias):
        start = snap.get("start_index", 0)
        step = snap.get("step", 1) or 1
        pts = []
        if sr.mode == "raw":
            codes = snap.get("codes") or []
            chans = snap.get("channels") or []
            for j, (ch, code) in enumerate(zip(chans, codes)):
                if int(ch) == int(sr.mux):
                    idx = start + j * step
                    # Monitor / short modes have no AIN pin voltage; plot VIN.
                    offset = 0.0 if int(sr.mux) >= 12 else bias
                    pts.append((idx, idx / rate,
                                int(code) * lsb + offset))
        else:
            xs, ys = (snap.get("series") or {}).get(sr.key,
                                                    ((), ()))
            for j, (xv, yv) in enumerate(zip(xs, ys)):
                try:
                    xf, yf = float(xv), float(yv)
                except Exception:
                    continue
                if math.isfinite(xf) and math.isfinite(yf):
                    pts.append((start + j * step, xf, yf))
        return pts

    def _check_trigger(self, window, snap):
        state = window.state
        spec = window.spec
        if spec.trigger_mode == "关闭" or state.frozen:
            return
        sr = None
        for item in spec.series:
            if item.key == spec.trigger_source:
                sr = item
                break
        if sr is None and spec.series:
            sr = spec.series[0]
        if sr is None:
            return
        rate = self._plot_rate_value()
        bias = self._plot_bias()
        pts = self._series_points(sr, snap, rate, self._plot_lsb(), bias)
        if not pts:
            return
        thr = float(spec.trigger_value)
        mode = spec.trigger_mode
        crossing = None
        prev = None
        scan_from = state.trigger_scan_index
        for idx, _x, y in pts:
            if idx <= scan_from:
                prev = (idx, y)
                continue
            if prev is not None:
                if mode == "上升沿" and prev[1] < thr <= y:
                    crossing = (idx, y)
                    break
                if mode == "下降沿" and prev[1] > thr >= y:
                    crossing = (idx, y)
                    break
            if mode == "电平" and y >= thr:
                crossing = (idx, y)
                break
            prev = (idx, y)
        if crossing is None:
            state.trigger_scan_index = max(state.trigger_scan_index,
                                           pts[-1][0])
            return
        idx, val = crossing
        quick = self.worker.stats.quick()
        count = self._window_count(quick, rate, spec.timebase)
        start = max(0, idx - count + 1)
        frozen = self.worker.stats.snapshot_range(start, idx,
                                                  max_points=4000)
        state.frozen_snapshot = frozen
        state.index_view = None
        state.frozen = True
        state.trigger_index = idx
        state.trigger_scan_index = idx
        window.set_frozen(True, frozen)
        self._log("warn", f"波形「{spec.title}」触发（{val:.6g}），显示已暂停")
        self._plot_dirty = True

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

    def _fmt_axis_value(self, value, span):
        if not math.isfinite(value):
            return "—"
        a = abs(value)
        span = abs(span)
        if a >= 1e5 or (span and span < 1e-5):
            return f"{value:.4g}"
        if span >= 100:
            return f"{value:.0f}"
        if span >= 10:
            return f"{value:.1f}"
        if span >= 1:
            return f"{value:.2f}"
        if span >= 0.01:
            return f"{value:.4f}"
        if span >= 1e-4:
            return f"{value:.6f}"
        return f"{value:.4g}"

    def _render_wave(self, window, snap):
        state = window.state
        spec = window.spec
        c = state.canvas
        if c is None:
            return
        try:
            c.delete("all")
            w = c.winfo_width()
            h = c.winfo_height()
        except Exception:
            return
        if w < 120 or h < 90:
            return
        s = self.ui_scale
        ml, mr = int(58 * s), int(16 * s)
        mt, mb = int(26 * s), int(28 * s)
        pw, ph = w - ml - mr, h - mt - mb
        if pw < 20 or ph < 20:
            return
        c.create_rectangle(ml, mt, ml + pw, mt + ph, outline=COL_BORDER)

        rate = self._plot_rate_value()
        bias = self._plot_bias()
        lsb = self._plot_lsb()
        series_all = []
        series_visible = []
        all_pts = []
        for i, sr in enumerate(spec.series):
            pts = self._series_points(sr, snap, rate, lsb, bias)
            series_all.append((sr, i, pts))
            if sr.key not in spec.hidden and pts:
                series_visible.append((sr, i, pts))
                all_pts.extend(pts)

        def draw_legend():
            state.legend_hits = []
            if not spec.show_legend or not spec.series:
                return
            lx = ml + int(8 * s)
            ly = mt + int(8 * s)
            for sr, i, pts in series_all:
                hidden = sr.key in spec.hidden
                color = COL_MUTED if hidden else WAVE_COLORS[i % len(WAVE_COLORS)]
                c.create_line(lx, ly, lx + int(14 * s), ly, fill=color,
                              width=max(2, int(2 * s)))
                txt = c.create_text(lx + int(18 * s), ly,
                                    text=(sr.label or sr.key) +
                                    ("（隐藏）" if hidden else ""),
                                    anchor="w",
                                    fill=COL_MUTED if hidden else COL_FG,
                                    font=(FONT, 8))
                bb = c.bbox(txt)
                if bb:
                    state.legend_hits.append((lx - 2, ly - 9,
                                              bb[2] + 2, ly + 9, sr.key))
                ly += int(16 * s)

        if not all_pts:
            c.create_text(ml + pw / 2, mt + ph / 2,
                          text="等待数据…\n请确认该通道已使能并开始采集",
                          fill="#5d6472", font=(FONT, 10), justify="center")
            draw_legend()
            state.last = None
            return

        xs = [p[1] for p in all_pts]
        ys = [p[2] for p in all_pts]
        xmin, xmax = min(xs), max(xs)
        vmin, vmax = min(ys), max(ys)
        if xmax <= xmin:
            dx = max(1e-9, abs(xmin) * 1e-6)
            xmin -= dx
            xmax += dx
        if window.auto_var.get() or state.y_view is None:
            span = vmax - vmin
            if span <= 0:
                span = max(abs(vmax) * 0.02, 1e-9)
            pad = span * 0.08
            ymin, ymax = vmin - pad, vmax + pad
            state.y_view = None
        else:
            ymin, ymax = state.y_view
        if ymax <= ymin:
            ymax = ymin + 1e-9

        def X(v):
            return ml + (v - xmin) / (xmax - xmin) * pw

        def Y(v):
            return mt + (ymax - v) / (ymax - ymin) * ph

        step_y = self._nice_step(ymax - ymin, 5)
        start_y = math.ceil(ymin / step_y) * step_y
        v = start_y
        tick = 0
        while v <= ymax + step_y * 0.01 and tick < 20:
            y = Y(v)
            c.create_line(ml, y, ml + pw, y, fill=COL_GRID)
            c.create_text(ml - int(8 * s), y,
                          text=self._fmt_axis_value(v, ymax - ymin),
                          anchor="e", fill="#6d7585", font=(MONO, 8))
            v += step_y
            tick += 1
        nx = 6
        xspan = xmax - xmin
        for i in range(nx + 1):
            x = ml + pw * i / nx
            c.create_line(x, mt, x, mt + ph, fill=COL_GRID)
            xv = xmin + xspan * i / nx
            c.create_text(x, mt + ph + int(9 * s),
                          text=self._fmt_axis_value(xv, xspan),
                          anchor="n", fill="#6d7585", font=(MONO, 8))

        x_title = f"X ({spec.x_unit})" if spec.x_unit else "X"
        y_title = spec.y_unit or ""
        c.create_text(ml, mt - int(13 * s), text=y_title,
                      anchor="w", fill="#7d8698", font=(FONT, 8, "bold"))
        c.create_text(ml + pw, mt - int(13 * s), text=x_title,
                      anchor="e", fill="#5d6472", font=(FONT, 8))

        for sr, i, pts in series_visible:
            color = WAVE_COLORS[i % len(WAVE_COLORS)]
            px = [X(p[1]) for p in pts]
            py = [Y(p[2]) for p in pts]
            if len(pts) >= 2:
                flat = []
                for a, b in zip(px, py):
                    flat.extend((a, b))
                c.create_line(*flat, fill=color,
                              width=max(1.0, 1.4 * s), joinstyle="round")
            elif len(pts) == 1:
                r = max(2, int(3 * s))
                c.create_oval(px[0] - r, py[0] - r, px[0] + r, py[0] + r,
                              fill=color, outline="")

        draw_legend()

        if spec.trigger_mode != "关闭":
            thr = float(spec.trigger_value)
            if ymin <= thr <= ymax:
                ty = Y(thr)
                c.create_line(ml, ty, ml + pw, ty, fill=COL_WARN,
                              dash=(4, 3))
                c.create_text(ml + pw - int(4 * s), ty - int(3 * s),
                              text=f"触发 {thr:.6g}", anchor="se",
                              fill=COL_WARN, font=(FONT, 7))

        if state.frozen:
            c.create_text(ml + pw - int(6 * s), mt + int(8 * s),
                          text="已触发 / 暂停", anchor="ne",
                          fill=COL_WARN, font=(FONT, 8, "bold"))

        state.last = {
            "ml": ml, "mt": mt, "pw": pw, "ph": ph,
            "xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax,
            "rate": rate, "pts": all_pts,
        }


    # -- demo --------------------------------------------------------------
    def _demo_connect(self):
        self._log("info", "演示模式：使用模拟数据源")
        labels = list(self.port_combo.cget("values"))
        if SIM_LABEL in labels:
            self.port_var.set(SIM_LABEL)
        if not self.wave_windows:
            muxes = self._enabled_muxes()
            if muxes:
                mux = muxes[0]
                label = MUX_SHORT.get(mux, str(mux))
                self._add_wave_window(WaveSpec(
                    wid=0, title="演示 · " + label,
                    series=[SeriesSpec(key="demo0", mux=mux, mode="raw",
                                       label=label)]), save=False)
        self.demo_pending = True
        self._toggle_connect()

    # -- shutdown ----------------------------------------------------------
    def _on_close(self):
        for attr in ("_tick_after", "_danmaku_after"):
            aid = getattr(self, attr, None)
            if aid:
                try:
                    self.root.after_cancel(aid)
                except Exception:
                    pass
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

    # Formula evaluator (C expression subset)
    fvars = dict(FORMULA_VARIABLE_TYPES)
    fctx = {"ain0": 1.7, "ain1": 1.65}
    fcases = [
        ("1+2*3", {}, 7),
        ("1/2", {}, 0),
        ("1.0/2", {}, 0.5),
        ("-5%2", {}, -1),
        ("0x10 + 010", {}, 24),
        ("1e3", {}, 1000.0),
        ("2?10:20", {}, 10),
        ("1&&0||1", {}, 1),
        ("(int)3.9", {}, 3),
        ("(double)1/2", {}, 0.5),
        ("sqrt(9)", {}, 3.0),
        ("round(2.5)", {}, 3.0),
        ("round(-2.5)", {}, -3.0),
        ("M_PI > 3", {}, 1),
        ("ain0 - ain1", fctx, 0.05),
    ]
    for expr, cctx, exp in fcases:
        try:
            val = formula_engine.compile_expr(expr, fvars).eval(cctx)
            check(f"formula {expr}", abs(float(val) - exp) < 1e-9, f"{val!r}")
        except Exception as exc:
            check(f"formula {expr}", False, str(exc))
    try:
        formula_engine.compile_expr("1+", fvars)
        check("formula invalid rejected", False)
    except Exception:
        check("formula invalid rejected", True)

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
    parser.add_argument("--no-update-check", action="store_true",
                        help="跳过启动时的更新检查（开发/测试）")
    parser.add_argument("--updated", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--apply-update", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--target", default="", help=argparse.SUPPRESS)
    parser.add_argument("--old-pid", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--parent-pid", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--expected-version", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.apply_update:
        if update_client is None:
            sys.exit(2)
        target = args.target or sys.executable
        code = update_client.apply_update(
            target, args.old_pid, args.expected_version or APP_VERSION,
            parent_pid=args.parent_pid)
        sys.exit(code)

    if args.selftest:
        sys.exit(run_selftest())

    if update_client is not None:
        try:
            update_client.record_startup(APP_VERSION, APP_TAG, APP_COMMIT)
        except Exception:
            pass

    enable_dpi_awareness()
    root = tk.Tk()
    app = App(root, demo=args.demo,
              no_update_check=args.no_update_check,
              updated=args.updated)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app._on_close()


if __name__ == "__main__":
    main()
