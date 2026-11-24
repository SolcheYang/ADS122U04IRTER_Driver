# -*- coding: utf-8 -*-
"""Update-check and self-update helpers for the ADS122U04 acquisition tool.

The update feed is a small JSON manifest generated from a git tag and attached
to a GitHub release (see tools/build.py and .github/workflows/release.yml).

The application downloads the new executable, verifies its SHA-256, and then
re-launches that *new* executable with ``--apply-update``.  The new executable
waits for the old process to exit, backs up and replaces the installed exe, and
relaunches it.  The new process writes a startup marker; if it does not appear,
the previous executable is restored.
"""

from __future__ import annotations

import ctypes
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

MAX_MANIFEST_BYTES = 512 * 1024
MAX_UPDATE_BYTES = 256 * 1024 * 1024
USER_AGENT = "ADS122U04-ADC-Tool-Updater/1.0"
EXE_NAME = "ADS122U04_ADC_Tool.exe"

_VERSION_RE = re.compile(r"^[vV]?(\d+)\.(\d+)\.(\d+)(?:[-+].*)?$")
_SHA_RE = re.compile(r"^[0-9a-fA-F]{64}$")


class UpdateError(Exception):
    """A recoverable update error."""


class NoReleaseFound(UpdateError):
    """No published release / manifest exists (not an error on first use)."""


@dataclass
class UpdateInfo:
    tag: str
    version: str
    version_tuple: tuple
    commit: str
    commit_short: str
    commit_message: str
    commit_date: str
    notes: str
    url: str
    sha256: str
    size: int
    mandatory: bool
    min_supported: str
    repo: str
    raw: dict


def parse_version(text) -> tuple | None:
    if text is None:
        return None
    m = _VERSION_RE.match(str(text).strip())
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def is_newer(remote_version, current_version) -> bool:
    rv = parse_version(remote_version)
    cv = parse_version(current_version)
    return rv is not None and cv is not None and rv > cv


def _validate_https(url: str, what="更新地址"):
    try:
        p = urllib.parse.urlparse(url)
    except Exception as exc:
        raise UpdateError(f"{what}无效：{exc}") from exc
    if p.scheme == "https" and p.hostname:
        return
    # Allow plain HTTP only for local testing / mirrors on localhost.
    if p.scheme == "http" and p.hostname in ("127.0.0.1", "localhost", "::1"):
        return
    raise UpdateError(f"{what}必须使用 HTTPS")


def _clean_text(value, limit=8000) -> str:
    if value is None:
        return ""
    return str(value).strip()[:limit]


def fetch_manifest(url: str, timeout: float = 5.0) -> UpdateInfo:
    """Fetch and validate the latest release manifest."""
    _validate_https(url)
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": USER_AGENT,
            "Accept": "application/json",
            "Cache-Control": "no-cache",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            status = getattr(resp, "status", 200)
            if status != 200:
                raise UpdateError(f"服务器返回 HTTP {status}")
            data = resp.read(MAX_MANIFEST_BYTES + 1)
    except urllib.error.HTTPError as exc:
        if exc.code in (403, 404):
            raise NoReleaseFound("尚未发布任何版本") from exc
        raise UpdateError(f"检查更新失败：HTTP {exc.code}") from exc
    except urllib.error.URLError as exc:
        raise UpdateError(f"无法连接更新服务器：{exc.reason}") from exc
    except TimeoutError as exc:
        raise UpdateError("检查更新超时") from exc

    if len(data) > MAX_MANIFEST_BYTES:
        raise UpdateError("更新清单过大")
    try:
        obj = json.loads(data.decode("utf-8"))
    except Exception as exc:
        raise UpdateError("更新清单格式错误") from exc
    if not isinstance(obj, dict):
        raise UpdateError("更新清单格式错误")
    if int(obj.get("schema", 1)) > 1:
        raise UpdateError("更新清单版本不受支持")

    tag = _clean_text(obj.get("tag"), 80)
    version = _clean_text(obj.get("version") or tag, 80).lstrip("vV")
    vt = parse_version(version)
    if not tag or vt is None:
        raise UpdateError("更新清单缺少有效版本号")

    exe_url = _clean_text(obj.get("url"), 2000)
    _validate_https(exe_url, "下载地址")
    sha = _clean_text(obj.get("sha256"), 80).lower()
    if not _SHA_RE.match(sha):
        raise UpdateError("更新清单缺少有效的 SHA-256")
    try:
        size = int(obj.get("size") or 0)
    except Exception:
        size = 0
    if size <= 0 or size > MAX_UPDATE_BYTES:
        raise UpdateError("更新文件大小无效")

    return UpdateInfo(
        tag=tag,
        version=version,
        version_tuple=vt,
        commit=_clean_text(obj.get("commit"), 80),
        commit_short=_clean_text(obj.get("commit_short"), 20) or
        _clean_text(obj.get("commit"), 80)[:7],
        commit_message=_clean_text(obj.get("commit_message"), 500),
        commit_date=_clean_text(obj.get("commit_date"), 80),
        notes=_clean_text(obj.get("notes"), 12000),
        url=exe_url,
        sha256=sha,
        size=size,
        mandatory=bool(obj.get("mandatory", True)),
        min_supported=_clean_text(obj.get("min_supported"), 40),
        repo=_clean_text(obj.get("repo"), 200),
        raw=obj,
    )


def make_update_dir() -> Path:
    base = Path(os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()) / \
        "ADS122U04_Tool" / "updates"
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    d = base / stamp
    d.mkdir(parents=True, exist_ok=True)
    # Best-effort cleanup of old update directories.
    try:
        now = time.time()
        for child in base.iterdir():
            if child.is_dir() and now - child.stat().st_mtime > 7 * 86400:
                shutil.rmtree(child, ignore_errors=True)
    except Exception:
        pass
    return d


def download_update(info: UpdateInfo, dest_dir: Path, progress=None,
                    cancel=None, timeout: float = 30.0) -> Path:
    """Download the new exe, verify size + SHA-256, return the file path."""
    _validate_https(info.url, "下载地址")
    dest_dir = Path(dest_dir)
    dest_dir.mkdir(parents=True, exist_ok=True)
    final = dest_dir / "ADS122U04_ADC_Tool_update.exe"
    part = dest_dir / "download.part"
    req = urllib.request.Request(
        info.url,
        headers={"User-Agent": USER_AGENT, "Accept": "application/octet-stream"},
    )
    sha = hashlib.sha256()
    received = 0
    last_report = 0.0
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            status = getattr(resp, "status", 200)
            if status != 200:
                raise UpdateError(f"下载失败：HTTP {status}")
            try:
                total = int(resp.headers.get("Content-Length") or info.size)
            except Exception:
                total = info.size
            if total > MAX_UPDATE_BYTES:
                raise UpdateError("更新文件过大")
            with open(part, "wb") as f:
                while True:
                    if cancel is not None and cancel.is_set():
                        raise UpdateError("更新已取消")
                    chunk = resp.read(256 * 1024)
                    if not chunk:
                        break
                    received += len(chunk)
                    if received > MAX_UPDATE_BYTES:
                        raise UpdateError("更新文件过大")
                    if received > info.size + 4096:
                        raise UpdateError("下载数据与清单大小不符")
                    sha.update(chunk)
                    f.write(chunk)
                    if progress is not None:
                        now = time.time()
                        if now - last_report >= 0.15:
                            last_report = now
                            progress(received, total)
                f.flush()
                os.fsync(f.fileno())
    except urllib.error.HTTPError as exc:
        raise UpdateError(f"下载失败：HTTP {exc.code}") from exc
    except urllib.error.URLError as exc:
        raise UpdateError(f"下载失败：{exc.reason}") from exc
    except UpdateError:
        try:
            part.unlink(missing_ok=True)
        except Exception:
            pass
        raise
    except Exception as exc:
        try:
            part.unlink(missing_ok=True)
        except Exception:
            pass
        raise UpdateError(f"下载失败：{exc}") from exc

    if received != info.size:
        part.unlink(missing_ok=True)
        raise UpdateError(f"文件大小不符：收到 {received} B，应为 {info.size} B")
    digest = sha.hexdigest().lower()
    if digest != info.sha256.lower():
        part.unlink(missing_ok=True)
        raise UpdateError("SHA-256 校验失败，更新包可能损坏")
    try:
        with open(part, "rb") as f:
            if f.read(2) != b"MZ":
                raise UpdateError("下载的文件不是 Windows 可执行文件")
        os.replace(part, final)
    except UpdateError:
        part.unlink(missing_ok=True)
        raise
    except Exception as exc:
        part.unlink(missing_ok=True)
        raise UpdateError(f"保存更新文件失败：{exc}") from exc
    if progress is not None:
        progress(received, received)
    return final


# ---------------------------------------------------------------------------
# Startup marker / process waiting
# ---------------------------------------------------------------------------
def app_data_dir() -> Path:
    base = Path(os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()) / \
        "ADS122U04_Tool"
    base.mkdir(parents=True, exist_ok=True)
    return base


def startup_marker_path() -> Path:
    return app_data_dir() / "startup.json"


def record_startup(version: str, tag: str = "", commit: str = ""):
    """Write a marker used by the updater to verify the new exe started."""
    try:
        path = startup_marker_path()
        tmp = path.with_suffix(".tmp")
        data = {
            "version": str(version),
            "tag": str(tag or ""),
            "commit": str(commit or ""),
            "pid": os.getpid(),
            "time": time.time(),
        }
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        pass


def _wait_for_process_exit(pid: int, timeout: float) -> bool:
    if not pid or pid <= 0 or os.name != "nt":
        return True
    SYNCHRONIZE = 0x00100000
    kernel32 = ctypes.windll.kernel32
    handle = kernel32.OpenProcess(SYNCHRONIZE, False, int(pid))
    if not handle:
        return True  # process already gone / inaccessible
    try:
        WAIT_OBJECT_0 = 0
        result = kernel32.WaitForSingleObject(handle, int(timeout * 1000))
        return result == WAIT_OBJECT_0
    finally:
        kernel32.CloseHandle(handle)


def _process_image_path(pid: int):
    """Return the image path of pid (Windows), or None."""
    if os.name != "nt" or not pid or pid <= 0:
        return None
    PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
    kernel32 = ctypes.windll.kernel32
    handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
    if not handle:
        return None
    try:
        buf = ctypes.create_unicode_buffer(32768)
        size = ctypes.c_ulong(32768)
        if kernel32.QueryFullProcessImageNameW(handle, 0, buf,
                                               ctypes.byref(size)):
            try:
                return Path(buf.value).resolve()
            except Exception:
                return Path(buf.value)
        return None
    finally:
        kernel32.CloseHandle(handle)


def _replace_retry(tmp: Path, target: Path, timeout: float = 15.0):
    """os.replace with retries while the old exe image is still mapped."""
    end = time.time() + timeout
    last = None
    while True:
        try:
            os.replace(tmp, target)
            return
        except PermissionError as exc:
            last = exc
            if time.time() >= end:
                raise
            time.sleep(0.3)
        except OSError as exc:
            last = exc
            if time.time() >= end:
                raise
            time.sleep(0.3)


def _wait_for_startup(expected_version: str, since: float,
                      timeout: float = 25.0) -> bool:
    path = startup_marker_path()
    end = time.time() + timeout
    while time.time() < end:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if (str(data.get("version")) == str(expected_version) and
                    float(data.get("time", 0)) >= since - 2.0):
                return True
        except Exception:
            pass
        time.sleep(0.4)
    return False


def _launch(path: Path, args=None):
    cmd = [str(path)] + list(args or [])
    subprocess.Popen(cmd, cwd=str(path.parent), close_fds=True,
                     creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))


def _restore_and_launch(target: Path, backup: Path):
    try:
        if backup.exists():
            tmp = target.with_name(target.name + ".restore")
            shutil.copy2(backup, tmp)
            _replace_retry(tmp, target, timeout=10.0)
    except Exception:
        pass
    try:
        _launch(target, ["--no-update-check"])
    except Exception:
        pass


def apply_update(target_exe, old_pid: int, expected_version: str,
                 new_exe=None, log_path=None, parent_pid: int = 0) -> int:
    """Updater mode: wait for the old PID, replace target, relaunch, verify."""
    target = Path(target_exe).resolve()
    new = Path(new_exe or sys.executable).resolve()
    if log_path is None:
        log_path = app_data_dir() / "update.log"
    log_path = Path(log_path)

    def log(msg):
        try:
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(f"[{datetime.now().isoformat(timespec='seconds')}] {msg}\n")
        except Exception:
            pass

    log(f"apply_update start target={target} new={new} old_pid={old_pid} "
        f"parent_pid={parent_pid} expected={expected_version}")

    if _wait_for_process_exit(old_pid, 30.0) is False:
        log("old process did not exit; aborting")
        return 2
    # In a PyInstaller onefile build the bootloader parent also keeps the exe
    # image mapped for a short while after the Python child exits.
    if parent_pid and parent_pid not in (old_pid, os.getpid()):
        img = _process_image_path(parent_pid)
        try:
            same = img is not None and img.resolve() == target
        except Exception:
            same = img == target
        if same:
            log(f"waiting for bootloader parent {parent_pid}")
            _wait_for_process_exit(parent_pid, 30.0)
    if not new.exists() or new.stat().st_size < 1024:
        log("new executable missing/small; aborting")
        return 3

    backup = target.with_name(target.name + ".bak")
    launched_at = time.time()
    backup_ok = False
    try:
        if target.exists():
            shutil.copy2(target, backup)
            backup_ok = backup.stat().st_size == target.stat().st_size
            if not backup_ok:
                log("WARNING: backup size mismatch")
        tmp = target.with_name(target.name + ".new")
        shutil.copy2(new, tmp)
        _replace_retry(tmp, target, timeout=20.0)
        log("target replaced")
    except Exception as exc:
        # os.replace is atomic: if it failed, the original target is intact.
        log(f"replacement failed: {exc}")
        if backup_ok:
            _restore_and_launch(target, backup)
        else:
            try:
                _launch(target, ["--no-update-check"])
            except Exception:
                pass
        return 4

    try:
        _launch(target, ["--updated", "--no-update-check"])
    except Exception as exc:
        log(f"launch failed: {exc}; restoring")
        _restore_and_launch(target, backup)
        return 5

    if _wait_for_startup(expected_version, launched_at, 25.0):
        log("new version started successfully")
        return 0

    log("new version did not report startup; rolling back")
    _restore_and_launch(target, backup)
    return 6


def launch_updater(new_exe: Path, target_exe: Path, old_pid: int,
                   expected_version: str, parent_pid: int = 0) -> None:
    """Start the downloaded new exe in updater mode and let it take over."""
    cmd = [
        str(new_exe),
        "--apply-update",
        "--target", str(target_exe),
        "--old-pid", str(old_pid),
        "--expected-version", str(expected_version),
    ]
    if parent_pid and parent_pid > 0:
        cmd += ["--parent-pid", str(parent_pid)]
    creation = 0
    if os.name == "nt":
        creation = (getattr(subprocess, "DETACHED_PROCESS", 0) |
                    getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
    subprocess.Popen(cmd, close_fds=True, creationflags=creation,
                     cwd=str(Path(target_exe).resolve().parent))


def check_writable(path: Path) -> bool:
    try:
        path = Path(path)
        path.mkdir(parents=True, exist_ok=True)
        test = path / ".ads122u04_write_test"
        test.write_text("x", encoding="ascii")
        test.unlink()
        return True
    except Exception:
        return False
