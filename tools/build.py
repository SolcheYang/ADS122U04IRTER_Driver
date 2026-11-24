#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build the ADS122U04 tool with git-tag-derived version metadata.

Typical use:
    python tools/build.py                 # development build
    python tools/build.py --release       # requires an exact vX.Y.Z tag and clean tree
    python tools/build.py --force-release --version 1.0.1   # local testing only

A release build also produces:
    dist/ADS122U04_ADC_Tool.exe
    dist/latest.json          # manifest used by the in-app updater
    dist/release_notes.md     # GitHub release body
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
BUILD = ROOT / "build"
DIST = ROOT / "dist"
APP_NAME = "ADS122U04_ADC_Tool"
EXE_NAME = APP_NAME + ".exe"
TAG_RE = re.compile(r"^[vV]?(\d+)\.(\d+)\.(\d+)$")


def git(*args, default=None):
    try:
        return subprocess.check_output(
            ["git", *args], cwd=str(ROOT), text=True,
            stderr=subprocess.DEVNULL).strip()
    except Exception:
        if default is not None:
            return default
        raise


def git_ok(*args) -> bool:
    try:
        subprocess.check_call(["git", *args], cwd=str(ROOT),
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL)
        return True
    except Exception:
        return False


def parse_tag(tag):
    m = TAG_RE.match((tag or "").strip())
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def repo_slug():
    env = os.environ.get("GITHUB_REPOSITORY", "").strip()
    if env:
        return env
    url = git("remote", "get-url", "origin", default="")
    m = re.search(r"github\.com[:/](.+?)(?:\.git)?$", url)
    return m.group(1) if m else ""


def current_tag():
    env_tag = os.environ.get("GITHUB_REF_NAME", "").strip()
    if os.environ.get("GITHUB_REF_TYPE", "") == "tag" and env_tag:
        return env_tag
    return git("describe", "--tags", "--exact-match", "HEAD", default="")


def is_dirty():
    return bool(git("status", "--porcelain", default=""))


def write_version_resource(path: Path, version_tuple, version_text):
    major, minor, patch = version_tuple
    content = f"""# UTF-8
VSVersionInfo(
  ffi=FixedFileInfo(
    filevers=({major}, {minor}, {patch}, 0),
    prodvers=({major}, {minor}, {patch}, 0),
    mask=0x3f,
    flags=0x0,
    OS=0x40004,
    fileType=0x1,
    subtype=0x0,
    date=(0, 0)
  ),
  kids=[
    StringFileInfo([
      StringTable('080404B0', [
        StringStruct('CompanyName', 'ADS122U04 Tool'),
        StringStruct('FileDescription', 'ADS122U04 UART ADC acquisition tool'),
        StringStruct('FileVersion', '{major}.{minor}.{patch}.0'),
        StringStruct('InternalName', '{APP_NAME}'),
        StringStruct('OriginalFilename', '{EXE_NAME}'),
        StringStruct('ProductName', 'ADS122U04 UART ADC acquisition tool'),
        StringStruct('ProductVersion', '{major}.{minor}.{patch}.0'),
        StringStruct('Comments', 'build {version_text}')
      ])
    ]),
    VarFileInfo([VarStruct('Translation', [2052, 1200])])
  ]
)
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description="Build ADS122U04 tool")
    ap.add_argument("--release", action="store_true",
                    help="release build: exact vX.Y.Z tag, clean tree")
    ap.add_argument("--force-release", action="store_true",
                    help="local testing: release metadata even without a clean tag")
    ap.add_argument("--version", default="",
                    help="override version (testing only)")
    ap.add_argument("--no-build", action="store_true",
                    help="only generate metadata (testing)")
    args = ap.parse_args()

    tag = current_tag()
    tag_vt = parse_tag(tag)
    dirty = is_dirty()

    if args.release:
        if not tag_vt:
            sys.exit("ERROR: --release requires HEAD to be tagged as vX.Y.Z "
                     f"(current tag: {tag!r})")
        if dirty:
            sys.exit("ERROR: working tree is dirty; commit/stash before a release build")

    if args.version:
        vt = parse_tag(args.version)
        if not vt:
            sys.exit("ERROR: --version must be X.Y.Z")
        version = f"{vt[0]}.{vt[1]}.{vt[2]}"
        if args.force_release:
            # Local testing only: make the tag/version fields consistent.
            tag = f"v{version}"
            tag_vt = vt
    elif tag_vt:
        vt = tag_vt
        version = f"{vt[0]}.{vt[1]}.{vt[2]}"
    else:
        vt = (0, 0, 0)
        desc = git("describe", "--tags", "--always", "--dirty", default="0.0.0")
        version = desc if desc else "0.0.0-dev"

    release = bool(args.release and tag_vt and not dirty) or bool(args.force_release)
    if args.force_release:
        version = version.split("-")[0]
        vt = parse_tag(version) or vt
        release = True
        print("WARNING: --force-release is for local testing; tag/clean state ignored")

    if not args.force_release and not tag_vt and not args.version:
        release = False

    commit = git("rev-parse", "HEAD", default="")
    commit_short = commit[:7]
    commit_msg = git("log", "-1", "--format=%s", default="")
    commit_body = git("log", "-1", "--format=%B", default="")
    commit_date = git("log", "-1", "--format=%cI", default="")
    tag_notes = ""
    if tag:
        tag_notes = git("tag", "-l", "--format=%(contents)", tag, default="").strip()
    notes = tag_notes or commit_body or commit_msg
    notes = notes[:12000]
    repo = repo_slug()
    build_time = datetime.now(timezone.utc).isoformat(timespec="seconds")

    info = {
        "app": APP_NAME,
        "version": version,
        "tag": tag or "",
        "commit": commit,
        "commit_short": commit_short,
        "commit_message": commit_msg,
        "commit_date": commit_date,
        "build_time": build_time,
        "dirty": dirty,
        "release": release,
        "repo": repo,
        "notes": notes,
    }

    BUILD.mkdir(parents=True, exist_ok=True)
    build_info_path = BUILD / "build_info.json"
    build_info_path.write_text(json.dumps(info, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    # A source-run copy (ignored by git) makes local testing convenient.
    try:
        (SRC / "build_info.json").write_text(
            json.dumps(info, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass

    version_file = BUILD / "version_info.txt"
    write_version_resource(version_file, vt, version)

    print(f"Version   : {version}")
    print(f"Tag       : {tag or '(none)'}")
    print(f"Commit    : {commit_short} {commit_msg}")
    print(f"Release   : {release}")
    print(f"Repo      : {repo}")
    print(f"Build info: {build_info_path}")

    if not args.no_build:
        DIST.mkdir(parents=True, exist_ok=True)
        spec_dir = BUILD
        cmd = [
            sys.executable, "-m", "PyInstaller",
            "--noconfirm", "--clean", "--onefile", "--noconsole",
            "--name", APP_NAME,
            "--icon", str(ROOT / "icon.ico"),
            "--add-data", f"{build_info_path}{os.pathsep}.",
            "--add-data", f"{ROOT / 'icon.ico'}{os.pathsep}.",
            "--version-file", str(version_file),
            "--distpath", str(DIST),
            "--workpath", str(BUILD / "pyinstaller"),
            "--specpath", str(spec_dir),
            str(SRC / "ads122u04_tool.py"),
        ]
        print("Running PyInstaller...")
        subprocess.run(cmd, cwd=str(ROOT), check=True)

    exe = DIST / EXE_NAME
    if not exe.exists():
        print(f"Built exe not found: {exe}")
        return 0

    size = exe.stat().st_size
    digest = sha256_file(exe)
    print(f"EXE       : {exe} ({size} bytes)")
    print(f"SHA-256   : {digest}")

    if release:
        if not repo:
            print("WARNING: cannot determine GitHub repo; manifest URL will be empty")
        url = (f"https://github.com/{repo}/releases/download/{tag}/{EXE_NAME}"
               if repo and tag else "")
        manifest = {
            "schema": 1,
            "app": APP_NAME,
            "tag": tag or f"v{version}",
            "version": version,
            "commit": commit,
            "commit_short": commit_short,
            "commit_message": commit_msg,
            "commit_date": commit_date,
            "notes": notes,
            "url": url,
            "sha256": digest,
            "size": size,
            "mandatory": True,
            "min_supported": "1.0.0",
            "repo": repo,
            "published_at": build_time,
        }
        manifest_path = DIST / "latest.json"
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        notes_path = DIST / "release_notes.md"
        notes_path.write_text(
            f"# {APP_NAME} {tag or version}\n\n"
            f"- commit: `{commit_short}` — {commit_msg}\n"
            f"- date: {commit_date}\n\n{notes}\n",
            encoding="utf-8")
        print(f"Manifest  : {manifest_path}")
        print(f"Notes     : {notes_path}")
    else:
        print("Development build: no latest.json generated; update check disabled")

    return 0


if __name__ == "__main__":
    sys.exit(main())
