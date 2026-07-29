"""Config, storage layout, and path handling shared by the device API (server.py)
and the web interface (web.py)."""

import hashlib
import hmac
import os
import re
import sqlite3
import time
from pathlib import Path

from flask import abort, request

DATA_DIR = Path(os.getenv("DATA_DIR", "./data")).resolve()
DB_PATH = Path(os.getenv("DB_PATH", DATA_DIR / "logserver.db"))
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8080"))

# What the device should call back to. Must be reachable *from the comma device*,
# so localhost will not work -- use the LAN IP or public hostname.
BASE_URL = os.getenv("BASE_URL", f"http://127.0.0.1:{PORT}").rstrip("/")

# Signs upload/download URLs and the web session cookie. Persisted so both
# survive a restart.
SECRET_PATH = DATA_DIR / "secret.key"

# Optional shared secret. If set, a device must send ?registration_token=<value>
# to be allowed to register.
REGISTRATION_TOKEN = os.getenv("REGISTRATION_TOKEN") or None

# Password for the web interface. Unset means the UI stays switched off.
WEB_PASSWORD = os.getenv("WEB_PASSWORD") or None

URL_TTL = 60 * 60  # signed upload/download URLs are good for an hour

SEGMENT_RE = re.compile(r"^(?P<time>\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})--(?P<seg>\d+)$")
TIME_RE = re.compile(r"^\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}$")
DONGLE_RE = re.compile(r"^[0-9a-f]{16}$")
FILENAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")

FLAT_FOLDERS = ("boot", "crash")

# Which response key each filename belongs under, matching connect/replay.
FILE_GROUPS = {
  "rlog": "logs", "rlog.bz2": "logs", "raw_log.bz2": "logs",
  "qlog": "qlogs", "qlog.bz2": "qlogs",
  "fcamera.hevc": "cameras", "video.hevc": "cameras",
  "dcamera.hevc": "dcameras",
  "ecamera.hevc": "ecameras",
  "qcamera.ts": "qcameras",
}

# Human labels for the file table in the web UI.
FILE_LABELS = {
  "qlog": "qlog", "qlog.bz2": "qlog",
  "rlog": "rlog", "rlog.bz2": "rlog", "raw_log.bz2": "rlog",
  "qcamera.ts": "road (low res)",
  "fcamera.hevc": "road", "video.hevc": "road",
  "dcamera.hevc": "driver",
  "ecamera.hevc": "wide",
}


def get_secret() -> bytes:
  if not SECRET_PATH.exists():
    SECRET_PATH.parent.mkdir(parents=True, exist_ok=True)
    SECRET_PATH.write_bytes(os.urandom(32))
    SECRET_PATH.chmod(0o600)
  return SECRET_PATH.read_bytes()


# ---------------------------------------------------------------- db

def db():
  conn = sqlite3.connect(DB_PATH)
  conn.row_factory = sqlite3.Row
  return conn


def init_db():
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  with db() as conn:
    conn.execute("""
      CREATE TABLE IF NOT EXISTS devices (
        dongle_id   TEXT PRIMARY KEY,
        serial      TEXT UNIQUE,
        public_key  TEXT NOT NULL,
        imei        TEXT,
        imei2       TEXT,
        created_at  INTEGER NOT NULL,
        last_seen   INTEGER NOT NULL
      )
    """)


def get_device(dongle_id):
  with db() as conn:
    return conn.execute("SELECT * FROM devices WHERE dongle_id = ?", (dongle_id,)).fetchone()


def all_devices():
  with db() as conn:
    return conn.execute("SELECT * FROM devices ORDER BY last_seen DESC").fetchall()


def dongle_id_for_serial(serial: str) -> str:
  """Derive a stable 16-hex-char dongle id from the hardware serial, so a device
  that re-registers (new keys, reflashed nvme) keeps its existing route history."""
  return hashlib.sha256(("openpilot-logserver:" + serial).encode()).hexdigest()[:16]


# ---------------------------------------------------------------- signed urls

def sign_url(method: str, dongle_id: str, path: str, expires: int) -> str:
  msg = f"{method}\n{dongle_id}\n{path}\n{expires}".encode()
  return hmac.new(get_secret(), msg, hashlib.sha256).hexdigest()


def check_signature(method: str, dongle_id: str, path: str):
  try:
    expires = int(request.args.get("expires", "0"))
  except ValueError:
    abort(403, "bad expiry")
  if expires < time.time():
    abort(403, "url expired")
  expected = sign_url(method, dongle_id, path, expires)
  if not hmac.compare_digest(expected, request.args.get("sig", "")):
    abort(403, "bad signature")


# ---------------------------------------------------------------- paths

def safe_segment_path(dongle_id: str, key: str) -> Path:
  """`key` is what the uploader sends: either '<segment-dir>/<filename>' e.g.
  '2024-01-01--12-00-00--3/qlog.bz2', or one of the flat 'boot/' and 'crash/'
  folders. Reject anything else so a malformed key can't escape DATA_DIR."""
  if not DONGLE_RE.match(dongle_id):
    abort(400, "bad dongle id")

  parts = key.strip("/").split("/")
  if len(parts) != 2:
    abort(400, "bad path")

  seg, fn = parts
  if not FILENAME_RE.match(fn) or fn in (".", ".."):
    abort(400, "bad path")

  if seg in FLAT_FOLDERS:
    out = DATA_DIR / dongle_id / seg / fn
  else:
    m = SEGMENT_RE.match(seg)
    if not m:
      abort(400, "bad path")
    out = DATA_DIR / dongle_id / m.group("time") / m.group("seg") / fn

  # belt and braces: the resolved path must still live under DATA_DIR
  if not str(out.resolve()).startswith(str(DATA_DIR) + os.sep):
    abort(400, "bad path")
  return out


def route_dir(dongle_id: str, time_str: str) -> Path:
  if not DONGLE_RE.match(dongle_id) or not TIME_RE.match(time_str):
    abort(400, "bad route")
  return DATA_DIR / dongle_id / time_str


def segment_dirs(dongle_id):
  """Yield (time_str, segment_num, path) for every segment of every route."""
  root = DATA_DIR / dongle_id
  if not root.is_dir():
    return
  for time_dir in sorted(root.iterdir()):
    if not time_dir.is_dir() or time_dir.name in FLAT_FOLDERS:
      continue
    for seg_dir in sorted(time_dir.iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else -1):
      if seg_dir.is_dir() and seg_dir.name.isdigit():
        yield time_dir.name, seg_dir.name, seg_dir


def route_summary(dongle_id, time_str):
  """Segment count, total bytes, and which file kinds are present for one route."""
  segs, size, kinds = [], 0, set()
  for t, seg, seg_dir in segment_dirs(dongle_id):
    if t != time_str:
      continue
    segs.append(int(seg))
    for f in seg_dir.iterdir():
      if f.is_file():
        size += f.stat().st_size
        kinds.add(f.name)
  return {
    "time_str": time_str,
    "segments": sorted(segs),
    "count": len(segs),
    "size": size,
    "kinds": kinds,
    "has_video": "qcamera.ts" in kinds,
  }


def routes_for(dongle_id):
  times = sorted({t for t, _, _ in segment_dirs(dongle_id)}, reverse=True)
  return [route_summary(dongle_id, t) for t in times]
