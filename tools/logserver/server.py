#!/usr/bin/env python3
"""
A minimal self-hosted replacement for the comma/retropilot API, just big enough
to let an openpilot device register itself and upload its logs.

Implements the endpoints this fork actually calls:

  POST /v2/pilotauth/                     selfdrive/athena/registration.py
  GET  /v1.4/<dongle_id>/upload_url/      selfdrive/loggerd/uploader.py
  PUT  /upload/<dongle_id>/<path>         (the URL we hand back above)
  GET  /v1.1/devices/<dongle_id>/         selfdrive/ui/qt/widgets/prime.cc
  GET  /v1/devices/<dongle_id>/owner      selfdrive/ui/qt/widgets/prime.cc
  GET  /v1.1/devices/<dongle_id>/stats    selfdrive/ui/qt/widgets/drive_stats.cc
  GET  /v1/route/<route>/files            tools/lib/route.py, replay
  GET  /v1/devices/<dongle_id>/bootlogs   tools/lib/bootlog.py
  GET  /<dongle_id>/<time>/<seg>/<fn>     download URL handed out above

Run:
  pip install -r requirements.txt
  BASE_URL=http://192.168.1.50:8080 ./server.py

Point the device at it:
  API_HOST=http://192.168.1.50:8080
"""

import hashlib
import hmac
import json
import os
import re
import sqlite3
import time
from pathlib import Path

import jwt
from flask import Flask, Response, abort, jsonify, request, send_file

# ---------------------------------------------------------------- config

DATA_DIR = Path(os.getenv("DATA_DIR", "./data")).resolve()
DB_PATH = Path(os.getenv("DB_PATH", DATA_DIR / "logserver.db"))
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8080"))

# What the device should call back to. Must be reachable *from the comma device*,
# so localhost will not work -- use the LAN IP or public hostname.
BASE_URL = os.getenv("BASE_URL", f"http://127.0.0.1:{PORT}").rstrip("/")

# Signs the short-lived upload/download URLs. Persisted so URLs survive a restart.
SECRET_PATH = DATA_DIR / "secret.key"

# Optional shared secret. If set, a device must send ?registration_token=<value>
# to be allowed to register. Without it, any device that can reach the server
# can claim a dongle id.
REGISTRATION_TOKEN = os.getenv("REGISTRATION_TOKEN")

URL_TTL = 60 * 60  # signed upload/download URLs are good for an hour

SEGMENT_RE = re.compile(r"^(?P<time>\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})--(?P<seg>\d+)$")
FILENAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")

app = Flask(__name__)


def get_secret() -> bytes:
  if not SECRET_PATH.exists():
    SECRET_PATH.parent.mkdir(parents=True, exist_ok=True)
    SECRET_PATH.write_bytes(os.urandom(32))
    SECRET_PATH.chmod(0o600)
  return SECRET_PATH.read_bytes()


# ---------------------------------------------------------------- storage

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


def dongle_id_for_serial(serial: str) -> str:
  """Derive a stable 16-hex-char dongle id from the hardware serial, so a device
  that re-registers (new keys, reflashed nvme) keeps its existing route history."""
  return hashlib.sha256(("openpilot-logserver:" + serial).encode()).hexdigest()[:16]


# ---------------------------------------------------------------- auth

def verify_device_token(dongle_id: str):
  """The device signs a JWT with the RSA key it registered. Verify it against the
  public key we stored at registration time."""
  auth = request.headers.get("Authorization", "")
  if not auth.startswith("JWT "):
    abort(401, "missing JWT")

  device = get_device(dongle_id)
  if device is None:
    abort(404, "unknown device")

  try:
    claims = jwt.decode(auth[4:], device["public_key"], algorithms=["RS256"])
  except jwt.PyJWTError as e:
    abort(401, f"bad token: {e}")

  if claims.get("identity") != dongle_id:
    abort(403, "token identity mismatch")

  with db() as conn:
    conn.execute("UPDATE devices SET last_seen = ? WHERE dongle_id = ?", (int(time.time()), dongle_id))
  return device


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
  parts = key.strip("/").split("/")
  if len(parts) != 2:
    abort(400, "bad path")

  seg, fn = parts
  if not FILENAME_RE.match(fn) or fn in (".", ".."):
    abort(400, "bad path")

  if seg in ("boot", "crash"):
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


# ---------------------------------------------------------------- registration

@app.route("/v2/pilotauth/", methods=["POST"])
def pilotauth():
  # NB: common/api sends everything as query params, even on POST.
  args = request.args if request.args else request.form

  serial = args.get("serial")
  public_key = args.get("public_key")
  register_token = args.get("register_token")
  if not serial or not public_key or not register_token:
    abort(400, "missing serial/public_key/register_token")

  if REGISTRATION_TOKEN and not hmac.compare_digest(args.get("registration_token", ""), REGISTRATION_TOKEN):
    # 403 makes the device fall back to the UNREGISTERED_DONGLE_ID rather than retry forever
    abort(403, "registration not permitted")

  # Prove the device holds the private key for the public key it's handing us.
  try:
    claims = jwt.decode(register_token, public_key, algorithms=["RS256"])
  except jwt.PyJWTError as e:
    abort(400, f"bad register_token: {e}")
  if not claims.get("register"):
    abort(400, "register_token missing register claim")

  dongle_id = dongle_id_for_serial(serial)
  now = int(time.time())
  with db() as conn:
    conn.execute("""
      INSERT INTO devices (dongle_id, serial, public_key, imei, imei2, created_at, last_seen)
      VALUES (?, ?, ?, ?, ?, ?, ?)
      ON CONFLICT(dongle_id) DO UPDATE SET
        public_key = excluded.public_key,
        imei       = excluded.imei,
        imei2      = excluded.imei2,
        last_seen  = excluded.last_seen
    """, (dongle_id, serial, public_key, args.get("imei"), args.get("imei2"), now, now))

  (DATA_DIR / dongle_id).mkdir(parents=True, exist_ok=True)
  app.logger.info("registered %s (serial %s)", dongle_id, serial)
  return jsonify({"dongle_id": dongle_id, "access_token": "unused"})


# ---------------------------------------------------------------- upload

@app.route("/v1.4/<dongle_id>/upload_url/")
def upload_url(dongle_id):
  verify_device_token(dongle_id)

  key = request.args.get("path")
  if not key:
    abort(400, "missing path")

  dest = safe_segment_path(dongle_id, key)
  if dest.exists() and dest.stat().st_size > 0:
    # 412 tells the uploader "already have it", and it marks the file uploaded
    # locally so the deleter can reclaim the space.
    return Response("already uploaded", status=412)

  expires = int(time.time()) + URL_TTL
  sig = sign_url("PUT", dongle_id, key, expires)
  return jsonify({
    "url": f"{BASE_URL}/upload/{dongle_id}/{key}?expires={expires}&sig={sig}",
    "headers": {},
  })


@app.route("/upload/<dongle_id>/<path:key>", methods=["PUT"])
def do_upload(dongle_id, key):
  check_signature("PUT", dongle_id, key)

  dest = safe_segment_path(dongle_id, key)
  dest.parent.mkdir(parents=True, exist_ok=True)

  # Stream to a temp file so a dropped connection never leaves a truncated
  # segment that later looks like a complete upload.
  tmp = dest.with_suffix(dest.suffix + ".part")
  size = 0
  with open(tmp, "wb") as f:
    while True:
      chunk = request.stream.read(64 * 1024)
      if not chunk:
        break
      f.write(chunk)
      size += len(chunk)
  tmp.replace(dest)

  app.logger.info("uploaded %s/%s (%.1f MB)", dongle_id, key, size / 1e6)
  return Response(status=201)


# ---------------------------------------------------------------- routes / files

def segment_dirs(dongle_id):
  root = DATA_DIR / dongle_id
  if not root.is_dir():
    return
  for time_dir in sorted(root.iterdir()):
    if not time_dir.is_dir() or time_dir.name in ("boot", "crash"):
      continue
    for seg_dir in sorted(time_dir.iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else -1):
      if seg_dir.is_dir():
        yield time_dir.name, seg_dir.name, seg_dir


def download_url(dongle_id, time_str, seg, fn):
  expires = int(time.time()) + URL_TTL
  key = f"{time_str}--{seg}/{fn}"
  sig = sign_url("GET", dongle_id, key, expires)
  # tools/lib/route.py parses this as .../<dongle_id>/<time>/<seg>/<fn>
  return f"{BASE_URL}/{dongle_id}/{time_str}/{seg}/{fn}?expires={expires}&sig={sig}"


# Which response key each filename belongs under, matching what connect/replay expect.
FILE_GROUPS = {
  "rlog": "logs", "rlog.bz2": "logs", "raw_log.bz2": "logs",
  "qlog": "qlogs", "qlog.bz2": "qlogs",
  "fcamera.hevc": "cameras", "video.hevc": "cameras",
  "dcamera.hevc": "dcameras",
  "ecamera.hevc": "ecameras",
  "qcamera.ts": "qcameras",
}


@app.route("/v1/route/<route>/files")
def route_files(route):
  # canonical name is "<dongle_id>|<time_str>"
  if "|" in route:
    dongle_id, _, time_str = route.partition("|")
  else:
    abort(400, "expected <dongle_id>|<route>")

  out = {k: [] for k in set(FILE_GROUPS.values())}
  for t, seg, seg_dir in segment_dirs(dongle_id):
    if t != time_str:
      continue
    for f in sorted(seg_dir.iterdir()):
      group = FILE_GROUPS.get(f.name)
      if group and f.is_file():
        out[group].append(download_url(dongle_id, t, seg, f.name))
  return jsonify(out)


@app.route("/v1/devices/<dongle_id>/routes_segments")
def routes_segments(dongle_id):
  routes = {}
  for t, seg, _ in segment_dirs(dongle_id):
    r = routes.setdefault(t, {"fullname": f"{dongle_id}|{t}", "maxqlog": -1, "segment_numbers": []})
    r["segment_numbers"].append(int(seg))
    r["maxqlog"] = max(r["maxqlog"], int(seg))
  return jsonify(sorted(routes.values(), key=lambda r: r["fullname"], reverse=True))


@app.route("/v1/devices/<dongle_id>/bootlogs")
def bootlogs(dongle_id):
  boot_dir = DATA_DIR / dongle_id / "boot"
  if not boot_dir.is_dir():
    return jsonify([])
  expires = int(time.time()) + URL_TTL
  return jsonify([
    {"url": f"{BASE_URL}/boot/{dongle_id}/{f.name}?expires={expires}"
            f"&sig={sign_url('GET', dongle_id, 'boot/' + f.name, expires)}"}
    for f in sorted(boot_dir.iterdir()) if f.is_file()
  ])


@app.route("/boot/<dongle_id>/<fn>")
@app.route("/crash/<dongle_id>/<fn>")
def download_flat(dongle_id, fn):
  folder = request.path.split("/")[1]
  check_signature("GET", dongle_id, f"{folder}/{fn}")
  path = safe_segment_path(dongle_id, f"{folder}/{fn}")
  if not path.is_file():
    abort(404)
  return send_file(path, conditional=True)


@app.route("/<dongle_id>/<time_str>/<seg>/<fn>")
def download(dongle_id, time_str, seg, fn):
  check_signature("GET", dongle_id, f"{time_str}--{seg}/{fn}")
  path = safe_segment_path(dongle_id, f"{time_str}--{seg}/{fn}")
  if not path.is_file():
    abort(404)
  return send_file(path, conditional=True)


# ---------------------------------------------------------------- UI stubs

@app.route("/v1.1/devices/<dongle_id>/")
def device_info(dongle_id):
  device = get_device(dongle_id)
  if device is None:
    abort(404)
  return jsonify({
    "dongle_id": dongle_id,
    "is_paired": True,
    "prime": True,
    "prime_type": 1,
    "device_type": "unknown",
    "serial": device["serial"],
  })


@app.route("/v1/devices/<dongle_id>/owner")
def device_owner(dongle_id):
  return jsonify({"points": 0, "username": "self-hosted"})


@app.route("/v1.1/devices/<dongle_id>/stats")
def device_stats(dongle_id):
  # Real numbers would mean parsing every qlog; report segment counts instead
  # so the offroad UI has something coherent to show.
  segs = [s for s in segment_dirs(dongle_id)]
  routes = len({t for t, _, _ in segs})
  minutes = len(segs)  # segments are ~1 minute each
  blank = {"routes": routes, "distance": 0, "minutes": minutes}
  return jsonify({"all": blank, "week": dict(blank)})


@app.route("/health")
def health():
  with db() as conn:
    n = conn.execute("SELECT COUNT(*) c FROM devices").fetchone()["c"]
  return jsonify({"ok": True, "devices": n, "data_dir": str(DATA_DIR)})


@app.errorhandler(400)
@app.errorhandler(401)
@app.errorhandler(403)
@app.errorhandler(404)
def json_error(e):
  return jsonify({"error": e.description}), e.code


init_db()  # at import, so `gunicorn server:app` works too

if __name__ == "__main__":
  print(f"data dir: {DATA_DIR}")
  print(f"base url: {BASE_URL}  (must be reachable from the device)")
  if not REGISTRATION_TOKEN:
    print("warning: REGISTRATION_TOKEN unset, any device that can reach this server can register")
  app.run(host=HOST, port=PORT, threaded=True)
