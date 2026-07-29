#!/usr/bin/env python3
"""
A minimal self-hosted replacement for the comma/retropilot API, just big enough
to let an openpilot device register itself and upload its logs -- plus a web
interface (web.py) for browsing and downloading what it collected.

Implements the endpoints this fork actually calls:

  POST /v2/pilotauth/                     selfdrive/athena/registration.py
  GET  /v1.4/<dongle_id>/upload_url/      selfdrive/loggerd/uploader.py
  PUT  /upload/<dongle_id>/<path>         (the URL we hand back above)
  GET  /v1.1/devices/<dongle_id>/         selfdrive/ui/qt/widgets/prime.cc
  GET  /v1/devices/<dongle_id>/owner      selfdrive/ui/qt/widgets/prime.cc
  GET  /v1.1/devices/<dongle_id>/stats    selfdrive/ui/qt/widgets/drive_stats.cc
  GET  /v1/route/<route>/files            tools/lib/route.py, replay
  GET  /v1/devices/<dongle_id>/bootlogs   tools/lib/bootlog.py

Run:
  pip install -r requirements.txt
  BASE_URL=http://192.168.1.50:8080 ./server.py

Point the device at it:
  API_HOST=http://192.168.1.50:8080
"""

import hmac
import json
import os
import time

import jwt
from flask import Flask, Response, abort, jsonify, redirect, request, send_file
from werkzeug.middleware.proxy_fix import ProxyFix

import store
from store import (BASE_URL, DATA_DIR, FILE_GROUPS, URL_TTL, check_signature, db,
                   get_device, safe_segment_path, segment_dirs, sign_url)

app = Flask(__name__)
app.secret_key = store.get_secret()
app.config.update(
  SESSION_COOKIE_HTTPONLY=True,
  SESSION_COOKIE_SAMESITE="Lax",
  # Deliberately NOT keyed off BASE_URL. Marking the cookie Secure makes browsers
  # drop it on any plain-http request, which silently breaks reaching the server
  # directly on a LAN address even though the tunnel is https. Opt in with
  # SECURE_COOKIE=1 when every client is guaranteed to arrive over https.
  SESSION_COOKIE_SECURE=os.getenv("SECURE_COOKIE", "").lower() in ("1", "true", "yes"),
)

# cloudflared terminates TLS and forwards over http; without this Flask builds
# redirects with the wrong scheme when it sits behind the tunnel.
app.wsgi_app = ProxyFix(app.wsgi_app, x_proto=1, x_host=1)


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

  if store.REGISTRATION_TOKEN and not hmac.compare_digest(
      args.get("registration_token", ""), store.REGISTRATION_TOKEN):
    # 403 makes the device fall back to the UNREGISTERED_DONGLE_ID rather than retry forever
    abort(403, "registration not permitted")

  # Prove the device holds the private key for the public key it's handing us.
  try:
    claims = jwt.decode(register_token, public_key, algorithms=["RS256"])
  except jwt.PyJWTError as e:
    abort(400, f"bad register_token: {e}")
  if not claims.get("register"):
    abort(400, "register_token missing register claim")

  dongle_id = store.dongle_id_for_serial(serial)
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

def download_url(dongle_id, time_str, seg, fn):
  expires = int(time.time()) + URL_TTL
  key = f"{time_str}--{seg}/{fn}"
  sig = sign_url("GET", dongle_id, key, expires)
  # tools/lib/route.py parses this as .../<dongle_id>/<time>/<seg>/<fn>
  return f"{BASE_URL}/{dongle_id}/{time_str}/{seg}/{fn}?expires={expires}&sig={sig}"


@app.route("/v1/route/<route>/files")
def route_files(route):
  # canonical name is "<dongle_id>|<time_str>"
  if "|" not in route:
    abort(400, "expected <dongle_id>|<route>")
  dongle_id, _, time_str = route.partition("|")

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
  segs = list(segment_dirs(dongle_id))
  routes = len({t for t, _, _ in segs})
  minutes = len(segs)  # segments are ~1 minute each
  blank = {"routes": routes, "distance": 0, "minutes": minutes}
  return jsonify({"all": blank, "week": dict(blank)})


@app.route("/health")
def health():
  with db() as conn:
    n = conn.execute("SELECT COUNT(*) c FROM devices").fetchone()["c"]
  return jsonify({"ok": True, "devices": n, "data_dir": str(DATA_DIR)})


@app.route("/")
def index():
  return redirect("/ui/")


@app.errorhandler(400)
@app.errorhandler(401)
@app.errorhandler(403)
@app.errorhandler(404)
def json_error(e):
  # The web interface renders its own error pages; only the API speaks JSON.
  if request.path.startswith("/ui/"):
    return e
  return jsonify({"error": e.description}), e.code


store.init_db()  # at import, so `gunicorn server:app` works too

import web  # noqa: E402  (imported after app exists to avoid a circular import)
app.register_blueprint(web.bp)

if __name__ == "__main__":
  print(f"data dir: {DATA_DIR}")
  print(f"base url: {BASE_URL}  (must be reachable from the device)")
  if not store.REGISTRATION_TOKEN:
    print("warning: REGISTRATION_TOKEN unset, any device that can reach this server can register")
  if not store.WEB_PASSWORD:
    print("warning: WEB_PASSWORD unset, the web interface at /ui/ is disabled")
  app.run(host=store.HOST, port=store.PORT, threaded=True)
