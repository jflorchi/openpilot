#!/usr/bin/env python3
"""Checks for the web interface: auth gating, browsing, downloads, and video."""

import json
import os
import shutil
import subprocess
import tempfile
from datetime import datetime, timedelta

import jwt
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

tmpdir = tempfile.mkdtemp()
os.environ["DATA_DIR"] = tmpdir
os.environ["BASE_URL"] = "http://testserver"
os.environ["WEB_PASSWORD"] = "hunter2"

import server  # noqa: E402
import store  # noqa: E402

client = server.app.test_client()

key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
private_key = key.private_bytes(
  serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
  serialization.NoEncryption()).decode()
public_key = key.public_key().public_bytes(
  serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo).decode()

register_token = jwt.encode(
  {"register": True, "exp": datetime.utcnow() + timedelta(hours=1)}, private_key, algorithm="RS256")
r = client.post("/v2/pilotauth/", query_string={
  "imei": "1", "imei2": "2", "serial": "web-test",
  "public_key": public_key, "register_token": register_token})
dongle_id = json.loads(r.data)["dongle_id"]

TIME = "2024-03-05--09-15-00"

# --- everything behind the login wall stays there
protected = ["/ui/", f"/ui/d/{dongle_id}", f"/ui/d/{dongle_id}/r/{TIME}",
             f"/ui/dl/{dongle_id}/{TIME}/0/qlog.bz2", f"/ui/video/{dongle_id}/{TIME}.mp4"]
for path in protected:
  r = client.get(path)
  assert r.status_code == 302 and "/ui/login" in r.headers["Location"], f"{path} -> {r.status_code}"
print("auth:       anonymous access redirects to login")

# --- wrong password doesn't authenticate
r = client.post("/ui/login", data={"password": "wrong"})
assert r.status_code == 401
assert client.get("/ui/").status_code == 302
print("auth:       wrong password rejected")

# --- correct password does
r = client.post("/ui/login", data={"password": "hunter2"}, follow_redirects=True)
assert r.status_code == 200 and b"Devices" in r.data
print("auth:       login works")

# --- device list renders, with the dongle on it
r = client.get("/ui/")
assert dongle_id.encode() in r.data

# --- empty state before any upload
r = client.get(f"/ui/d/{dongle_id}")
assert r.status_code == 200 and b"uploaded anything yet" in r.data
print("empty:      device with no drives explains what to do")

# --- upload two segments through the real device path
def upload(seg, name, payload):
  auth = {"Authorization": "JWT " + jwt.encode(
    {"identity": dongle_id, "exp": datetime.utcnow() + timedelta(hours=1)},
    private_key, algorithm="RS256")}
  r = client.get(f"/v1.4/{dongle_id}/upload_url/",
                 query_string={"path": f"{TIME}--{seg}/{name}"}, headers=auth)
  url = json.loads(r.data)["url"].replace("http://testserver", "")
  assert client.put(url, data=payload).status_code == 201

have_ffmpeg = shutil.which("ffmpeg") is not None
for seg in (0, 1):
  upload(seg, "qlog.bz2", b"qlog-%d" % seg)
  if have_ffmpeg:
    # a real 1-second H.264 transport stream, so the concat is genuinely exercised
    ts = os.path.join(tmpdir, f"seg{seg}.ts")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-f", "lavfi", "-i", f"testsrc=size=160x120:rate=10:duration=1",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-f", "mpegts", ts],
                   check=True, capture_output=True)
    upload(seg, "qcamera.ts", open(ts, "rb").read())

# --- the drive now shows up
r = client.get(f"/ui/d/{dongle_id}")
assert r.status_code == 200 and TIME.encode() in r.data
print("browse:     drive listed after upload")

r = client.get(f"/ui/d/{dongle_id}/r/{TIME}")
assert r.status_code == 200 and b"qlog" in r.data
print("browse:     route page lists segment files")

# --- download returns the bytes we uploaded
r = client.get(f"/ui/dl/{dongle_id}/{TIME}/0/qlog.bz2")
assert r.status_code == 200 and r.data == b"qlog-0", r.data
assert "attachment" in r.headers.get("Content-Disposition", "")
print("download:   serves the uploaded bytes as an attachment")

# --- path traversal through the UI download route
for bad in ["..%2f..%2fetc%2fpasswd", "....//etc"]:
  assert client.get(f"/ui/dl/{dongle_id}/{TIME}/0/{bad}").status_code in (400, 404)
print("traversal:  rejected")

# --- video stitching
if have_ffmpeg:
  r = client.get(f"/ui/video/{dongle_id}/{TIME}.mp4")
  assert r.status_code == 200, r.data[:300]
  assert r.headers["Content-Type"] == "video/mp4"
  assert len(r.data) > 500 and r.data[4:8] == b"ftyp", "not an mp4"
  assert os.path.isfile(os.path.join(tmpdir, dongle_id, TIME, "route.mp4")), "not cached"
  # served from cache the second time, and seekable
  r = client.get(f"/ui/video/{dongle_id}/{TIME}.mp4", headers={"Range": "bytes=0-99"})
  assert r.status_code == 206, r.status_code
  print("video:      stitched, cached, and seekable")
else:
  print("video:      skipped (no ffmpeg on this host)")

# --- unknown device 404s rather than leaking a directory listing
assert client.get("/ui/d/0000000000000000").status_code == 404

# --- logout drops the session
assert client.post("/ui/logout").status_code == 302
assert client.get("/ui/").status_code == 302
print("auth:       logout clears the session")

# --- device API is untouched by the session cookie
assert client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": "x/y"}).status_code == 401
print("isolation:  browser session grants no device API access")

print("\nall web checks passed")

# --- the signing key must be identical across concurrent workers (gunicorn forks
# several, each calling get_secret() at import on a possibly-empty volume)
import concurrent.futures  # noqa: E402
import importlib  # noqa: E402
import pathlib  # noqa: E402

for _ in range(20):
  pathlib.Path(store.SECRET_PATH).unlink(missing_ok=True)
  with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex:
    keys = set(ex.map(lambda _: store.get_secret(), range(8)))
  assert len(keys) == 1, f"workers disagreed on the signing key: {len(keys)} distinct"
  assert keys.pop() == pathlib.Path(store.SECRET_PATH).read_bytes(), "memory != disk"
assert not list(pathlib.Path(store.DATA_DIR).glob(".secret.*")), "left temp files behind"
print("secret:     stable across concurrent workers")

# --- password file is read verbatim, dodging compose's $ expansion
pw_path = os.path.join(tmpdir, "web_password")
with open(pw_path, "w") as f:
  f.write("p$a s#w'ord\n")   # the exact characters compose would mangle
os.environ.pop("WEB_PASSWORD", None)
os.environ["WEB_PASSWORD_FILE"] = pw_path
importlib.reload(store)
assert store.WEB_PASSWORD == "p$a s#w'ord", repr(store.WEB_PASSWORD)
print("password:   file read verbatim, trailing newline stripped")
