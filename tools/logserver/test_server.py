#!/usr/bin/env python3
"""End-to-end check: register a fake device, then walk the exact call sequence
selfdrive/loggerd/uploader.py makes, using the real JWT signing path."""

import json
import os
import tempfile
from datetime import datetime, timedelta

import jwt
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

tmpdir = tempfile.mkdtemp()
os.environ["DATA_DIR"] = tmpdir
os.environ["BASE_URL"] = "http://testserver"

import server  # noqa: E402
import store  # noqa: E402

client = server.app.test_client()

# a device keypair, as /persist/comma/id_rsa
key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
private_key = key.private_bytes(
  serialization.Encoding.PEM,
  serialization.PrivateFormat.TraditionalOpenSSL,
  serialization.NoEncryption(),
).decode()
public_key = key.public_key().public_bytes(
  serialization.Encoding.PEM,
  serialization.PublicFormat.SubjectPublicKeyInfo,
).decode()

# --- registration (selfdrive/athena/registration.py)
register_token = jwt.encode(
  {"register": True, "exp": datetime.utcnow() + timedelta(hours=1)}, private_key, algorithm="RS256")
r = client.post("/v2/pilotauth/", query_string={
  "imei": "1", "imei2": "2", "serial": "abc123",
  "public_key": public_key, "register_token": register_token,
})
assert r.status_code == 200, r.data
dongle_id = json.loads(r.data)["dongle_id"]
print(f"registered: {dongle_id}")

# re-registering is idempotent
r2 = client.post("/v2/pilotauth/", query_string={
  "imei": "1", "imei2": "2", "serial": "abc123",
  "public_key": public_key, "register_token": register_token,
})
assert json.loads(r2.data)["dongle_id"] == dongle_id, "dongle id not stable"

# --- upload (selfdrive/loggerd/uploader.py)
def device_token():
  now = datetime.utcnow()
  return jwt.encode({"identity": dongle_id, "nbf": now, "iat": now,
                     "exp": now + timedelta(hours=1)}, private_key, algorithm="RS256")

auth = {"Authorization": "JWT " + device_token()}
key_path = "2024-01-01--12-00-00--3/qlog.bz2"

r = client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": key_path}, headers=auth)
assert r.status_code == 200, r.data
url = json.loads(r.data)["url"].replace("http://testserver", "")

payload = b"fake qlog contents" * 1000
r = client.put(url, data=payload)
assert r.status_code == 201, r.data
on_disk = os.path.join(tmpdir, dongle_id, "2024-01-01--12-00-00", "3", "qlog.bz2")
assert open(on_disk, "rb").read() == payload
print(f"uploaded:   {on_disk}")

# second request for the same file gets 412 -> uploader marks it done locally
r = client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": key_path}, headers=auth)
assert r.status_code == 412, r.status_code
print("dedupe:     412 as expected")

# --- bootlogs use a flat "boot/<name>" key, not a segment dir
boot_key = "boot/2024-01-01--12-00-00.bz2"
r = client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": boot_key}, headers=auth)
assert r.status_code == 200, r.data
boot_url = json.loads(r.data)["url"].replace("http://testserver", "")
assert client.put(boot_url, data=b"boot log").status_code == 201
assert os.path.isfile(os.path.join(tmpdir, dongle_id, "boot", "2024-01-01--12-00-00.bz2"))

r = client.get(f"/v1/devices/{dongle_id}/bootlogs")
listed = json.loads(r.data)
assert len(listed) == 1, listed
r = client.get(listed[0]["url"].replace("http://testserver", ""))
assert r.status_code == 200 and r.data == b"boot log"
print("bootlogs:   uploaded, listed, downloaded")

# boot/crash must not leak into the route listing
r = client.get(f"/v1/devices/{dongle_id}/routes_segments")
assert all("boot" not in x["fullname"] for x in json.loads(r.data)), r.data

# --- auth rejections
assert client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": key_path}).status_code == 401
other = rsa.generate_private_key(public_exponent=65537, key_size=2048).private_bytes(
  serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
  serialization.NoEncryption()).decode()
forged = jwt.encode({"identity": dongle_id, "exp": datetime.utcnow() + timedelta(hours=1)},
                    other, algorithm="RS256")
assert client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": key_path},
                  headers={"Authorization": "JWT " + forged}).status_code == 401
print("auth:       unsigned and forged tokens rejected")

# --- path traversal
for bad in ["../../etc/passwd", "2024-01-01--12-00-00--3/../../x", "/etc/passwd"]:
  r = client.get(f"/v1.4/{dongle_id}/upload_url/", query_string={"path": bad}, headers=auth)
  assert r.status_code == 400, f"{bad} -> {r.status_code}"
print("traversal:  rejected")

# --- tampered upload URL
tampered = url.replace("qlog.bz2", "rlog.bz2")
assert client.put(tampered, data=b"x").status_code == 403
print("signature:  tampered url rejected")

# --- route listing + download (tools/lib/route.py)
r = client.get(f"/v1/route/{dongle_id}|2024-01-01--12-00-00/files")
files = json.loads(r.data)
assert len(files["qlogs"]) == 1, files
dl = files["qlogs"][0].replace("http://testserver", "")
r = client.get(dl)
assert r.status_code == 200 and r.data == payload
print("route/files: listed and downloaded")

# --- UI stubs respond
for path in [f"/v1.1/devices/{dongle_id}/", f"/v1/devices/{dongle_id}/owner",
             f"/v1.1/devices/{dongle_id}/stats", f"/v1/devices/{dongle_id}/bootlogs", "/health"]:
  assert client.get(path).status_code == 200, path
print("ui stubs:   ok")


# --- REGISTRATION_TOKEN gating (matches selfdrive/athena/registration.py)
store.REGISTRATION_TOKEN = "s3cret"
base = {"imei": "1", "imei2": "2", "serial": "gated-device",
        "public_key": public_key, "register_token": register_token}

# no token -> 403, which registration.py turns into UNREGISTERED_DONGLE_ID
assert client.post("/v2/pilotauth/", query_string=base).status_code == 403
# wrong token -> 403
assert client.post("/v2/pilotauth/", query_string={**base, "registration_token": "nope"}).status_code == 403
# correct token -> registers
r = client.post("/v2/pilotauth/", query_string={**base, "registration_token": "s3cret"})
assert r.status_code == 200, r.data
print("reg token:  gates registration correctly")
store.REGISTRATION_TOKEN = None

print("\nall checks passed")
