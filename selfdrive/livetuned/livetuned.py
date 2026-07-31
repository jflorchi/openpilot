#!/usr/bin/env python3
"""Tiny web server for adjusting lateral tuning from a phone.

Connect the device to a phone hotspot, then browse to http://<device ip>:8086 from
any device on that hotspot. Stdlib only and idle-blocked on accept(), so it costs
nothing when nobody has the page open.
"""
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cereal.messaging as messaging
from common.live_tune import KNOBS, KNOBS_BY_KEY, log_snapshot, read_defaults, read_state, write_state
from selfdrive.swaglog import cloudlog

PORT = int(os.getenv("LIVE_TUNE_PORT", "8086"))
INDEX_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")

# stop polling cereal once nobody has looked at the page for this long
IDLE_TIMEOUT = 10.0


class Status:
  """Background poll of carState/controlsState, only while someone is watching."""

  def __init__(self):
    self.lock = threading.Lock()
    self.data = {"alive": False}
    self.last_request = 0.0

  def touch(self):
    self.last_request = time.monotonic()

  def get(self):
    self.touch()
    with self.lock:
      return dict(self.data)

  def run(self):
    sm = messaging.SubMaster(['carState', 'controlsState'])
    while True:
      if time.monotonic() - self.last_request > IDLE_TIMEOUT:
        with self.lock:
          self.data = {"alive": False}
        time.sleep(1.0)
        continue

      sm.update(200)
      data = {"alive": sm.alive['controlsState'] and sm.alive['carState']}
      if data["alive"]:
        cs = sm['carState']
        ctrl = sm['controlsState']
        data.update({
          "engaged": bool(ctrl.enabled),
          "active": bool(ctrl.active),
          "vEgoMph": round(cs.vEgo * 2.2369, 1),
          "steeringAngleDeg": round(cs.steeringAngleDeg, 1),
          "steeringPressed": bool(cs.steeringPressed),
        })
        lat = ctrl.lateralControlState
        if lat.which() == 'torqueState':
          t = lat.torqueState
          data.update({
            "output": round(t.output, 3),
            "desiredLatAccel": round(t.desiredLateralAccel, 2),
            "actualLatAccel": round(t.actualLateralAccel, 2),
            "latAccelError": round(t.desiredLateralAccel - t.actualLateralAccel, 2),
            "saturated": bool(t.saturated),
          })
      with self.lock:
        self.data = data
      time.sleep(0.2)


status = Status()
state_write_lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"
  server_version = "livetuned"

  def log_message(self, fmt, *args):
    pass  # the default logger writes to stderr on every request

  def _send(self, code, body, content_type):
    if isinstance(body, str):
      body = body.encode()
    self.send_response(code)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def _send_json(self, obj, code=200):
    self._send(code, json.dumps(obj), "application/json")

  def _read_json(self):
    try:
      length = int(self.headers.get("Content-Length", 0))
      return json.loads(self.rfile.read(length) or b"{}")
    except (TypeError, ValueError):
      return {}

  def _state_payload(self):
    state = read_state()
    return {
      "knobs": [k.as_dict() for k in KNOBS],
      "defaults": read_defaults(),
      "values": state["values"],
      "enabled": state["enabled"],
      "status": status.get(),
    }

  def do_GET(self):
    if self.path.split("?")[0] == "/api/state":
      self._send_json(self._state_payload())
    elif self.path in ("/", "/index.html"):
      try:
        with open(INDEX_PATH, "rb") as f:
          self._send(200, f.read(), "text/html; charset=utf-8")
      except OSError:
        self._send(500, "missing index.html", "text/plain")
    else:
      self._send(404, "not found", "text/plain")

  def do_POST(self):
    path = self.path.split("?")[0]
    body = self._read_json()

    # ThreadingHTTPServer can process requests from multiple clients concurrently.
    # Keep each read/modify/write transaction together so adjustments cannot overwrite
    # one another, and serialize snapshot appends for the same reason.
    with state_write_lock:
      state = read_state()

      if path == "/api/set":
        values = dict(state["values"])
        incoming = body.get("values", {})
        if "key" in body:
          incoming = {body["key"]: body.get("value")}
        for key, val in incoming.items():
          if key not in KNOBS_BY_KEY:
            continue
          if val is None:  # null clears the override, falling back to stock
            values.pop(key, None)
          else:
            values[key] = val
        # any adjustment implicitly arms the tuner, otherwise the slider does nothing
        write_state({"enabled": True, "values": values})
      elif path == "/api/enabled":
        write_state({"enabled": bool(body.get("enabled")), "values": state["values"]})
      elif path == "/api/reset":
        write_state({"enabled": False, "values": {}})
      elif path == "/api/snapshot":
        log_snapshot(str(body.get("note", ""))[:200], state["values"])
      else:
        self._send(404, "not found", "text/plain")
        return

      payload = self._state_payload()

    self._send_json(payload)


def local_ips():
  ips = []
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))  # doesn't send anything, just picks the default route
    ips.append(s.getsockname()[0])
    s.close()
  except OSError:
    pass
  try:
    for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
      ip = info[4][0]
      if not ip.startswith("127.") and ip not in ips:
        ips.append(ip)
  except OSError:
    pass
  return ips


def main():
  threading.Thread(target=status.run, daemon=True).start()

  server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
  server.daemon_threads = True

  urls = ", ".join("http://%s:%d" % (ip, PORT) for ip in local_ips()) or "no network"
  cloudlog.warning("livetuned listening on %s", urls)
  print("live tuner: %s" % urls)

  server.serve_forever()


if __name__ == "__main__":
  main()
