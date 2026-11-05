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
from common import diag_tx
from common.realtime import sec_since_boot
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
    # Plain conflated sockets, not SubMaster: with SubMaster's poller on this msgq build only
    # controlsState ever woke the poll, carState never updated and the page always said offroad.
    socks = {s: messaging.sub_sock(s, conflate=True, timeout=100) for s in ('carState', 'controlsState')}
    latest = {}
    while True:
      if time.monotonic() - self.last_request > IDLE_TIMEOUT:
        with self.lock:
          self.data = {"alive": False}
        time.sleep(1.0)
        continue

      for name, sock in socks.items():
        msg = messaging.recv_one(sock)
        if msg is not None:
          latest[name] = msg
      now = sec_since_boot()
      fresh = all(n in latest and now - latest[n].logMonoTime * 1e-9 < 0.5 for n in socks)
      data = {"alive": fresh}
      if fresh:
        cs = latest['carState'].carState
        ctrl = latest['controlsState'].controlsState
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


# OBD-II on the powertrain bus, see selfdrive/car/glen/OBD_NOTES.md
OBD_REQ = 0x7DF   # functional request
OBD_RESP = 0x7E8  # ECM, the only responder on this car
OBD_BUS = 0
OBD_TIMEOUT = 1.5  # s, includes the hop through controlsd


def decode_dtc(b1, b2):
  return "%s%d%X%X%X" % ("PCBU"[b1 >> 6], (b1 >> 4) & 0x3, b1 & 0xF, b2 >> 4, b2 & 0xF)


class ObdError(Exception):
  pass


class Obd:
  """Single-frame OBD requests. TX goes through controlsd (common/diag_tx.py), RX is read
  straight off 'can'. Sockets are opened once and kept, msgq has a fixed number of reader slots."""

  def __init__(self):
    self.lock = threading.Lock()
    self.can_sock = None
    self.cs_sock = None

  def _connect(self):
    if self.can_sock is None:
      self.can_sock = messaging.sub_sock('can', timeout=100)
      self.cs_sock = messaging.sub_sock('carState', conflate=True, timeout=100)

  def car_state(self):
    """Latest carState, or None if controlsd isn't publishing (offroad: nothing would get sent)."""
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
      msg = messaging.recv_one(self.cs_sock)
      if msg is not None and sec_since_boot() - msg.logMonoTime * 1e-9 < 0.5:
        return msg.carState
    return None

  def query(self, payload, mode, pid=None):
    """Send one single-frame request, return the ECM's response payload (after the PCI byte).
    Raises ObdError on a negative response or a timeout. Multi-frame responses are returned
    as just the first frame's data, prefixed with None so callers can tell."""
    messaging.drain_sock_raw(self.can_sock)
    diag_tx.request([(OBD_REQ, bytes([len(payload)] + payload).ljust(8, b'\0'), OBD_BUS)])

    end = time.monotonic() + OBD_TIMEOUT
    while time.monotonic() < end:
      for msg in messaging.drain_sock(self.can_sock, wait_for_one=True):
        for c in msg.can:
          if c.address != OBD_RESP or c.src != OBD_BUS or len(c.dat) < 3:
            continue
          dat = bytes(c.dat)
          pci = dat[0] >> 4
          if pci == 0:
            data = dat[1:1 + (dat[0] & 0xF)]
          elif pci == 1:
            data = dat[2:]
          else:
            continue
          if data[0] == 0x7F and data[1] == mode:
            raise ObdError("mode %02X refused, code %02X" % (mode, data[2]))
          if data[0] != mode + 0x40 or (pid is not None and data[1] != pid):
            continue
          return data if pci == 0 else (None, data)
    raise ObdError("no response to mode %02X" % mode)

  def status(self):
    """MIL and stored DTC count from mode 01 PID 01, plus the codes if they fit in one frame."""
    d = self.query([0x01, 0x01], 0x01, 0x01)
    out = {"mil": bool(d[2] & 0x80), "count": d[2] & 0x7F, "codes": None}
    try:
      r = self.query([0x03], 0x03)
      if not isinstance(r, tuple):  # single frame: 43 <n> <codes...>
        raw = r[2:]
        out["codes"] = [decode_dtc(raw[i], raw[i + 1]) for i in range(0, len(raw) - 1, 2)
                        if raw[i] or raw[i + 1]]
    except ObdError:
      pass  # the count above is still good, just not the names
    return out

  def clear(self):
    if not self.lock.acquire(blocking=False):
      return {"ok": False, "msg": "already running"}
    try:
      self._connect()

      cs = self.car_state()
      if cs is None:
        return {"ok": False, "msg": "openpilot isn't onroad, turn the key on"}

      # allowed while moving on purpose: clearing the P0420 from the device beats doing it with
      # a scan tool at speed. If the ECM objects it answers 7F 04 22 and query() raises.
      before = self.status()
      self.query([0x04], 0x04)
      time.sleep(0.2)
      after = self.status()
      cloudlog.warning("livetuned: cleared DTCs at %.1f m/s, before %s after %s", cs.vEgo, before, after)
      return {"ok": True, "msg": "cleared", "before": before, "after": after}
    except ObdError as e:
      return {"ok": False, "msg": str(e)}
    finally:
      self.lock.release()


obd = Obd()


# china-sas steering angle board (~/Developer/china-sas/firmware, selfdrive/car/glen/SAS_NOTES.md):
# service commands on 0x6F0, replies on 0x6F1, bus 0. CMD_SET_ZERO makes the board average 300 ms
# of a still wheel and store that as centre (flash), then ack with mux 0x7F.
SAS_CMD_ID = 0x6F0
SAS_DIAG_ID = 0x6F1
SAS_BUS = 0
SAS_CMD_SET_ZERO = 0x01
SAS_ACK_TIMEOUT = 2.0  # s: hop through controlsd + 300 ms capture + flash write
SAS_ZERO_RESULT = {
  0x03: "the wheel moved during the 0.3 s capture, hold it still and retry",
  0x04: "a magnet sensor is faulted, not zeroed",
  0x05: "flash write failed, not zeroed",
  0x06: "a zero is already in progress",
}


def toyota_checksum(addr, dat):
  s = len(dat)
  while addr:
    s += addr & 0xFF
    addr >>= 8
  return (s + sum(dat[:-1])) & 0xFF


class SasZero:
  """Steering angle zero on the china-sas board. Shares Obd's sockets and lock (msgq reader slots
  are limited and both drain 'can')."""

  def run(self):
    if not obd.lock.acquire(blocking=False):
      return {"ok": False, "msg": "busy, try again"}
    try:
      obd._connect()
      cs = obd.car_state()
      if cs is None:
        return {"ok": False, "msg": "openpilot isn't onroad, turn the key on"}
      st = status.get()
      if st.get("engaged") or st.get("active"):
        return {"ok": False, "msg": "openpilot is engaged, disengage first (the angle jumps to 0)"}
      before = round(cs.steeringAngleDeg, 1)

      d = bytearray([0xA5, SAS_CMD_SET_ZERO, 0, 0, 0, 0, 0, 0])
      d[7] = toyota_checksum(SAS_CMD_ID, d)
      messaging.drain_sock_raw(obd.can_sock)
      diag_tx.request([(SAS_CMD_ID, bytes(d), SAS_BUS)])

      end = time.monotonic() + SAS_ACK_TIMEOUT
      result = None
      while result is None and time.monotonic() < end:
        for msg in messaging.drain_sock(obd.can_sock, wait_for_one=True):
          for c in msg.can:
            dat = bytes(c.dat)
            if c.address == SAS_DIAG_ID and c.src == SAS_BUS and len(dat) == 8 and \
               dat[0] == 0x7F and dat[1] == SAS_CMD_SET_ZERO:
              result = dat[2]
      if result is None:
        return {"ok": False, "msg": "no reply from the steering angle board"}
      if result != 0:
        return {"ok": False, "msg": SAS_ZERO_RESULT.get(result, "refused, code %d" % result)}

      time.sleep(0.3)  # let a few 0x23 frames through
      cs = obd.car_state()
      after = round(cs.steeringAngleDeg, 1) if cs is not None else None
      cloudlog.warning("livetuned: SAS zeroed at %.1f m/s, angle %s -> %s", cs.vEgo if cs else -1, before, after)
      return {"ok": True, "msg": "zeroed", "before": before, "after": after}
    finally:
      obd.lock.release()


sas_zero = SasZero()


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

    if path == "/api/clear_dtc":
      self._send_json(obd.clear())
      return
    if path == "/api/sas_zero":
      self._send_json(sas_zero.run())
      return

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
