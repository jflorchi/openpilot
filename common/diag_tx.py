#!/usr/bin/env python3
"""Hands one-off diagnostic CAN frames to controlsd so it can send them.

Only one process may publish 'sendcan' (a second msgq publisher replaces controlsd's and
steering stops), so anything else that needs to put a frame on the bus writes it here and
controlsd appends it to its next send. Same file approach as common/live_tune.py.

Requests are single-shot: controlsd deletes the file once it has sent the frames, and drops
anything older than MAX_AGE so a request made offroad can't fire on the next drive.
"""
import json
import os
import time

from common.live_tune import LIVE_TUNE_DIR, _atomic_write

DIAG_TX_PATH = os.path.join(LIVE_TUNE_DIR, "diag_tx.json")
MAX_AGE = 1.0  # s


def request(frames):
  """frames: list of (address, bytes, bus)."""
  _atomic_write(DIAG_TX_PATH, json.dumps({
    "t": time.time(),
    "frames": [[int(addr), bytes(dat).hex(), int(bus)] for addr, dat, bus in frames],
  }))


class DiagTxReader:
  """Polled from controlsd at 100Hz; one stat() per poll_dt while idle."""

  def __init__(self, poll_dt=0.05):
    self.poll_dt = poll_dt
    self.last_poll = 0.0

  def update(self, now):
    if now - self.last_poll < self.poll_dt:
      return []
    self.last_poll = now

    if not os.path.exists(DIAG_TX_PATH):
      return []
    try:
      with open(DIAG_TX_PATH) as f:
        req = json.load(f)
      os.unlink(DIAG_TX_PATH)
    except (OSError, ValueError):
      return []

    try:
      if abs(time.time() - float(req["t"])) > MAX_AGE:
        return []
      return [(int(addr), 0, bytes.fromhex(dat), int(bus)) for addr, dat, bus in req["frames"]]
    except (KeyError, TypeError, ValueError):
      return []
