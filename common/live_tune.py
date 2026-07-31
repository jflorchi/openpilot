#!/usr/bin/env python3
"""Shared state for the live lateral tuner.

The web server (selfdrive/livetuned) writes a small JSON file, controlsd reads it.
A file is used instead of Params so nothing has to be rebuilt to add a knob, and so
the state survives a controlsd restart mid-drive.

Only keys the user has actually touched are stored. Anything absent falls back to
the value the car port (or torqued) would have used, so an empty file == stock.
"""
import json
import os
import time

from common.numpy_fast import clip

LIVE_TUNE_DIR = os.getenv("LIVE_TUNE_DIR", "/data")
LIVE_TUNE_PATH = os.path.join(LIVE_TUNE_DIR, "live_tune.json")
LIVE_TUNE_DEFAULTS_PATH = os.path.join(LIVE_TUNE_DIR, "live_tune_defaults.json")
LIVE_TUNE_LOG_PATH = os.path.join(LIVE_TUNE_DIR, "live_tune_log.jsonl")


class Knob:
  def __init__(self, key, label, lo, hi, step, group, default, unit="", desc=""):
    self.key = key
    self.label = label
    self.lo = lo
    self.hi = hi
    self.step = step
    self.group = group
    self.default = default
    self.unit = unit
    self.desc = desc

  def clamp(self, v):
    return float(clip(float(v), self.lo, self.hi))

  def as_dict(self):
    return {"key": self.key, "label": self.label, "min": self.lo, "max": self.hi,
            "step": self.step, "group": self.group, "default": self.default,
            "unit": self.unit, "desc": self.desc}


# The ranges here are the safety limits: nothing outside them can reach the controller,
# no matter what gets POSTed or hand-edited into the JSON.
#
# The defaults mirror what the car port actually sets, so the UI shows the right "stock"
# value before controlsd has ever run. controlsd overwrites them from CarParams at startup
# (write_defaults below), which is authoritative -- these are just the offroad fallback.
#   latAccelFactor / friction / steerActuatorDelay: selfdrive/car/glen/interface.py
#   kp / ki / kf:                                   set_torque_tune, latcontrol_torque.py
#   lowSpeedFactor*:                                LOW_SPEED_FACTOR_*, latcontrol_torque.py
KNOBS = [
  Knob("latAccelFactor", "Lat Accel Factor", 1.0, 5.0, 0.05, "torque", 3.6,
       desc="Lateral accel the car makes at full torque. Lower = more steer for the same request."),
  Knob("friction", "Friction", 0.0, 0.20, 0.002, "torque", 0.012,
       desc="Torque added in the direction of the error. Behaves like extra P around center."),
  Knob("kp", "P gain", 0.0, 3.0, 0.05, "pid", 1.0,
       desc="Proportional gain on lat accel error."),
  Knob("ki", "I gain", 0.0, 1.0, 0.01, "pid", 0.1,
       desc="Integral gain. Too high wanders, too low holds an offset in long curves."),
  Knob("kf", "FF gain", 0.0, 2.0, 0.05, "pid", 1.0,
       desc="Feedforward scale. This is what does most of the steering."),
  Knob("steerActuatorDelay", "Actuator Delay", 0.0, 0.60, 0.01, "plan", 0.30, unit="s",
       desc="Lookahead. Sets the planner horizon and divides into psi, so it acts as a gain too."),
  Knob("lowSpeedFactorLow", "Low Speed Factor @ 0-10 m/s", 0.0, 800.0, 10.0, "plan", 500.0,
       desc="Curvature weighting in the error at low speed."),
  Knob("lowSpeedFactorHigh", "Low Speed Factor @ 20 m/s", 0.0, 800.0, 10.0, "plan", 200.0,
       desc="Curvature weighting in the error at highway speed."),
]

KNOBS_BY_KEY = {k.key: k for k in KNOBS}


def sanitize(values):
  """Drop unknown keys and clamp everything else to its safe range."""
  out = {}
  for key, val in (values or {}).items():
    knob = KNOBS_BY_KEY.get(key)
    if knob is None:
      continue
    try:
      out[key] = knob.clamp(val)
    except (TypeError, ValueError):
      continue
  return out


def _atomic_write(path, text):
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    f.write(text)
    f.flush()
    os.fsync(f.fileno())
  os.rename(tmp, path)


def write_state(state):
  state = {"enabled": bool(state.get("enabled", False)),
           "values": sanitize(state.get("values")),
           "t": time.time()}
  _atomic_write(LIVE_TUNE_PATH, json.dumps(state))
  return state


def read_state():
  try:
    with open(LIVE_TUNE_PATH) as f:
      state = json.load(f)
    return {"enabled": bool(state.get("enabled", False)),
            "values": sanitize(state.get("values")),
            "t": float(state.get("t", 0.0))}
  except (OSError, ValueError, AttributeError):
    return {"enabled": False, "values": {}, "t": 0.0}


def write_defaults(defaults):
  """Called by controlsd once it knows CarParams, so the UI can show and restore stock."""
  try:
    _atomic_write(LIVE_TUNE_DEFAULTS_PATH, json.dumps(sanitize(defaults)))
  except OSError:
    pass


def read_defaults():
  """Stock values for the UI. Declared defaults, overlaid with whatever controlsd
  published from the live CarParams, so every knob always has a stock value to show."""
  defaults = {k.key: k.default for k in KNOBS}
  try:
    with open(LIVE_TUNE_DEFAULTS_PATH) as f:
      defaults.update(sanitize(json.load(f)))
  except (OSError, ValueError, AttributeError):
    pass
  return defaults


def log_snapshot(note, values):
  """Append a timestamped marker so a tuning session can be lined up with the drive later."""
  line = json.dumps({"t": time.time(), "note": note, "values": sanitize(values)})
  try:
    with open(LIVE_TUNE_LOG_PATH, "a") as f:
      f.write(line + "\n")
  except OSError:
    pass


class LiveTuneReader:
  """Polls the tune file at a low rate. Safe to call at 100Hz from controlsd."""

  def __init__(self, poll_dt=0.2):
    self.poll_dt = poll_dt
    self.last_poll = 0.0
    self.last_mtime = None
    self.values = {}

  def update(self, now):
    if now - self.last_poll < self.poll_dt:
      return self.values
    self.last_poll = now

    try:
      mtime = os.path.getmtime(LIVE_TUNE_PATH)
    except OSError:
      self.last_mtime = None
      self.values = {}
      return self.values

    if mtime != self.last_mtime:
      self.last_mtime = mtime
      state = read_state()
      self.values = state["values"] if state["enabled"] else {}
    return self.values

  def get(self):
    return self.values
