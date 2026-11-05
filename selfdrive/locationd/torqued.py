#!/usr/bin/env python3
"""Live torque parameter estimator, ported from upstream openpilot master (selfdrive/locationd/torqued.py).

The estimation logic is upstream's. Differences for this fork:
- inputs: liveLocationKalman instead of deviceMotion + extrinsicsCalibration (its
  angularVelocityCalibrated is already calibrated), and carControl.actuatorsOutput instead of carOutput
- publishes on this fork's liveTorqueParameters (liveValid rather than upstream's valid)
- GLEN's two strong-left buckets need fewer points, because the roads it drives rarely fill them
"""
from __future__ import annotations

import os
import numpy as np
from collections import deque, defaultdict

import cereal.messaging as messaging
from cereal import car, log
from common.params import Params
from common.realtime import config_realtime_process, DT_MDL
from common.filter_simple import FirstOrderFilter
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.locationd.helpers import PointBuckets, ParameterEstimator, capnp_reader

HISTORY = 5  # secs
POINTS_PER_BUCKET = 1500
MIN_POINTS_TOTAL = 4000
MIN_POINTS_TOTAL_QLOG = 600
FIT_POINTS_TOTAL = 2000
FIT_POINTS_TOTAL_QLOG = 600
MIN_VEL = 15  # m/s
FRICTION_FACTOR = 1.5  # ~85% of data coverage
FACTOR_SANITY = 0.3
FACTOR_SANITY_QLOG = 0.5
FRICTION_SANITY = 0.5
FRICTION_SANITY_QLOG = 0.8
STEER_MIN_THRESHOLD = 0.02
MIN_FILTER_DECAY = 50
MAX_FILTER_DECAY = 250
LAT_ACC_THRESHOLD = 1
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = np.array([100, 300, 500, 500, 500, 500, 300, 100])
MIN_ENGAGE_BUFFER = 2  # secs

# controlsd applies the filtered values (web tuner pins still win); until the buckets validate they stay at the
# car port's values
USE_PARAMS = True

# 36 min of commute filled the strong-left buckets to 5/100 and 104/300, and the old cache only had
# 43/100 after many drives, so stock thresholds would never validate on these roads.
GLEN_MIN_BUCKET_POINTS = np.array([10, 100, 500, 500, 500, 500, 300, 100])

VERSION = 2  # bump this to invalidate old parameter caches (2: upstream port, drops the pre-ZSS-fix points)
CACHE_EVERY_FRAMES = 240  # 60 s at 4 Hz


def slope2rot(slope):
  sin = np.sqrt(slope ** 2 / (slope ** 2 + 1))
  cos = np.sqrt(1 / (slope ** 2 + 1))
  return np.array([[cos, -sin], [sin, cos]])


def offline_torque_params(CP):
  if CP.lateralTuning.which() != 'torque':
    return 0.0, 0.0
  return CP.lateralTuning.torque.latAccelFactor, CP.lateralTuning.torque.friction


class TorqueBuckets(PointBuckets):
  def add_point(self, x, y):
    for bound_min, bound_max in self.x_bounds:
      if (x >= bound_min) and (x < bound_max):
        self.buckets[(bound_min, bound_max)].append([x, 1.0, y])
        break


class TorqueEstimator(ParameterEstimator):
  def __init__(self, CP, decimated=False, track_all_points=False):
    self.hist_len = int(HISTORY / DT_MDL)
    self.lag = 0.0
    self.track_all_points = track_all_points  # for offline analysis, without max lateral accel or max steer torque filters
    min_bucket_points = GLEN_MIN_BUCKET_POINTS if CP.carFingerprint == 'GLEN' else MIN_BUCKET_POINTS
    if decimated:
      self.min_bucket_points: list[float] = (min_bucket_points / 10).tolist()
      self.min_points_total = MIN_POINTS_TOTAL_QLOG
      self.fit_points = FIT_POINTS_TOTAL_QLOG
      self.factor_sanity = FACTOR_SANITY_QLOG
      self.friction_sanity = FRICTION_SANITY_QLOG

    else:
      self.min_bucket_points = min_bucket_points.tolist()
      self.min_points_total = MIN_POINTS_TOTAL
      self.fit_points = FIT_POINTS_TOTAL
      self.factor_sanity = FACTOR_SANITY
      self.friction_sanity = FRICTION_SANITY

    self.resets = 0.0
    self.use_params = USE_PARAMS and CP.lateralTuning.which() == 'torque'
    self.offline_latAccelFactor, self.offline_friction = offline_torque_params(CP)

    self.reset()

    initial_params = {
      'latAccelFactor': self.offline_latAccelFactor,
      'latAccelOffset': 0.0,
      'frictionCoefficient': self.offline_friction,
      'points': []
    }
    self.decay = MIN_FILTER_DECAY
    self.min_lataccel_factor = (1.0 - self.factor_sanity) * self.offline_latAccelFactor
    self.max_lataccel_factor = (1.0 + self.factor_sanity) * self.offline_latAccelFactor
    self.min_friction = (1.0 - self.friction_sanity) * self.offline_friction
    self.max_friction = (1.0 + self.friction_sanity) * self.offline_friction

    # try to restore cached params
    params = Params()
    params_cache = params.get("LiveTorqueCarParams")
    torque_cache = params.get("LiveTorqueParameters")
    if params_cache is not None and torque_cache is not None:
      try:
        with capnp_reader(log.Event, torque_cache) as log_evt:
          cache_ltp = log_evt.liveTorqueParameters
          with capnp_reader(car.CarParams, params_cache) as cache_CP:
            restore = self.get_restore_key(cache_CP, cache_ltp.version) == self.get_restore_key(CP, VERSION)
          if restore:
            if cache_ltp.liveValid:
              initial_params = {
                'latAccelFactor': cache_ltp.latAccelFactorFiltered,
                'latAccelOffset': cache_ltp.latAccelOffsetFiltered,
                'frictionCoefficient': cache_ltp.frictionCoefficientFiltered
              }
            cached_points = [list(point) for point in cache_ltp.points]
            initial_params['points'] = cached_points
            self.decay = cache_ltp.decay
            self.filtered_points.load_points(cached_points)
            cloudlog.info("restored torque params from cache")
      except Exception:
        cloudlog.exception("failed to restore cached torque params")
        params.remove("LiveTorqueCarParams")
        params.remove("LiveTorqueParameters")

    self.filtered_params = {}
    for param in initial_params:
      self.filtered_params[param] = FirstOrderFilter(initial_params[param], self.decay, DT_MDL)

  @staticmethod
  def get_restore_key(CP, version):
    # keyed on the seed values, so changing them (or the car's tune) starts learning over
    a, b = offline_torque_params(CP)
    return (CP.carFingerprint, CP.lateralTuning.which(), a, b, version)

  def reset(self):
    self.resets += 1.0
    self.decay = MIN_FILTER_DECAY
    self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
    self.filtered_points = TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                                         min_points=self.min_bucket_points,
                                         min_points_total=self.min_points_total,
                                         points_per_bucket=POINTS_PER_BUCKET,
                                         rowsize=3)
    self.all_torque_points = []

  def estimate_params(self):
    points = self.filtered_points.get_points(self.fit_points)
    # total least square solution as both x and y are noisy observations
    # this is empirically the slope of the hysteresis parallelogram as opposed to the line through the diagonals
    try:
      _, _, v = np.linalg.svd(points, full_matrices=False)
      slope, offset = -v.T[0:2, 2] / v.T[2, 2]
      _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
      friction_coeff = np.std(spread) * FRICTION_FACTOR
    except np.linalg.LinAlgError as e:
      cloudlog.exception(f"Error computing lateral torque parameters: {e}")
      slope = offset = friction_coeff = np.nan
    return slope, offset, friction_coeff

  def update_params(self, params):
    self.decay = min(self.decay + DT_MDL, MAX_FILTER_DECAY)
    for param, value in params.items():
      self.filtered_params[param].update(value)
      self.filtered_params[param].update_alpha(self.decay)

  def handle_log(self, t, which, msg):
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["lat_active"].append(msg.latActive)
      # upstream reads this from carOutput, which this fork doesn't have
      self.raw_points["carOutput_t"].append(t + self.lag)
      self.raw_points["steer_torque"].append(-msg.actuatorsOutput.steer)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      # TODO: check if high aEgo affects resulting lateral accel
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(msg.steeringPressed)
    elif which == "lateralDelay":
      self.lag = msg.lateralDelay
    # calculate lateral accel from past steering torque
    elif which == "liveLocationKalman":
      av = msg.angularVelocityCalibrated
      is_valid = av.valid and msg.orientationNED.valid and msg.inputsOK and msg.sensorsOK and msg.posenetOK
      if len(self.raw_points['steer_torque']) == self.hist_len and is_valid:
        yaw_rate = av.value[2]
        # device roll, not calibrated roll, as upstream does; latAccelOffset absorbs the mount misalignment
        roll = msg.orientationNED.value[0]
        # check lat active up to now (without lag compensation)
        lat_active = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t + self.lag, DT_MDL),
                               self.raw_points['carControl_t'], self.raw_points['lat_active']).astype(bool)
        steer_override = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t + self.lag, DT_MDL),
                                   self.raw_points['carState_t'], self.raw_points['steer_override']).astype(bool)
        vego = np.interp(t, self.raw_points['carState_t'], self.raw_points['vego'])
        steer = np.interp(t, self.raw_points['carOutput_t'], self.raw_points['steer_torque']).item()
        lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY).item()
        if all(lat_active) and not any(steer_override) and (vego > MIN_VEL) and (abs(steer) > STEER_MIN_THRESHOLD):
          if abs(lateral_acc) <= LAT_ACC_THRESHOLD:
            self.filtered_points.add_point(steer, lateral_acc)

          if self.track_all_points:
            self.all_torque_points.append([steer, lateral_acc])

  def get_msg(self, valid=True, with_points=False):
    msg = messaging.new_message('liveTorqueParameters')
    msg.valid = valid
    liveTorqueParameters = msg.liveTorqueParameters
    liveTorqueParameters.version = VERSION
    liveTorqueParameters.useParams = self.use_params

    # Calculate raw estimates when possible, only update filters when enough points are gathered
    if self.filtered_points.is_calculable():
      latAccelFactor, latAccelOffset, frictionCoeff = self.estimate_params()
      liveTorqueParameters.latAccelFactorRaw = float(latAccelFactor)
      liveTorqueParameters.latAccelOffsetRaw = float(latAccelOffset)
      liveTorqueParameters.frictionCoefficientRaw = float(frictionCoeff)

      if self.filtered_points.is_valid():
        if any(val is None or np.isnan(val) for val in [latAccelFactor, latAccelOffset, frictionCoeff]):
          cloudlog.exception("Lateral torque parameters are invalid.")
          liveTorqueParameters.liveValid = False
          self.reset()
        else:
          liveTorqueParameters.liveValid = True
          latAccelFactor = np.clip(latAccelFactor, self.min_lataccel_factor, self.max_lataccel_factor)
          frictionCoeff = np.clip(frictionCoeff, self.min_friction, self.max_friction)
          self.update_params({'latAccelFactor': latAccelFactor, 'latAccelOffset': latAccelOffset, 'frictionCoefficient': frictionCoeff})

    if with_points:
      liveTorqueParameters.points = self.filtered_points.get_points()[:, [0, 2]].tolist()

    liveTorqueParameters.latAccelFactorFiltered = float(self.filtered_params['latAccelFactor'].x)
    liveTorqueParameters.latAccelOffsetFiltered = float(self.filtered_params['latAccelOffset'].x)
    liveTorqueParameters.frictionCoefficientFiltered = float(self.filtered_params['frictionCoefficient'].x)
    liveTorqueParameters.totalBucketPoints = len(self.filtered_points)
    liveTorqueParameters.calPerc = self.filtered_points.get_valid_percent()
    liveTorqueParameters.decay = self.decay
    liveTorqueParameters.maxResets = self.resets
    return msg


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  DEBUG = bool(int(os.getenv("DEBUG", "0")))

  pm = messaging.PubMaster(['liveTorqueParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'liveLocationKalman', 'lateralDelay'], poll=['liveLocationKalman'])

  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  estimator = TorqueEstimator(CP)
  params.put("LiveTorqueCarParams", CP.as_builder().to_bytes())

  inputs = ['carControl', 'carState', 'liveLocationKalman']
  while True:
    sm.update()
    if sm.all_checks(inputs):
      for which in sm.updated.keys():
        # lagd is optional: without it the lag stays at its last value
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          estimator.handle_log(t, which, sm[which])

    # 4Hz driven by liveLocationKalman
    if sm.frame % 5 == 0:
      pm.send('liveTorqueParameters', estimator.get_msg(valid=sm.all_checks(inputs), with_points=DEBUG))

    # Cache points every 60 seconds while onroad
    if sm.frame % CACHE_EVERY_FRAMES == 0:
      msg = estimator.get_msg(valid=sm.all_checks(inputs), with_points=True)
      params.put("LiveTorqueParameters", msg.to_bytes())


if __name__ == "__main__":
  main()
