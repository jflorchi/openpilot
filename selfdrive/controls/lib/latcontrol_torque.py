import math
from collections import deque

import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_CTRL
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# Ported from upstream openpilot master (selfdrive/controls/lib/latcontrol_torque.py).
#
# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.
#
# Differences from upstream for this fork: kp/ki/kf/latAccelFactor/friction can be pinned from the web
# tuner (common/live_tune.py), and kp scales upstream's whole speed table so it is the gain at 30+ m/s.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
FRICTION_THRESHOLD = 0.2  # opendbc car/lateral.py


# Scale on the low-speed end of the P table, blended back to 1 between these speeds. Upstream's table
# plus the low learned latAccelFactor asks for more torque than the steer rate limit can deliver at
# 6-15 m/s, so a bump or a turn exit starts a ~1.5 Hz bang-bang cycle with the motor a quarter cycle late.
LOW_SPEED_KP_SCALE = 0.5
LOW_SPEED_KP_BLEND = [10., 17.]


def kp_table(kp, low_speed_scale=LOW_SPEED_KP_SCALE):
  """Upstream's speed-dependent P gain, scaled so kp is the gain at 30+ m/s and low_speed_scale is the
  extra scale at <= 10 m/s."""
  speeds = sorted(set(INTERP_SPEEDS + LOW_SPEED_KP_BLEND))
  base = np.interp(speeds, INTERP_SPEEDS, KP_INTERP)
  scale = np.interp(speeds, LOW_SPEED_KP_BLEND, [low_speed_scale, 1.])
  return [speeds, [float(k * s * kp / KP) for k, s in zip(base, scale)]]


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def get_friction(lateral_accel_error, lateral_accel_deadzone, friction_threshold, torque_params):
  # friction is in torque units, so it is scaled into lat accel space here (as upstream)
  friction_interp = np.interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction * torque_params.latAccelFactor, torque_params.friction * torque_params.latAccelFactor]
  )
  return float(friction_interp)


def set_torque_tune(tune, MAX_LAT_ACCEL=2.5, FRICTION=.1, steering_angle_deadzone_deg=0.0):
  tune.init('torque')
  tune.torque.useSteeringAngle = True
  tune.torque.kp = KP
  tune.torque.kf = 1.0
  tune.torque.ki = KI
  tune.torque.latAccelFactor = MAX_LAT_ACCEL
  tune.torque.latAccelOffset = 0.0
  tune.torque.friction = FRICTION
  tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.dt = DT_CTRL
    self.torque_params = CP.lateralTuning.torque
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.pid = PIDController(kp_table(self.torque_params.kp), self.torque_params.ki, k_f=self.torque_params.kf,
                             rate=1 / self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    # baselines the live tuner falls back to when a knob is cleared
    self.base_gains = (self.torque_params.kp, self.torque_params.ki, self.torque_params.kf)
    self.base_lat_accel_factor = self.torque_params.latAccelFactor
    self.base_friction = self.torque_params.friction

  def torque_to_lat_accel(self, lateral_accel):
    # the car port's linear map, without its own friction term (friction is added in lat accel space below)
    return self.torque_from_lateral_accel(lateral_accel, self.torque_params, 0.0, 0.0, friction_compensation=False)

  def update_limits(self):
    # upstream: lateral_accel_from_torque(+-steer_max); the car port's map is linear
    self.pid.pos_limit = self.steer_max * self.torque_params.latAccelFactor
    self.pid.neg_limit = -self.steer_max * self.torque_params.latAccelFactor

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # The latest torqued values become the fallback for cleared web overrides.
    self.base_lat_accel_factor = latAccelFactor
    self.base_friction = friction
    self.torque_params.latAccelOffset = latAccelOffset

  def update_live_tune(self, live_tune):
    """Apply web tuner overrides. Runs after torqued so a pinned value wins."""
    self.torque_params.latAccelFactor = live_tune.get('latAccelFactor', self.base_lat_accel_factor)
    self.torque_params.friction = live_tune.get('friction', self.base_friction)
    self.update_limits()

    # set gains in place rather than rebuilding the PID, which would dump the integrator
    kp = live_tune.get('kp', self.base_gains[0])
    ki = live_tune.get('ki', self.base_gains[1])
    kf = live_tune.get('kf', self.base_gains[2])
    self.pid._k_p = kp_table(kp, live_tune.get('kpLowSpeedScale', LOW_SPEED_KP_SCALE))
    self.pid._k_i = [[0], [ki]]
    self.pid.k_f = kf

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk,
             lat_delay=0.0, curvature_limited=False):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    # the desired curvature is where the car should be after lat_delay: compare the measurement against what
    # was requested lat_delay ago, not against the request that hasn't had time to act yet
    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len + 1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx + 1] - self.lat_accel_request_buffer[lookahead_idx - 1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_to_lat_accel(output_lataccel)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = self._check_saturation_2(self.steer_max - abs(output_torque) < 1e-3 or curvature_limited,
                                                   CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
