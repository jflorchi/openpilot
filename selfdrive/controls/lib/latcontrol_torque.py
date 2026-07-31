import math

from cereal import log
from common.numpy_fast import interp
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_FACTOR_LOW = 500.
LOW_SPEED_FACTOR_HIGH = 200.


def set_torque_tune(tune, MAX_LAT_ACCEL=2.5, FRICTION=.1, steering_angle_deadzone_deg=0.0):
  tune.init('torque')
  tune.torque.useSteeringAngle = True
  tune.torque.kp = 1.0
  tune.torque.kf = 1.0
  tune.torque.ki = 0.1
  tune.torque.latAccelFactor = MAX_LAT_ACCEL
  tune.torque.latAccelOffset = 0.0
  tune.torque.friction = FRICTION
  tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # baselines the live tuner falls back to when a knob is cleared
    self.base_gains = (self.torque_params.kp, self.torque_params.ki, self.torque_params.kf)
    self.base_lat_accel_factor = self.torque_params.latAccelFactor
    self.base_friction = self.torque_params.friction
    self.base_low_speed_factor = (LOW_SPEED_FACTOR_LOW, LOW_SPEED_FACTOR_HIGH)
    self.low_speed_factor = self.base_low_speed_factor

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # The latest valid torqued values become the fallback for cleared web overrides.
    self.base_lat_accel_factor = latAccelFactor
    self.base_friction = friction
    self.torque_params.latAccelOffset = latAccelOffset

  def update_live_tune(self, live_tune):
    """Apply web tuner overrides. Runs after torqued so a pinned value wins."""
    self.torque_params.latAccelFactor = live_tune.get('latAccelFactor', self.base_lat_accel_factor)
    self.torque_params.friction = live_tune.get('friction', self.base_friction)

    self.low_speed_factor = (live_tune.get('lowSpeedFactorLow', self.base_low_speed_factor[0]),
                             live_tune.get('lowSpeedFactorHigh', self.base_low_speed_factor[1]))

    # set gains in place rather than rebuilding the PID, which would dump the integrator
    kp = live_tune.get('kp', self.base_gains[0])
    ki = live_tune.get('ki', self.base_gains[1])
    kf = live_tune.get('kf', self.base_gains[2])
    self.pid._k_p = [[0], [kp]]
    self.pid._k_i = [[0], [ki]]
    self.pid.k_f = kf

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, [0, 10, 20], [self.low_speed_factor[0],
                                                       self.low_speed_factor[0],
                                                       self.low_speed_factor[1]])
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      pid_log.error = self.torque_from_lateral_accel(error, self.torque_params, error, lateral_accel_deadzone, friction_compensation=False)
      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params, error, lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation_2(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
