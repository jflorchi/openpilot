import math
import numpy as np
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL, DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.modeld.constants import T_IDXS

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
V_CRUISE_MAX = 145  # kph
V_CRUISE_MIN = 8  # kph
V_CRUISE_ENABLE_MIN = 40  # kph

LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  car.CarState.ButtonEvent.Type.accelCruise: math.ceil,
  car.CarState.ButtonEvent.Type.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  car.CarState.ButtonEvent.Type.accelCruise: +1,
  car.CarState.ButtonEvent.Type.decelCruise: -1,
}


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  if not enabled:
    return v_cruise_kph

  long_press = False
  button_type = None

  # should be CV.MPH_TO_KPH, but this causes rounding errors
  v_cruise_delta = 1. if metric else 1.6

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph # end long press
      button_type = b.type.raw
      break
  else:
    for k in button_timers.keys():
      if button_timers[k] and button_timers[k] % CRUISE_LONG_PRESS == 0:
        button_type = k
        long_press = True
        break

  if button_type:
    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and v_cruise_kph % v_cruise_delta != 0: # partial interval
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]
    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type in (car.CarState.ButtonEvent.Type.accelCruise, car.CarState.ButtonEvent.Type.resumeCruise) and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))



def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates, steer_actuator_delay=None):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N

  if steer_actuator_delay is None:
    steer_actuator_delay = CP.steerActuatorDelay

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = steer_actuator_delay + .2

  current_curvature = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  desired_curvature_rate = curvature_rates[0]

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  curvature_diff_from_psi = psi / (max(v_ego, 1e-1) * delay) - current_curvature
  desired_curvature = current_curvature + 2 * curvature_diff_from_psi

  v_ego = max(v_ego, 0.1)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2)
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                          -max_curvature_rate,
                                          max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                     current_curvature - max_curvature_rate * DT_MDL,
                                     current_curvature + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


# Desired curvature from the model's plan, and its limits, ported from upstream openpilot master
# (selfdrive/controls/lib/drive_helpers.py). Upstream computes this in modeld for models without an
# 'action' output, which is the case for the ported supercombo (C2PORT); here controlsd does it.
MIN_SPEED = 1.0
MAX_CURVATURE = 0.2  # This is a turn radius smaller than most cars can achieve
MIN_STABLE_DELAY = 0.3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2, EU guidelines
ACCELERATION_DUE_TO_GRAVITY = 9.81


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val


def clip_curvature(v_ego, prev_curvature, new_curvature, roll):
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2 * curv_from_psi - psi_rate / vego


def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return float(curv_from_psis(psi_target, psi_rate, vego, action_t))
