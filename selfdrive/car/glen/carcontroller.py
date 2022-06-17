from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, make_can_msg
from selfdrive.car.glen.glencan import create_steer_command, create_gas_interceptor_command
from selfdrive.car.glen.values import CAR, MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams
from opendbc.can.packer import CANPacker
VisualAlert = car.CarControl.HUDControl.VisualAlert

PEDAL_SCALE = 2.0

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # gas and brake
    if CS.CP.enableGasInterceptor and active:
      MAX_INTERCEPTOR_GAS = 0.5
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = ((actuators.accel / PEDAL_SCALE) + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, CarControllerParams)
    self.steer_rate_limited = new_steer != apply_steer

    if not active or CS.steer_state in (9, 25):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    can_sends = []

    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present so ACC can be engaged
      if CS.CP.openpilotLongitudinalControl:
        self.accel = pcm_accel_cmd

    if frame % 2 == 0 and CS.CP.enableGasInterceptor and CS.CP.openpilotLongitudinalControl:
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, frame // 2))
      self.gas = interceptor_gas_cmd

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    return new_actuators, can_sends
