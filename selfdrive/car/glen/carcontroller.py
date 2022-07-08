from cereal import car
from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits, make_can_msg
from selfdrive.car.glen.glencan import create_steer_command, create_gas_interceptor_command
from selfdrive.car.glen.values import MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
    def __init__(self, dbc_name, CP, VM):
        self.CP = CP
        self.frame = 0
        self.torque_rate_limits = CarControllerParams(self.CP)
        self.steer_rate_limited = False
        self.last_steer = 0

        self.packer = CANPacker(dbc_name)
        self.gas = 0
        self.accel = 0
        self.last_enabled = False

    def update(self, CC, CS):
        actuators = CC.actuators
        hud_control = CC.hudControl
        pcm_cancel_cmd = CC.cruiseControl.cancel

        # gas and brake
        if self.CP.enableGasInterceptor and CC.longActive:
            MAX_INTERCEPTOR_GAS = 0.5
            PEDAL_SCALE = interp(CS.out.vEgo, [0.0, 40.0], [0.15, 0.45, 0.0])
            pedal_command = (actuators.accel * PEDAL_SCALE)
            interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
        else:
            interceptor_gas_cmd = 0.
        pcm_accel_cmd = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

        new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
        apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.torque_rate_limits)
        self.steer_rate_limited = new_steer != apply_steer

        if not CC.latActive:
            apply_steer = 0
            apply_steer_req = 0
        else:
            apply_steer_req = 1

        self.last_steer = apply_steer

        can_sends = [create_steer_command(self.packer, apply_steer, apply_steer_req, self.frame)]

        if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
            lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged
            if self.CP.openpilotLongitudinalControl:
                self.accel = pcm_accel_cmd

        if self.frame % 2 == 0 and self.CP.enableGasInterceptor and CS.CP.openpilotLongitudinalControl:
            can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
            self.gas = interceptor_gas_cmd

        if self.last_enabled and not CC.enabled: # openpilot was disabled during this loop send messaged
          can_sends.append(make_can_msg(0x002, b'\xFF', 2))

        new_actuators = actuators.copy()
        new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
        new_actuators.accel = self.accel
        new_actuators.gas = self.gas

        self.last_enabled = CC.enabled

        self.frame += 1
        return new_actuators, can_sends
