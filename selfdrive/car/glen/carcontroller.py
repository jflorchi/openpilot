from cereal import car
from common.live_tune import LiveTuneReader
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits, make_can_msg
from selfdrive.car.glen.glencan import create_steer_command, create_gas_interceptor_command
from selfdrive.car.glen.values import CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


def pedal_hold_and_gain(v_ego, hold_scale=1.0, gain_scale=1.0):
    """Interceptor pedal = hold + accel * gain (see CarControllerParams). Scales come from the web tuner."""
    hold = interp(v_ego, CarControllerParams.PEDAL_HOLD_BP, CarControllerParams.PEDAL_HOLD_V) * hold_scale
    gain = interp(v_ego, CarControllerParams.PEDAL_GAIN_BP, CarControllerParams.PEDAL_GAIN_V) * gain_scale
    return hold, gain


def coast_accel(v_ego):
    """The accel command at which the pedal reaches 0. Any lower request does nothing (no brakes), so the long PID
    is limited here; below it, the integrator wound to -3.5 m/s^2 while the car sat above the set speed and then
    took a minute to unwind."""
    hold, gain = pedal_hold_and_gain(v_ego)
    return -hold / gain


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
        self.live_tune = LiveTuneReader()

    def update(self, CC, CS):
        actuators = CC.actuators
        hud_control = CC.hudControl
        pcm_cancel_cmd = CC.cruiseControl.cancel

        # gas only: there is no brake actuator, so anything below the hold pedal is a coast
        if self.CP.enableGasInterceptor and CC.longActive:
            tune = self.live_tune.update(sec_since_boot())
            hold, gain = pedal_hold_and_gain(CS.out.vEgo, tune.get('pedalHoldScale', 1.0), tune.get('pedalGainScale', 1.0))
            interceptor_gas_cmd = clip(hold + actuators.accel * gain, 0., CarControllerParams.MAX_INTERCEPTOR_GAS)
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
