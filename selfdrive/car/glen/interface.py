#!/usr/bin/env python3
from cereal import car
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, \
  get_safety_config
from selfdrive.car.glen.tunes import LatTunes, LongTunes, set_long_tune, set_lat_tune
from selfdrive.car.glen.values import CAR, EPS_SCALE, CarControllerParams
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.config import Conversions as CV

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
    @staticmethod
    def get_pid_accel_limits(CP, current_speed, cruise_speed):
        return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

    @staticmethod
    def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
        tire_stiffness_factor = 0.444
        if car_fw is None:
            car_fw = []
        ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

        ret.carName = "glen"
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.allOutput)]
        ret.safetyConfigs[0].safetyParam = EPS_SCALE

        ret.steerActuatorDelay = 0.28  # Default delay, Prius has larger delay
        ret.steerLimitTimer = 0.4
        ret.stoppingControl = True  # Toyota starts braking more when it thinks you want to stop

        if candidate == CAR.GLEN:
            ret.wheelbase = 2.6
            ret.steerRatio = 17.4
            tire_stiffness_factor = 0.444
            ret.mass = 2745. * CV.LB_TO_KG + STD_CARGO_KG
            set_lat_tune(ret.lateralTuning, LatTunes.GLEN_MODEL)

        ret.steerRateCost = 1.
        ret.centerToFront = ret.wheelbase * 0.44

        ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
        ret.tireStiffnessFront, ret.tireStiffnessRear = \
            scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                 tire_stiffness_factor=tire_stiffness_factor)

        ret.enableBsm = False
        ret.enableDsu = False
        ret.enableGasInterceptor = True
        ret.openpilotLongitudinalControl = True
        ret.minEnableSpeed = -1.

        set_long_tune(ret.longitudinalTuning, LongTunes.GLEN_PEDAL)

        return ret

    # returns a car.CarState
    def update(self, c, can_strings):
        # ******************* do can recv *******************
        self.cp.update_strings(can_strings)
        self.cp_cam.update_strings(can_strings)
        self.cp_body.update_strings(can_strings)

        ret = self.CS.update(self.cp, self.cp_cam, self.cp_body)

        ret.canValid = self.cp.can_valid and self.cp_cam.can_valid and self.cp_body.can_valid
        ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

        # events
        events = self.create_common_events(ret)

        if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
            events.add(EventName.lowSpeedLockout)
        if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
            events.add(EventName.belowEngageSpeed)
            if c.actuators.accel > 0.3:
                # some margin on the actuator to not false trigger cancellation while stopping
                events.add(EventName.speedTooLow)
            if ret.vEgo < 0.001:
                # while in standstill, send a user alert
                events.add(EventName.manualRestart)

        ret.events = events.to_msg()

        self.CS.out = ret.as_reader()
        return self.CS.out

    # pass in a car.CarControl
    # to be called @ 100hz
    def apply(self, c):
        hud_control = c.hudControl
        ret = self.CC.update(c.enabled, c.active, self.CS, self.frame,
                             c.actuators, c.cruiseControl.cancel,
                             hud_control.visualAlert, hud_control.leftLaneVisible,
                             hud_control.rightLaneVisible, hud_control.leadVisible,
                             hud_control.leftLaneDepart, hud_control.rightLaneDepart)

        self.frame += 1
        return ret
