import time

from cereal import car
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import mean
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.glen.linked_ring import LinkedRing
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.glen.values import DBC, STEER_THRESHOLD, EPS_SCALE
from common.conversions import Conversions as CV

class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)
        self.steer_state = 0
        self.pcm_acc_status = 0
        can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
        self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
        self.eps_torque_scale = EPS_SCALE / 100.

        self.needs_angle_offset = True
        self.accurate_steer_angle_seen = False
        self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

        self.low_speed_lockout = False
        self.acc_type = 1

        self.angle_offset = 0.
        self.last_time = -1
        self.last_angle = 0
        self.last_rate = 0
        self.rolling_sum = LinkedRing(5)

    def update(self, cp0, cp1, cp2):
        ret = car.CarState.new_message()

        ret.doorOpen = any([cp0.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp0.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                            cp0.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp0.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
        ret.seatbeltUnlatched = cp0.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0

        ret.brakePressed = cp0.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
        ret.brakeHoldActive = cp0.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
        ret.gas = (cp2.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp2.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
        ret.gasPressed = ret.gas > 15

        ret.wheelSpeeds = self.get_wheel_speeds(
            cp0.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FL"],
            cp0.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FR"],
            cp0.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RL"],
            cp0.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RR"],
        )
        ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

        ret.standstill = ret.vEgoRaw < 0.001

        ret.steeringAngleDeg = cp2.vl["SECONDARY_STEER_ANGLE"]["ANGLE"]
        if cp2.vl["SECONDARY_STEER_ANGLE"]["SIGN"] == 1:
            ret.steeringAngleDeg = 65535 - ret.steeringAngleDeg
            ret.steeringAngleDeg = 0 - ret.steeringAngleDeg
            ret.steeringAngleDeg = ret.steeringAngleDeg * 0.021973997
        else:
            ret.steeringAngleDeg = ret.steeringAngleDeg * 0.021973997
        if self.last_time == -1:
            ret.steeringRateDeg = 0  # set to 0 since it isn't moving yet
            self.last_time = round(time.time() * 1000)
        else:
            cur_time = round(time.time() * 1000)
            elapsed = cur_time - self.last_time
            elapsed /= 1000.0
            cur_angle = (ret.steeringAngleDeg - self.last_angle) / elapsed
            self.rolling_sum.add_val(cur_angle)
            ret.steeringRateDeg = cur_angle
            self.last_time = cur_time
        self.last_angle = ret.steeringAngleDeg
        ret.steeringAngleDeg -= 2.41

        #print(ret.steeringAngleDeg)

        can_gear = int(cp0.vl["GEAR_PACKET"]["GEAR"])
        ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
        ret.leftBlinker = cp2.vl["BLINKERS_STATE"]["SIGNAL_LEFT"] == 255.0
        ret.rightBlinker = cp2.vl["BLINKERS_STATE"]["SIGNAL_RIGHT"] == 255.0

        ret.steeringTorque = cp0.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
        ret.steeringTorqueEps = cp0.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
        # we could use the override bit from dbc, but it's triggered at too high torque values
        ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
        ret.steerFaultTemporary = cp0.vl["EPS_STATUS"]["LKA_STATE"] in (0, 9, 21, 25)
        # 17 is a fault from a prolonged high torque delta between cmd and user
        ret.steerFaultPermanent = cp0.vl["EPS_STATUS"]["LKA_STATE"] == 17

        #print(str(ret.steerFaultTemporary) + " - " + str(ret.steerFaultPermanent))

        ret.cruiseState.available = cp2.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
        ret.cruiseState.speed = cp2.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

        self.pcm_acc_status = cp2.vl["PCM_CRUISE"]["CRUISE_STATE"]
        ret.cruiseState.standstill = False
        vehicle_cruise = bool(cp0.vl["PCM_CRUISE_SM"]["CRUISE_CONTROL_STATE"] != 0)
        stalk_cruise = bool(cp2.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
        ret.cruiseState.enabled = vehicle_cruise or stalk_cruise
        ret.cruiseState.nonAdaptive = cp2.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)
        ret.genericToggle = True
        ret.stockAeb = False

        ret.espDisabled = cp0.vl["ESP_CONTROL"]["TC_DISABLED"] != 0
        # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
        self.steer_state = cp0.vl["EPS_STATUS"]["LKA_STATE"]

        return ret

    @staticmethod
    def get_can_parser(CP):
        signals = [
            ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
            ("GEAR", "GEAR_PACKET"),
            ("BRAKE_PRESSED", "BRAKE_MODULE"),
            ("DOOR_OPEN_FL", "BODY_CONTROL_STATE"),
            ("DOOR_OPEN_FR", "BODY_CONTROL_STATE"),
            ("DOOR_OPEN_RL", "BODY_CONTROL_STATE"),
            ("DOOR_OPEN_RR", "BODY_CONTROL_STATE"),
            ("SEATBELT_DRIVER_UNLATCHED", "BODY_CONTROL_STATE"),
            ("PARKING_BRAKE", "BODY_CONTROL_STATE"),
            ("TC_DISABLED", "ESP_CONTROL"),
            ("BRAKE_HOLD_ACTIVE", "ESP_CONTROL"),
            ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
            ("STEER_RATE", "STEER_ANGLE_SENSOR"),
            ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"),
            ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR"),
            ("STEER_ANGLE", "STEER_TORQUE_SENSOR"),
            ("STEER_ANGLE_INITIALIZING", "STEER_TORQUE_SENSOR"),
            ("LKA_STATE", "EPS_STATUS"),
            ("AUTO_HIGH_BEAM", "LIGHT_STALK"),
            ("GAS_PEDAL", "GAS_PEDAL"),
            ("WHEEL_SPEED_FL", "WHEEL_SPEEDS_FRONT"),
            ("WHEEL_SPEED_FR", "WHEEL_SPEEDS_FRONT"),
            ("WHEEL_SPEED_RL", "WHEEL_SPEEDS_REAR"),
            ("WHEEL_SPEED_RR", "WHEEL_SPEEDS_REAR"),
            ("CRUISE_CONTROL_STATE", "PCM_CRUISE_SM")
        ]
        checks = [
            ("GEAR_PACKET", 1),
            ("LIGHT_STALK", 1),
            ("BODY_CONTROL_STATE", 3),
            ("ESP_CONTROL", 3),
            ("EPS_STATUS", 25),
            ("GAS_PEDAL", 33),
            ("PCM_CRUISE_SM", 2),
            ("BRAKE_MODULE", 40),
            ("STEER_ANGLE_SENSOR", 80),
            ("WHEEL_SPEEDS_FRONT", 50),
            ("WHEEL_SPEEDS_REAR", 50),
            ("STEER_TORQUE_SENSOR", 50)
        ]
        return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

    @staticmethod
    def get_cam_can_parser(CP):
        signals = [

        ]
        checks = [

        ]
        return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1)

    @staticmethod
    def get_body_can_parser(CP):
        signals = [
            ("ZORRO_STEER", "SECONDARY_STEER_ANGLE"),
            ("SIGN", "SECONDARY_STEER_ANGLE"),
            ("ANGLE", "SECONDARY_STEER_ANGLE"),
            ("CRUISE_ACTIVE", "PCM_CRUISE"),
            ("CRUISE_STATE", "PCM_CRUISE"),
            ("GAS_RELEASED", "PCM_CRUISE"),
            ("MAIN_ON", "PCM_CRUISE_2"),
            ("SET_SPEED", "PCM_CRUISE_2"),
            ("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2"),
            ("INTERCEPTOR_GAS", "GAS_SENSOR"),
            ("INTERCEPTOR_GAS2", "GAS_SENSOR"),
            ("SIGNAL_LEFT", "BLINKERS_STATE"),
            ("SIGNAL_RIGHT", "BLINKERS_STATE")
        ]
        checks = [
            ("PCM_CRUISE", 4),
            ("PCM_CRUISE_2", 4),
            ("BLINKERS_STATE", 4),
            ("SECONDARY_STEER_ANGLE", 20),
            ("GAS_SENSOR", 40)
        ]
        return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
