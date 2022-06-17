from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.config import Conversions as CV

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
    ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
    ACCEL_MIN = -3.5  # m/s2

    STEER_MAX = 1500
    STEER_DELTA_UP = 10  # 1.5s time to peak torque
    STEER_DELTA_DOWN = 25  # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
    STEER_ERROR_MAX = 350  # max delta between torque cmd and torque motor


class CAR:
    GLEN = "GLEN"


STATIC_DSU_MSGS = []

FW_VERSIONS = {
  CAR.GLEN: {}
}

STEER_THRESHOLD = 100

DBC = {
    CAR.GLEN: dbc_dict('toyota_corolla_2010', None)
}

EPS_SCALE = 73
