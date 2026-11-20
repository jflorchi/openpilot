from cereal import car
from selfdrive.car import dbc_dict
from common.conversions import Conversions as CV

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
    ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
    ACCEL_MIN = -3.5  # m/s2

    # Gas interceptor model, fitted Nov 2026 on 5 commutes (Nov 4/5/10), using only time when openpilot
    # alone had the throttle (stock cruise off, 0x399). Pedal lag ~0.75 s.
    # PEDAL_HOLD: the pedal that holds speed on a flat road. Before this the pedal was accel * scale, so
    # the integrator had to build up ~0.3-0.6 m/s^2 of fake "accel" just to hold speed, and it stalled
    # 1-2 km/h short of the set speed. Measured: ~.07 @15, ~.09-.10 @20-26, ~.20 @30, ~.25 @34 m/s.
    # The table sits slightly under the fit so the integrator tops it up instead of overshooting.
    # (no data below 14 m/s; it ramps to 0 at a stop so a zero-accel request can't creep the car)
    PEDAL_HOLD_BP = [0., 5., 10., 15., 20., 25., 30., 35.]
    PEDAL_HOLD_V = [0.0, 0.04, 0.06, 0.07, 0.085, 0.10, 0.15, 0.22]
    # PEDAL_GAIN: pedal per m/s^2 above the hold pedal. Measured accel per unit pedal is ~3 below
    # 26 m/s, 1.5 @30 and 1.2 @34. Again a bit under, giving a plant gain of ~0.8-0.9.
    PEDAL_GAIN_BP = [0., 20., 25., 30., 35.]
    PEDAL_GAIN_V = [0.30, 0.30, 0.40, 0.55, 0.65]
    MAX_INTERCEPTOR_GAS = 0.5

    STEER_MAX = 1500
    STEER_ERROR_MAX = 350  # max delta between torque cmd and torque motor

    def __init__(self, CP):
        if CP.lateralTuning.which == 'torque':
            self.STEER_DELTA_UP = 15       # 1.0s time to peak torque
            self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
        else:
            self.STEER_DELTA_UP = 10       # 1.5s time to peak torque
            self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)



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

EPS_SCALE = 88
