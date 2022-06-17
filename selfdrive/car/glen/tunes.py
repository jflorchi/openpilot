# !/usr/bin/env python3
from enum import Enum


class LongTunes(Enum):
    GLEN_PEDAL = 0


class LatTunes(Enum):
    GLEN_PID = 0
    GLEN_MODEL = 1


###### LONG ######
def set_long_tune(tune, name):
    # Improved longitudinal tune
    if name == LongTunes.GLEN_PEDAL:
        tune.deadzoneBP = [0., 8.05]
        tune.deadzoneV = [.0, .14]
        tune.kpBP = [0., 5., 20.]
        tune.kpV = [1.3, 1.0, 0.7]
        tune.kiBP = [0., 5., 12., 20., 27.]
        tune.kiV = [.35, .23, .20, .17, .1]
    else:
        raise NotImplementedError('This longitudinal tune does not exist')


###### LAT ######
def set_lat_tune(tune, name):
    if name == LatTunes.GLEN_PID:
        tune.pid.kpV = [0.2]
        tune.pid.kiV = [0.05]
        tune.pid.kf = 0.00003
    elif name == LatTunes.GLEN_MODEL:
      tune.init('model')
      tune.model.useRates = False  # TODO: makes model sluggish, see comments in latcontrol_model.py
      tune.model.multiplier = 1.
      tune.model.name = "corolla_model_v5"
    else:
        raise NotImplementedError('This lateral tune does not exist')
