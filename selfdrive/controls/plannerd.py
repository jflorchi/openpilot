#!/usr/bin/env python3
from cereal import car
from common.params import Params
from common.realtime import Priority, config_realtime_process
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.longitudinal_planner import Planner
from selfdrive.controls.lib.lateral_planner import LateralPlanner
from selfdrive.hardware import TICI
import cereal.messaging as messaging


def plannerd_thread(sm=None, pm=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  # LanelessMode: 0 = lanelines, 1 = laneless, 2 = auto (confidence-based)
  laneless_mode = int(params.get('LanelessMode', encoding='utf8') or 1)
  use_lanelines = laneless_mode == 0
  wide_camera = params.get_bool('WideCameraOnly') if TICI else False

  cloudlog.event("e2e mode", on=not use_lanelines)

  longitudinal_planner = Planner(CP)
  lateral_planner = LateralPlanner(CP, use_lanelines=use_lanelines, wide_camera=wide_camera)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'modelV2'],
                             poll=['radarState', 'modelV2'], ignore_avg_freq=['radarState'])

  if pm is None:
    pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan'])

  frame = 0
  while True:
    sm.update()

    # allow the laneless mode to take effect while onroad (checked ~1x/sec)
    frame += 1
    if frame % 20 == 0:
      laneless_mode = int(params.get('LanelessMode', encoding='utf8') or 1)
      lateral_planner.dynamic_lanelines = laneless_mode == 2
      # in auto mode lateral_planner picks the mode from laneline confidence;
      # otherwise the selector decides directly
      if not lateral_planner.dynamic_lanelines:
        use_lanelines = laneless_mode == 0
        if use_lanelines != lateral_planner.use_lanelines:
          lateral_planner.use_lanelines = use_lanelines
          cloudlog.event("e2e mode", on=not use_lanelines)

    if sm.updated['modelV2']:
      lateral_planner.update(sm)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm)
      longitudinal_planner.publish(sm, pm)


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
