"""Slow lane-position trim on top of the model's plan.

The plan centres the *camera* in the lane, and a comma two's road camera sits about 6 cm left of the
car's centre line (lane_planner's EON CAMERA_OFFSET). That offset was only ever applied by the old
lateral planner, which the upstream controller port took out of the steering path. Nov 2026 logs:
with openpilot steering, the lane centre sits 0-7 cm right of the camera (the car slightly right of
centre). With the driver steering it sits 6-15 cm right of the camera.

This trims a small lateral-accel bias into the desired curvature until the measured lane centre
sits `target` metres to the right of the camera. It is an integrator, so it finds the bias that
offsets the model's own centring pull, whatever that strength is. It only adapts on clear,
near-straight road while the driver isn't steering, and it holds its value otherwise. It decays
when the lines are lost for a while.

Frames: model y is + right, and controls curvature is + right (controlsd negates calc_curvature),
so a lane centre further right than the target (y too big) needs a + (rightward) bias.
"""
from common.numpy_fast import clip
from common.realtime import DT_MDL

LANE_OFFSET_TARGET = 0.08    # m, lane centre to the right of the camera; 0 disables the trim
MAX_BIAS = 0.10              # m/s^2 lateral accel, the most the trim can ever ask for
RATE = 0.10                  # m/s^2 per s per m of position error (0.1 m off -> 0.01 m/s^2 per s)
MIN_SPEED = 12.              # m/s
MIN_PROB = 0.6
MAX_DES_LAT_ACCEL = 0.3      # m/s^2: only learn on near-straight road; the model cuts curves on purpose
LOST_DECAY_AFTER = 5.        # s without good lines before the bias starts decaying
DECAY_TAU = 10.              # s
FILTER_TAU = 1.0             # s, low-pass on the measured lane centre


class LaneOffset:
  def __init__(self):
    self.bias = 0.0          # m/s^2 lateral accel, + right
    self.centre = None       # filtered lane centre y (m, + right of camera)
    self.lost_t = 0.0

  def update(self, md, v_ego, active, steering_pressed, lane_changing, des_lat_accel, target):
    """Call once per new modelV2. Returns the curvature bias (1/m, + right)."""
    lines_ok = (len(md.laneLines) == 4 and len(md.laneLineProbs) == 4 and
                md.laneLineProbs[1] > MIN_PROB and md.laneLineProbs[2] > MIN_PROB and
                len(md.laneLines[1].y) > 0 and len(md.laneLines[2].y) > 0)
    width = (md.laneLines[2].y[0] - md.laneLines[1].y[0]) if lines_ok else 0.
    lines_ok = lines_ok and 2.6 < width < 4.6

    if target == 0. or not active:
      # disabled, or openpilot isn't steering: let any bias bleed off and start the filter fresh
      self.bias -= self.bias * DT_MDL / DECAY_TAU
      self.centre = None
      return self.curvature(v_ego)

    if lines_ok:
      c = (md.laneLines[1].y[0] + md.laneLines[2].y[0]) / 2.
      self.centre = c if self.centre is None else self.centre + (c - self.centre) * DT_MDL / FILTER_TAU
      self.lost_t = 0.
    else:
      self.lost_t += DT_MDL

    learn = (lines_ok and self.centre is not None and v_ego > MIN_SPEED and not steering_pressed
             and not lane_changing and abs(des_lat_accel) < MAX_DES_LAT_ACCEL)
    if learn:
      err = self.centre - target   # + => lane centre too far right of camera => car too far left
      self.bias = clip(self.bias + RATE * err * DT_MDL, -MAX_BIAS, MAX_BIAS)
    elif self.lost_t > LOST_DECAY_AFTER:
      self.bias -= self.bias * DT_MDL / DECAY_TAU

    return self.curvature(v_ego)

  def curvature(self, v_ego):
    if v_ego < MIN_SPEED:
      return 0.
    return self.bias / (v_ego ** 2)
