"""Shared pieces of upstream's torqued and lagd (openpilot master selfdrive/locationd/helpers.py).

Ported to this fork's Python 3.8. Upstream's pose calibrator is left out: both estimators read
liveLocationKalman, whose angularVelocityCalibrated is already in the calibrated frame.
"""
from __future__ import annotations

from contextlib import contextmanager
from functools import lru_cache
from typing import Any, Sequence

import numpy as np


@lru_cache(maxsize=None)
def fft_next_good_size(n: int) -> int:
  """
  smallest composite of 2, 3, 5, 7, 11 that is >= n
  inspired by pocketfft
  """
  if n <= 6:
    return n
  best, f2 = 2 * n, 1
  while f2 < best:
    f23 = f2
    while f23 < best:
      f235 = f23
      while f235 < best:
        f2357 = f235
        while f2357 < best:
          f235711 = f2357
          while f235711 < best:
            best = f235711 if f235711 >= n else best
            f235711 *= 11
          f2357 *= 7
        f235 *= 5
      f23 *= 3
    f2 *= 2
  return best


def parabolic_peak_interp(R, max_index):
  if max_index == 0 or max_index == len(R) - 1:
    return max_index

  y_m1, y_0, y_p1 = R[max_index - 1], R[max_index], R[max_index + 1]
  offset = 0.5 * (y_p1 - y_m1) / (2 * y_0 - y_p1 - y_m1)

  return max_index + offset


@contextmanager
def capnp_reader(struct, dat: bytes):
  """struct.from_bytes that works on both pycapnp APIs: the device's 1.1 returns the reader,
  newer versions return a context manager around it."""
  r = struct.from_bytes(dat)
  if hasattr(r, '__enter__') and not hasattr(r, 'to_bytes'):
    with r as msg:
      yield msg
  else:
    yield r


class NPQueue:
  def __init__(self, maxlen: int, rowsize: int) -> None:
    self.maxlen = maxlen
    self.arr = np.empty((0, rowsize))

  def __len__(self) -> int:
    return len(self.arr)

  def append(self, pt: list[float]) -> None:
    if len(self.arr) < self.maxlen:
      self.arr = np.append(self.arr, [pt], axis=0)
    else:
      self.arr[:-1] = self.arr[1:]
      self.arr[-1] = pt


class PointBuckets:
  def __init__(self, x_bounds: list[tuple[float, float]], min_points: Sequence[float], min_points_total: int, points_per_bucket: int, rowsize: int) -> None:
    assert len(x_bounds) == len(min_points)
    self._rng = np.random.default_rng()
    self.x_bounds = x_bounds
    self.buckets = {bounds: NPQueue(maxlen=points_per_bucket, rowsize=rowsize) for bounds in x_bounds}
    self.buckets_min_points = dict(zip(x_bounds, min_points))
    self.min_points_total = min_points_total

  def __len__(self) -> int:
    return sum([len(v) for v in self.buckets.values()])

  def is_valid(self) -> bool:
    individual_buckets_valid = all(len(v) >= min_pts for v, min_pts in zip(self.buckets.values(), self.buckets_min_points.values()))
    total_points_valid = self.__len__() >= self.min_points_total
    return individual_buckets_valid and total_points_valid

  def get_valid_percent(self) -> int:
    total_points_perc = min(self.__len__() / self.min_points_total * 100, 100)
    individual_buckets_perc = min(min(len(v) / min_pts * 100 for v, min_pts in
                                      zip(self.buckets.values(), self.buckets_min_points.values())), 100)
    return int((total_points_perc + individual_buckets_perc) / 2)

  def is_calculable(self) -> bool:
    return all(len(v) > 0 for v in self.buckets.values())

  def add_point(self, x: float, y: float) -> None:
    raise NotImplementedError

  def get_points(self, num_points: int | None = None) -> Any:
    points = np.vstack([x.arr for x in self.buckets.values()])
    if num_points is None:
      return points
    return points[self._rng.choice(np.arange(len(points)), min(len(points), num_points), replace=False)]

  def load_points(self, points: Sequence[Sequence[float]]) -> None:
    for point in points:
      self.add_point(*point)


class ParameterEstimator:
  """ Base class for parameter estimators """
  def reset(self) -> None:
    raise NotImplementedError

  def handle_log(self, t: int, which: str, msg) -> None:
    raise NotImplementedError

  def get_msg(self, valid: bool, with_points: bool):
    raise NotImplementedError
