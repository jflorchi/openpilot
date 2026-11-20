"""Shared test feed for the port graphs (make_ref.py, thneed_check.py, splitk.py): real_frames.npz holds the
25 x 8 desire tokens of the older contract; Cheddar-era graphs take 9 windows of them ([9,200], row 8 = the 25 most
recent). Output layouts differ too, so the plan slice used for the summary line is picked by output size."""
import numpy as np

ACTION_T = np.array([[0.41, 0.525]], np.float32)  # upstream: lat/long delay + frame delay + half a frame
PLAN = {2576: slice(1576, 1576 + 495), 2580: slice(917, 917 + 495)}


def desire_tokens(desire, n_floats):
  a = np.asarray(desire, np.float32).reshape(-1)
  if a.size == n_floats:
    return a
  assert a.size == 200 and n_floats == 1800, (a.size, n_floats)
  pooled = np.concatenate([np.zeros((8, 8), np.float32), a.reshape(25, 8)])
  return np.stack([pooled[k:k + 25].reshape(200) for k in range(9)])


def feed(d, i, sizes, imgs):
  """sizes: {input name: number of floats}"""
  f = {'imgs': imgs, 'desire_tokens': desire_tokens(d[f'desire{i}'], sizes['desire_tokens']),
       'feat_hist': d[f'feat{i}'], 'traffic_convention': np.array([[1, 0]], np.float32), 'action_t': ACTION_T}
  return {k: v for k, v in f.items() if k in sizes}
