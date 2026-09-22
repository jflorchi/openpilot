#!/usr/bin/env python3
"""Rebuild the comma two port graph from recipe.pkl + the original driving_supercombo.onnx, with no onnx/protobuf install.

usage: build_model.py recipe.pkl driving_supercombo.onnx supercombo_c2.onnxpkl
"""
import pickle
import sys

import numpy as np

DTYPES = {1: np.float32, 7: np.int64, 9: np.bool_, 10: np.float16}


def varint(b, i):
  x = s = 0
  while True:
    c = b[i]
    i += 1
    x |= (c & 0x7f) << s
    s += 7
    if c < 0x80:
      return x, i


def fields(b):
  """Yield (field number, wire type, value) of one protobuf message. LEN values are memoryviews."""
  i = 0
  while i < len(b):
    key, i = varint(b, i)
    fn, wt = key >> 3, key & 7
    if wt == 0:
      v, i = varint(b, i)
    elif wt == 2:
      n, i = varint(b, i)
      v, i = b[i:i + n], i + n
    elif wt == 1:
      v, i = b[i:i + 8], i + 8
    elif wt == 5:
      v, i = b[i:i + 4], i + 4
    else:
      raise ValueError(f'wire type {wt}')
    yield fn, wt, v


def tensor(b):
  """onnx TensorProto: dims=1, data_type=2, int64_data=7, name=8, raw_data=9."""
  t = {'dims': [], 'int64': []}
  for fn, wt, v in fields(b):
    if fn == 1:
      if wt == 2:  # packed
        j = 0
        while j < len(v):
          d, j = varint(v, j)
          t['dims'].append(d)
      else:
        t['dims'].append(v)
    elif fn == 2:
      t['data_type'] = v
    elif fn == 7:
      if wt == 2:
        j = 0
        while j < len(v):
          d, j = varint(v, j)
          t['int64'].append(d if d < 2**63 else d - 2**64)
      else:
        t['int64'].append(v if v < 2**63 else v - 2**64)
    elif fn == 8:
      t['name'] = bytes(v).decode()
    elif fn == 9:
      t['raw'] = v
  if 'raw' in t:
    arr = np.frombuffer(t['raw'], DTYPES[t['data_type']])
  else:
    arr = np.array(t['int64'], np.int64)
  return t['name'], arr.reshape(t['dims'])


def original_weights(path):
  b = memoryview(open(path, 'rb').read())
  graph = next(v for fn, _, v in fields(b) if fn == 7)            # ModelProto.graph
  return dict(tensor(v) for fn, _, v in fields(graph) if fn == 5)  # GraphProto.initializer


def main(recipe_path, onnx_path, out_path):
  recipe = pickle.load(open(recipe_path, 'rb'))
  weights = original_weights(onnx_path)
  for t in recipe.graph.initializer:
    gen = getattr(t, 'gen', None)
    if gen is not None:  # channel-duplication weights for rewritten multiplier convs
      _, cout, group = gen
      arr = np.zeros((cout, group, 1, 1), np.float32)
      arr[np.arange(cout), np.arange(cout) // (cout // group)] = 1
      t.raw_data, t.float_data, t.int64_data = arr.tobytes(), [], []
      del t.gen
    elif t.ref is not None:
      arr = weights[t.ref].astype(DTYPES[t.data_type]).reshape(t.dims)
      t.raw_data, t.float_data, t.int64_data = arr.tobytes(), [], []
    del t.ref
  with open(out_path, 'wb') as f:
    pickle.dump(recipe, f, protocol=4)
  print(f'wrote {out_path}: {len(recipe.graph.node)} nodes, {len(recipe.graph.initializer)} initializers')


if __name__ == '__main__':
  main(*sys.argv[1:4])
