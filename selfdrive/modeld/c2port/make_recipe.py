#!/usr/bin/env python3
"""Split the rewritten model into a small recipe + references to the public original weights.

usage: make_recipe.py driving_supercombo.onnx supercombo_c2_tg.onnx recipe.pkl
The device rebuilds the full graph pickle with build_model.py from recipe.pkl + the original ONNX.
"""
import pickle
import sys
from types import SimpleNamespace as NS

import numpy as np
import onnx
from onnx import numpy_helper

orig = {t.name: numpy_helper.to_array(t) for t in onnx.load(sys.argv[1]).graph.initializer}
m = onnx.load(sys.argv[2])
g = m.graph


def init(t):
  arr = numpy_helper.to_array(t)
  src = orig.get(t.name)
  if src is not None and src.size == arr.size and np.array_equal(src.astype(arr.dtype).reshape(arr.shape), arr):
    return NS(name=t.name, ref=t.name, dims=list(t.dims), data_type=t.data_type)
  if t.name.startswith('c2port/dup_w'):  # channel-duplication conv weights: generated on the device (too big for the proxy)
    cout, group = arr.shape[:2]
    expect = np.zeros_like(arr)
    expect[np.arange(cout), np.arange(cout) // (cout // group)] = 1
    assert np.array_equal(arr, expect)
    return NS(name=t.name, ref=None, gen=('dup', cout, group), dims=list(t.dims), data_type=t.data_type)
  return NS(name=t.name, ref=None, raw_data=arr.tobytes(), float_data=[], int64_data=[], dims=list(t.dims), data_type=t.data_type)


def attr(a):
  return NS(name=a.name, type=a.type, ints=list(a.ints), i=a.i, f=a.f, s=a.s, floats=list(a.floats), t=init(a.t) if a.type == 4 else None)


def vi(v):
  return NS(name=v.name, type=NS(tensor_type=NS(shape=NS(dim=[NS(dim_value=d.dim_value) for d in v.type.tensor_type.shape.dim]))))


inits = [init(t) for t in g.initializer]
graph = NS(initializer=inits, input=[vi(v) for v in g.input], output=[vi(v) for v in g.output],
           node=[NS(op_type=n.op_type, name=n.name, input=list(n.input), output=list(n.output), attribute=[attr(a) for a in n.attribute])
                 for n in g.node])
with open(sys.argv[3], 'wb') as f:
  pickle.dump(NS(graph=graph), f, protocol=4)
nref = sum(i.ref is not None for i in inits)
ngen = sum(getattr(i, 'gen', None) is not None for i in inits)
print(f'{sys.argv[3]}: {nref} weights by reference, {ngen} generated, {len(inits) - nref - ngen} inline '
      f'({sum(len(i.raw_data) for i in inits if i.ref is None and getattr(i, "gen", None) is None) / 1e3:.1f} kB)')
