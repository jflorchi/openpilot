#!/usr/bin/env python3
"""Export an ONNX model to a pickle of plain Python objects (no protobuf needed to read it).
Only the fields tinygrad 2e9b7637's extra/onnx.py and openpilot/compile.py read are kept."""
import pickle, sys
from types import SimpleNamespace as NS
import onnx
m = onnx.load(sys.argv[1])
def attr(a):
  return NS(name=a.name, type=a.type, ints=list(a.ints), i=a.i, f=a.f, s=a.s, floats=list(a.floats),
            t=init(a.t) if a.type == 4 else None)
def init(t):
  return NS(name=t.name, raw_data=t.raw_data, float_data=list(t.float_data), int64_data=list(t.int64_data), dims=list(t.dims), data_type=t.data_type)
def vi(v):
  return NS(name=v.name, type=NS(tensor_type=NS(shape=NS(dim=[NS(dim_value=d.dim_value) for d in v.type.tensor_type.shape.dim]))))
g = m.graph
graph = NS(initializer=[init(t) for t in g.initializer], input=[vi(v) for v in g.input], output=[vi(v) for v in g.output],
           node=[NS(op_type=n.op_type, name=n.name, input=list(n.input), output=list(n.output), attribute=[attr(a) for a in n.attribute]) for n in g.node])
with open(sys.argv[2], 'wb') as f:
  pickle.dump(NS(graph=graph), f, protocol=4)
print('wrote', sys.argv[2])
