#!/usr/bin/env python3
"""Remove the (x - 127.5) / 63.75 input normalization from the graph; modeld's YUV loader applies it instead.
The model input 'imgs' then expects normalized values. usage: strip_input_norm.py in.onnx out.onnx"""
import sys
import numpy as np
import onnx
from onnx import numpy_helper, shape_inference
m = onnx.load(sys.argv[1]); g = m.graph
inits = {t.name: numpy_helper.to_array(t) for t in g.initializer}
sub = next(n for n in g.node if n.op_type == 'Sub' and n.input[0] == 'imgs')
div = next(n for n in g.node if n.op_type == 'Div' and n.input[0] == sub.output[0])
assert np.allclose(inits[sub.input[1]], 127.5) and np.allclose(inits[div.input[1]], 63.75)
for n in g.node:
  for i, x in enumerate(n.input):
    if x == div.output[0]:
      n.input[i] = 'imgs'
g.node.remove(sub); g.node.remove(div)
used = {x for n in g.node for x in n.input}
for t in [t for t in g.initializer if t.name not in used]:
  g.initializer.remove(t)
del g.value_info[:]
m = shape_inference.infer_shapes(m)
onnx.checker.check_model(m)
onnx.save(m, sys.argv[2])
print('wrote', sys.argv[2], '- imgs input now normalized: (yuv - 127.5) / 63.75')
