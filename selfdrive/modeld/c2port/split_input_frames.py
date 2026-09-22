#!/usr/bin/env python3
"""Feed the input frames channel-last, so the image backend's input conversion is a plain copy.

With NCHW input [1,24,128,256], tinygrad permutes it into the conv's image layout (H, W*C/4, 4) with a gather kernel:
~9 ms per frame on the Adreno 530. Here the input becomes imgs [1,128,256,32]: per pixel the 4 frames (narrow t-4,
narrow t, wide t-4, wide t) x 8 channels (the 6 YUV channels + 2 unused). The first conv's weight is spread over the 32
channels (zeros for the unused ones), so the model is unchanged, and the transpose back to NCHW cancels against the
conv's own layout change.

Tried and rejected: one input [4,128,256,8] sliced per frame (a slice at a non-zero offset isn't contiguous for
tinygrad, so it still copies); four inputs with one conv each (the summing Add chain becomes its own 0.5M-element
kernel). usage: split_input_frames.py in.onnx out.onnx
"""
import sys

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto, shape_inference

m = onnx.load(sys.argv[1])
g = m.graph
inits = {t.name: t for t in g.initializer}
conv = next(n for n in g.node if 'imgs' in n.input)
assert conv.op_type == 'Conv' and sum('imgs' in n.input for n in g.node) == 1
w = numpy_helper.to_array(inits[conv.input[1]])
assert w.shape[1] == 24, w.shape

w32 = np.zeros((w.shape[0], 32) + w.shape[2:], np.float32)
for f in range(4):
  w32[:, 8 * f:8 * f + 6] = w[:, 6 * f:6 * f + 6]
w32_name = 'c2port/stem_w32'
g.initializer.append(numpy_helper.from_array(w32, w32_name))
transpose = helper.make_node('Transpose', ['imgs'], ['c2port/imgs_chw'], perm=[0, 3, 1, 2], name='c2port/imgs_transpose')
conv.input[0], conv.input[1] = 'c2port/imgs_chw', w32_name
g.node.insert(list(g.node).index(conv), transpose)
for i in g.input:
  if i.name == 'imgs':
    i.CopyFrom(helper.make_tensor_value_info('imgs', TensorProto.FLOAT, [1, 128, 256, 32]))
used = {x for n in g.node for x in n.input}
for t in [t for t in g.initializer if t.name not in used]:
  g.initializer.remove(t)
del g.value_info[:]
m = shape_inference.infer_shapes(m)
onnx.checker.check_model(m)
onnx.save(m, sys.argv[2])
print(f'wrote {sys.argv[2]}: imgs is now [1,128,256,32] (row, col, frame*8 + channel; channels 6,7 of each frame unused)')
