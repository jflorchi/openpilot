#!/usr/bin/env python3
"""Reference outputs of a final (normalized, channel-last) port graph on the saved real frames, with tinygrad in fp32,
for thneed_check.py. Writes a copy of real_frames.npz with out0/out1 replaced.

usage: make_ref.py model.onnxpkl real_frames.npz out.npz   (env as in check_dump4.py: OPT=99 OPENCL=1 UNSAFE_FLOAT4=1 NATIVE_EXPLOG=1)
"""
import sys

import numpy as np
sys.path[:0] = ['/data/c2port', '/data/c2port/tinygrad_repo', '/data/c2port/pylib']
import onnx
from extra.onnx import get_run_onnx
from tinygrad.tensor import Tensor

Tensor.no_grad = True
model, frames, out_path = sys.argv[1:4]
graph = onnx.load(open(model, 'rb'))
run = get_run_onnx(graph)
from port_feed import feed as port_feed
sizes = {v.name: int(np.prod([dd.dim_value for dd in v.type.tensor_type.shape.dim])) for v in graph.graph.input}
d = dict(np.load(frames))
for i in range(2):
  imgs = (d[f'imgs{i}'].astype(np.float32) - 127.5) / 63.75
  fr = imgs.reshape(4, 6, 128, 256).transpose(2, 3, 0, 1)
  imgs = np.concatenate([fr, np.zeros((128, 256, 4, 2), np.float32)], -1).reshape(1, 128, 256, 32)
  shapes = {v.name: [dd.dim_value for dd in v.type.tensor_type.shape.dim] for v in graph.graph.input}
  feed = {k: np.asarray(v, np.float32).reshape(shapes[k]) for k, v in port_feed(d, i, sizes, imgs).items()}
  out = run(feed)['outputs'].numpy()
  assert out.shape in ((1, 2576), (1, 2580)) and np.isfinite(out).all(), (out.shape, np.isfinite(out).all())
  d[f'out{i}'] = out
  print(f'reference frame {i}: |out| max {np.abs(out).max():.1f}')
np.savez(out_path, **d)
print('wrote', out_path)
