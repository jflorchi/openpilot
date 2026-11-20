#!/usr/bin/env python3
"""Validate a C2PORT_DUMP from the live bench:
 - history: for every frame, feature token 7's source offset == image slot t-4's source offset (both advance per frame)
 - accuracy: C++ outputs vs tinygrad fp32 (validated reference) on the dumped inputs"""
import sys
import numpy as np
sys.path[:0] = ['/data/c2port', '/data/c2port/tinygrad_repo', '/data/c2port/pylib']
ddir, n = sys.argv[1], int(sys.argv[2])
ld = lambda i, w: np.fromfile(f'{ddir}/{i:03d}_{w}.bin', np.float32)
outs = [ld(i, 'out') for i in range(n)]
# hidden state / plan offsets by output layout (2576: Sep 2026 supercombo and split merges, 2580: Cheddar)
H, PLAN = {2576: (slice(1064, 1576), slice(1576, 2071)), 2580: (slice(2066, 2578), slice(917, 1412))}[outs[0].size]
feats = [ld(i, 'feat').reshape(8, 512) for i in range(n)]
imgs = [ld(i, 'imgs').reshape(128 * 256, 4, 8).transpose(1, 0, 2).reshape(4, -1) for i in range(n)]  # channel-last -> per frame
def src(i, test):
  hits = [i - j for j in range(max(0, i - 10), i) if test(j)]
  return hits[0] if len(hits) == 1 else hits
rows, ok = [], True
for i in range(8, n):
  r = src(i, lambda j: np.array_equal(imgs[i][0], imgs[j][1]))
  f = src(i, lambda j: np.array_equal(feats[i][7], outs[j][H]))
  rows.append((i, r, f)); ok &= (r == f and isinstance(r, int))
print('frame: image t-4 offset / feature token-7 offset:', ' '.join(f'{i}:{r}/{f}' for i, r, f in rows))
print('HISTORY OK:', ok)
des = [ld(i, 'desire').reshape(-1, 25, 8) for i in range(n)]
if des[0].shape[0] > 1:  # Cheddar: window w+1 is window w shifted by one pooled token
  print('DESIRE WINDOWS OK:', all(np.array_equal(d[1:, :-1], d[:-1, 1:]) for d in des))
import os
import onnx
from extra.onnx import get_run_onnx
from tinygrad.tensor import Tensor
Tensor.no_grad = True
graph = onnx.load(open(os.environ.get('C2PORT_MODEL', '/data/c2port/supercombo_c2_final.onnxpkl'), 'rb'))
shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim] for v in graph.graph.input}
run = get_run_onnx(graph)
for i in (0, 8, 16, n - 1):
  at = ld(i, 'action_t') if os.path.exists(f'{ddir}/{i:03d}_action_t.bin') else np.array([0.275, 0.525], np.float32)
  feed = {'imgs': ld(i, 'imgs'), 'desire_tokens': ld(i, 'desire'), 'feat_hist': feats[i], 'traffic_convention': ld(i, 'traffic'), 'action_t': at}
  feed = {k: v.reshape(shapes[k]) for k, v in feed.items() if k in shapes}
  ref = run(feed)['outputs'].numpy()[0]
  print(f'frame {i}: C++ vs fp32 rel {np.linalg.norm(outs[i] - ref) / np.linalg.norm(ref):.2e}  plan-mean max|diff| {np.abs(outs[i][PLAN] - ref[PLAN]).max():.3f}  action_t {at}', flush=True)
print('CHECK4_DONE')
