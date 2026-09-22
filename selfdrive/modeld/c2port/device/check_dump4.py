#!/usr/bin/env python3
"""Validate a C2PORT_DUMP from the live bench:
 - history: for every frame, feature token 7's source offset == image slot t-4's source offset (both advance per frame)
 - accuracy: C++ outputs vs tinygrad fp32 (validated reference) on the dumped inputs"""
import sys
import numpy as np
sys.path[:0] = ['/data/c2port', '/data/c2port/tinygrad_repo', '/data/c2port/pylib']
ddir, n = sys.argv[1], int(sys.argv[2])
FRAME, H = 6 * 128 * 256, slice(1064, 1576)
ld = lambda i, w: np.fromfile(f'{ddir}/{i:03d}_{w}.bin', np.float32)
outs = [ld(i, 'out') for i in range(n)]
feats = [ld(i, 'feat').reshape(8, 512) for i in range(n)]
imgs = [ld(i, 'imgs').reshape(4, FRAME) for i in range(n)]
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
import onnx
from extra.onnx import get_run_onnx
from tinygrad.tensor import Tensor
Tensor.no_grad = True
run = get_run_onnx(onnx.load(open('/data/c2port/supercombo_c2_final.onnxpkl', 'rb')))
for i in (0, 8, 16, n - 1):
  feed = {'imgs': imgs[i].reshape(1, 24, 128, 256), 'desire_tokens': ld(i, 'desire').reshape(1, 200), 'feat_hist': feats[i],
          'traffic_convention': ld(i, 'traffic').reshape(1, 2), 'action_t': np.array([[0.275, 0.525]], np.float32)}
  ref = run(feed)['outputs'].numpy()[0]
  print(f'frame {i}: C++ vs fp32 rel {np.linalg.norm(outs[i] - ref) / np.linalg.norm(ref):.2e}  plan-mean max|diff| {np.abs(outs[i][1576:2071] - ref[1576:2071]).max():.3f}', flush=True)
print('CHECK4_DONE')
