#!/usr/bin/env python3
"""Per-kernel GPU time of a thneed. usage: kprof.py model.thneed out.json"""
import json, os, sys, time
sys.path[:0] = ['/data/c2port']
import numpy as np
import pyopencl as cl
import thneed_check as tc

path, out_path = sys.argv[1], sys.argv[2]
inputs, run = tc.load_thneed(path)
mem, programs, kernels = run.keepalive
q = tc.q
raw = open(path, 'rb').read()
import struct
jsz = struct.unpack('I', raw[:4])[0]
jdat = json.loads(raw[4:4 + jsz].decode('latin_1'))
del raw

rng = np.random.default_rng(0)
for name, (buf, size) in inputs.items():
  cl.enqueue_copy(q, buf, rng.standard_normal(size // 4).astype(np.float32) * 0.5)
q.finish()

N = int(os.environ.get('RUNS', 30))
times = np.zeros((N + 5, len(kernels)))
spans = []
for r in range(N + 5):
  ev = [cl.enqueue_nd_range_kernel(q, k, g, l) for k, g, l in kernels]
  q.finish()
  times[r] = [(e.profile.end - e.profile.start) / 1e6 for e in ev]
  spans.append((ev[-1].profile.end - ev[0].profile.start) / 1e6)
times, spans = times[5:], spans[5:]

med = np.median(times, 0)
recs = []
for i, (k, (kern, g, l)) in enumerate(zip(jdat['kernels'], kernels)):
  src = jdat['programs'][k['name']]
  recs.append({'i': i, 'name': k['name'], 'gws': list(g), 'lws': list(l) if l else None,
               'ms': float(med[i]), 'ms_p90': float(np.percentile(times[:, i], 90)), 'src_len': len(src),
               'src': src})
json.dump({'sum_ms': float(np.median(times.sum(1))), 'span_ms': float(np.median(spans)), 'runs': N, 'kernels': recs},
          open(out_path, 'w'))
print(f'kernels {len(kernels)}  sum of kernel time median {np.median(times.sum(1)):.2f} ms  '
      f'first-start..last-end span {np.median(spans):.2f} ms')
print('KPROF_DONE')
