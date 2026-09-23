#!/usr/bin/env python3
"""Split-K replacements for the weight-bound 1x1 convs of a tinygrad thneed.

tinygrad's image_conv gives each thread one packed output channel and loops over every input channel, so a conv with a
single output pixel (the policy / temporal matmuls) runs as a handful of workgroups and streams its weights at ~5 GB/s.
Here S threads share one output channel, each takes every S-th input channel, and the partial sums are added in local
memory. The kernel signature and the fused epilogue (_ewop) are kept, so the thneed's args are unchanged; only the
program source and the work sizes are rewritten.

For each candidate kernel this runs the model up to it on real frames, times the original and a few (S, workgroup)
configs, checks each against the original's output and keeps the fastest one that matches.

usage: splitk.py in.thneed real_frames.npz out.thneed [report.json]
"""
import json
import os
import struct
import sys
import time

sys.path[:0] = ['/data/c2port', '/data/c2port/pylib']
import numpy as np
import pyopencl as cl
import thneed_check as tc

MIN_PACKED_IN = 16      # smaller convs take ~10 us either way
MIN_GAIN = 0.95         # replace only if at least 5% faster
MAX_REL_ERR = 2e-3      # vs the original kernel's fp16 output
RUNS = 15
SPLITS = (4, 8, 16, 32, 64)
WGS = (64, 128, 256)

BODY_START = '  float4 outputValues[NUM_OUTPUTS];'
BODY_END = '  // output to memory (with binop)'


def shape(src, key):
  i = src.index(f'short {key} = ') + len(f'short {key} = ')
  return int(src[i:src.index(';', i)])


def eligible(src):
  if '#define ONLY_1X1_CONV' not in src or '#define DEPTHWISE' in src or '#define BATCH' in src:
    return False
  if BODY_START not in src or BODY_END not in src:
    return False
  s = {k: shape(src, k) for k in ('filterSizeX', 'filterSizeY', 'strideX', 'strideY', 'paddingX', 'paddingY',
                                  'numOutputColumns', 'numPackedInputChannelsForGroup')}
  return (s['filterSizeX'] == s['filterSizeY'] == s['strideX'] == s['strideY'] == 1 and s['paddingX'] == s['paddingY'] == 0
          and s['numOutputColumns'] == 1 and s['numPackedInputChannelsForGroup'] >= MIN_PACKED_IN)


def splitk_source(src, S, WG):
  """Same kernel, split-K body. Work: dim0 = packed output channel * S (S consecutive lanes per channel), dim2 = row."""
  body = f'''  float4 outputValues[NUM_OUTPUTS];
  for (short i = 1; i < NUM_OUTPUTS; ++i) {{
    outputValues[i] = (float4)(0, 0, 0, 0);
  }}

  const int lid = get_local_id(0);
  const short k = lid % {S};
  short packedOutputChannel = get_global_id(0) / {S};
  short startOutputColumn = mul24((short)get_global_id(1), NUM_OUTPUTS);
  short groupNum = (packedOutputChannel / numPackedOutputChannelsForGroup);
  short startPackedInputChannel = mul24(groupNum, numPackedInputChannelsForGroup);
  short outputRow = get_global_id(2);

  int2 inputLocation;
  inputLocation.y = outputRow;
  int2 weightLocation;
  weightLocation.y = packedOutputChannel;

  float4 acc = (float4)(0, 0, 0, 0);
  for (short packedInputChannel = k; packedInputChannel < numPackedInputChannelsForGroup; packedInputChannel += {S}) {{
    inputLocation.x = startPackedInputChannel + packedInputChannel;
    float4 inputValues = read_imagef(input, smp, inputLocation);
    weightLocation.x = packedInputChannel * 4;
    float4 w0 = read_imagef(weights, smp, weightLocation); ++weightLocation.x;
    float4 w1 = read_imagef(weights, smp, weightLocation); ++weightLocation.x;
    float4 w2 = read_imagef(weights, smp, weightLocation); ++weightLocation.x;
    float4 w3 = read_imagef(weights, smp, weightLocation);
    acc.x += inputValues.x * w0.x; acc.x += inputValues.y * w0.y; acc.x += inputValues.z * w0.z; acc.x += inputValues.w * w0.w;
    acc.y += inputValues.x * w1.x; acc.y += inputValues.y * w1.y; acc.y += inputValues.z * w1.z; acc.y += inputValues.w * w1.w;
    acc.z += inputValues.x * w2.x; acc.z += inputValues.y * w2.y; acc.z += inputValues.z * w2.z; acc.z += inputValues.w * w2.w;
    acc.w += inputValues.x * w3.x; acc.w += inputValues.y * w3.y; acc.w += inputValues.z * w3.z; acc.w += inputValues.w * w3.w;
  }}

  __local float4 partial[{WG}];
  partial[lid] = acc;
  barrier(CLK_LOCAL_MEM_FENCE);
  for (short s = {S // 2}; s > 0; s >>= 1) {{
    if (k < s) partial[lid] += partial[lid + s];
    barrier(CLK_LOCAL_MEM_FENCE);
  }}
  if (k != 0) return;
  outputValues[0] = partial[lid];

'''
  return src[:src.index(BODY_START)] + body + src[src.index(BODY_END):]


def configs(packed_in, packed_out):
  for S in SPLITS:
    if S > packed_in:
      continue
    for WG in WGS:
      W = WG // S
      while W > 1 and packed_out % W:
        W //= 2
      if W < 1:
        continue
      yield S, W * S


def main():
  path, frames, out_path = sys.argv[1:4]
  report_path = sys.argv[4] if len(sys.argv) > 4 else None
  os.environ.setdefault('NATIVE_IMAGES', '1')
  inputs, run = tc.load_thneed(path)
  mem, programs, kernels = run.keepalive
  ctx, q = tc.ctx, tc.q

  raw = open(path, 'rb').read()
  jsz = struct.unpack('I', raw[:4])[0]
  jdat = json.loads(raw[4:4 + jsz].decode('latin_1'))
  # written like tinygrad's openpilot/compile.py; the ids are raw pointer bytes, so the round trip must be exact
  assert json.dumps(jdat, ensure_ascii=False).encode('latin_1') == raw[4:4 + jsz], 'header does not round-trip'
  objs = {o['id']: o for o in jdat['objects']}

  # real frame, same feed as thneed_check (final model: modeld normalizes)
  d = np.load(frames)
  imgs = (d['imgs0'].astype(np.float32) - 127.5) / 63.75
  fr = imgs.reshape(4, 6, 128, 256).transpose(2, 3, 0, 1)
  imgs = np.concatenate([fr, np.zeros((128, 256, 4, 2), np.float32)], -1).reshape(1, 128, 256, 32)
  feed = {'imgs': imgs, 'desire_tokens': d['desire0'], 'feat_hist': d['feat0'],
          'traffic_convention': np.array([[1, 0]], np.float32), 'action_t': np.array([[0.275, 0.525]], np.float32)}
  for name, (buf, size) in inputs.items():
    cl.enqueue_copy(q, buf, np.ascontiguousarray(feed[name], dtype=np.float32))

  def set_args(kern, k):
    for i, (a, s) in enumerate(zip(k['args'], k['args_size'])):
      if s == 8:
        kern.set_arg(i, mem[a])
      elif a == '':
        kern.set_arg(i, cl.LocalMemory(s))
      elif s == 4:
        kern.set_arg(i, np.uint32(struct.unpack('I', a.encode('latin_1'))[0]))
      elif s == 2:
        kern.set_arg(i, np.uint16(struct.unpack('H', a.encode('latin_1'))[0]))

  def read_out(k):
    o = objs[k['args'][0]]
    out = np.empty((o['height'], o['width'], 4), np.float16)
    cl.enqueue_copy(q, out, mem[k['args'][0]], origin=(0, 0), region=(o['width'], o['height']))
    return out.astype(np.float32)

  def time_kernel(kern, gws, lws):
    t = []
    for _ in range(RUNS):
      e = cl.enqueue_nd_range_kernel(q, kern, gws, lws)
      e.wait()
      t.append((e.profile.end - e.profile.start) / 1e6)
    return float(np.median(t))

  # per program: {config: [total ms over instances, max rel err]}
  results = {}
  keep = []
  for idx, (k, (kern, gws, lws)) in enumerate(zip(jdat['kernels'], kernels)):
    src = jdat['programs'][k['name']]
    if not (k['name'].startswith('image_conv') and eligible(src)):
      cl.enqueue_nd_range_kernel(q, kern, gws, lws)
      continue
    base_ms = time_kernel(kern, gws, lws)  # original leaves its output in place
    ref = read_out(k)
    ref_norm = np.linalg.norm(ref) + 1e-12
    res = results.setdefault(k['name'], {'base': 0.0, 'cfg': {}, 'instances': 0,
                                         'packed_in': shape(src, 'numPackedInputChannelsForGroup'),
                                         'packed_out': shape(src, 'totalNumPackedOutputChannels'),
                                         'rows': shape(src, 'numOutputRows')})
    res['base'] += base_ms
    res['instances'] += 1
    for S, WG in configs(res['packed_in'], res['packed_out']):
      key = f'{S},{WG}'
      if key not in res['cfg']:
        res['cfg'][key] = {'ms': 0.0, 'err': 0.0, 'ok': True}
      c = res['cfg'][key]
      if not c['ok']:
        continue
      try:
        prog = cl.Program(ctx, splitk_source(src, S, WG)).build()
        nk = getattr(prog, k['name'])
        set_args(nk, k)
        keep.append((prog, nk))
        ngws = (gws[0] * S, gws[1], gws[2])
        nlws = (WG, 1, 1)
        ms = time_kernel(nk, ngws, nlws)
        err = float(np.linalg.norm(read_out(k) - ref) / ref_norm)
      except Exception as e:
        print(f'{k["name"]} S={S} WG={WG}: {type(e).__name__} {str(e)[:200]}')
        c['ok'] = False
        continue
      c['ms'] += ms
      c['err'] = max(c['err'], err)
      if err > MAX_REL_ERR:
        c['ok'] = False
    cl.enqueue_nd_range_kernel(q, kern, gws, lws)  # restore the original output for the kernels downstream
    q.finish()

  # choose per program, then patch
  chosen = {}
  for name, res in results.items():
    ok = {c: v for c, v in res['cfg'].items() if v['ok']}
    best = min(ok, key=lambda c: ok[c]['ms']) if ok else None
    res['best'] = best
    if best and ok[best]['ms'] < MIN_GAIN * res['base']:
      chosen[name] = tuple(int(x) for x in best.split(','))
    bms = ok[best]['ms'] if best else float('nan')
    print(f'{name:16s} x{res["instances"]} in {res["packed_in"]:3d} out {res["packed_out"]:3d} rows {res["rows"]}: '
          f'{res["base"]:.3f} ms -> {bms:.3f} ms (S,WG={best}, err {ok[best]["err"] if best else 0:.1e})'
          f'{"  REPLACED" if name in chosen else ""}')
  before = sum(r['base'] for r in results.values())
  after = sum(r['cfg'][f'{chosen[n][0]},{chosen[n][1]}']['ms'] if n in chosen else r['base'] for n, r in results.items())
  print(f'targets: {before:.2f} ms -> {after:.2f} ms ({len(chosen)}/{len(results)} programs replaced)')

  for name, (S, WG) in chosen.items():
    jdat['programs'][name] = splitk_source(jdat['programs'][name], S, WG)
  for k in jdat['kernels']:
    if k['name'] in chosen:
      S, WG = chosen[k['name']]
      k['global_work_size'][0] *= S
      k['local_work_size'] = [WG, 1, 1]
  hdr = json.dumps(jdat, ensure_ascii=False).encode('latin_1')
  with open(out_path, 'wb') as f:
    f.write(struct.pack('I', len(hdr)))
    f.write(hdr)
    f.write(raw[4 + jsz:])
  if report_path:
    json.dump({'results': results, 'chosen': chosen}, open(report_path, 'w'), indent=1)
  print('SPLITK_DONE')


if __name__ == '__main__':
  main()
