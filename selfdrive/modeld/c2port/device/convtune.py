#!/usr/bin/env python3
"""Register-blocked replacements for the general image convs of a tinygrad thneed.

tinygrad's image_conv gives each thread 4 output pixels x 1 packed output channel: per input channel and filter tap it
reads 4 input and 4 weight texels for 64 FMAs, and the convs run texture-bound at ~50 GMAC/s. Here a thread can take
NO output pixels x OC packed output channels (each input read is shared by OC channels, each weight read by NO pixels),
optionally with fp16 math. The kernel signature and the fused epilogue (_ewop) are kept, so only the program source and
the work sizes in the thneed header change. Depthwise, batched and split-K kernels are left alone.

For each candidate kernel this runs the model up to it on real frames, times the original and the variants (with a
few workgroup sizes), checks each against the original's output and keeps the fastest one that matches. Writes two
models: out.thneed (fp32 math only) and out_half.thneed (fp16 math allowed).

usage: convtune.py in.thneed real_frames.npz out.thneed [report.json]
"""
import json
import os
import struct
import sys

sys.path[:0] = ['/data/c2port', '/data/c2port/pylib']
import numpy as np
import pyopencl as cl
import thneed_check as tc

MIN_GAIN = 0.95         # replace only if at least 5% faster
MAX_REL_ERR = 2e-3      # fp32 math, vs the original kernel's fp16 output
MAX_REL_ERR_HALF = 4e-3
RUNS = 15
VARIANTS = [(no, oc, h) for h in (0, 1) for no in (4, 8) for oc in (1, 2) if (no, oc, h) != (4, 1, 0)]

BODY_START = '  float4 outputValues[NUM_OUTPUTS];'
BODY_END = '  // output to memory (with binop)'
SHAPE_KEYS = ('filterSizeX', 'filterSizeY', 'paddingX', 'paddingY', 'strideX', 'strideY', 'dilationX', 'dilationY',
              'numPackedInputChannelsForGroup', 'totalNumPackedInputChannels', 'numPackedOutputChannelsForGroup',
              'totalNumPackedOutputChannels', 'numOutputColumns', 'numOutputRows')


def shape(src, key):
  i = src.index(f'short {key} = ') + len(f'short {key} = ')
  return int(src[i:src.index(';', i)])


def eligible(src):
  if any(f'#define {d}' in src for d in ('DEPTHWISE', 'BATCH')) or 'partial[' in src:
    return False
  if BODY_START not in src or BODY_END not in src or '#define NUM_OUTPUTS 4\n' not in src:
    return False
  return all(f'short {k} = ' in src for k in SHAPE_KEYS)


def tuned_source(src, NO, OC, half):
  T, rd = ('half4', 'read_imageh') if half else ('float4', 'read_imagef')
  acc = [[f'a{oc}_{i}' for i in range(NO)] for oc in range(OC)]
  L = []
  L.append(f'  {T} ' + ', '.join(f'{a} = ({T})(0)' for row in acc for a in row) + ';')
  L.append(f'''  short pocBase = mul24((short)get_global_id(0), (short){OC});
  short groupNum = pocBase / numPackedOutputChannelsForGroup;
  short startPackedInputChannel = mul24(groupNum, numPackedInputChannelsForGroup);
  short startOutputColumn = mul24((short)get_global_id(1), NUM_OUTPUTS);
  short startX = mad24(mad24(startOutputColumn, strideX, -paddingX), totalNumPackedInputChannels, startPackedInputChannel);
  short strideWithChannels = mul24(strideX, totalNumPackedInputChannels);
  short dilatedStepX = mul24(totalNumPackedInputChannels, dilationX);
  short outputRow = get_global_id(2);
  int2 inputLocation;
  inputLocation.y = mad24(outputRow, strideY, -paddingY);
  int wx = 0;
  for (short rfRow = 0; rfRow < filterSizeY; ++rfRow) {{
    for (short packedInputChannel = 0; packedInputChannel < numPackedInputChannelsForGroup; ++packedInputChannel) {{
      short startXForChannel = startX + packedInputChannel;
      for (short rfColumn = 0; rfColumn < filterSizeX; ++rfColumn) {{
        inputLocation.x = mad24(rfColumn, dilatedStepX, startXForChannel);''')
  for i in range(NO):
    L.append(f'        {T} in{i} = {rd}(input, smp, inputLocation); inputLocation.x += strideWithChannels;')
  for oc in range(OC):
    for j in range(4):
      L.append(f'        {T} w{oc}_{j} = {rd}(weights, smp, (int2)(wx + {j}, pocBase + {oc}));')
    for i in range(NO):
      a = acc[oc][i]
      L.append(f'        {a} += ({T})(dot(in{i}, w{oc}_0), dot(in{i}, w{oc}_1), dot(in{i}, w{oc}_2), dot(in{i}, w{oc}_3));')
  L.append('''        wx += 4;
      }
    }
    inputLocation.y += dilationY;
  }
''')
  body = '\n'.join(L)

  head = src[:src.index(BODY_START)].replace('#define NUM_OUTPUTS 4\n', f'#define NUM_OUTPUTS {NO}\n')
  epilogue = src[src.index(BODY_END):].rstrip()
  assert epilogue.endswith('}')
  epilogue = epilogue[:-1]
  tail = ['  float4 outputValues[NUM_OUTPUTS];']
  for oc in range(OC):
    tail.append('  {')
    tail.append(f'  short packedOutputChannel = pocBase + {oc};')
    for i in range(NO):
      tail.append(f'  outputValues[{i}] = convert_float4({acc[oc][i]});')
    tail.append(epilogue)
    tail.append('  }')
  return head + body + '\n'.join(tail) + '\n}\n'


def lws_options(gws, maxwg):
  """A few workgroup sizes that divide gws (OpenCL 1.2; serialize.cc always passes one)."""
  def divs(n):
    return [d for d in (1, 2, 4, 8, 16, 32, 64) if n % d == 0]
  opts = set()
  for total in (32, 64, 128, 256):
    if total > maxwg:
      continue
    for a in divs(gws[0]):
      for b in divs(gws[1]):
        c = total // (a * b)
        if c >= 1 and a * b * c == total and gws[2] % c == 0:
          opts.add((a, b, c))
  # keep the ones closest to the original's shape: favour dim0 (channels), then rows
  return sorted(opts, key=lambda o: (-o[0], -o[2]))[:24]


def main():
  path, frames, out_path = sys.argv[1:4]
  report_path = sys.argv[4] if len(sys.argv) > 4 else None
  os.environ.setdefault('NATIVE_IMAGES', '1')
  inputs, run = tc.load_thneed(path)
  mem, programs, kernels = run.keepalive
  ctx, q = tc.ctx, tc.q
  dev = ctx.devices[0]

  raw = open(path, 'rb').read()
  jsz = struct.unpack('I', raw[:4])[0]
  jdat = json.loads(raw[4:4 + jsz].decode('latin_1'))
  assert json.dumps(jdat, ensure_ascii=False).encode('latin_1') == raw[4:4 + jsz], 'header does not round-trip'
  objs = {o['id']: o for o in jdat['objects']}

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

  results = {}  # per program: base ms, best per variant
  keep = []
  for k, (kern, gws, lws) in zip(jdat['kernels'], kernels):
    src = jdat['programs'][k['name']]
    if not (k['name'].startswith('image_conv') and eligible(src)):
      cl.enqueue_nd_range_kernel(q, kern, gws, lws)
      continue
    s = {key: shape(src, key) for key in SHAPE_KEYS}
    base_ms = time_kernel(kern, gws, lws)
    ref = read_out(k)
    ref_norm = np.linalg.norm(ref) + 1e-12
    res = results.setdefault(k['name'], {'base': 0.0, 'instances': 0, 'shape': s, 'gws': list(gws),
                                         'lws': list(lws) if lws else None, 'var': {}})
    res['base'] += base_ms
    res['instances'] += 1
    assert res['instances'] == 1, 'program used by several kernels'
    for NO, OC, half in VARIANTS:
      if OC > 1 and s['numPackedOutputChannelsForGroup'] % OC:
        continue
      ngws = (gws[0] // OC, -(-s['numOutputColumns'] // NO), gws[2])
      if gws[0] % OC or gws[2] != s['numOutputRows']:
        continue
      try:
        prog = cl.Program(ctx, tuned_source(src, NO, OC, half)).build()
      except Exception as e:
        print(f'{k["name"]} NO={NO} OC={OC} half={half}: build {type(e).__name__} {str(e)[:300]}')
        continue
      nk = getattr(prog, k['name'])
      set_args(nk, k)
      keep.append((prog, nk))
      maxwg = nk.get_work_group_info(cl.kernel_work_group_info.WORK_GROUP_SIZE, dev)
      best = None
      for nlws in lws_options(ngws, maxwg):
        try:
          ms = time_kernel(nk, ngws, nlws)
        except Exception as e:
          continue
        if best is None or ms < best[0]:
          best = (ms, nlws)
      if best is None:
        continue
      cl.enqueue_nd_range_kernel(q, nk, ngws, best[1])
      err = float(np.linalg.norm(read_out(k) - ref) / ref_norm)
      res['var'][f'{NO},{OC},{half}'] = {'ms': best[0], 'lws': list(best[1]), 'gws': list(ngws),
                                         'err': err, 'maxwg': maxwg}
    cl.enqueue_nd_range_kernel(q, kern, gws, lws)  # restore the original output for the kernels downstream
    q.finish()
    v = res['var']
    bestv = min(v, key=lambda x: v[x]['ms']) if v else None
    print(f'{k["name"]:16s} {s["filterSizeX"]}x{s["filterSizeY"]}/{s["strideX"]} in {s["numPackedInputChannelsForGroup"]:3d} '
          f'out {s["totalNumPackedOutputChannels"]:3d} {s["numOutputColumns"]}x{s["numOutputRows"]}: {base_ms:.3f} ms | ' +
          ' '.join(f'{x}:{v[x]["ms"]:.3f}/{v[x]["err"]:.0e}' for x in v), flush=True)

  outputs = {out_path: False, out_path.replace('.thneed', '_half.thneed'): True}
  report = {'results': results}
  for opath, allow_half in outputs.items():
    chosen = {}
    for name, res in results.items():
      ok = {x: v for x, v in res['var'].items()
            if (allow_half or x.endswith(',0')) and v['err'] <= (MAX_REL_ERR_HALF if x.endswith(',1') else MAX_REL_ERR)}
      if ok:
        b = min(ok, key=lambda x: ok[x]['ms'])
        if ok[b]['ms'] < MIN_GAIN * res['base']:
          chosen[name] = b
    before = sum(r['base'] for r in results.values())
    after = sum(results[n]['var'][chosen[n]]['ms'] if n in chosen else r['base'] for n, r in results.items())
    print(f'{opath}: targets {before:.2f} ms -> {after:.2f} ms ({len(chosen)}/{len(results)} programs replaced)')
    report['chosen_half' if allow_half else 'chosen'] = chosen

    jd = json.loads(raw[4:4 + jsz].decode('latin_1'))
    for name, x in chosen.items():
      NO, OC, half = map(int, x.split(','))
      jd['programs'][name] = tuned_source(jd['programs'][name], NO, OC, half)
    for k in jd['kernels']:
      if k['name'] in chosen:
        v = results[k['name']]['var'][chosen[k['name']]]
        k['global_work_size'] = v['gws']
        k['local_work_size'] = v['lws']
    hdr = json.dumps(jd, ensure_ascii=False).encode('latin_1')
    with open(opath, 'wb') as f:
      f.write(struct.pack('I', len(hdr)))
      f.write(hdr)
      f.write(raw[4 + jsz:])
  if report_path:
    json.dump(report, open(report_path, 'w'), indent=1)
  print('CONVTUNE_DONE')


if __name__ == '__main__':
  main()
