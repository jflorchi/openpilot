#!/usr/bin/env python3
"""Load a thneed exactly the way selfdrive/modeld/thneed/serialize.cc does (programs from source without options,
images created from buffers with the stored row pitch, CL_HALF_FLOAT) and run it on real frames.

Inputs/outputs come from jdat['inputs'] / jdat['outputs'] (what the upstream tinygrad-era loader reads).
usage: thneed_check.py model.thneed real_frames.npz
"""
import json
import os
import struct
import sys
import time

sys.path[:0] = ['/data/c2port/pylib']
import numpy as np
import pyopencl as cl

def load_thneed(path):
  """-> (inputs {name: (cl_mem, size)}, run(feed) -> (output, gpu_ms))"""
  global ctx, q, fmt
  raw = open(path, 'rb').read()
  jsz = struct.unpack('I', raw[:4])[0]
  jdat = json.loads(raw[4:4 + jsz].decode('latin_1'))
  ptr = 4 + jsz

  ctx = cl.create_some_context(interactive=False)
  q = cl.CommandQueue(ctx, properties=cl.command_queue_properties.PROFILING_ENABLE)
  fmt = cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.HALF_FLOAT)


  import ctypes
  _libcl = ctypes.CDLL('/system/vendor/lib64/libOpenCL.so')
  class _Fmt(ctypes.Structure):
    _fields_ = [('order', ctypes.c_uint), ('type', ctypes.c_uint)]
  class _Desc(ctypes.Structure):
    _fields_ = [('image_type', ctypes.c_uint), ('width', ctypes.c_size_t), ('height', ctypes.c_size_t), ('depth', ctypes.c_size_t),
                ('array_size', ctypes.c_size_t), ('row_pitch', ctypes.c_size_t), ('slice_pitch', ctypes.c_size_t),
                ('num_mip_levels', ctypes.c_uint), ('num_samples', ctypes.c_uint), ('buffer', ctypes.c_void_p)]
  _libcl.clCreateImage.restype = ctypes.c_void_p
  _libcl.clCreateImage.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(_Fmt), ctypes.POINTER(_Desc), ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

  def image_from_buffer(buf, width, height, row_pitch):
    """clCreateImage(IMAGE2D, CL_RGBA/CL_HALF_FLOAT, desc.buffer = buf), exactly as serialize.cc (pyopencl won't do pitch+buffer)."""
    fmt_ = _Fmt(int(cl.channel_order.RGBA), int(cl.channel_type.HALF_FLOAT))
    desc = _Desc(int(cl.mem_object_type.IMAGE2D), width, height, 0, 0, row_pitch, 0, 0, 0, buf.int_ptr)
    err = ctypes.c_int(0)
    h = _libcl.clCreateImage(ctx.int_ptr, int(cl.mem_flags.READ_WRITE), ctypes.byref(fmt_), ctypes.byref(desc), None, ctypes.byref(err))
    assert err.value == 0 and h, f'clCreateImage failed {err.value} ({width}x{height} pitch {row_pitch})'
    return cl.Image.from_int_ptr(h, retain=False)

  NATIVE = int(os.environ.get('NATIVE_IMAGES', 0))  # 1: images not backed by buffers (tiled), weights uploaded unpadded
  mem = {}
  for o in jdat['objects']:
    sz = o['size']
    if NATIVE and o['arg_type'] == 'image2d_t' and not o.get('buffer_id'):
      img = cl.Image(ctx, cl.mem_flags.READ_WRITE, fmt, shape=(o['width'], o['height']))
      if o['needs_load']:
        rows = np.frombuffer(raw, np.float16, sz // 2, ptr).reshape(o['height'], o['row_pitch'] // 2)
        cl.enqueue_copy(q, img, np.ascontiguousarray(rows[:, :o['width'] * 4]), origin=(0, 0), region=(o['width'], o['height']))
        ptr += sz
      mem[o['id']] = img
      continue
    if o.get('buffer_id'):
      buf = mem[o['buffer_id']]
    elif o['needs_load']:
      buf = cl.Buffer(ctx, cl.mem_flags.READ_WRITE | cl.mem_flags.COPY_HOST_PTR, hostbuf=np.frombuffer(raw, np.uint8, sz, ptr).copy())
      ptr += sz
    else:
      buf = cl.Buffer(ctx, cl.mem_flags.READ_WRITE, sz)
    if o['arg_type'] in ('image2d_t', 'image1d_t'):
      # same as serialize.cc: CL_HALF_FLOAT image backed by the buffer, with the stored row pitch
      assert o['arg_type'] == 'image2d_t', o['arg_type']
      buf = image_from_buffer(buf, o['width'], o['height'], o['row_pitch'])
    mem[o['id']] = buf

  programs = {name: cl.Program(ctx, src).build() for name, src in jdat['programs'].items()}
  assert not jdat.get('binaries'), 'expected a source-only thneed'

  kernels = []
  for k in jdat['kernels']:
    kern = getattr(programs[k['name']], k['name'])
    for i, (a, s) in enumerate(zip(k['args'], k['args_size'])):
      if s == 8:
        kern.set_arg(i, mem[a])
      elif a == '':
        kern.set_arg(i, cl.LocalMemory(s))
      elif s == 4:
        kern.set_arg(i, np.uint32(struct.unpack('I', a.encode('latin_1'))[0]))
      elif s == 2:
        kern.set_arg(i, np.uint16(struct.unpack('H', a.encode('latin_1'))[0]))
      else:
        raise ValueError(f'arg size {s}')
    kernels.append((kern, k['global_work_size'], k['local_work_size']))

  inputs = {i['name']: (mem[i['buffer_id']], i['size']) for i in jdat['inputs']}
  out_buf, out_size = mem[jdat['outputs'][0]['buffer_id']], jdat['outputs'][0]['size']
  print('inputs:', {k: v[1] for k, v in inputs.items()}, 'output bytes:', out_size, 'kernels:', len(kernels))

  def run(feed):
    for name, arr in feed.items():
      buf, size = inputs[name]
      a = np.ascontiguousarray(arr, dtype=np.float32)
      assert a.nbytes == size, (name, a.nbytes, size)
      cl.enqueue_copy(q, buf, a)
    events = [cl.enqueue_nd_range_kernel(q, kern, gws, lws) for kern, gws, lws in kernels]
    out = np.empty(out_size // 4, np.float32)
    cl.enqueue_copy(q, out, out_buf)
    q.finish()
    return out, sum(e.profile.end - e.profile.start for e in events) / 1e6

  run.keepalive = (mem, programs, kernels)  # pyopencl set_arg holds no Python refs; freed images crash the driver
  return inputs, run


if __name__ == '__main__':
  path, frames = sys.argv[1], sys.argv[2]
  inputs, run = load_thneed(path)
  d = np.load(frames)
  plan = slice(1576, 1576 + 495)
  for i in range(2):
    imgs = d[f'imgs{i}'].astype(np.float32)
    if int(os.environ.get('NORMALIZED', 0)): imgs = (imgs - 127.5) / 63.75  # final model: modeld normalizes
    if inputs['imgs'][1] == 128 * 256 * 32 * 4:  # channel-last model input: per pixel 4 frames x (6 channels + 2 unused)
      fr = imgs.reshape(4, 6, 128, 256).transpose(2, 3, 0, 1)
      imgs = np.concatenate([fr, np.zeros((128, 256, 4, 2), np.float32)], -1).reshape(1, 128, 256, 32)
    feed = {'imgs': imgs, 'desire_tokens': d[f'desire{i}'], 'feat_hist': d[f'feat{i}'],
            'traffic_convention': np.array([[1, 0]], np.float32), 'action_t': np.array([[0.275, 0.525]], np.float32)}
    feed = {k: v for k, v in feed.items() if k in inputs}
    out, _ = run(feed)
    ref = d[f'out{i}'][0]
    print(f'real frame {i}: rel L2 {np.linalg.norm(out - ref) / np.linalg.norm(ref):.2e}  plan-mean max|diff| {np.abs(out[plan] - ref[plan]).max():.4f}  finite {np.isfinite(out).all()}')
  gpu = [run(feed)[1] for _ in range(20)]
  t0 = time.monotonic()
  for _ in range(20):
    run(feed)
  wall = (time.monotonic() - t0) / 20 * 1000
  print(f'GPU time per run: median {np.median(gpu):.2f} ms  min {min(gpu):.2f}  max {max(gpu):.2f} | python wall {wall:.1f} ms')
  print('THNEED_CHECK_DONE')
