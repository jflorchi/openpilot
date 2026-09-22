#!/usr/bin/env python3
"""Rewrite openpilot's current driving_supercombo.onnx into a graph SNPE 1.43 can plausibly convert.

- History buffers move to the host (modeld). New inputs:
    imgs          [1,24,128,256] f32  narrow(t-4, t) + wide(t-4, t), 6 YUV channels each, raw 0..255
    desire_tokens [1,200]        f32  100-step desire history, max-pooled over groups of 4
    feat_hist     [1,8,512]      f32  features from t-32, t-28, ..., t-4 (every 4th of the 96-step history)
    traffic_convention [1,2], action_t [1,2]
  The feature to push into the history each step is outputs[1064:1576].
- fp16 -> fp32, Casts removed.
- Gelu(tanh) and ReduceL2 decomposed fp16-safely (no intermediate past 65504), LayerNormalization decomposed; Where(mask) -> Add(const); Expand dropped (broadcast);
  scalar Gather -> Slice + Squeeze.
- Grouped convs with a channel multiplier (tinygrad 2022 image backend gets them wrong) -> 1x1 duplicate + depthwise.
- Down-converted to opset 11 (axes as attributes).
"""
import sys
from collections import defaultdict

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto, shape_inference

SRC, DST = sys.argv[1], sys.argv[2]
OPSET = 11

m = onnx.load(SRC)
g = m.graph
uid = [0]
def name(base):
  uid[0] += 1
  return f'c2port/{base}_{uid[0]}'

inits = {t.name: t for t in g.initializer}
consts = {n.output[0]: n for n in g.node if n.op_type == 'Constant'}

def const_value(t):
  if t in inits:
    return numpy_helper.to_array(inits[t])
  if t in consts:
    return numpy_helper.to_array(consts[t].attribute[0].t)
  raise KeyError(t)

def add_init(arr, base):
  n = name(base)
  g.initializer.append(numpy_helper.from_array(np.asarray(arr), n))
  return n

def _is_multiplier_conv(n):
  group = next((a.i for a in n.attribute if a.name == 'group'), 1)
  w = inits[n.input[1]].dims
  return group > 1 and w[1] == 1 and w[0] > group

new_nodes = []
def rename_input(old, new):
  for n in g.node:
    for i, x in enumerate(n.input):
      if x == old:
        n.input[i] = new

# ---- 1. host-managed history: new inputs and rewiring ----
f32 = TensorProto.FLOAT
old_inputs = {i.name: i for i in g.input}
del g.input[:]
g.input.extend([helper.make_tensor_value_info('imgs', f32, [1, 24, 128, 256]),
                helper.make_tensor_value_info('desire_tokens', f32, [1, 200]),
                helper.make_tensor_value_info('feat_hist', f32, [1, 8, 512]),
                old_inputs['traffic_convention'], old_inputs['action_t']])
rename_input('model/cat', 'imgs')
rename_input('model/p_view', 'desire_tokens')
new_nodes.append(helper.make_node('Concat', ['feat_hist', 'model/hidden_t'], ['c2port/p_index'], axis=1, name='c2port/p_index'))
rename_input('model/p_index', 'c2port/p_index')
keep_out = [o for o in g.output if o.name == 'outputs']
del g.output[:]
g.output.extend(keep_out)

# ---- 2. op rewrites ----
out_nodes = []
for n in list(g.node):
  op = n.op_type
  if op == 'Gelu':
    # tanh-GELU, fp16-safe: 0.5*x*(1+tanh(z)) == x*sigmoid(2z), z = sqrt(2/pi)*(x+0.044715*x^3). The cubic is evaluated on
    # clip(x,-10,10): beyond that sigmoid(2z) is already exactly 0/1 in fp32, and x^3 no longer overflows fp16 (65504).
    x, y = n.input[0], n.output[0]
    xc, x2, x3, a, b, t, sg = (name('gelu') for _ in range(7))
    lo, hi = add_init(np.float32(-10.0), 'lo'), add_init(np.float32(10.0), 'hi')
    c = add_init(np.float32(0.044715), 'c')
    k2 = add_init(np.float32(2 * np.sqrt(2 / np.pi)), 'k2')
    out_nodes += [helper.make_node('Clip', [x, lo, hi], [xc]),
                  helper.make_node('Mul', [xc, xc], [x2]), helper.make_node('Mul', [x2, xc], [x3]),
                  helper.make_node('Mul', [x3, c], [a]), helper.make_node('Add', [xc, a], [b]),
                  helper.make_node('Mul', [b, k2], [t]), helper.make_node('Sigmoid', [t], [sg]),
                  helper.make_node('Mul', [x, sg], [y])]
  elif op == 'LayerNormalization':
    x, scale, bias = n.input[0], n.input[1], n.input[2]
    eps = next((a.f for a in n.attribute if a.name == 'epsilon'), 1e-5)
    axis = next((a.i for a in n.attribute if a.name == 'axis'), -1)
    mu, d, d2, var, ve, sd, nrm, sc = (name('ln') for _ in range(8))
    e = add_init(np.float32(eps), 'eps')
    out_nodes += [helper.make_node('ReduceMean', [x], [mu], axes=[axis], keepdims=1),
                  helper.make_node('Sub', [x, mu], [d]), helper.make_node('Mul', [d, d], [d2]),
                  helper.make_node('ReduceMean', [d2], [var], axes=[axis], keepdims=1),
                  helper.make_node('Add', [var, e], [ve]), helper.make_node('Sqrt', [ve], [sd]),
                  helper.make_node('Div', [d, sd], [nrm]), helper.make_node('Mul', [nrm, scale], [sc]),
                  helper.make_node('Add', [sc, bias], [n.output[0]])]
  elif op == 'ReduceL2':  # 16*sqrt(sum((x/16)^2)): the plain sum of squares reaches ~2e5, past fp16's 65504
    x = n.input[0]
    axes = const_value(n.input[1]).tolist() if len(n.input) > 1 else [a.ints for a in n.attribute if a.name == 'axes'][0]
    keep = next((a.i for a in n.attribute if a.name == 'keepdims'), 1)
    xs, sq, s_, r = name('l2'), name('l2'), name('l2'), name('l2')
    inv, sc = add_init(np.float32(1 / 16), 'l2_inv'), add_init(np.float32(16), 'l2_scale')
    out_nodes += [helper.make_node('Mul', [x, inv], [xs]), helper.make_node('Mul', [xs, xs], [sq]),
                  helper.make_node('ReduceSum', [sq], [s_], axes=list(axes), keepdims=keep),
                  helper.make_node('Sqrt', [s_], [r]), helper.make_node('Mul', [r, sc], [n.output[0]])]
  elif op == 'Where':  # masked_fill(mask, -inf) -> x + (mask ? -1e4 : 0)
    mask = const_value(n.input[0]).astype(bool)
    fill = float(const_value(n.input[1]))
    assert np.isneginf(fill), fill
    bias = add_init(np.where(mask, np.float32(-1e4), np.float32(0)).astype(np.float32), 'attn_mask')
    out_nodes.append(helper.make_node('Add', [n.input[2], bias], [n.output[0]]))
  elif op == 'Expand':  # only used to broadcast the L2 norm; Div broadcasts on its own
    out_nodes.append(helper.make_node('Identity', [n.input[0]], [n.output[0]]))
  elif op == 'Gather':
    idx = int(const_value(n.input[1]))
    axis = next((a.i for a in n.attribute if a.name == 'axis'), 0)
    sl = name('gather')
    s, e = add_init(np.array([idx], np.int64), 's'), add_init(np.array([idx + 1 if idx != -1 else 2**62], np.int64), 'e')
    ax = add_init(np.array([axis], np.int64), 'ax')
    out_nodes += [helper.make_node('Slice', [n.input[0], s, e, ax], [sl]),
                  helper.make_node('Squeeze', [sl], [n.output[0]], axes=[axis])]
  elif op == 'Cast':
    out_nodes.append(helper.make_node('Identity', [n.input[0]], [n.output[0]]))
  elif op in ('ReduceMean', 'ReduceSum', 'ReduceMax') and len(n.input) > 1:
    axes = const_value(n.input[1]).tolist()
    keep = next((a.i for a in n.attribute if a.name == 'keepdims'), 1)
    out_nodes.append(helper.make_node(op, [n.input[0]], list(n.output), axes=axes, keepdims=keep))
  elif op in ('Squeeze', 'Unsqueeze') and len(n.input) > 1:
    out_nodes.append(helper.make_node(op, [n.input[0]], list(n.output), axes=const_value(n.input[1]).tolist()))
  elif op == 'Conv' and n.input[1] in inits and _is_multiplier_conv(n):
    # grouped conv with channel multiplier m (cin/group == 1, cout = m*cin): tinygrad 2022's image backend gets these wrong.
    # Exact rewrite: 1x1 conv duplicating channel i into i*m..i*m+m-1, then a plain depthwise conv (group = cout).
    w = numpy_helper.to_array(inits[n.input[1]])
    group = next(a.i for a in n.attribute if a.name == 'group')
    cout, mult = w.shape[0], w.shape[0] // group
    dup = np.zeros((cout, group, 1, 1), np.float32)
    dup[np.arange(cout), np.arange(cout) // mult] = 1
    dup_w, dup_out = add_init(dup, 'dup_w'), name('dup')
    out_nodes.append(helper.make_node('Conv', [n.input[0], dup_w], [dup_out], group=1, kernel_shape=[1, 1], pads=[0, 0, 0, 0], strides=[1, 1]))
    attrs = {a.name: helper.get_attribute_value(a) for a in n.attribute if a.name not in ('group', 'auto_pad')}
    out_nodes.append(helper.make_node('Conv', [dup_out] + list(n.input[1:]), list(n.output), group=cout, **attrs))
  elif op == 'Reshape':
    keep_attrs = [a for a in n.attribute if a.name != 'allowzero']
    del n.attribute[:]
    n.attribute.extend(keep_attrs)
    out_nodes.append(n)
  else:
    out_nodes.append(n)
del g.node[:]
g.node.extend(out_nodes + new_nodes)

# ---- 3. prune everything not needed for 'outputs', then fold Identity ----
prod = {o: n for n in g.node for o in n.output}
need, stack = set(), ['outputs']
while stack:
  t = stack.pop()
  if t in prod and id(prod[t]) not in need:
    need.add(id(prod[t]))
    stack.extend(prod[t].input)
nodes = [n for n in g.node if id(n) in need]
alias = {n.output[0]: n.input[0] for n in nodes if n.op_type == 'Identity' and n.output[0] != 'outputs'}
def resolve(t):
  while t in alias:
    t = alias[t]
  return t
nodes = [n for n in nodes if not (n.op_type == 'Identity' and n.output[0] in alias)]
for n in nodes:
  for i, x in enumerate(n.input):
    n.input[i] = resolve(x)

# topological sort (new nodes were appended out of order)
avail = {i.name for i in g.input} | {t.name for t in g.initializer} | {''}
ordered, pending = [], nodes
while pending:
  rest = []
  for n in pending:
    (ordered if all(x in avail for x in n.input) else rest).append(n)
    if ordered and ordered[-1] is n:
      avail.update(n.output)
  if len(rest) == len(pending):
    raise RuntimeError(f'cycle or missing input near {rest[0].op_type} {list(rest[0].input)}')
  pending = rest
del g.node[:]
g.node.extend(ordered)

# ---- 4. fp16 -> fp32 for weights and constants ----
used = {x for n in g.node for x in n.input}
for t in list(g.initializer):
  if t.name not in used:
    g.initializer.remove(t)
  elif t.data_type == TensorProto.FLOAT16:
    t.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(t).astype(np.float32), t.name))
for n in g.node:
  if n.op_type == 'Constant' and n.attribute[0].t.data_type == TensorProto.FLOAT16:
    n.attribute[0].t.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(n.attribute[0].t).astype(np.float32)))
for n in g.node:
  if not n.name:
    n.name = name(n.op_type)
del g.value_info[:]

# ---- 5. opset 11 ----
del m.opset_import[:]
m.opset_import.extend([helper.make_opsetid('', OPSET)])
m.ir_version = 7
m = shape_inference.infer_shapes(m)
onnx.checker.check_model(m, full_check=True)
onnx.save(m, DST)
ops = sorted({n.op_type for n in m.graph.node})
print(f'wrote {DST}: {len(m.graph.node)} nodes, {sum(int(np.prod(t.dims)) for t in m.graph.initializer):,} params, opset {OPSET}')
print('ops:', ops)
