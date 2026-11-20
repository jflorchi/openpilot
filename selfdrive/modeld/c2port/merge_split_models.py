#!/usr/bin/env python3
"""Merge an upstream split driving model (driving_vision.onnx + driving_policy.onnx, Dec 2025 - Jun 2026 era, e.g.
MacroStiff) into one graph with the comma two port's input contract, so the rest of the pipeline and modeld (C2PORT)
are unchanged:

    imgs               [1,24,128,256] f32  narrow (t-4, t) + wide (t-4, t), 6 YUV channels each, raw 0..255
    desire_tokens      [1,200]        f32  25 x 8 desire pulses (100-step 20 Hz history, max-pooled over groups of 4)
    feat_hist          [8,512]        f32  vision features of frames t-32, t-28, .., t-4
    traffic_convention [1,2]          f32
    outputs            [1,2576]       f32  vision outputs (1576, hidden_state at 1064) + policy outputs (1000)

The policy reads only features_buffer[:, -9:] (8 history tokens + the current frame's feature), so its 25-step buffer
becomes feat_hist + the vision model's hidden_state, computed in the same graph. The policy's one transformer block is
rebuilt in 2D for the last query only (same restructuring as make_policy_2d.py); its heads are copied as they are.

Also rewritten here (old tinygrad / fp16 safety): fp16 -> fp32; Constant nodes -> initializers; tanh-GELU
0.5*x*(1+tanh(k*(x+c*x^3))) -> x*sigmoid(2k*(xc+c*xc^3)), xc = clip(x,-10,10) (x^3 overflows fp16 past |x|~40);
GlobalAveragePool -> ReduceMean; Flatten -> Reshape; Clip without max -> Relu. ReduceL2, Expand, multiplier convs and
the opset are left to make_c2_onnx.py, which runs next.

usage: merge_split_models.py driving_vision.onnx driving_policy.onnx merged.onnx
"""
import sys

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto, shape_inference

VISION, POLICY, DST = sys.argv[1:4]
f32 = TensorProto.FLOAT
POL = 'pol/'  # prefix for every policy tensor name (both models use names like /Constant_output_0)


def load(path):
  m = onnx.load(path)
  m = shape_inference.infer_shapes(m)
  g = m.graph
  shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim] for v in list(g.value_info) + list(g.input) + list(g.output)}
  inits = {t.name: numpy_helper.to_array(t) for t in g.initializer}
  for n in g.node:  # Constant nodes -> values
    if n.op_type == 'Constant':
      inits[n.output[0]] = numpy_helper.to_array(n.attribute[0].t)
  nodes = [n for n in g.node if n.op_type != 'Constant']
  return m, nodes, inits, shapes


vm, vnodes, vinits, vshapes = load(VISION)
pm, pnodes, pinits, pshapes = load(POLICY)
assert [i.name for i in vm.graph.input] == ['img', 'big_img'], [i.name for i in vm.graph.input]
assert [i.name for i in pm.graph.input] == ['desire_pulse', 'traffic_convention', 'features_buffer']
assert vshapes['outputs'] == [1, 1576] and pshapes['outputs'] == [1, 1000], (vshapes['outputs'], pshapes['outputs'])

out_nodes, out_inits, uid = [], {}, [0]


def fresh(base):
  uid[0] += 1
  return f'ms/{base}_{uid[0]}'


def const(arr, base, dtype=np.float32):
  name = fresh(base)
  out_inits[name] = np.asarray(arr, dtype=dtype)
  return name


def op(op_type, inputs, out=None, **attrs):
  out = out or fresh(op_type.lower())
  out_nodes.append(helper.make_node(op_type, inputs, [out], name=out, **attrs))
  return out


def scalar(inits, t):
  return float(np.asarray(inits[t]).ravel()[0]) if t in inits else None


def safe_gelu(x, c, k, out=None):
  xc = op('Clip', [x, const([-10.0], 'lo'), const([10.0], 'hi')])
  cube = op('Mul', [op('Mul', [xc, xc]), xc])
  inner = op('Add', [xc, op('Mul', [cube, const([c], 'c')])])
  return op('Mul', [x, op('Sigmoid', [op('Mul', [inner, const([2 * k], 'k2')])])], out=out)


def find_tanh_gelus(nodes, inits):
  """Match 0.5 * (x * (1 + tanh(k * (x + c * (x * (x * x)))))) as exported by torch (any operand order).
  Returns {output tensor: (x, c, k, set of node ids)}."""
  prod = {o: n for n in nodes for o in n.output}
  cons = {}
  for n in nodes:
    for i in n.input:
      cons.setdefault(i, []).append(n)

  def other(n, known):  # the input of a binary node that isn't `known`
    a, b = n.input
    return b if a == known else a

  found = {}
  for th in (n for n in nodes if n.op_type == 'Tanh'):
    mk = prod[th.input[0]]                                     # k * inner
    k_name = next(i for i in mk.input if i in inits)
    inner = prod[other(mk, k_name)]                            # x + c*x^3
    assert inner.op_type == 'Add'
    a, b = inner.input
    cmul = prod[a] if a in prod and prod[a].op_type == 'Mul' and any(i in inits for i in prod[a].input) else prod[b]
    x = other(inner, cmul.output[0])
    c_name = next(i for i in cmul.input if i in inits)
    x3 = prod[other(cmul, c_name)]
    x2 = prod[other(x3, x)]
    assert x3.op_type == 'Mul' and x2.op_type == 'Mul' and list(x2.input) == [x, x], (x3.input, x2.input)
    (add1,) = cons[th.output[0]]                               # 1 + tanh
    assert add1.op_type == 'Add' and scalar(inits, other(add1, th.output[0])) == 1.0
    (mulx,) = cons[add1.output[0]]                             # x * (1 + tanh)
    assert mulx.op_type == 'Mul' and other(mulx, add1.output[0]) == x
    (half,) = cons[mulx.output[0]]                             # 0.5 * ...
    assert half.op_type == 'Mul' and scalar(inits, other(half, mulx.output[0])) == 0.5
    ids = {id(n) for n in (x2, x3, cmul, inner, mk, th, add1, mulx, half)}
    found[half.output[0]] = (x, scalar(inits, c_name), scalar(inits, k_name), ids)
  return found


def convert(nodes, inits, shapes, rename, skip):
  """Copy nodes (except ids in skip) into out_nodes with the generic rewrites; returns nothing."""
  gelus = find_tanh_gelus(nodes, inits)
  drop = set().union(*(g[3] for g in gelus.values())) if gelus else set()
  for n in nodes:
    if id(n) in skip:
      continue
    if id(n) in drop:
      if n.output[0] in gelus:
        x, c, k, _ = gelus[n.output[0]]
        safe_gelu(rename(x), c, k, out=rename(n.output[0]))
      continue
    ins, outs = [rename(i) if i else '' for i in n.input], [rename(o) for o in n.output]
    attrs = {a.name: helper.get_attribute_value(a) for a in n.attribute}
    if n.op_type == 'GlobalAveragePool':
      op('ReduceMean', ins, out=outs[0], axes=[2, 3], keepdims=1)
    elif n.op_type == 'Flatten':
      assert attrs.get('axis', 1) == 1
      shp = shapes[n.output[0]]
      assert len(shp) == 2 and all(shp), shp
      if shapes[n.input[0]] == shp:
        op('Identity', ins, out=outs[0])
      else:
        op('Reshape', [ins[0], const(shp, 'shape', np.int64)], out=outs[0])
    elif n.op_type == 'Clip' and (len(n.input) < 3 or not n.input[2]):
      assert scalar(inits, n.input[1]) == 0.0
      op('Relu', [ins[0]], out=outs[0])
    elif n.op_type == 'Cast':
      op('Identity', ins, out=outs[0])
    elif n.op_type == 'Concat' and any(i in inits and inits[i].size == 0 for i in n.input):
      # empty 'pad' [1,0] (old tinygrad's loader can't take a zero-size initializer)
      keep = [(i, r) for i, r in zip(n.input, ins) if not (i in inits and inits[i].size == 0)]
      for i, _ in keep:
        if i in inits:
          out_inits[rename(i)] = inits[i]
      op('Concat', [r for _, r in keep], out=outs[0], **attrs)
    else:
      for i in n.input:
        if i in inits:
          out_inits[rename(i)] = inits[i]
      out_nodes.append(helper.make_node(n.op_type, ins, outs, name=outs[0], **attrs))


# ---- vision: Cast(img), Cast(big_img), Concat -> input 'imgs'
cat = next(n for n in vnodes if n.op_type == 'Concat' and len(n.input) == 2 and
           all(i.startswith('/Cast') for i in n.input))
casts = [n for n in vnodes if n.op_type == 'Cast' and n.output[0] in cat.input]
assert [c.input[0] for c in casts] == ['img', 'big_img']
v_alias = {cat.output[0]: 'imgs', 'outputs': 'vis/outputs'}
convert(vnodes, vinits, vshapes, lambda t: v_alias.get(t, t), {id(n) for n in [cat] + casts})
hidden = '/summarizer/Div_output_0'
vcat = next(n for n in vnodes if n.output[0] == 'outputs')
assert vcat.input[-2] == hidden and vshapes['/summarizer/_mu/Gemm_output_0'] == [1, 512]  # hidden = _mu / |_mu|

# ---- policy: 2D transformer block for the last token, then the original heads
pprod = {n.output[0]: n for n in pnodes}
pcons = {}
for n in pnodes:
  for i in n.input:
    pcons.setdefault(i, []).append(n)


def P(t):  # policy tensor name in the merged graph
  return t if t in ('desire_tokens', 'traffic_convention', 'feat_hist') else POL + t


def W(name):  # policy weight, copied into the merged graph
  out_inits[P(name)] = pinits[name]
  return P(name)


def wt(node, idx=1):
  return W(node.input[idx])


def bias_of(add):  # Add(bias, x) as exported
  return W(next(i for i in add.input if i in pinits))


def only(pred):
  r = [n for n in pnodes if pred(n)]
  assert len(r) == 1, [n.output[0] for n in r]
  return r[0]


def path(name, op_type):
  return only(lambda n: n.op_type == op_type and n.output[0] == name)


gather_feats = only(lambda n: n.op_type == 'Gather' and n.input[0] == 'features_buffer')
assert pinits[gather_feats.input[1]].tolist() == list(range(-9, 0)), pinits[gather_feats.input[1]]
assert pshapes['features_buffer'] == [1, 25, 512] and pshapes['desire_pulse'] == [1, 25, 8]
reshape_desire = only(lambda n: n.op_type == 'Reshape' and n.input[0] == 'desire_pulse')
assert pinits[reshape_desire.input[1]].tolist() == [1, -1]
TS = '/temporal_summarizer'
fe0 = path(f'{TS}/_feats_encode/_feats_encode.0/MatMul_output_0', 'MatMul')
fe0b = path(f'{TS}/_feats_encode/_feats_encode.0/Add_output_0', 'Add')
fe2 = path(f'{TS}/_feats_encode/_feats_encode.2/MatMul_output_0', 'MatMul')
fe2b = path(f'{TS}/_feats_encode/_feats_encode.2/Add_output_0', 'Add')
de0 = path(f'{TS}/_desire_encode/_desire_encode.0/Gemm_output_0', 'Gemm')
de2 = path(f'{TS}/_desire_encode/_desire_encode.2/Gemm_output_0', 'Gemm')
te0 = path(f'{TS}/_traffic_encode/_traffic_encode.0/Gemm_output_0', 'Gemm')
te2 = path(f'{TS}/_traffic_encode/_traffic_encode.2/Gemm_output_0', 'Gemm')
pos_add = path(f'{TS}/Add_output_0', 'Add')
pos = next(i for i in pos_add.input if i in pinits)
T0 = f'{TS}/transformer/transformer.0'
ln1 = path(f'{T0}/attn/layer_norm/LayerNormalization_output_0', 'LayerNormalization')
qkv = path(f'{T0}/attn/c_attn/MatMul_output_0', 'MatMul')
qkvb = path(f'{T0}/attn/c_attn/Add_output_0', 'Add')
qkv_shape = path(f'{T0}/attn/Reshape_output_0', 'Reshape')
assert pinits[qkv_shape.input[1]].tolist() == [1, 9, 3, 8, 64]
assert [a.ints for a in path(f'{T0}/attn/Transpose_output_0', 'Transpose').attribute][0] == [2, 0, 3, 1, 4]
scale = scalar(pinits, next(i for i in path(f'{T0}/attn/Mul_output_0', 'Mul').input if i in pinits))
where = path(f'{T0}/attn/Where_output_0', 'Where')
mask = pinits[where.input[0]].astype(bool)
assert mask.shape == (1, 1, 9, 9) and not mask[0, 0, 8].any(), 'last query must see all 9 tokens'
assert [a.ints for a in path(f'{T0}/attn/Transpose_2_output_0', 'Transpose').attribute][0] == [0, 2, 1, 3]
cproj = path(f'{T0}/attn/c_proj/MatMul_output_0', 'MatMul')
cprojb = path(f'{T0}/attn/c_proj/Add_output_0', 'Add')
ln2 = path(f'{T0}/mlp/layer_norm/LayerNormalization_output_0', 'LayerNormalization')
fc = path(f'{T0}/mlp/c_fc/MatMul_output_0', 'MatMul')
fcb = path(f'{T0}/mlp/c_fc/Add_output_0', 'Add')
mproj = path(f'{T0}/mlp/c_proj/MatMul_output_0', 'MatMul')
mprojb = path(f'{T0}/mlp/c_proj/Add_output_0', 'Add')
pgelus = find_tanh_gelus(pnodes, pinits)
assert len(pgelus) == 1
(gx, gc, gk, _), = pgelus.values()
assert gx == fcb.output[0]
last = path(f'{TS}/Gather_output_0', 'Gather')
assert scalar(pinits, last.input[1]) == 8
for g in (de0, de2, te0, te2):
  assert {a.name: helper.get_attribute_value(a) for a in g.attribute}.get('transB') == 1
for ln in (ln1, ln2):
  assert {a.name: helper.get_attribute_value(a) for a in ln.attribute}.get('axis', -1) == -1
eps = {a.name: helper.get_attribute_value(a) for a in ln1.attribute}['epsilon']


def linear(x, mm, add):
  return op('Add', [op('MatMul', [x, wt(mm)]), bias_of(add)])


def gemm(x, g):
  return op('Gemm', [x, W(g.input[1]), W(g.input[2])], transB=1)


def silu(x):
  return op('Mul', [x, op('Sigmoid', [x])])


def layernorm(x, ln):
  mu = op('ReduceMean', [x], axes=[-1], keepdims=1)
  d = op('Sub', [x, mu])
  var = op('ReduceMean', [op('Mul', [d, d])], axes=[-1], keepdims=1)
  sd = op('Sqrt', [op('Add', [var, const([eps], 'eps')])])
  return op('Add', [op('Mul', [op('Div', [d, sd]), W(ln.input[1])]), W(ln.input[2])])


def cols(x, a, b):
  return op('Slice', [x, const([a], 's', np.int64), const([b], 'e', np.int64), const([1], 'ax', np.int64)])


def rows(x, a, b):
  return op('Slice', [x, const([a], 's', np.int64), const([b], 'e', np.int64), const([0], 'ax', np.int64)])


heads, dh = 8, 64
x_in = op('Concat', ['feat_hist', hidden], axis=0)                                    # [9,512]
e = linear(silu(linear(x_in, fe0, fe0b)), fe2, fe2b)
desire_emb = gemm(silu(gemm('desire_tokens', de0)), de2)                             # [1,512]
tc_emb = gemm(silu(gemm('traffic_convention', te0)), te2)                            # [1,512]
pos2d = P(pos)  # same name and values, [1,9,512] -> [9,512], so make_recipe.py stores it as a reference
out_inits[pos2d] = pinits[pos].reshape(9, 512)
x = op('Add', [op('Add', [op('Add', [e, pos2d]), desire_emb]), tc_emb])               # [9,512]
q_kv = linear(layernorm(x, ln1), qkv, qkvb)                                          # [9,1536]
k, v = cols(q_kv, 512, 1024), cols(q_kv, 1024, 1536)
q = rows(cols(q_kv, 0, 512), 8, 9)                                                   # [1,512]
hsum = np.zeros((heads * dh, heads), np.float32)
hsum[np.arange(heads * dh), np.arange(heads * dh) // dh] = 1
s = op('MatMul', [op('Mul', [k, q]), const(hsum, 'hsum')])                            # [9,8] scores[key, head]
s = op('Mul', [op('Transpose', [s], perm=[1, 0]), const([scale], 'scale')])         # [8,9]
a = op('Softmax', [s], axis=-1)
a_exp = op('MatMul', [op('Transpose', [a], perm=[1, 0]), const(hsum.T.copy(), 'hexp')])  # [9,512]
attn = op('ReduceSum', [op('Mul', [a_exp, v])], axes=[0], keepdims=1)                # [1,512]
r1 = op('Add', [rows(x, 8, 9), linear(attn, cproj, cprojb)])
h = safe_gelu(linear(layernorm(r1, ln2), fc, fcb), gc, gk)
op('Add', [r1, linear(h, mproj, mprojb)], out=P(last.output[0]))                     # [1,512], was Gather(.., 8)

# heads: everything downstream of the block output, copied
heads_ids, stack = set(), [last.output[0]]
while stack:
  for n in pcons.get(stack.pop(), []):
    if id(n) not in heads_ids:
      heads_ids.add(id(n))
      stack.extend(n.output)
convert([n for n in pnodes if id(n) in heads_ids], pinits, pshapes, P, set())
op('Concat', ['vis/outputs', P('outputs')], out='outputs', axis=1)

# ---- assemble
graph = helper.make_graph(
  out_nodes, 'macrostiff_c2',
  [helper.make_tensor_value_info('imgs', f32, [1, 24, 128, 256]),
   helper.make_tensor_value_info('desire_tokens', f32, [1, 200]),
   helper.make_tensor_value_info('feat_hist', f32, [8, 512]),
   helper.make_tensor_value_info('traffic_convention', f32, [1, 2])],
  [helper.make_tensor_value_info('outputs', f32, [1, 2576])],
  [numpy_helper.from_array(arr.astype(np.float32) if arr.dtype == np.float16 else arr, name) for name, arr in out_inits.items()])
m = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 11)])  # every op left has its opset-11 form
m.ir_version = 7
m = shape_inference.infer_shapes(m)
onnx.checker.check_model(m, full_check=True)
onnx.save(m, DST)
print(f'wrote {DST}: {len(m.graph.node)} nodes, {sum(int(np.prod(t.dims)) for t in m.graph.initializer):,} params, '
      f'{len(find_tanh_gelus(vnodes, vinits))} vision GELUs rewritten')
