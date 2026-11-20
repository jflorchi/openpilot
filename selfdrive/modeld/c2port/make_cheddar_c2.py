#!/usr/bin/env python3
"""Rewrite upstream's Cheddar driving_supercombo.onnx (Nov 2026, recurrent-state contract) into the comma two port's
input contract. The result then goes through make_c2_onnx.py, strip_input_norm.py, split_input_frames.py and
make_recipe.py like any other port graph.

Cheddar's own interface keeps its history inside the model (state_img_q [2,5,6,128,256] u8, state_desire_q [132,1,8],
state_feat_q [128,1,512] in, next_state_* out). What it reads from them is the same as the port's host buffers:
  - images: queue entries 0 and 4 after pushing the new frame = frames t-4 and t, narrow then wide -> imgs [1,24,128,256]
  - features: every 4th queue entry, last 8 = frames t-32 .. t-4, plus the current frame's feature -> 9 tokens
  - desire: the 132-step pulse queue (current pulse last) max-pooled over groups of 4 -> 33 tokens; token k of the
    policy reads pooled[k .. k+24] -> desire_tokens [9,200] (row k = 25 x 8)
so the port's contract becomes

    imgs               [1,24,128,256] f32  narrow (t-4, t) + wide (t-4, t), 6 YUV channels each, raw 0..255
    desire_tokens      [9,200]        f32  9 windows of 25 pooled desire tokens (row 8 = the 25 most recent)
    feat_hist          [8,512]        f32  vision features of frames t-32, t-28, .., t-4
    traffic_convention [1,2]          f32
    action_t           [1,2]          f32  lateral / longitudinal action time, s (upstream: delay + 0.05 + 0.025)
    outputs            [1,2580]       f32  output_slices metadata layout (hidden_state at 2066)

Rewrites for tinygrad 2022 / fp16 (no Exp, Erf, Abs, ReduceMax, GatherND or activation x activation matmuls there):
  - the two temporal summarizers (4 + 1 transformer layers, causal, per-head q/k LayerNorm) rebuilt in 2D over [9,512]
    rows: attention per query over its visible keys only, with head sums done by constant matmuls; the last layer of
    each only for the last token, which is all the heads read
  - erf-GELU (vision) -> relu(x) - |x| * 0.5 * erfc(|x|/sqrt2), erfc by Abramowitz-Stegun 7.1.26 (|err| < 1.5e-7) with
    exp(-y) = sigmoid(-y) / sigmoid(y); tanh-GELU -> x * sigmoid(2k(xc + c xc^3)), xc = clip(x, -10, 10)
  - LayerNormalization decomposed; Cast -> Identity; fp16 -> fp32; opset 11 forms for Reduce*/Squeeze/Unsqueeze
Multiplier convs and the rest are left to make_c2_onnx.py.

usage: make_cheddar_c2.py driving_supercombo.onnx cheddar_merged.onnx
"""
import os
import sys

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto, shape_inference

SRC, DST = sys.argv[1:3]
# BATCHED_ATTENTION=1: all (query, key) pairs of a layer at once (attend_all). Measured on the device: only 0.5 ms
# faster (the ~200 small kernels it removes cost ~10 us each) and less accurate in fp16 (3.0e-3 vs 1.1e-3 on real
# frames), so the default stays per query.
BATCHED_ATTENTION = os.environ.get('BATCHED_ATTENTION') is not None
f32 = TensorProto.FLOAT

m = shape_inference.infer_shapes(onnx.load(SRC))
g = m.graph
shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim] for v in list(g.value_info) + list(g.input) + list(g.output)}
inits = {t.name: numpy_helper.to_array(t) for t in g.initializer}
for n in g.node:
  if n.op_type == 'Constant':
    inits[n.output[0]] = numpy_helper.to_array(n.attribute[0].t)
nodes = [n for n in g.node if n.op_type != 'Constant']
prod = {o: n for n in nodes for o in n.output}
cons = {}
for n in nodes:
  for i in n.input:
    cons.setdefault(i, []).append(n)

assert [i.name for i in g.input] == ['new_img', 'desire', 'traffic_convention', 'action_t', 'state_img_q', 'state_desire_q',
                                     'state_feat_q'], [i.name for i in g.input]
assert shapes['outputs'] == [1, 2580] and shapes['state_desire_q'] == [132, 1, 8] and shapes['state_feat_q'] == [128, 1, 512]

out_nodes, out_inits, uid = [], {}, [0]


def fresh(base):
  uid[0] += 1
  return f'ch/{base}_{uid[0]}'


def const(arr, base, dtype=np.float32):
  name = fresh(base)
  out_inits[name] = np.asarray(arr, dtype=dtype)
  return name


def W(name, reshape=None):
  """an original weight, copied under its own name (make_recipe.py stores it as a reference)"""
  arr = inits[name]
  out_inits[name] = arr.reshape(reshape) if reshape is not None else arr
  return name


def op(op_type, inputs, out=None, **attrs):
  out = out or fresh(op_type.lower())
  out_nodes.append(helper.make_node(op_type, inputs, [out], name=out, **attrs))
  return out


def attrs_of(n):
  return {a.name: helper.get_attribute_value(a) for a in n.attribute}


def only(pred):
  r = [n for n in nodes if pred(n)]
  assert len(r) == 1, [n.output[0] for n in r]
  return r[0]


def consumer(t, op_type=None):
  c = cons[t]
  assert len(c) == 1 and (op_type is None or c[0].op_type == op_type), (t, [(x.op_type, x.output[0]) for x in c])
  return c[0]


def other(n, known):
  a, b = n.input
  return b if a == known else a


# ---- building blocks
def tanh_gelu(x, out=None):
  xc = op('Clip', [x, const([-10.0], 'lo'), const([10.0], 'hi')])
  cube = op('Mul', [op('Mul', [xc, xc]), xc])
  inner = op('Add', [xc, op('Mul', [cube, const([0.044715], 'c')])])
  return op('Mul', [x, op('Sigmoid', [op('Mul', [inner, const([2 * np.sqrt(2 / np.pi)], 'k2')])])], out=out)


AS_P = 0.3275911
AS_A = [0.254829592, -0.284496736, 1.421413741, -1.453152027, 1.061405429]


def erf_gelu(x, out=None):
  # gelu(x) = x * Phi(x) = relu(x) - |x| * Phi(-|x|),  Phi(-a) = 0.5 * erfc(a / sqrt2) ~= 0.5 * poly(t) * exp(-a^2 / 2)
  r = op('Relu', [x])
  a = op('Add', [r, op('Relu', [op('Mul', [x, const([-1.0], 'neg')])])])
  ac = op('Clip', [a, const([0.0], 'lo'), const([10.0], 'hi')])                    # Phi(-10) ~ 7.6e-24
  t = op('Div', [const([1.0], 'one'), op('Add', [const([1.0], 'one'), op('Mul', [ac, const([AS_P / np.sqrt(2)], 'p')])])])
  poly = const([AS_A[4]], 'a5')
  for coef in AS_A[3::-1]:
    poly = op('Add', [op('Mul', [poly, t]), const([coef], 'a')])
  poly = op('Mul', [poly, t])
  y = op('Mul', [op('Mul', [ac, ac]), const([0.5], 'half')])
  e = op('Div', [op('Sigmoid', [op('Mul', [y, const([-1.0], 'neg')])]), op('Sigmoid', [y])])  # exp(-y)
  q = op('Mul', [op('Mul', [poly, e]), const([0.5], 'half')])
  return op('Sub', [r, op('Mul', [a, q])], out=out)


def layernorm(x, gamma, beta, eps, out=None):
  mu = op('ReduceMean', [x], axes=[-1], keepdims=1)
  d = op('Sub', [x, mu])
  var = op('ReduceMean', [op('Mul', [d, d])], axes=[-1], keepdims=1)
  sd = op('Sqrt', [op('Add', [var, const([eps], 'eps')])])
  return op('Add', [op('Mul', [op('Div', [d, sd]), gamma]), beta], out=out)


def ln_node(x, n, out=None):
  a = attrs_of(n)
  assert a.get('axis', -1) == -1
  return layernorm(x, W(n.input[1]), W(n.input[2]), a['epsilon'], out=out)


def silu(x):
  return op('Mul', [x, op('Sigmoid', [x])])


def rows(x, a, b):
  return op('Slice', [x, const([a], 's', np.int64), const([b], 'e', np.int64), const([0], 'ax', np.int64)])


def cols(x, a, b):
  return op('Slice', [x, const([a], 's', np.int64), const([b], 'e', np.int64), const([1], 'ax', np.int64)])


# ---- 1. generic copy of everything outside the two temporal summarizers
SUMM = {'off': 'model.on_policy.off_policy.temporal_summarizer', 'on': 'model.on_policy.temporal_summarizer'}
# the token blocks end where each summarizer's last token is taken: Gather(x, -1, axis=1) on a [1,9,512] tensor
ends = {}
for key, P in SUMM.items():
  ln_last = None
  for n in nodes:
    if n.op_type == 'LayerNormalization' and n.input[1].startswith(f'{P}.transformer.layers.') and n.input[1].endswith('mlp.norm.weight'):
      ln_last = n
  res_in = ln_last.input[0]                                         # residual before the last MLP
  fc = consumer(ln_last.output[0], 'MatMul')
  fcb = consumer(fc.output[0], 'Add')
  gl = consumer(fcb.output[0], 'Gelu')
  pj = consumer(gl.output[0], 'MatMul')
  pjb = consumer(pj.output[0], 'Add')
  res = consumer(pjb.output[0], 'Add')
  gat = [c for c in cons[res.output[0]] if c.op_type == 'Gather']
  assert len(gat) == 1 and int(inits[gat[0].input[1]]) == -1 and attrs_of(gat[0])['axis'] == 1
  ends[key] = gat[0].output[0]
# the traffic convention / action_t selects of the on-policy summarizer are plain aliases
alias = {}
for n in nodes:
  if n.op_type == 'Gather' and n.input[0] in prod and prod[n.input[0]].op_type == 'Unsqueeze':
    src = prod[n.input[0]].input[0]
    if src in prod and prod[src].op_type == 'Cast' and prod[src].input[0] in ('traffic_convention', 'action_t'):
      alias[n.output[0]] = prod[src].input[0]
  if n.op_type == 'Cast' and n.input[0] in ('traffic_convention', 'action_t'):
    alias[n.output[0]] = n.input[0]
cat_imgs = only(lambda n: n.op_type == 'Concat' and shapes.get(n.output[0]) == [1, 24, 128, 256])
cast_imgs = consumer(cat_imgs.output[0], 'Cast')
alias[cast_imgs.output[0]] = 'imgs'
stops = set(ends.values()) | set(alias) | {i.name for i in g.input}

keep, stack = set(), ['outputs']
while stack:
  t = stack.pop()
  if t in stops or t not in prod:
    continue
  n = prod[t]
  if id(n) not in keep:
    keep.add(id(n))
    stack.extend(n.input)
generic = [n for n in nodes if id(n) in keep]
assert not any(n.input[0].startswith('state_') or n.input[0] in ('new_img', 'desire') for n in generic)

R = lambda t: alias.get(t, t)
for n in generic:
  ins, outs, a = [R(i) if i else '' for i in n.input], list(n.output), attrs_of(n)
  for i in n.input:
    if i in inits and i not in out_inits:
      out_inits[i] = inits[i]
  if n.op_type == 'Gelu':
    (erf_gelu if a.get('approximate', b'none') in (b'none', 'none') else tanh_gelu)(ins[0], out=outs[0])
  elif n.op_type == 'LayerNormalization':
    ln_node(ins[0], n, out=outs[0])
  elif n.op_type == 'Cast':
    op('Identity', ins, out=outs[0])
  elif n.op_type in ('ReduceMean', 'ReduceSum') and len(n.input) > 1:
    op(n.op_type, [ins[0]], out=outs[0], axes=inits[n.input[1]].tolist(), keepdims=a.get('keepdims', 1))
  elif n.op_type in ('Squeeze', 'Unsqueeze') and len(n.input) > 1:
    op(n.op_type, [ins[0]], out=outs[0], axes=inits[n.input[1]].tolist())
  elif n.op_type == 'Reshape':
    op('Reshape', ins, out=outs[0])
  elif n.op_type == 'Gemm' and len(ins) == 2:  # tinygrad 2022's Gemm needs the bias input
    assert a.get('transB') == 1 and not a.get('transA')
    op('Gemm', ins + [const(np.zeros(inits[n.input[1]].shape[0]), 'zero_b')], out=outs[0], **a)
  elif n.op_type in ('Gather', 'GatherND', 'Expand', 'ReduceMax', 'Where'):
    raise NotImplementedError(f'{n.op_type} {n.output[0]} in the generic part')
  else:
    if n.op_type == 'MatMul':
      assert n.input[1] in inits and len(shapes[n.input[0]]) == 2, n.output[0]
    out_nodes.append(helper.make_node(n.op_type, ins, outs, name=outs[0], **a))

# vision feature of the current frame [1,512] (= the token the policy appends, and the hidden_state output)
feat = op('Reshape', [only(lambda n: n.op_type == 'Reshape' and n.output[0] == 'view_21').input[0], const([1, 512], 'shape', np.int64)])

# ---- 2. temporal summarizers in 2D
heads, dh, T = 8, 64, 9
hsum = np.zeros((heads * dh, heads), np.float32)
hsum[np.arange(heads * dh), np.arange(heads * dh) // dh] = 1
HSUM, HEXP, HMEAN = const(hsum, 'hsum'), const(hsum.T.copy(), 'hexp'), const(hsum / dh, 'hmean')
x_tok = op('Concat', ['feat_hist', feat], axis=0)                                                    # [9,512]


def bias_add(bias_name):
  (n,) = [c for c in cons[bias_name] if c.op_type == 'Add']
  mm = prod[other(n, bias_name)]
  assert mm.op_type == 'MatMul' and mm.input[1] in inits
  return mm.input[1]


def mlp_block(x, ln):
  """x + c_proj(gelu(c_fc(ln(x)))) with bias-free c_fc / c_proj (summarizer mlp1/mlp2), on [9,512]"""
  fc = consumer(ln.output[0], 'MatMul')
  gl = consumer(fc.output[0], 'Gelu')
  pj = consumer(gl.output[0], 'MatMul')
  assert attrs_of(gl)['approximate'] in (b'tanh', 'tanh')
  h = tanh_gelu(op('MatMul', [ln_node(x, ln), W(fc.input[1])]))
  return op('Add', [x, op('MatMul', [h, W(pj.input[1])])])


def head_ln(x, gamma, beta, eps):
  """LayerNorm over each head's 64 channels of [n,512] rows (q_norm / k_norm), with constant matmuls"""
  mu = op('MatMul', [op('MatMul', [x, HMEAN]), HEXP])
  d = op('Sub', [x, mu])
  var = op('MatMul', [op('MatMul', [op('Mul', [d, d]), HMEAN]), HEXP])
  sd = op('Sqrt', [op('Add', [var, const([eps], 'eps')])])
  return op('Add', [op('Mul', [op('Div', [d, sd]), const(np.tile(inits[gamma], heads), 'qkg')]), const(np.tile(inits[beta], heads), 'qkb')])


def attend(qn, kn, v, i, scale):
  """attention output [1,512] of query row i over keys 0..i (causal)"""
  if i == 0:
    return rows(v, 0, 1)                                            # softmax over one key
  q = rows(qn, i, i + 1)
  k, vv = rows(kn, 0, i + 1), rows(v, 0, i + 1)
  s = op('Mul', [op('Transpose', [op('MatMul', [op('Mul', [k, q]), HSUM])], perm=[1, 0]), const([scale], 'scale')])  # [8,i+1]
  a = op('Softmax', [s], axis=-1)
  a_exp = op('MatMul', [op('Transpose', [a], perm=[1, 0]), HEXP])                                    # [i+1,512]
  return op('ReduceSum', [op('Mul', [a_exp, vv])], axes=[0], keepdims=1)


def attend_all(qn, kn, v, scale):
  """causal attention of all T queries at once, [T,512] -> [T,512]: the T*T (query, key) pairs are laid out as rows
  i*T+j, built and summed with constant matmuls (the constant always as the second operand), so a layer is ~16
  kernels instead of ~9 per query. Masked pairs get -1e4 (exp underflows to exactly 0, fp16-safe)."""
  TT = T * T
  rq, rk, gsum = np.zeros((T, TT), np.float32), np.zeros((T, TT), np.float32), np.zeros((TT, T), np.float32)
  for i in range(T):
    for j in range(T):
      rq[i, i * T + j] = rk[j, i * T + j] = gsum[i * T + j, i] = 1
  RQ, RK, GS = const(rq, 'rep_q'), const(rk, 'rep_k'), const(gsum, 'sum_k')
  t = lambda x: op('Transpose', [x], perm=[1, 0])
  prod_t = op('Mul', [op('MatMul', [t(qn), RQ]), op('MatMul', [t(kn), RK])])                       # [512,TT]
  s = op('MatMul', [t(prod_t), HSUM])                                                              # [TT,8]
  s = op('Reshape', [t(s), const([heads * T, T], 'shape', np.int64)])                              # [8T,T] row h*T+i
  mask = np.tile(np.where(np.triu(np.ones((T, T), bool), 1), np.float32(-1e4), np.float32(0)), (heads, 1))
  a = op('Softmax', [op('Add', [op('Mul', [s, const([scale], 'scale')]), const(mask, 'causal')])], axis=-1)
  a = t(op('Reshape', [a, const([heads, TT], 'shape', np.int64)]))                                  # [TT,8]
  m = op('Mul', [op('MatMul', [a, HEXP]), t(op('MatMul', [t(v), RK]))])                             # [TT,512]
  return t(op('MatMul', [t(m), GS]))                                                               # [T,512]


def summarizer(key, out):
  P = SUMM[key]
  ln1 = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{P}.mlp1.norm.weight')
  ln2 = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{P}.mlp2.norm.weight')
  # the GatherND of the last 9 tokens: [-9 .. -1] of 33
  gnd = prod[prod[ln1.input[0]].input[0]]
  assert gnd.op_type == 'GatherND' and inits[gnd.input[1]].ravel().tolist() == list(range(-9, 0))
  x = mlp_block(mlp_block(x_tok, ln1), ln2)
  # positional embedding: the [1,9,512] initializer added right after
  r = consumer(consumer(ln2.output[0]).output[0])  # MatMul -> Gelu
  res2 = consumer(consumer(r.output[0]).output[0], 'Add')
  view = consumer(res2.output[0], 'Reshape')
  pos_add = consumer(view.output[0], 'Add')
  pos = other(pos_add, view.output[0])
  assert inits[pos].shape == (1, 9, 512)
  x = op('Add', [x, W(pos, reshape=(9, 512))])
  # desire / traffic / action_t embeddings
  d0 = bias_add(f'{P}.desire_encoder.net.0.bias')
  dadd = only(lambda n: n.op_type == 'Add' and f'{P}.desire_encoder.net.0.bias' in n.input)
  dsilu = only(lambda n: n.op_type == 'Mul' and dadd.output[0] in n.input)
  d2 = consumer(dsilu.output[0], 'MatMul')
  assert f'{P}.desire_encoder.net.2.bias' not in inits
  de = op('MatMul', [silu(op('Add', [op('MatMul', ['desire_tokens', W(d0)]), W(f'{P}.desire_encoder.net.0.bias')])), W(d2.input[1])])
  x = op('Add', [x, de])
  for enc, src in (('traffic_encoder', 'traffic_convention'), ('action_t_encoder', 'action_t')):
    g0 = only(lambda n: n.op_type == 'Gemm' and n.input[1] == f'{P}.{enc}.net.0.weight')
    g2 = only(lambda n: n.op_type == 'Gemm' and n.input[1] == f'{P}.{enc}.net.2.weight')
    assert attrs_of(g0).get('transB') == 1 and attrs_of(g2).get('transB') == 1 and len(g2.input) == 2
    e = op('Gemm', [silu(op('Gemm', [src, W(g0.input[1]), W(g0.input[2])], transB=1)), W(g2.input[1]),
                    const(np.zeros(512), 'zero_b')], transB=1)
    x = op('Add', [x, e])
  # transformer layers
  layers = sorted({int(n.input[1].split('.layers.')[1].split('.')[0]) for n in nodes if n.op_type == 'LayerNormalization'
                   and n.input[1].startswith(f'{P}.transformer.layers.')})
  assert layers == list(range(len(layers)))
  for li in layers:
    L = f'{P}.transformer.layers.{li}'
    last = li == layers[-1]
    lna = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{L}.attention.norm.weight')
    qkv = op('Add', [op('MatMul', [ln_node(x, lna), W(bias_add(f'{L}.attention.c_attn.bias'))]), W(f'{L}.attention.c_attn.bias')])
    qln = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{L}.attention.q_norm.weight')
    kln = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{L}.attention.k_norm.weight')
    mt = consumer(consumer(kln.output[0], 'Transpose').output[0], 'MatMul')
    smul = consumer(mt.output[0], 'Mul')
    scale = float(inits[other(smul, mt.output[0])])
    wh = consumer(smul.output[0], 'Where')
    mask = inits[wh.input[0]].astype(bool)
    assert mask.shape == (1, 1, T, T) and (mask[0, 0] == np.triu(np.ones((T, T), bool), 1)).all(), 'expected a causal mask'
    q_rows = qkv if not last else rows(qkv, T - 1, T)
    qn = head_ln(cols(q_rows, 0, 512), qln.input[1], qln.input[2], attrs_of(qln)['epsilon'])
    kn = head_ln(cols(qkv, 512, 1024), kln.input[1], kln.input[2], attrs_of(kln)['epsilon'])
    v = cols(qkv, 1024, 1536)
    if last:  # q is the last token's row only; it sees every key
      s = op('Mul', [op('Transpose', [op('MatMul', [op('Mul', [kn, qn]), HSUM])], perm=[1, 0]), const([scale], 'scale')])
      a = op('Softmax', [s], axis=-1)
      att = op('ReduceSum', [op('Mul', [op('MatMul', [op('Transpose', [a], perm=[1, 0]), HEXP]), v])], axes=[0], keepdims=1)
      x = rows(x, T - 1, T)
    else:
      att = (attend_all(qn, kn, v, scale) if BATCHED_ATTENTION
             else op('Concat', [attend(qn, kn, v, i, scale) for i in range(T)], axis=0))           # [9,512]
    x = op('Add', [x, op('Add', [op('MatMul', [att, W(bias_add(f'{L}.attention.c_proj.bias'))]), W(f'{L}.attention.c_proj.bias')])])
    lnm = only(lambda n: n.op_type == 'LayerNormalization' and n.input[1] == f'{L}.mlp.norm.weight')
    h = tanh_gelu(op('Add', [op('MatMul', [ln_node(x, lnm), W(bias_add(f'{L}.mlp.c_fc.bias'))]), W(f'{L}.mlp.c_fc.bias')]))
    x = op('Add', [x, op('Add', [op('MatMul', [h, W(bias_add(f'{L}.mlp.c_proj.bias'))]), W(f'{L}.mlp.c_proj.bias')])],
           out=out if last else None)


for key, end in ends.items():
  summarizer(key, end)

# ---- 3. assemble: topological order (the heads were copied before the summarizers that feed them were built)
avail = {'imgs', 'desire_tokens', 'feat_hist', 'traffic_convention', 'action_t', ''} | set(out_inits)
ordered, pending = [], out_nodes
while pending:
  rest = []
  for n in pending:
    if all(x in avail for x in n.input):
      ordered.append(n)
      avail.update(n.output)
    else:
      rest.append(n)
  if len(rest) == len(pending):
    raise RuntimeError(f'missing input near {rest[0].op_type} {list(rest[0].input)}')
  pending = rest
out_nodes = ordered
for name in list(out_inits):
  if out_inits[name].dtype == np.float16:
    out_inits[name] = out_inits[name].astype(np.float32)
graph = helper.make_graph(
  out_nodes, 'cheddar_c2',
  [helper.make_tensor_value_info('imgs', f32, [1, 24, 128, 256]),
   helper.make_tensor_value_info('desire_tokens', f32, [9, 200]),
   helper.make_tensor_value_info('feat_hist', f32, [8, 512]),
   helper.make_tensor_value_info('traffic_convention', f32, [1, 2]),
   helper.make_tensor_value_info('action_t', f32, [1, 2])],
  [helper.make_tensor_value_info('outputs', f32, [1, 2580])],
  [numpy_helper.from_array(arr, name) for name, arr in out_inits.items()])
mo = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 11)])
mo.ir_version = 7
for p in m.metadata_props:
  mo.metadata_props.add(key=p.key, value=p.value)
mo = shape_inference.infer_shapes(mo)
onnx.checker.check_model(mo, full_check=True)
onnx.save(mo, DST)
ops = sorted({n.op_type for n in mo.graph.node})
print(f'wrote {DST}: {len(mo.graph.node)} nodes, {sum(int(np.prod(t.dims)) for t in mo.graph.initializer):,} params\nops: {ops}')
