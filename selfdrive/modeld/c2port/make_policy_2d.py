#!/usr/bin/env python3
"""Rewrite the policy transformer of supercombo_c2.onnx into 2D ops that tinygrad 2022's image backend can run.

The policy has one transformer block over 9 tokens (8 history features + the current one), and only the last token's
output is used. Exact restructuring:
  - tokens are [9,512] rows (no batch dim); embedding MLP, LayerNorm 1 and the QKV projection run over all 9 rows
  - attention only for the last query: scores = (K * q) @ Hsum, Hsum [512,8] sums each head's 64 channels;
    softmax over the 9 keys; out = sum_rows((A^T @ Hsum^T) * V)
  - output projection, residual, LayerNorm 2 and the GELU MLP on the last row only
No activation x activation matmuls, no rank > 2 tensors. feat_hist becomes [8,512].

usage: make_policy_2d.py supercombo_c2.onnx supercombo_c2_2d.onnx
"""
import sys

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto, shape_inference

m = onnx.load(sys.argv[1])
g = m.graph
inits = {t.name: t for t in g.initializer}
by_out = {n.output[0].split('/')[-1]: n for n in g.node}
W = lambda node, i: node.input[i]                               # initializer name of a node input
val = lambda name: numpy_helper.to_array(inits[name])

def node(suffix, op):
  n = by_out[suffix]
  assert n.op_type == op, (suffix, n.op_type, op)
  return n

# weights of the original block, located by the node outputs they feed (asserted, so a different model fails loudly)
emb_w1, emb_b1 = W(node('p_val_13', 'MatMul'), 1), W(node('p_linear', 'Add'), 1)
emb_w2, emb_b2 = W(node('p_val_16', 'MatMul'), 1), W(node('p_linear_1', 'Add'), 1)
pos = W(node('p_add', 'Add'), 1)
ln1_g, ln1_b = W(node('ln_245', 'Mul'), 1), W(node('p_layer_norm', 'Add'), 1)
qkv_w, qkv_b = W(node('p_val_29', 'MatMul'), 1), W(node('p_linear_6', 'Add'), 1)
scale = float(val(W(node('p_mul', 'Mul'), 1)).ravel()[0])
mask = val(W(node('p_masked_fill', 'Add'), 1))
o_w, o_b = W(node('p_val_58', 'MatMul'), 1), W(node('p_linear_7', 'Add'), 1)
ln2_g, ln2_b = W(node('ln_255', 'Mul'), 1), W(node('p_layer_norm_1', 'Add'), 1)
m1_w, m1_b = W(node('p_val_62', 'MatMul'), 1), W(node('p_linear_8', 'Add'), 1)
m2_w, m2_b = W(node('p_val_64', 'MatMul'), 1), W(node('p_linear_9', 'Add'), 1)
eps = float(val(node('ln_242', 'Add').input[1]).ravel()[0])
desire_emb = node('p_unsqueeze', 'Unsqueeze').input[0]         # [1,512]
tc_emb = node('p_unsqueeze_1', 'Unsqueeze').input[0]           # [1,512]
hidden = node('hidden', 'Slice').output[0]                     # current vision feature [1,512]
p_select = node('p_select', 'Squeeze').output[0]               # block output consumed by the policy heads
assert val(qkv_w).shape == (512, 1536) and val(pos).shape == (1, 9, 512) and mask.shape == (1, 1, 9, 9)
heads, dh = 8, 64

new, uid = [], [0]
def t(base):
  uid[0] += 1
  return f'p2d/{base}_{uid[0]}'
def const(arr, base):
  name = t(base)
  g.initializer.append(numpy_helper.from_array(np.asarray(arr, dtype=np.asarray(arr).dtype), name))
  return name
def op(op_type, inputs, **attrs):
  out = t(op_type.lower())
  new.append(helper.make_node(op_type, inputs, [out], **attrs))
  return out
def linear(x, w, b):
  return op('Add', [op('MatMul', [x, w]), b])
def layernorm(x, gamma, beta):
  mu = op('ReduceMean', [x], axes=[-1], keepdims=1)
  d = op('Sub', [x, mu])
  var = op('ReduceMean', [op('Mul', [d, d])], axes=[-1], keepdims=1)
  sd = op('Sqrt', [op('Add', [var, const(np.float32([eps]), 'eps')])])
  return op('Add', [op('Mul', [op('Div', [d, sd]), gamma]), beta])
def gelu(x):  # fp16-safe tanh-GELU, same form as make_c2_onnx.py
  xc = op('Clip', [x, const(np.float32([-10]), 'lo'), const(np.float32([10]), 'hi')])
  cube = op('Mul', [op('Mul', [xc, xc]), xc])
  inner = op('Add', [xc, op('Mul', [cube, const(np.float32([0.044715]), 'c')])])
  return op('Mul', [x, op('Sigmoid', [op('Mul', [inner, const(np.float32([2 * np.sqrt(2 / np.pi)]), 'k2')])])])
def rows(x, start, end):
  return op('Slice', [x, const(np.int64([start]), 's'), const(np.int64([end]), 'e'), const(np.int64([0]), 'ax')])
def cols(x, start, end):
  return op('Slice', [x, const(np.int64([start]), 's'), const(np.int64([end]), 'e'), const(np.int64([1]), 'ax')])

# ---- token embedding over the 9 rows
x_in = op('Concat', ['feat_hist', hidden], axis=0)                                     # [9,512]
e = linear(x_in, emb_w1, emb_b1)
e = op('Mul', [e, op('Sigmoid', [e])])                                                 # SiLU
e = linear(e, emb_w2, emb_b2)
pos2d = const(val(pos).reshape(9, 512).astype(np.float32), 'pos')  # same values as the original initializer
x = op('Add', [op('Add', [op('Add', [e, pos2d]), desire_emb]), tc_emb])                # [9,512]

# ---- attention, last query only
l1 = layernorm(x, ln1_g, ln1_b)
qkv = linear(l1, qkv_w, qkv_b)                                                         # [9,1536]
k, v = cols(qkv, 512, 1024), cols(qkv, 1024, 1536)                                     # [9,512]
q = rows(cols(qkv, 0, 512), 8, 9)                                                      # [1,512]
hsum = np.zeros((heads * dh, heads), np.float32)
hsum[np.arange(heads * dh), np.arange(heads * dh) // dh] = 1
s = op('MatMul', [op('Mul', [k, q]), const(hsum, 'hsum')])                              # [9,8]  scores[key, head]
s = op('Add', [op('Mul', [op('Transpose', [s], perm=[1, 0]), const(np.float32([scale]), 'scale')]),
               const(mask[0, 0, 8:9, :].astype(np.float32), 'mask_row')])               # [8,9]
a = op('Softmax', [s], axis=-1)                                                        # [8,9]
a_exp = op('MatMul', [op('Transpose', [a], perm=[1, 0]), const(hsum.T.copy(), 'hexp')])  # [9,512]
attn = op('ReduceSum', [op('Mul', [a_exp, v])], axes=[0], keepdims=1)                  # [1,512]

# ---- rest of the block on the last row
r1 = op('Add', [rows(x, 8, 9), linear(attn, o_w, o_b)])
mlp = linear(gelu(linear(layernorm(r1, ln2_g, ln2_b), m1_w, m1_b)), m2_w, m2_b)
new.append(helper.make_node('Add', [r1, mlp], [p_select], name='p2d/out'))

# ---- splice: drop the old block (everything between the inputs and p_select that only fed the block)
prod = {o: n for n in g.node for o in n.output}
old_block = set()
stack = [p_select]
keep_srcs = {hidden, desire_emb, tc_emb, 'feat_hist'}
while stack:
  tname = stack.pop()
  if tname in keep_srcs or tname not in prod:
    continue
  n = prod[tname]
  if id(n) not in old_block:
    old_block.add(id(n))
    stack.extend(n.input)
nodes = [n for n in g.node if id(n) not in old_block]
insert_at = next(i for i, n in enumerate(nodes) if p_select in n.input)
nodes[insert_at:insert_at] = new
del g.node[:]
g.node.extend(nodes)
for i in g.input:
  if i.name == 'feat_hist':
    i.CopyFrom(helper.make_tensor_value_info('feat_hist', TensorProto.FLOAT, [8, 512]))
used = {x for n in g.node for x in n.input}
for tt in [tt for tt in g.initializer if tt.name not in used]:
  g.initializer.remove(tt)
for n in g.node:
  if not n.name:
    n.name = n.output[0]
del g.value_info[:]
m = shape_inference.infer_shapes(m)
onnx.checker.check_model(m, full_check=True)
onnx.save(m, sys.argv[2])
ranks = {len(v.type.tensor_type.shape.dim) for v in m.graph.value_info if any(v.name == n.output[0] for n in new)}
print(f'wrote {sys.argv[2]}: removed {len(old_block)} nodes, added {len(new)}, ranks in new block {sorted(ranks)}')
