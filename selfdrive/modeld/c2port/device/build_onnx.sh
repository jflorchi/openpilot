#!/data/data/com.termux/files/usr/bin/bash
# Build a comma two thneed from a single upstream driving_supercombo.onnx (e.g. Cheddar, via make_cheddar_c2.py).
# Offroad only; the installed model is not touched (install with install_c2port.sh after copying the result over
# /data/c2port/supercombo_c2port.thneed).
#
# usage: build_onnx.sh <name> <sha256>
#   needs /data/c2port/<name>/recipe.pkl and the usual /data/c2port setup (tinygrad + patch, pyopencl, real_frames.npz)
#   The ONNX is fetched by its LFS oid (= sha256) from upstream's LFS server (huggingface, see upstream .lfsconfig);
#   github's media URLs 404 for models stored there.
# result: /data/c2port/<name>/supercombo_c2port.thneed (split-K applied), checked against tinygrad fp32 on real frames
set -eu
NAME=$1 SHA=$2
[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "car is onroad, not building"; exit 1; }
D=/data/c2port/$NAME
cd $D
FILE=driving_supercombo.onnx
if ! echo "$SHA  $FILE" | sha256sum -c --status 2>/dev/null; then
  HREF=$(curl -fsS -X POST -H "Accept: application/vnd.git-lfs+json" -H "Content-Type: application/vnd.git-lfs+json" \
    https://huggingface.co/commaai/openpilot-lfs.git/info/lfs/objects/batch \
    -d "{\"operation\":\"download\",\"transfers\":[\"basic\"],\"objects\":[{\"oid\":\"$SHA\",\"size\":0}]}" \
    | python3 -c "import json,sys; print(json.load(sys.stdin)['objects'][0]['actions']['download']['href'])")
  curl -fL --retry 5 -o $FILE.part "$HREF"
  echo "$SHA  $FILE.part" | sha256sum -c
  mv $FILE.part $FILE
fi
echo "model ok"

export PYTHONPATH=/data/c2port:/data/c2port/pylib:/data/c2port/tinygrad_repo
export HOME=/data/c2port/home XDG_CACHE_HOME=/data/c2port/home/.cache PYOPENCL_NO_CACHE=1
python3 /data/c2port/build_model.py recipe.pkl $FILE model.onnxpkl
(cd /data/c2port/tinygrad_repo && git checkout -q -- . && git apply ../tinygrad_c2port.patch)
# compile.py's random feed needs the graph's desire_tokens shape (Cheddar: 9 windows)
export DESIRE_SHAPE=$(python3 -c "import pickle; r = pickle.load(open('recipe.pkl', 'rb')); print(','.join(str(d.dim_value) for v in r.graph.input if v.name == 'desire_tokens' for d in v.type.tensor_type.shape.dim))")
echo "desire_tokens shape $DESIRE_SHAPE"
(cd /data/c2port/tinygrad_repo && SAVE_SOURCE=1 OPT=99 OPENCL=1 NATIVE_EXPLOG=1 UNSAFE_FLOAT4=1 OPTWG=1 FLOAT16=1 \
  python3 openpilot/compile.py $D/model.onnxpkl $D/nosplitk.thneed > $D/compile.log 2>&1) \
  || { tail -20 $D/compile.log; exit 1; }
grep -E "total runtime|saving" compile.log | tail -2
OPT=99 OPENCL=1 UNSAFE_FLOAT4=1 NATIVE_EXPLOG=1 python3 /data/c2port/make_ref.py model.onnxpkl /data/c2port/real_frames.npz real_frames_ref.npz \
  2>&1 | grep -vE "unused DT|CompilerWarning|warn\("
echo "--- before split-K"
NORMALIZED=1 NATIVE_IMAGES=1 python3 /data/c2port/thneed_check.py nosplitk.thneed real_frames_ref.npz 2>&1 | grep -E "inputs:|real frame|GPU time|Error|assert"
python3 /data/c2port/splitk.py nosplitk.thneed /data/c2port/real_frames.npz supercombo_c2port.thneed splitk.json 2>&1 | grep -vE "unused DT|CompilerWarning|warn\(" | tail -3
echo "--- after split-K"
NORMALIZED=1 NATIVE_IMAGES=1 python3 /data/c2port/thneed_check.py supercombo_c2port.thneed real_frames_ref.npz 2>&1 | grep -E "real frame|GPU time|Error|assert"
ls -la supercombo_c2port.thneed && md5sum supercombo_c2port.thneed
echo BUILD_ONNX_DONE
