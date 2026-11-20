#!/data/data/com.termux/files/usr/bin/bash
# Build a comma two thneed from an upstream split (vision + policy) driving model, e.g. MacroStiff. Offroad only; the
# installed model is not touched (install with install_c2port.sh after copying the result over
# /data/c2port/supercombo_c2port.thneed).
#
# usage: build_split.sh <name> <commit> <vision sha256> <policy sha256>
#   needs /data/c2port/<name>/recipe.pkl (make_recipe.py on the merged graph, see ../README.md) and the usual
#   /data/c2port setup (fetch.sh: tinygrad + patch, install_pyopencl.sh, real_frames.npz)
# result: /data/c2port/<name>/supercombo_c2port.thneed (split-K applied), checked against tinygrad fp32 on real frames
set -eu
NAME=$1 COMMIT=$2 VSHA=$3 PSHA=$4
[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "car is onroad, not building"; exit 1; }
D=/data/c2port/$NAME
cd $D
for f in vision:$VSHA policy:$PSHA; do
  part=${f%%:*} sha=${f#*:}
  file=driving_$part.onnx
  if ! echo "$sha  $file" | sha256sum -c --status 2>/dev/null; then
    curl -fL --retry 5 -o $file.part https://media.githubusercontent.com/media/commaai/openpilot/$COMMIT/selfdrive/modeld/models/$file
    echo "$sha  $file.part" | sha256sum -c
    mv $file.part $file
  fi
done
echo "models ok"

export PYTHONPATH=/data/c2port:/data/c2port/pylib:/data/c2port/tinygrad_repo
export HOME=/data/c2port/home XDG_CACHE_HOME=/data/c2port/home/.cache PYOPENCL_NO_CACHE=1
python3 /data/c2port/build_model.py recipe.pkl driving_vision.onnx,driving_policy.onnx model.onnxpkl
(cd /data/c2port/tinygrad_repo && git checkout -q -- . && git apply ../tinygrad_c2port.patch)
(cd /data/c2port/tinygrad_repo && SAVE_SOURCE=1 OPT=99 OPENCL=1 NATIVE_EXPLOG=1 UNSAFE_FLOAT4=1 OPTWG=1 FLOAT16=1 \
  python3 openpilot/compile.py $D/model.onnxpkl $D/nosplitk.thneed > $D/compile.log 2>&1) \
  || { tail -20 $D/compile.log; exit 1; }
grep -E "total runtime|saving" compile.log | tail -2
OPT=99 OPENCL=1 UNSAFE_FLOAT4=1 NATIVE_EXPLOG=1 python3 /data/c2port/make_ref.py model.onnxpkl /data/c2port/real_frames.npz real_frames_ref.npz \
  2>&1 | grep -vE "unused DT|CompilerWarning|warn\("
echo "--- before split-K"
NORMALIZED=1 NATIVE_IMAGES=1 python3 /data/c2port/thneed_check.py nosplitk.thneed real_frames_ref.npz 2>&1 | grep -E "real frame|GPU time|Error|assert"
python3 /data/c2port/splitk.py nosplitk.thneed /data/c2port/real_frames.npz supercombo_c2port.thneed splitk.json 2>&1 | grep -vE "unused DT|CompilerWarning|warn\(" | tail -3
echo "--- after split-K"
NORMALIZED=1 NATIVE_IMAGES=1 python3 /data/c2port/thneed_check.py supercombo_c2port.thneed real_frames_ref.npz 2>&1 | grep -E "real frame|GPU time|Error|assert"
ls -la supercombo_c2port.thneed && md5sum supercombo_c2port.thneed
echo BUILD_SPLIT_DONE
