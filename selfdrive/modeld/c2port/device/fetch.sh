#!/data/data/com.termux/files/usr/bin/bash
# Fetch public inputs for the comma two port into /data/c2port. Safe to rerun.
set -e
cd /data/c2port
MODEL_URL=https://media.githubusercontent.com/media/commaai/openpilot/66fd3d1/openpilot/selfdrive/modeld/models/driving_supercombo.onnx
MODEL_SHA=65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce
if ! echo "$MODEL_SHA  driving_supercombo.onnx" | sha256sum -c --status 2>/dev/null; then
  curl -fL --retry 5 -o driving_supercombo.onnx.part "$MODEL_URL"
  echo "$MODEL_SHA  driving_supercombo.onnx.part" | sha256sum -c
  mv driving_supercombo.onnx.part driving_supercombo.onnx
fi
echo "model ok"
if [ ! -d tinygrad_repo/.git ]; then
  rm -rf tinygrad_repo && mkdir tinygrad_repo && cd tinygrad_repo
  git init -q && git remote add origin https://github.com/tinygrad/tinygrad.git
  git fetch -q --depth 1 origin 2e9b7637b3c3c8895fda9f964215db3a35fe3441
  git checkout -q FETCH_HEAD
  git apply ../tinygrad_c2port.patch
  cd ..
fi
git -C tinygrad_repo log -1 --format='tinygrad %h %s'
git -C tinygrad_repo diff --stat | tail -1
python3 build_model.py recipe.pkl driving_supercombo.onnx supercombo_c2.onnxpkl
echo FETCH_DONE
