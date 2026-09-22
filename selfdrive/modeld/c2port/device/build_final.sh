#!/data/data/com.termux/files/usr/bin/bash
# Build the final comma two port thneed (input normalized by modeld) and check it the way serialize.cc loads it.
cd /data/c2port
export PYTHONPATH=/data/c2port:/data/c2port/pylib:/data/c2port/tinygrad_repo
export HOME=/data/c2port/home XDG_CACHE_HOME=/data/c2port/home/.cache PYOPENCL_NO_CACHE=1
python3 build_model.py recipe_final.pkl driving_supercombo.onnx supercombo_c2_final.onnxpkl
cd tinygrad_repo
SAVE_SOURCE=1 OPT=99 OPENCL=1 NATIVE_EXPLOG=1 UNSAFE_FLOAT4=1 OPTWG=1 FLOAT16=1 \
  python3 openpilot/compile.py /data/c2port/supercombo_c2_final.onnxpkl /data/c2port/supercombo_c2port.thneed > /data/c2port/build_final_raw.log 2>&1
grep -E "total runtime|saving|Error|assert" /data/c2port/build_final_raw.log | tail -4
cd /data/c2port && ls -la supercombo_c2port.thneed && sha256sum supercombo_c2port.thneed | cut -c1-16
NORMALIZED=1 NATIVE_IMAGES=1 python3 thneed_check.py supercombo_c2port.thneed real_frames.npz 2>&1 | grep -vE "unused DT|CompilerWarning|warn\("
echo ALL_DONE
