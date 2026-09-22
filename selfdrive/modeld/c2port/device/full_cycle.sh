#!/data/data/com.termux/files/usr/bin/bash
# rebuild model + thneed, check it, install into the build copy, rebuild modeld, live bench with dump check
set -u
cd /data/c2port
[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "onroad, not starting"; exit 1; }
cd tinygrad_repo && git checkout -q -- . && git apply ../tinygrad_c2port.patch && cd ..
[ "${SKIP_THNEED:-0}" = 1 ] || ./build_final.sh 2>&1 | grep -vE "unused DT|CompilerWarning|warn\(" | grep -vE "^\s*$"
FILES="selfdrive/modeld/SConscript selfdrive/modeld/models/commonmodel.cc selfdrive/modeld/models/commonmodel.h selfdrive/modeld/models/driving.cc selfdrive/modeld/models/driving.h selfdrive/modeld/thneed/serialize.cc selfdrive/modeld/thneed/thneed.h selfdrive/modeld/thneed/thneed.cc selfdrive/modeld/transforms/loadyuv.cc selfdrive/modeld/transforms/loadyuv.cl selfdrive/modeld/transforms/loadyuv.h"
cd /data/c2port/op && for f in $FILES; do cp /data/openpilot/$f $f; done && git apply ../modeld_c2port.patch && echo "patch applied"
cp /data/c2port/supercombo_c2port.thneed /data/c2port/op/selfdrive/modeld/models/
cd /data/c2port && ./live_test.sh
echo CYCLE_DONE
