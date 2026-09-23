#!/data/data/com.termux/files/usr/bin/bash
# Install the ported model into /data/openpilot next to the default one. Offroad only.
# Expects the modeld sources (this branch) copied to /data/c2port/deploy/selfdrive/modeld/... and the compiled model at
# /data/c2port/supercombo_c2port.thneed. Does NOT enable it: `touch /data/c2port/USE_C2PORT` does (next modeld start).
# Undo everything with uninstall_c2port.sh.
set -eu
OP=/data/openpilot
DEPLOY=/data/c2port/deploy
BK=/data/c2port/backup
FILES="selfdrive/modeld/SConscript selfdrive/modeld/modeld
selfdrive/modeld/models/driving.cc selfdrive/modeld/models/driving.h
selfdrive/modeld/models/commonmodel.cc selfdrive/modeld/models/commonmodel.h
selfdrive/modeld/thneed/thneed.cc selfdrive/modeld/thneed/thneed.h selfdrive/modeld/thneed/serialize.cc
selfdrive/modeld/transforms/loadyuv.cc selfdrive/modeld/transforms/loadyuv.h selfdrive/modeld/transforms/loadyuv.cl"

[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "car is onroad, not installing"; exit 1; }
[ -f /data/c2port/supercombo_c2port.thneed ] || { echo "missing /data/c2port/supercombo_c2port.thneed"; exit 1; }
for f in $FILES; do [ -f $DEPLOY/$f ] || { echo "missing $DEPLOY/$f"; exit 1; }; done

# back up the originals once (a reinstall keeps the first, pre-port backup)
if [ ! -f $BK/modeld_src.tgz ]; then
  mkdir -p $BK
  (cd $OP && tar czf $BK/modeld_src.tgz $FILES)
  cp -p $OP/selfdrive/modeld/_modeld $BK/_modeld.default
  (cd $OP && git rev-parse HEAD > $BK/HEAD && git status --porcelain > $BK/git_status)
  echo "backed up originals to $BK"
fi

for f in $FILES; do cp $DEPLOY/$f $OP/$f; done
chmod +x $OP/selfdrive/modeld/modeld
cp /data/c2port/supercombo_c2port.thneed $OP/selfdrive/modeld/models/supercombo_c2port.thneed

cd $OP && scons -j4 selfdrive/modeld/_modeld selfdrive/modeld/_modeld_c2port > /data/c2port/scons_install.log 2>&1 \
  || { echo "BUILD FAILED (see /data/c2port/scons_install.log); run uninstall_c2port.sh to restore"; exit 1; }
ls -la $OP/selfdrive/modeld/_modeld $OP/selfdrive/modeld/_modeld_c2port
echo "installed. enable: touch /data/c2port/USE_C2PORT   disable: rm /data/c2port/USE_C2PORT"
