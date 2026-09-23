#!/data/data/com.termux/files/usr/bin/bash
# Put /data/openpilot's modeld back exactly as it was before install_c2port.sh. Offroad only. No rebuild needed.
set -eu
OP=/data/openpilot
BK=/data/c2port/backup
[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "car is onroad, not uninstalling"; exit 1; }
[ -f $BK/modeld_src.tgz ] && [ -f $BK/_modeld.default ] || { echo "no backup in $BK"; exit 1; }

rm -f /data/c2port/USE_C2PORT
(cd $OP && tar xzf $BK/modeld_src.tgz)
cp -p $BK/_modeld.default $OP/selfdrive/modeld/_modeld
rm -f $OP/selfdrive/modeld/_modeld_c2port $OP/selfdrive/modeld/models/supercombo_c2port.thneed \
      $OP/selfdrive/modeld/modeld_c2port.o $OP/selfdrive/modeld/models/driving_c2port.o
cd $OP && git status --porcelain selfdrive/modeld
echo "restored the original modeld (git status above should be empty for selfdrive/modeld)"
