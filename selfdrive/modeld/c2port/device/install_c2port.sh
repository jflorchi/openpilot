#!/data/data/com.termux/files/usr/bin/bash
# Install a newly built model (/data/c2port/supercombo_c2port.thneed) into /data/openpilot and rebuild modeld. Offroad
# only. The ported model is the only driving model; modeld fails to start without the thneed. Takes effect at the next
# modeld start (reboot).
set -eu
OP=/data/openpilot

[ "$(cat /data/params/d/IsOnroad)" = "1" ] && { echo "car is onroad, not installing"; exit 1; }
[ -f /data/c2port/supercombo_c2port.thneed ] || { echo "missing /data/c2port/supercombo_c2port.thneed"; exit 1; }

cp /data/c2port/supercombo_c2port.thneed $OP/selfdrive/modeld/models/supercombo_c2port.thneed
cd $OP && scons -j4 selfdrive/modeld/_modeld > /data/c2port/scons_install.log 2>&1 \
  || { echo "BUILD FAILED (see /data/c2port/scons_install.log)"; exit 1; }
ls -la $OP/selfdrive/modeld/_modeld $OP/selfdrive/modeld/models/supercombo_c2port.thneed
echo "installed; reboot (offroad) to load it"
