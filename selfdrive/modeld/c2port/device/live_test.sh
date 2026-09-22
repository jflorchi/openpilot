#!/data/data/com.termux/files/usr/bin/bash
# Bench: real camera -> ported modeld (from /data/c2port/op), car parked/offroad. camerad is stopped with SIGINT only.
set -u
onroad() { [ "$(cat /data/params/d/IsOnroad)" = "1" ]; }
onroad && { echo "car is onroad, not starting"; exit 1; }
cd /data/c2port/op && scons -j4 selfdrive/modeld/_modeld > /data/c2port/scons2.log 2>&1 || { echo "BUILD FAILED"; tail -20 /data/c2port/scons2.log; exit 1; }
echo "build ok"
rm -rf /data/c2port/dump && mkdir -p /data/c2port/dump
(cd /data/openpilot/selfdrive/camerad && exec ./camerad) > /data/c2port/camerad.log 2>&1 & CAM=$!
(cd /data/openpilot && PYTHONPATH=/data/openpilot exec python3 selfdrive/locationd/calibrationd.py) > /data/c2port/calibrationd.log 2>&1 & CAL=$!
sleep 3
(cd /data/c2port/op/selfdrive/modeld && C2PORT_DUMP=24 exec ./modeld) > /data/c2port/modeld.log 2>&1 & MOD=$!
echo "pids camerad $CAM calibrationd $CAL modeld $MOD"
PYTHONPATH=/data/openpilot python3 /data/c2port/live_monitor.py 40; MON=$?
kill -INT $MOD $CAL 2>/dev/null
kill -INT $CAM 2>/dev/null
for i in $(seq 1 20); do kill -0 $CAM 2>/dev/null || break; sleep 0.5; done
kill -0 $CAM 2>/dev/null && echo "WARNING: camerad still running after SIGINT (not force-killing, per process_config)" || echo "camerad stopped"
sleep 1; kill -0 $MOD 2>/dev/null && kill -TERM $MOD; kill -0 $CAL 2>/dev/null && kill -TERM $CAL
echo "monitor exit $MON"
echo "--- modeld log tail"; grep -vE "unused DT" /data/c2port/modeld.log | tail -8
ls /data/c2port/dump > /data/c2port/dumplist.txt; wc -l < /data/c2port/dumplist.txt
echo "--- dump check"
(cd /data/c2port && HOME=/data/c2port/home XDG_CACHE_HOME=/data/c2port/home/.cache OPT=99 OPENCL=1 UNSAFE_FLOAT4=1 NATIVE_EXPLOG=1 python3 -u check_dump4.py dump 24 2>&1 | grep -E "^frame|HISTORY|CHECK4|Error")
echo LIVE_DONE
