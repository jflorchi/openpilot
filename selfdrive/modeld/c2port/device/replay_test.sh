#!/data/data/com.termux/files/usr/bin/bash
# Replay a logged drive into the ported modeld (from /data/c2port/op) and record its modelV2. Car parked/offroad.
# usage: replay_test.sh <route timestamp> <first segment> <number of segments> [speed, default 0.5]
# Replay publishes ONLY what modeld needs (camera frames, calibration, desire): nothing that reaches the car
# (sendcan) or manager (deviceState, pandaStates). controlsd / plannerd are not started.
# The segments are staged in /data/c2port/replaydata (video symlinked, rlog bz2-compressed, which replay requires);
# /data/media is only read.
# env MODELD_CMD: how to start modeld (default: the build copy's launcher); TAG: suffix for the output file.
set -u
ROUTE=$1; SEG0=$2; NSEG=$3; SPEED=${4:-0.5}
onroad() { [ "$(cat /data/params/d/IsOnroad)" = "1" ]; }
onroad && { echo "car is onroad, not starting"; exit 1; }
pgrep -f "camerad|selfdrive/modeld/modeld|_modeld" > /dev/null && { echo "camerad/modeld already running, not starting"; exit 1; }
cd /data/c2port/op && scons -j4 selfdrive/modeld/_modeld > /data/c2port/scons2.log 2>&1 || { echo "BUILD FAILED"; tail -20 /data/c2port/scons2.log; exit 1; }

STAGE=/data/c2port/replaydata
rm -rf $STAGE && mkdir -p $STAGE
for s in $(seq $SEG0 $((SEG0 + NSEG - 1))); do
  src=/data/media/0/realdata/$ROUTE--$s; dst=$STAGE/$ROUTE--$s
  [ -f $src/fcamera.hevc ] && [ -f $src/rlog ] || { echo "missing $src"; exit 1; }
  mkdir -p $dst && ln -s $src/fcamera.hevc $dst/fcamera.hevc && bzip2 -c $src/rlog > $dst/rlog.bz2
done
echo "staged segments $SEG0..$((SEG0 + NSEG - 1))"

OUT=/data/c2port/replay_${ROUTE}_${SEG0}${TAG:+_$TAG}.npz
MODELD_CMD=${MODELD_CMD:-"cd /data/c2port/op/selfdrive/modeld && exec ./modeld"}
ALLOW=roadCameraState,roadEncodeIdx,liveCalibration,lateralPlan
tmux kill-session -t c2replay 2>/dev/null
sh -c "$MODELD_CMD" > /data/c2port/modeld_replay.log 2>&1 & MOD=$!
sleep 3
tmux new-session -d -s c2replay "cd /data/c2port/op/selfdrive/ui/replay && REPLAY_SPEED=$SPEED ./replay --data_dir $STAGE --allow $ALLOW --no-loop --start $((SEG0 * 60)) '47165e0be3802647|$ROUTE' 2> /data/c2port/replay.log"
echo "modeld $MOD ($MODELD_CMD), replay in tmux c2replay, speed $SPEED"
MAXS=$(python3 -c "print(int($NSEG * 60 / $SPEED + 90))")
PYTHONPATH=/data/openpilot python3 /data/c2port/replay_record.py $OUT $MAXS
tmux kill-session -t c2replay 2>/dev/null
kill -INT $MOD 2>/dev/null; sleep 2; kill -0 $MOD 2>/dev/null && kill -TERM $MOD
echo "--- modeld log tail"; grep -avE "unused DT" /data/c2port/modeld_replay.log | tail -5
echo "--- replay log tail"; tail -5 /data/c2port/replay.log
echo REPLAY_DONE $OUT
