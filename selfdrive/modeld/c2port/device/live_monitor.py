#!/usr/bin/env python3
"""Watch modelV2 from the bench modeld for a fixed time; abort (exit 3) if the car goes onroad."""
import sys, time
import numpy as np
sys.path.insert(0, '/data/openpilot')
import cereal.messaging as messaging
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30
sm = messaging.SubMaster(['modelV2', 'cameraOdometry'])
t0, n, exec_t, drops, finite, bad_frames = time.monotonic(), 0, [], [], True, 0
first = None
while time.monotonic() - t0 < dur:
  if open('/data/params/d/IsOnroad').read().strip() == '1':
    print('ONROAD -- aborting'); sys.exit(3)
  sm.update(100)
  if sm.updated['modelV2']:
    m = sm['modelV2']
    if first is None: first = time.monotonic() - t0
    n += 1
    exec_t.append(m.modelExecutionTime * 1000); drops.append(m.frameDropPerc)
    arrs = [np.array(m.position.x), np.array(m.position.y), np.array(m.laneLineProbs), np.array([l.prob for l in m.leadsV3])]
    ok = all(np.isfinite(a).all() for a in arrs) and len(m.position.x) == 33
    finite &= ok; bad_frames += (not ok)
    last = m
if n == 0:
  print('no modelV2 received'); sys.exit(2)
et = np.array(exec_t[5:] or exec_t)
print(f'modelV2: {n} msgs in {dur:.0f}s ({n/(dur-first):.1f} Hz after first at {first:.1f}s)')
print(f'modelExecutionTime ms: median {np.median(et):.1f}  p90 {np.percentile(et, 90):.1f}  max {et.max():.1f}')
print(f'frameDropPerc: median {np.median(drops):.1f}  max {max(drops):.1f}   all finite/33pts: {finite} (bad {bad_frames})')
print(f'last: pos.x[-1] {last.position.x[-1]:.1f} m  laneLineProbs {[round(p, 2) for p in last.laneLineProbs]}  lead probs {[round(l.prob, 2) for l in last.leadsV3]}')
print(f'cameraOdometry received: {sm.rcv_frame["cameraOdometry"] > 0}')
