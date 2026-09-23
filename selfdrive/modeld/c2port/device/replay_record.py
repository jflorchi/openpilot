#!/usr/bin/env python3
"""Record the ported modeld's modelV2 while replay feeds it a logged drive. usage: replay_record.py <out.npz> <max seconds>

Stops after <max seconds>, or 20 s after the last modelV2 once any arrived (replay reached the end).
"""
import sys
import time

import numpy as np
import cereal.messaging as messaging

out, max_s = sys.argv[1], float(sys.argv[2])
sm = messaging.SubMaster(['modelV2'])
rows = {k: [] for k in ('frame_id', 'x', 'y', 'vx', 'yaw', 'yaw_rate', 'exec_time', 'drop_perc',
                        'lead_prob', 'lead_x', 'lead_v', 'lead_a', 'hard_brake', 'engaged_prob')}
t0 = last = time.monotonic()
while time.monotonic() - t0 < max_s and not (rows['frame_id'] and time.monotonic() - last > 20):
  sm.update(100)
  if sm.updated['modelV2']:
    m = sm['modelV2']
    last = time.monotonic()
    if len(m.position.x) != 33:
      continue
    rows['frame_id'].append(m.frameId)
    rows['x'].append(list(m.position.x))
    rows['y'].append(list(m.position.y))
    rows['vx'].append(list(m.velocity.x))
    rows['yaw'].append(list(m.orientation.z))
    rows['yaw_rate'].append(list(m.orientationRate.z))
    rows['exec_time'].append(m.modelExecutionTime)
    rows['drop_perc'].append(m.frameDropPerc)
    lead = m.leadsV3[0] if len(m.leadsV3) else None   # what radard uses (t=0 lead)
    rows['lead_prob'].append(lead.prob if lead else float('nan'))
    rows['lead_x'].append(lead.x[0] if lead and len(lead.x) else float('nan'))
    rows['lead_v'].append(lead.v[0] if lead and len(lead.v) else float('nan'))
    rows['lead_a'].append(lead.a[0] if lead and len(lead.a) else float('nan'))
    rows['hard_brake'].append(float(m.meta.hardBrakePredicted))
    rows['engaged_prob'].append(m.meta.engagedProb)
    if len(rows['frame_id']) % 200 == 0:
      print(f"{len(rows['frame_id'])} frames, last frameId {m.frameId}, exec {m.modelExecutionTime * 1e3:.1f} ms", flush=True)
np.savez_compressed(out, **{k: np.array(v, np.float32 if k != 'frame_id' else np.int64) for k, v in rows.items()})
print(f"saved {len(rows['frame_id'])} frames to {out}", flush=True)
