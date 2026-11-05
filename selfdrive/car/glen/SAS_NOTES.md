# Steering angle: china-sas board (replaces stock SAS and micro_sas)

From 2026-11-01 a single board, the "china-sas" firmware in `~/Developer/china-sas/firmware`,
sits where the stock steering angle sensor was. It does two jobs, both on **bus 0**:

| ID | Rate | Content | Used by |
|---|---|---|---|
| 0x25 `STEER_ANGLE_SENSOR` | ~83 Hz, sent right after the skid ECU's 0x0B4 | stock format: status nibble, angle (1.5 deg/bit), reference angle in bytes 2-3, frame timing in bytes 4-6, Toyota checksum | skid control (ABS/VSC) ECU, `STATUS` here |
| 0x23 `SECONDARY_STEER_ANGLE` | 27.8 Hz | signed 32-bit, 360/16384 deg per count, sent only while the angle is valid | `steeringAngleDeg` |
| 0x423 / 0x4C6 | ~1 Hz | constant, as the stock sensor sends them | car |

micro_sas, with its 0x23 and 0x231 on bus 2, is no longer used. It is still wired on bus 1,
asleep (it only wakes on 0x001).

## carstate.py

* **Angle:** `SECONDARY_STEER_ANGLE` comes from `cp0` (bus 0), with a 20 Hz check, so the
  parser times out after 500 ms without it.
* **Decoding:** the angle is rebuilt from `SIGN` (bit 31) and the low 16 bits `ANGLE` as
  `counts = ANGLE - 65536 if SIGN`. This is exact for |angle| < 1440 deg. Don't use
  `ZORRO_STEER`: this libdbc mis-signs a full 32-bit signal, so 0 deg decodes as about
  -94,377,599 deg.
* **Gate:** steering is allowed only while the 0x25 status nibble (`STEER_ANGLE_SENSOR.STATUS`,
  bits 7..4 of byte 0) is 0. Status 0 means centre stored, tracking, and both magnet sensors
  agree. Anything else sets `steerFaultTemporary`. This replaces micro_sas's 0x231
  `CALIBRATED` gate.

## Things that tripped up the skid control ECU (ABS lamp + U0073)

The board's firmware README, section "How the 0x25 timing has to look", has the details.
In short:
* **Sync:** 0x25 has to be sent once per skid-ECU cycle, right after its 0x0B4. The cycle
  is 0x0B0, 0x0B2, 0x020, 0x0B4 every 12 ms. A free-running 0x25 at either 83 or 100 Hz
  lit the lamp.
* **Bytes 4-6 must keep changing** (the measured frame interval).
* **No angle chatter** between neighbouring 1.5 deg steps.
* **Termination:** the sensor connector is a bus end. The board's 120 ohm termination is
  required: the car reads 60 ohm only with it fitted.

In the logs, the ABS lamp request shows up as 0x223 byte 5 = 0x10 (0x3B7 byte 0 = 0x38).

## Service

The board answers on 0x6F0/0x6F1 (commands/replies) and takes updates on 0x6F2, using
`tools/sas_tool.py` in the firmware repo. On the comma, the tool and helper scripts live in
`/data/sas/`; openpilot has to be stopped while they use the panda.
