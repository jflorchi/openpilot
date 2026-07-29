# Glen — OBD-II diagnostics notes

Reference for reading and clearing DTCs on this car. Written up from a scanner session; see
**Provenance** at the bottom for what is confirmed and what is not, because a couple of the
values here have never been checked against a log.

## Bus and addressing

Everything below is **bus 0**. Diagnostic frames were also observed on bus 1 with identical
counts — same powertrain segment seen through the harness — so either works, but bus 0 is
where `0x2C4` lives, which puts detection and clearing on one interface.

| Address | Direction | Purpose |
|---|---|---|
| `0x7DF` | request | Functional (broadcast) OBD request. 8 bytes, zero padded. |
| `0x7E8`–`0x7EF` | response | ECU responses. Only `0x7E8` ever answered on this car. |
| `0x7E0` | request | ISO-TP **flow control** for a responder at `0x7E8` (see below). |
| `0x2C4` | broadcast | MIL state in byte 4 bit 6. Confirmed: 1 → 0 at 42.50s, 80 ms after the clear. |

Flow control goes to the *physical* address of whoever answered, which is `responder − 8`.
`0x7E8 → 0x7E0`. Sending it back to `0x7DF` does not work.

## Services

| Purpose | Request on `0x7DF` | Response |
|---|---|---|
| Stored DTCs | `01 03 00 00 00 00 00 00` | `04 43 01 04 20 …` |
| Pending DTCs | `01 07 00 00 00 00 00 00` | `04 47 01 04 20 …` |
| Permanent DTCs | `01 0A 00 00 00 00 00 00` | **no response at all** |
| MIL + DTC count | `02 01 01 00 00 00 00 00` | `06 41 01 81 07 E5 04 …` |
| Vehicle speed | `02 01 0D 00 00 00 00 00` | `03 41 0D <km/h>` |
| Clear DTCs | `01 04 00 00 00 00 00 00` | `01 44 …` |

A positive response is `0x40 + mode` (`03 → 43`, `07 → 47`, `04 → 44`). A negative response is
`7F <mode> <code>`; `0x31` means "request out of range" / not supported.

### Mode 0A returns nothing

Three requests at 32.29s, 33.91s and 35.55s, no reply to any of them — not even a negative
response, while Mode 03 and 07 answered normally either side of them. Treat a Mode 0A timeout on this ECM as
**"service unsupported"**, never as **"there are no permanent codes."** Those two states are
indistinguishable on the wire, and conflating them means an empty-set assumption based on
silence. If this ECM is ever swapped, re-check before assuming.

### Mode 01 PID 01 layout

`41 01 A B C D`

- **A** — bit 7 is the MIL, bits 0–6 are the stored DTC count. `0x81` = MIL on, 1 code.
- **B, C, D** — readiness monitor support/completion bits.

Across the clear at 42.42s:

```
36.07  0x7df  02 01 01 ...      42.58  0x7df  02 01 01 ...
36.08  0x7e8  06 41 01 81 07 E5 04     0x7e8  06 41 01 00 07 E5 E5
                          ^^       ^^                ^^       ^^
                          A        D                 A        D
```

`A` goes `0x81 → 0x00` (MIL off, count 0) and `D` goes `0x04 → 0xE5`, consistent with monitors
being reset. `B` and `C` are unchanged.

Note `0x3B7` is **not** a readiness field, despite an earlier note claiming it went
`0x1F → 0x00`. It transmits 1408 times across the route carrying the constant value
`58 1b 00 00 00 00 00 00`, unchanged before and after the clear.

## Decoding a DTC

Two bytes per code. First two bits pick the letter, next two the digit, remaining nibbles are hex:

```python
def decode_dtc(b1, b2):
    return "%s%d%X%X%X" % ("PCBU"[b1 >> 6], (b1 >> 4) & 0x3, b1 & 0xF, b2 >> 4, b2 & 0xF)

decode_dtc(0x04, 0x20)   # "P0420"  <- catalyst efficiency below threshold, bank 1
decode_dtc(0x01, 0x30)   # "P0130"
decode_dtc(0x43, 0x21)   # "C0321"
```

`00 00` is padding, not a code — filter out `P0000`.

## ISO-TP: the part that bites

With a single DTC the Mode 03 reply fits in one frame, so a naive parser appears to work. Three
or more codes overflow into multi-frame, and a single-frame-only parser will see a First Frame,
fail to parse it, and conclude the car has no other faults. If that result is feeding a decision
to clear, it clears a set it never actually read.

The frame's first nibble is the PCI type:

| PCI | Type | Layout |
|---|---|---|
| `0x` | Single frame | `0L <L bytes>` |
| `1x` | First frame | `1L LL <6 bytes>`, total length `0xLLL` |
| `2x` | Consecutive frame | `2N <7 bytes>`, `N` = sequence 0–15, wrapping |
| `3x` | Flow control | `30 <block size> <separation time>` |

A real multi-frame exchange from a VIN read on this car:

```
0x7E8  10 14 49 02 01 32 54 31   <- First Frame, total length 0x014 = 20
0x7E0  30 00 00 00 00 00 00 00   <- flow control, sent to responder-8
0x7E8  21 42 55 34 45 45 33 41   <- Consecutive Frame 1
0x7E8  22 43 32 30 34 32 32 36   <- Consecutive Frame 2
```

`30 00 00` = clear to send, no block limit, no minimum separation.

## Clearing

Mode 04 is **all or nothing**. There is no standard service to drop one DTC and leave the rest.
So "clear only P0420" cannot mean selective clearing — the only coherent reading is conditional:

> Read the complete set — stored ∪ pending ∪ permanent. If it is *exactly* `{P0420}`, clear.
> If anything else is present, do nothing.

Two details that matter if this is ever automated:

- Use **equality, not a subset test**. `"P0420" in codes` will happily wipe a P0420 that is
  sitting next to a misfire.
- A **failed read is not an empty set.** Distinguish "read returned no codes" from "read failed"
  and refuse on the latter. Ambiguity must not clear.

### Side effects of Mode 04

- All readiness monitors reset to not-ready, and stay that way until the car completes its drive
  cycles. For an OBD-II inspection this is not a workaround: "monitors incomplete" fails the
  same as a lit MIL. Alberta has no light-duty emissions test, so this is only relevant if the
  car is sold or registered somewhere that does.
- Fuel trims and adaptive values are reset on most ECMs. Expect somewhat different idle and
  driveability until they relearn. Clearing every drive cycle keeps the car permanently
  relearning.
- Freeze frame data is discarded, so the diagnostic trail for the fault is gone.
- **Do not send Mode 04 while moving.** Gate on PID 0D reading 0 km/h, and ideally park/neutral.
  The logged session did this correctly — the PID 0D read immediately before the clear (35.88s)
  returned `0x00`, and the fastest reading anywhere in that route was `0x48` = 72 km/h.

## Talking to the bus from a comma device

There is no socketcan here — CAN goes through the panda via boardd, so `python-can` will not
work. A process TXes by publishing to the `sendcan` socket and RXes from `can`:

```python
import cereal.messaging as messaging

can_sock = messaging.sub_sock('can', conflate=False, timeout=100)
sendcan = messaging.pub_sock('sendcan')

msg = messaging.new_message('sendcan', 1)
msg.sendcan[0].address = 0x7DF
msg.sendcan[0].dat = bytes([0x01, 0x03, 0, 0, 0, 0, 0, 0])
msg.sendcan[0].src = 0
sendcan.send(msg.to_bytes())
```

boardd must be running, which means the key must be on. Note this car is configured with
`SafetyModel.allOutput` (`interface.py`), so panda does **not** filter these frames — which is
why arbitrary diagnostic requests work at all, and why it is worth being deliberate about what
gets put on that bus.

## Provenance

The scanner session is captured in openpilot's own rlogs, route `2026-07-29--00-38-39`
(99 diagnostic frames on bus 0). Everything above was replayed out of it and confirmed:
the `0x7DF` / `0x7E8` byte sequences, Mode 03 and 07 both returning `01 04 20`, Mode 0A's
silence, the `06 41 01 81 07 E5 04` status response, the clear at 42.42s answering `01 44`,
the VIN multi-frame at 25.38s with flow control on `0x7E0`, `0x2C4` byte 4 bit 6 dropping
80 ms after the clear, and the same frames appearing on bus 1 with identical counts.

Two things in the original write-up did **not** survive checking:

- `0x3B7` does not carry readiness and never went `0x1F → 0x00`; it is constant at
  `58 1b 00 00 00 00 00 00` for all 1408 transmissions in that route.
- The clear was **not** sent at 49.9 mph under cruise. PID 0D read `0x00` immediately before it.

Still unverified:

- The exact capnp field names in the `sendcan` snippet above were not executed.
- `0x08` in the gear byte and other DTCs than P0420 have never been observed, so the multi-frame
  path here has only been exercised by the VIN read, not by a multi-DTC Mode 03 response.

ISO-TP framing, DTC decoding and Mode 04 semantics are standard and independent of this car.
