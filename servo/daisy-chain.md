# Running three HX-30HM servos on one bus

Notes for extending the servo lesson from one servo to three, sharing signal,
power and ground. The bus servo is designed for this — each unit has two
PH2.0 sockets wired in parallel — but three of them raise one electrical
problem and one protocol problem that a single servo never exposes.

Written against measurements from the real part, not just the vendor
documents, which turned out to be wrong in several places (see
[Reference measurements](#reference-measurements)).

---

## Already verified

SYNC WRITE was tested against a single servo before committing to any
wiring. A packet addressing IDs 1, 2 and 3 was sent; the one servo present
extracted its own block and ignored the other two:

```
ff ff fe 19 83 2a 06  01 b0 04 00 00 58 02  02 60 09 ...  03 10 0e ...  e6
└─ hdr ─┘ │  │  │  │  └─── ID 1 → 1200 ──┘  └─ ID 2 ───┘  └─ ID 3 ───┘  └ cs
          │  │  │  └ bytes per servo (6)
          │  │  └ start address 0x2A
          │  └ SYNC WRITE
          └ length = (6+1)x3 + 4 = 25
```

Result: no reply (broadcast is never answered — correct), and `cur_pos`
settled at 1201 against a target of 1200. So the framing and checksum maths
are right; what remains is IDs, power, and widening the FPGA's parameter
path.

---

## 1. Unique IDs — first, and one servo at a time

Every HX-30HM ships as **ID 1**. Three fresh servos on one bus are three
devices answering to the same address.

The failure mode is not obvious and is not recoverable by retrying: writing
an ID while all three are connected renames **all of them at once**, leaving
three servos that still share an address and no way to tell them apart.

```sh
make bridge-flash
# connect ONE servo
python3 tools/hx.py setid 2
# disconnect, connect the next one
python3 tools/hx.py setid 3
# now daisy-chain all three
python3 tools/hx.py scan          # expect 1, 2, 3
```

`setid` scans the bus first and refuses to run if more than one servo
answers, which makes the mistake hard to commit by accident.

ID lives at register `0x05` in NVS, so it takes the same unlock sequence as
control mode: `0x37 <- 0`, write, `0x37 <- 1`.

---

## 2. Power — the part that actually constrains the design

| Condition | Per servo | Three servos |
|---|---|---|
| Idle (electronics only) | ~0.1 A | 0.3 A |
| Unloaded motion | ~0.18 A | ~0.5 A |
| Stall | up to 6 A | **up to 18 A** |

Two independent limits are in play.

**The supply.** A DPS3010U tops out at 10 A, so three simultaneously stalled
servos exceed it. That is survivable — the supply folds back — but it means
the current limit is doing real work, not sitting there decoratively.

**The connectors, which are the tighter limit.** JST PH 2.0 mm contacts are
rated around **2 A**. In a daisy-chain the *first* connector carries the
current for all three servos, so a chain is capped near 2 A total regardless
of what the supply can deliver. Connectors fail by melting, not by tripping,
and nothing upstream will warn you.

### Recommended wiring

- **Unloaded bench demo:** daisy-chain everything. ~0.5 A total is well
  inside the connector rating. Set the supply limit to 2 A so the wiring is
  protected rather than the servos.
- **Anything with load on the shafts:** daisy-chain **signal and ground
  only**, and **star-feed power** to each servo from a distribution point.
  This is the same reasoning as the decoupling discussion in the main plan —
  keep the high-current loop short and out of shared conductors.

### Grounding

With three servos returning current through one ground wire, the ground
bounce landing on the FPGA's SIG reference is three times worse than with
one servo.

**Tie the FPGA ground at the supply / star point, not at the far end of the
servo chain.** Otherwise servo return current flows along the very wire the
FPGA uses as its logic reference, and the noise appears directly at the
receive pin's threshold.

Bulk capacitance scales too: 100 µF + 100 nF near *each* servo, or at
minimum at the head of the chain.

---

## 3. Protocol — SYNC WRITE

Three separate WRITE packets start the servos a few hundred microseconds
apart. One SYNC WRITE lands them on the same tick. The gain is **simultaneity,
not bandwidth** — for a leg, an arm or a gimbal that difference is visible in
the motion.

```
FF FF FE <Length> 83 <StartAddr> <L>
  <ID1> <D1_1..D1_L>
  <ID2> <D2_1..D2_L>
  ...
  <CheckSum>

Length   = (L + 1) x N + 4          N = servo count, L = bytes per servo
CheckSum = ~(everything from FE to the last data byte) & 0xFF
```

For coordinated motion, `StartAddr = 0x2A` and `L = 6` — the atomic
position/time/speed block. Broadcast ID means no replies, so there is no
turnaround to wait for and no risk of two servos answering at once.

For feedback, use three individual READs rather than SYNC READ. SYNC READ
returns N separate reply packets in ID order, which needs a multi-packet
collector in the RX path; at three servos that complexity is not worth it.

---

## 4. FPGA changes required

Modest, and confined to the parameter path.

| What | Currently | Needed | Why |
|---|---|---|---|
| `hx_bus` params bus | 64 bits (8 bytes) | 256 bits (32 bytes) | SYNC WRITE for 3 servos needs 2 + 3x7 = **23** parameter bytes |
| `nparam` | 4 bits (max 15) | 6 bits | 23 does not fit in 4 bits |
| `pidx` counter | 4 bits | 6 bits | follows `nparam` |
| Broadcast path | works | no change | `expect_reply = 0` already skips the reply wait and the timeout |
| Bridge `pbuf` | 64 bytes | no change | 29-byte packet; good to 8 servos ((6+1)x8+4 = 60) |
| `servo.v` | one target | per-servo targets | a sync-move macro replacing the single-target move |

Everything else — checksum accumulation, echo gating, the inter-frame gap —
is unchanged and already correct for multi-servo use.

Sizing note: 32 parameter bytes covers 4 servos (2 + 4x7 = 30). Beyond that,
swap the flattened bus for a small parameter RAM rather than growing the
vector further.

---

## 5. Suggested order

1. Assign IDs 1 / 2 / 3 individually with `setid`.
2. Wire: signal + ground chained, power star-fed, supply limit 2 A.
3. `python3 tools/hx.py scan` — all three must answer before going further.
4. `python3 tools/hx.py syncmove 1:1000 2:2000 3:3000` to confirm coordinated
   motion from the host.
5. Only then widen `hx_bus` and add the sync-move macro so the FPGA drives
   the chain standalone.

Steps 1–4 need no FPGA changes at all; the bridge bitstream already carries
them. That ordering means the wiring and the IDs are proven before any RTL
is touched, so a failure at step 5 can only be the RTL.

---

## Reference measurements

Taken from the real servo at 11.1 V over the bridge. Several of these
contradict Hiwonder's register table.

| Quantity | Value | Note |
|---|---|---|
| Top speed | ~2850 steps/s (~42 RPM) | Same in constant-speed and open-loop PWM mode, so it is a mechanical limit |
| Motor current at top speed, unloaded | 70–84 mA | Total supply draw ~0.1 A — a low reading is **not** evidence of a stall |
| Position resolution | 4096 steps / 360° | `cur_pos` counts multi-turn and exceeds 4095 |
| Voltage register `0x3E` | **millivolts** | Reads 11119 on an 11.12 V rail; the table says 0.1 V, which would give 1111.9 V |
| `0x21` control mode | **1 byte** | Table says 2. A 2-byte write happens to work because the extra byte lands on `0x22` |
| `0x13` max torque | **1 byte** | Table says 2 while also listing `0x14` separately — both cannot be true |

Signal integrity at 1 Mbaud with three loads has not been measured. If
checksum errors appear, shorten the bus before reaching for a lower baud
rate — changing baud means an NVS write to `0x06` on every servo, and
getting it wrong locks you out until you guess the new rate.
