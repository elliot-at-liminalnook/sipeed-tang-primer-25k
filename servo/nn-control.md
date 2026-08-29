# Driving the servos from a neural network

Architecture notes for the eventual pipeline:

```
  [ computer ]        neural network policy
       │  WiFi (UDP)
  [  ESP32   ]        network transport only
       │  UART, 3.3 V, 2 Mbaud, framed + CRC
  [  FPGA    ]        control tick, safety envelope, protocol
       │  1 Mbaud half-duplex        │ PWM
  [ servo chain ]              [ DC motors ]
```

The short version: the ESP32 is a radio, the computer is a policy, and the
FPGA is the only part of the chain that can promise timing or refuse a
dangerous command. Put the real-time work and the safety envelope there.

---

## What having a policy in the loop changes

Three pieces of guidance that are correct for teleoperation are wrong once a
network is generating the commands. Recording them here because the earlier
reasoning in this repo assumed a human at the controls.

**Telemetry becomes the observation path.** For teleop, servo state is a
nice-to-have polled at a few hertz. For a policy it is the *input*, needed
every tick. Each control cycle is now read-then-write, not write-only, and
the read is on the critical path.

**SYNC READ is therefore worth implementing.** `daisy-chain.md` argues
against it for three servos, on the grounds that three individual READs are
simpler and the complexity is not worth it. That holds when reads are
occasional. When every tick needs a full observation vector it inverts: one
SYNC READ packet queries the whole chain and the servos reply in ID order.
Reading 8 bytes from `0x38` returns position, speed, load and voltage in a
single transaction.

**A fixed-rate tick matters more than low latency.** Policies trained in
simulation assume a constant timestep. Jitter in when actions get applied
degrades sim-to-real transfer in a way that presents as a bad policy rather
than a bad timebase, which is a miserable thing to debug. The FPGA has a
50 MHz crystal and no competing work: it should generate an exact 50 or
100 Hz tick, sample telemetry at a fixed phase, and apply actions at a fixed
phase. Neither an RTOS on the ESP32 nor Python on the host can promise that.

---

## Latency budget at 50 Hz (20 ms period)

| Stage | Cost |
|---|---|
| SYNC READ, 3 servos | ~0.5–0.8 ms |
| FPGA → ESP32 @ 2 Mbaud | ~0.05 ms |
| WiFi → host (UDP) | 1–5 ms |
| Policy inference + host-side overhead | 1–5 ms |
| WiFi → ESP32 | 1–5 ms |
| ESP32 → FPGA | ~0.05 ms |
| SYNC WRITE, 3 servos | 0.29 ms |
| **Total** | **~4–12 ms of 20 ms** |

Comfortable at 50 Hz. 100 Hz is reachable, but the two WiFi hops start to
dominate and host-side Python overhead usually bites before the wire does.

For scale: a 100-step servo move takes ~35 ms of mechanics at the measured
2850 steps/s ceiling (see `daisy-chain.md`). **The mechanics dominate the
electronics by roughly 100x.** Optimising the wire past this point is
chasing a percent of the total; what actually matters is jitter.

### Speed levers, in order of payoff

1. **Broadcast SYNC WRITE for actions — never wait for a reply.** The
   protocol asks for a ≥1 ms wait before reading a response; broadcast is
   never answered, so that disappears entirely.
2. **UDP, and disable WiFi power-save on the ESP32.** Modem sleep alone
   turns a 2 ms hop into 100 ms spikes. This is a jitter fix more than a
   latency fix.
3. **Run the link at an exact divisor of 50 MHz.** 2 Mbaud is 50 MHz ÷ 25,
   comfortably within ESP32 capability over jumper wires. 5 Mbaud (÷10)
   works but leaves only 10 clocks per bit. Keep the *servo* bus at 1 Mbaud,
   which is both its maximum and exact (÷50).
4. **Compact opcodes rather than relaying whole servo packets.** 8 bytes
   versus 29 on the link. Minor, but it falls out of the design anyway.

---

## Link protocol (host/ESP32 ↔ FPGA)

Framed and checksummed, same discipline as the servo bus. An unframed UART
resynchronises badly, and a corrupted position command on a 30 kg·cm servo
is a mechanical event.

```
AA 55 | LEN | SEQ | OP | payload… | CRC8
```

`AA 55` rather than the servo bus's `FF FF`, so a mis-cabled link fails
obviously instead of half-working.

`SEQ` is a sequence number. UDP gives no ordering or delivery guarantee, so
it lets the FPGA drop stale or out-of-order actions rather than applying
them late, and lets the host align each observation with the action that
produced it.

### Host → FPGA

| OP | Name | Payload |
|---|---|---|
| `0x01` | SET_TARGETS | n, then (id, pos16) x n |
| `0x02` | SET_TARGETS_SPEED | n, then (id, pos16, speed16) x n |
| `0x03` | SPIN | id, speed16 |
| `0x04` | STOP | — (immediate, bypasses interpolation) |
| `0x05` | PWM | channel, duty16 |
| `0x10` | RAW | a complete HX packet, passed through untouched |
| `0x20` | CONFIG | limits, tick rate |
| `0x7F` | KEEPALIVE | — (resets the deadman with no motion) |

`RAW` earns its place: it keeps `tools/hx.py`-style register poking working
through the ESP32, which has already proved to be how questions actually get
answered on this project.

### FPGA → host

| OP | Name | Payload |
|---|---|---|
| `0x81` | OBS | tick32, then (pos16, speed16, load16, current16, temp8, status8) x n |
| `0x82` | FAULT | code, detail — deadman fired, servo error, link CRC failures |

`tick32` is the FPGA's own control-tick counter. It lets the host detect
stale observations and timestamp them against a clock that does not jitter.

---

## Safety envelope

All of it lives in the FPGA, because that is the only layer that cannot be
bypassed by a policy, a language runtime, or a dropped network.

**Deadman.** If no valid frame arrives within a timeout — 200 ms is a
sensible start — stop acting on the last command. The right failure action
is *hold position*, not release: on a legged or articulated robot, going
limp means collapse. After a longer timeout, reduce torque so a servo held
against an obstruction does not cook itself. Both thresholds configurable.

**Position clamps.** Per-servo min/max, enforced after interpolation. A
policy that is out of distribution will command joint angles that the
mechanism cannot reach.

**Rate limits.** Maximum change per tick, per servo. This is the one that
catches the worst policy failures — a full-scale step command is what breaks
linkages and strips gears.

**Torque and current limits.** Register `0x13` (max torque) and `0x1F`
(over-current) on each servo, set once at bring-up. Belt and braces
alongside the servo's own protections.

The ordering matters: interpolate, then clamp, then rate-limit, then emit.
Clamping before interpolation lets an interpolated path leave the envelope.

---

## Interpolation

The policy sets targets at 50 Hz; the FPGA generates a smooth, higher-rate
stream of intermediate setpoints between them. This is the standard
sim-to-real split — a policy setting targets, a fast low-level loop tracking
them — except that here the servo's own PID *is* the low-level loop and the
FPGA is feeding it cleanly.

Two properties worth having:

- Network jitter changes *where* the robot is going, never *how smoothly* it
  gets there.
- A dropped action degrades gracefully: the FPGA keeps executing the last
  trajectory rather than stuttering or freezing.

A transparent relay architecture structurally cannot do this — every servo
update would need its own network packet, so WiFi jitter lands directly on
the motion.

---

## Development path: tethered first

The tethered path already works. `bridge.v` plus the dock's FT2232 UART
gives the host a direct line to the FPGA today, with none of WiFi's jitter.

**If the link protocol is identical over both transports, the ESP32 becomes
a swap-in rather than a rewrite.** So:

1. Build the link layer, control tick, safety envelope and interpolation,
   driven over the existing USB UART.
2. Get the policy loop running against that deterministic link.
3. Add the ESP32 as a transport, changing one variable.

This avoids the trap of debugging a policy and a network simultaneously.
It also answers the obvious question of why the ESP32 is in the path at all:
for a tethered robot it is not, and USB is faster. It earns its place only
when the robot has to move around untethered.

---

## FPGA work required

| Component | Status |
|---|---|
| `uart_tx` / `uart_rx` | exist; parameterise for 2 Mbaud (÷25) |
| Link framer/parser + CRC8 | new |
| Fixed-rate control tick | new, small |
| Interpolator | new |
| Safety envelope (clamps, rate limit, deadman) | new |
| SYNC WRITE for actions | needs the wider parameter path from `daisy-chain.md` |
| SYNC READ for observations | new; multi-packet reply collector in the RX path |
| `hx_bus` parameter path | 64 bits → 256 bits (see `daisy-chain.md`) |
| Echo gating, checksums, inter-frame gap | done, unchanged |

The RX-side multi-packet collector is the only genuinely new protocol work:
SYNC READ returns N separate reply packets in ID order, where today the
parser handles exactly one.

---

## Open questions

- **Policy control rate.** 50 Hz is the common default for locomotion;
  vision-conditioned policies often run 10–30 Hz. Sets the tick and how much
  interpolation the FPGA does.
- **Host stack.** Plain UDP datagrams read by a Python script is the leanest
  thing that works and the easiest to point at PyTorch.
- **Observation content.** Position and velocity are the minimum. Load,
  current and temperature are nearly free to add, since one 8-byte read at
  `0x38` already returns most of them.
- **Servo count at deployment.** Three fits the 256-bit parameter path;
  beyond four, swap the flattened bus for a parameter RAM.
- **Signal integrity at 1 Mbaud with a full chain.** Unmeasured. See the
  note in `daisy-chain.md` before reaching for a lower baud rate.
