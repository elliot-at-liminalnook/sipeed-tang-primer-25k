# Sipeed Tang Primer 25K — learning projects

Verilog lessons for the [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html)
(Gowin GW5A-25 FPGA) on its dock, working up from a blinking LED to PWM motor
control — plus a labelled photo reference for the parts on the bench.

## Lessons

| Directory | What it covers |
|---|---|
| `blink/` | Clock divider, the `.cst` / `.sdc` constraint files, first bitstream |
| `pwm/`   | Counter-and-comparator PWM, duty cycle as a parameter |
| `motor/` | DC motor speed control: input synchroniser, debouncer, edge detector, 20 kHz PWM |
| `servo/` | Hiwonder HX-30HM bus servo: 1 Mbaud UART, packet framing and checksums, owning a shared half-duplex wire |

Two design notes live alongside that lesson:

- `servo/daisy-chain.md` — what changes when three servos share one bus: ID
  assignment, the connector current limit that caps a power daisy-chain, and
  SYNC WRITE for simultaneous motion.
- `servo/nn-control.md` — driving the servos from a neural network over
  WiFi via an ESP32: where the control tick and safety envelope belong, the
  link protocol, and why the tethered path comes first.

Each lesson has the same shape:

```
<lesson>/
  src/     <lesson>.v      the design
           <lesson>.cst    pin constraints (FPGA ball names)
           <lesson>.sdc    timing constraints
  tb/      <lesson>_tb.v   Icarus Verilog testbench
  build.tcl                Gowin synthesis + place-and-route
```

### Building

Synthesis and bitstream, using Gowin's Tcl shell:

```sh
gw_sh motor/build.tcl
```

Or with the **open toolchain**, which needs no vendor account and builds the
GW5A-25 directly — yosys, nextpnr-himbaechel, apicula and openFPGALoader all
ship in [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build/releases):

```sh
cd servo
make          # impl/servo.fs
make flash    # load to SRAM (volatile — gone at power-off)
make prog     # write to onboard flash (survives power cycling)
```

Two GW5A-25A quirks the Makefile encodes, both of which abort the build if
you get them wrong:

- **SSPI must be configured as GPIO**, and nextpnr and `gowin_pack`
  cross-check each other on it — set it in only one place and the pack step
  fails with "conflicting settings".
- The dock's LEDs sit on the **DONE and READY config pins**, so those have to
  be released to GPIO too (the same thing `build.tcl`'s `set_option` lines do).

Simulation only, no vendor tools needed:

```sh
cd motor && iverilog -o motor_tb.vvp tb/motor_tb.v src/motor.v && vvp motor_tb.vvp
gtkwave motor_tb.vcd
```

`servo/` has several source files and two testbenches — one for the wire
protocol, one for the button sequence end to end:

```sh
cd servo
iverilog -o servo_tb.vvp tb/servo_tb.v tb/hx_model.v \
         src/hx_bus.v src/uart_tx.v src/uart_rx.v && vvp servo_tb.vvp
iverilog -o top_tb.vvp   tb/top_tb.v   tb/hx_model.v \
         src/servo.v src/hx_bus.v src/uart_tx.v src/uart_rx.v && vvp top_tb.vvp
```

`tb/hx_model.v` is a behavioural HX-30HM that validates checksums and
replies. It is the substitute for a logic analyser: every packet is proven
against it before a real servo is asked to interpret one.

Build output lands in `<lesson>/impl/` and is gitignored — it is regenerable,
large, and its logs embed absolute paths.

## Photo reference

`photos/index.html` is a labelled, cropped reference for every part on the
bench — 86 images across FPGA, motors and motor controllers, servos, logic
glue, sensors, discretes, passives and test gear. Open it directly in a
browser.

Each photo is cropped to its subject and named for what it contains, e.g.
`motor-03-drv8301-drv8302-pinout-table.jpg`. `photos/build_gallery.py` holds
the manifest (crop, title, description per image) and regenerates both the
images and the HTML:

```sh
PHOTO_SRC=~/path/to/originals python3 photos/build_gallery.py
```

The originals are **not** in this repo — they carry GPS EXIF. The committed
copies are cropped, stripped of all metadata, and redacted where a delivery
slip happened to be in frame.

### Two useful things it settled

- **Where `F5` and `G5` actually are.** The dock's underside silkscreen (see
  `photos/images/fpga-02-...`) puts them in the first column of the
  bottom-right 6×2 header — `F5` top row, `G5` directly below.
- **The servo is a Parallax Feedback 360**, not a plain hobby servo. It is
  continuous-rotation, so the control pulse sets speed rather than angle, and
  it returns absolute shaft position on a feedback wire — enough to close a
  position loop in the FPGA with no separate encoder.

### Before wiring the servo lesson

The HX-30HM runs on **11.1 V from its own supply** and can pull 6 A. It never
shares a rail with the FPGA — the two share exactly one wire, a ground
reference. Its SIG line is expected to be 3.3 V logic (Hiwonder's own SDK
drives it straight from ESP32 and Raspberry Pi pins, neither of which is 5 V
tolerant), but **measure it before connecting anything to a GW5A pin**: a
3.3 V bank is not 5 V tolerant, and the series resistors in `servo.cst`'s
header comment are there to bound the damage if that assumption is wrong.

Note that `motor/src/motor.v` is written against a **DRV8833** H-bridge, which
is not among the photographed parts. The **L293D** is the closest drop-in; the
**DRV8301** board also present is a three-phase BLDC stage needing six PWM
inputs, not two.

## Related

Sipeed's official examples are used as reference but not vendored here:

```sh
git clone https://github.com/sipeed/TangPrimer-25K-example
```
