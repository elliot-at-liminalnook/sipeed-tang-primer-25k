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

Simulation only, no vendor tools needed:

```sh
cd motor && iverilog -o motor_tb.vvp tb/motor_tb.v src/motor.v && vvp motor_tb.vvp
gtkwave motor_tb.vcd
```

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

Note that `motor/src/motor.v` is written against a **DRV8833** H-bridge, which
is not among the photographed parts. The **L293D** is the closest drop-in; the
**DRV8301** board also present is a three-phase BLDC stage needing six PWM
inputs, not two.

## Related

Sipeed's official examples are used as reference but not vendored here:

```sh
git clone https://github.com/sipeed/TangPrimer-25K-example
```
