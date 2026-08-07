#!/usr/bin/env python3
"""
Build the labelled photo gallery for the Tang Primer 25K bench.

Reads the raw phone photos, crops each one down to its subject, writes it out
under a filename that says what it is, and emits index.html describing the lot.

Re-runnable: it only ever writes into images/ and index.html.
"""

import html
import os
import subprocess
from pathlib import Path

# Where the untouched phone originals live. They are deliberately NOT in this
# repo (they carry GPS EXIF); override with PHOTO_SRC if yours are elsewhere.
SRC = Path(os.environ.get("PHOTO_SRC", "~/Downloads/Photos-1-001")).expanduser()
OUT = Path(__file__).resolve().parent
IMG = OUT / "images"

# Regions blacked out before publishing, as fractions (x, y, w, h) of the
# FINAL cropped image. A delivery slip with a home address is lying under the
# dock in two of the bench shots.
REDACTIONS = {
    "fpga-03-tang-primer-25k-dock-wired-to-breadboard.jpg": [
        (0.615, 0.700, 0.235, 0.300)],
    "servo-03-servo-test-rig-with-tang-dock.jpg": [
        (0.650, 0.755, 0.195, 0.245)],
}

# (out_name, source_stem, crop_fraction(x, y, w, h), section, title, body)
#
# crop fractions are of the source's own width/height; (0,0,1,1) = no crop.
ITEMS = [
    # ------------------------------------------------------------------ FPGA
    ("fpga-01-tang-primer-25k-on-dock-with-level-shifter.jpg",
     "PXL_20260501_215841329.MP", (0.24, 0.20, 0.68, 0.64), "fpga",
     "Tang Primer 25K seated on its dock",
     "The Sipeed Tang Primer 25K core module (Gowin GW5A-25 FPGA, the square die "
     "in the middle) plugged into the Tang Primer 25K Dock. Visible on the dock: "
     "the USB-A host port, the USB-C debug/programming port, the on-board "
     "USB-Debugger, buttons S1/S2, and the two 220&nbsp;uF bulk caps. The blue "
     "board to the right is the TXS0108E level shifter; the FPGA I/O banks here "
     "run at 3.3&nbsp;V, so anything at 5&nbsp;V has to go through it."),

    ("fpga-02-tang-primer-25k-dock-underside-pin-labels.jpg",
     "PXL_20260531_145017259.MP", (0.00, 0.26, 0.98, 0.52), "fpga",
     "Dock underside &mdash; the pin-name silkscreen",
     "The most useful photo in the set for wiring. Sipeed prints every header "
     "pin's FPGA ball name on the back of the dock, which is how you turn a name "
     "in a <code>.cst</code> file into a physical hole. "
     "<strong>This settles the open question in "
     "<code>motor/src/motor.cst</code>:</strong> the motor pins <code>F5</code> "
     "and <code>G5</code> are the first column of the bottom-right 6&times;2 "
     "header &mdash; <code>F5</code> on the top row, <code>G5</code> directly "
     "below it, with that header continuing <code>G7 H8 H5 GND</code> along the "
     "top and <code>G8 H7 J5 GND 3V3</code> along the bottom. Also visible: the "
     "Debug block (<code>3V3 / GND / TCK / TMS / TDI / TDO</code>) and the "
     "<code>Rcfg / GND / Rdy / Done</code> pads."),

    ("fpga-03-tang-primer-25k-dock-wired-to-breadboard.jpg",
     "PXL_20260725_195702944", (0.00, 0.00, 1.00, 1.00), "fpga",
     "Dock jumpered out to a breadboard",
     "The dock on the right, four jumpers running to a solderless breadboard "
     "carrying the discrete driver circuit. This is the working arrangement for "
     "the PWM and motor lessons &mdash; the FPGA generates the switching "
     "waveform, the breadboard does the current. The grey block at the bottom is "
     "a redaction over a delivery slip that showed a home address."),

    # ------------------------------------------------- motors & motor control
    ("motor-01-drv8301-bldc-power-stage-top.jpg",
     "PXL_20260501_215912260.MP", (0.06, 0.31, 0.90, 0.55), "motor",
     "DRV8301 three-phase BLDC power stage",
     "A three-phase brushless motor driver board: six TO-263 power MOSFETs in "
     "three half-bridges, three R005 (5&nbsp;m&ohm;) current-sense shunts, two "
     "470&nbsp;uF / 63&nbsp;V bulk capacitors, a buck inductor for the on-board "
     "step-down, and the DRV8301 gate driver. Phase outputs OUTA / OUTB / OUTC "
     "sit on the left edge next to VIN and GND. This is a far bigger stage than "
     "the little H-bridges elsewhere in this set &mdash; it is built for a real "
     "BLDC/gimbal motor, not a toy DC motor."),

    ("motor-02-drv8301-bldc-power-stage-mosfets.jpg",
     "PXL_20260520_225740902", (0.14, 0.14, 0.76, 0.76), "motor",
     "DRV8301 board &mdash; component side, held for scale",
     "Same board, in hand. You can see the six MOSFETs across the top, the shunt "
     "resistors underneath each half-bridge, the gate-driver IC in the middle, "
     "and the 40-pin control header along the bottom edge that the logic side "
     "(FPGA or MCU) connects to."),

    ("motor-03-drv8301-drv8302-pinout-table.jpg",
     "PXL_20260531_152608915", (0.25, 0.18, 0.60, 0.66), "motor",
     "DRV8301 / DRV8302 header pinout &mdash; readable",
     "The board is dual-populated, so the back carries two pinout tables and you "
     "read the column matching whichever gate driver is fitted. The signals that "
     "matter for driving it: <code>INH-A/B/C</code> and <code>INL-A/B/C</code> "
     "(the six PWM inputs &mdash; high and low side per phase), "
     "<code>EN-GATE</code> (enable), <code>nFAULT</code> and <code>PWRGD</code> "
     "(status back to you), <code>nOCTW</code> (over-current / thermal warning), "
     "<code>SO-1</code> / <code>SO-2</code> and <code>IOUTA/B/C</code> (current "
     "sense out), <code>DC-CAL</code>, and on the DRV8301 an SPI port "
     "(<code>SDI</code> / <code>SDO</code> / <code>SCLK</code> / "
     "<code>nSCS</code>) for configuration. The DRV8302 swaps SPI for hardware "
     "pins <code>M_PWM</code>, <code>M_OC</code>, <code>OC_ADJ</code> and "
     "<code>GAIN</code>."),

    ("motor-04-drv8301-pinout-silkscreen-over-header.jpg",
     "PXL_20260520_225748215", (0.28, 0.28, 0.50, 0.48), "motor",
     "Pinout table next to the header it labels",
     "The same table shot against the header itself, so you can count along and "
     "map table row to physical pin."),

    ("motor-05-drv8301-board-underside-full.jpg",
     "PXL_20260520_225751211", (0.20, 0.03, 0.58, 0.94), "motor",
     "DRV8301 board &mdash; whole underside",
     "Context shot of the back of the driver board: mounting holes, the thermal "
     "via fields under each MOSFET, and the pinout tables down at the header end."),

    ("motor-06-l293d-quad-half-h-bridge-dip16.jpg",
     "PXL_20260725_153936024", (0.28, 0.36, 0.42, 0.30), "motor",
     "L293D quad half-H driver (DIP-16)",
     "The classic beginner's motor driver. Four half-bridges, usable as two full "
     "H-bridges &mdash; so one L293D runs two DC motors bidirectionally, or one "
     "bipolar stepper. About 600&nbsp;mA per channel with built-in flyback "
     "diodes (that is what the 'D' means). Separate logic and motor supply pins, "
     "and the two enable pins are where a PWM duty-cycle from the FPGA goes in. "
     "Slow and lossy by modern standards, but it takes 3.3&nbsp;V logic and is "
     "hard to destroy."),

    ("motor-07-l293d-with-sn74hc595-shift-register.jpg",
     "PXL_20260721_215526122", (0.19, 0.22, 0.60, 0.54), "motor",
     "L293D alongside an SN74HC595",
     "The pair as they are usually used together: the '595 shift register turns "
     "three FPGA pins into eight outputs, and those outputs feed the L293D's "
     "direction inputs. Handy when you are short of I/O, though note the PWM "
     "enable lines should still come straight from the FPGA &mdash; you do not "
     "want to shift-register a switching waveform."),

    ("motor-08-uln2003-stepper-driver-board.jpg",
     "PXL_20260721_203425308", (0.00, 0.19, 0.84, 0.60), "motor",
     "ULN2003 stepper driver board",
     "The board that ships with a 28BYJ-48 unipolar stepper. The ULN2003 is "
     "seven Darlington sinks &mdash; not an H-bridge, it can only pull each coil "
     "tap to ground, which is all a unipolar stepper needs. Inputs "
     "<code>IN1</code>&ndash;<code>IN4</code> on the left, the four A/B/C/D LEDs "
     "mirror the coil states (very useful when you are debugging a step "
     "sequence), the white 5-pin JST takes the motor, and the screw terminal "
     "takes 5&ndash;12&nbsp;V."),

    ("motor-09-as5600-magnetic-encoder-on-motor.jpg",
     "PXL_20260501_215900334.MP", (0.22, 0.26, 0.60, 0.40), "motor",
     "AS5600 magnetic rotary encoder, mounted on a motor",
     "A magnetic angle sensor board bolted to the back of a motor. The chip "
     "reads a diametrically-magnetised magnet on the shaft and reports absolute "
     "shaft angle over I2C &mdash; this is the feedback half of closed-loop "
     "control, which is what turns 'spin the motor' into 'hold this position'. "
     "Two JST ports so it can be daisy-chained, and the pinout is silkscreened "
     "between them."),

    ("motor-10-as5600-encoder-angled-view.jpg",
     "PXL_20260501_215907977.MP", (0.26, 0.26, 0.58, 0.44), "motor",
     "Encoder board &mdash; angled, showing the motor body",
     "Same assembly tilted, so you can see the encoder PCB standing off the "
     "motor bell on four brass pillars and the cable leaving toward the driver."),

    ("motor-11-as5600-encoder-pin-labels.jpg",
     "PXL_20260520_225835974", (0.05, 0.29, 0.82, 0.60), "motor",
     "Encoder pinout: OUT / PGO / GND / VCC and VCC / GND / SCL / SDA",
     "Straight-on shot of the silkscreen. Left column <code>OUT</code>, "
     "<code>PGO</code>, <code>GND</code>, <code>VCC</code>; right column "
     "<code>VCC</code>, <code>GND</code>, <code>SCL</code>, <code>SDA</code>. "
     "<code>OUT</code> is the analog/PWM angle output, <code>PGO</code> is the "
     "programming pin used to burn the zero position, and SCL/SDA are the normal "
     "I2C interface."),

    ("motor-12-as5600-encoder-closeup-in-hand.jpg",
     "PXL_20260531_153939766", (0.08, 0.38, 0.80, 0.54), "motor",
     "Encoder board in hand",
     "Closest view of the encoder module, with the JST cable colours visible "
     "(red / black / yellow / white) for tracing back to the pin labels."),

    # ----------------------------------------------------------------- servos
    ("servo-01-standard-hobby-servo-closeup.jpg",
     "PXL_20260725_195724442", (0.02, 0.04, 0.31, 0.35), "servo",
     "Parallax Feedback 360&deg; servo",
     "The case label reads <strong>Feedback 360 / Parallax Inc.</strong>, which "
     "makes this a good deal more interesting than a plain hobby servo. It is a "
     "<em>continuous-rotation</em> servo, so the 1.0&ndash;2.0&nbsp;ms control "
     "pulse sets <strong>speed and direction</strong>, not angle &mdash; and it "
     "adds a fourth wire carrying a feedback signal whose duty cycle reports "
     "<strong>absolute shaft position over the full 360&deg;</strong>. That "
     "means you can close a position loop entirely in the FPGA with no separate "
     "encoder: one counter generates the command pulse, a second measures the "
     "feedback pulse, and the difference drives the correction."),

    ("servo-02-servo-driven-from-breadboard-rig.jpg",
     "PXL_20260725_195709759", (0.00, 0.00, 1.00, 1.00), "servo",
     "Servo wired to the breadboard rig",
     "The clearest view of the servo's case label &mdash; <code>Feedback "
     "360</code> over <code>Parallax Inc.</code> Its lead is broken out to "
     "jumpers at the breadboard. Note the servo is powered from the breadboard "
     "rails rather than from an FPGA pin: a servo's stall current will brown out "
     "a dev board instantly, so only the signal and feedback wires should ever "
     "touch the FPGA."),

    ("servo-03-servo-test-rig-with-tang-dock.jpg",
     "PXL_20260725_195724442", (0.00, 0.00, 1.00, 1.00), "servo",
     "Whole servo test setup",
     "The full loop in one frame: Tang Primer 25K dock bottom-right, jumpers to "
     "the breadboard, breadboard circuit in the middle, servo in hand at the "
     "left. The grey block bottom-right is a redaction &mdash; a delivery slip "
     "with a home address was lying under the dock."),

    ("servo-04-breadboard-driver-circuit-detail.jpg",
     "PXL_20260725_195706771", (0.02, 0.10, 0.96, 0.80), "servo",
     "Breadboard driver circuit &mdash; detail",
     "The discrete stage between FPGA pin and load: an electrolytic across the "
     "rails for decoupling, a transistor, a series base resistor, and a flyback "
     "diode. This is the minimum honest interface between a 3.3&nbsp;V logic pin "
     "and an inductive load."),

    ("servo-05-breadboard-driver-circuit-overhead.jpg",
     "PXL_20260725_195730561", (0.02, 0.02, 0.96, 0.90), "servo",
     "Breadboard circuit from above",
     "Overhead view of the same breadboard, showing the jumper routing between "
     "the power rails, the transistor stage and the header going to the dock."),

    # --------------------------------------------- logic glue / FPGA I/O gear
    ("interface-01-txs0108e-8ch-level-shifter.jpg",
     "PXL_20260520_225818801", (0.28, 0.24, 0.54, 0.48), "interface",
     "TXS0108E 8-channel bidirectional level shifter",
     "The workhorse for connecting a 3.3&nbsp;V FPGA to 5&nbsp;V parts. "
     "<code>VA</code> side to 3.3&nbsp;V with <code>A1</code>&ndash;"
     "<code>A8</code>, <code>VB</code> side to 5&nbsp;V with <code>B1</code>&ndash;"
     "<code>B8</code>, and <code>OE</code> to enable. Auto-direction sensing, so "
     "it works for bidirectional buses like I2C &mdash; but it is weakly driven "
     "and does not like fast push-pull signals, so use it for slow control "
     "lines, not for a clock or a PWM edge you care about."),

    ("interface-02-txs0108e-level-shifter-in-hand.jpg",
     "PXL_20260501_215847637.MP", (0.28, 0.42, 0.52, 0.26), "interface",
     "Level shifter in hand, for scale",
     "The HW-221 breakout is about the size of a thumbnail. Pin labels are "
     "printed on both long edges."),

    ("interface-03-txs0108e-level-shifter-underside.jpg",
     "PXL_20260501_215850960", (0.28, 0.45, 0.50, 0.26), "interface",
     "Level shifter &mdash; blank underside",
     "Nothing on the back but the two mounting holes; included so the board can "
     "be identified from either side in a parts bin."),

    ("interface-04-bss138-4ch-logic-level-converter.jpg",
     "PXL_20260725_153901204", (0.36, 0.28, 0.30, 0.36), "interface",
     "4-channel MOSFET level converter (BSS138 type)",
     "The other level-shifting option here: four discrete BSS138 MOSFETs with "
     "pull-ups, <code>HV1</code>&ndash;<code>HV4</code> on one side and "
     "<code>LV1</code>&ndash;<code>LV4</code> on the other. Open-drain by "
     "construction, so it is genuinely good for I2C and poor for anything fast."),

    ("interface-05-sn74ahct125-quad-buffer.jpg",
     "PXL_20260725_153312348", (0.25, 0.34, 0.48, 0.34), "interface",
     "SN74AHCT125N quad tri-state buffer",
     "Four independently-enabled buffers. The useful trick: AHCT inputs are "
     "TTL-threshold, so a 3.3&nbsp;V FPGA output reliably registers as a logic "
     "high while the chip runs from 5&nbsp;V and drives a clean 5&nbsp;V output. "
     "That makes it the right part &mdash; not the TXS0108E &mdash; for pushing "
     "a fast one-way signal like WS2812 data or a clock up to 5&nbsp;V."),

    ("interface-06-sn74hc595-shift-register.jpg",
     "PXL_20260725_154035491", (0.28, 0.36, 0.44, 0.30), "interface",
     "SN74HC595N 8-bit serial-in / parallel-out shift register",
     "Three pins in, eight latched outputs out. Trivial to drive from an FPGA "
     "(shift on one clock, then pulse the latch), and cascadable, which is why "
     "it shows up wherever there are more LEDs or direction lines than I/O."),

    ("interface-07-mcp3208-12bit-spi-adc.jpg",
     "PXL_20260725_153828640", (0.32, 0.30, 0.36, 0.34), "interface",
     "MCP3208 &mdash; 8-channel 12-bit SPI ADC",
     "The FPGA has no analog input at all, so this is how it reads a "
     "potentiometer, a current-sense voltage, or the analog output of the "
     "encoder. Eight single-ended channels (or four differential), 12-bit, "
     "~100&nbsp;ksps, plain SPI &mdash; a straightforward state machine to write "
     "in Verilog. DIP-16 so it goes straight into a breadboard."),

    ("interface-08-mcp3208-adc-in-hand.jpg",
     "PXL_20260721_215957714", (0.28, 0.36, 0.56, 0.30), "interface",
     "MCP3208 in hand",
     "Scale reference, and the <code>MCP3208-CI/P</code> / date-code marking "
     "clearly legible."),

    ("interface-09-mcp3208-adc-on-antistatic-bag.jpg",
     "PXL_20260520_225829357", (0.34, 0.20, 0.32, 0.55), "interface",
     "MCP3208 on its antistatic bag",
     "Same part as stored. Included because this is what it looks like in the "
     "bin rather than on white paper."),

    ("interface-10-mcp3208-adc-alt-angle.jpg",
     "PXL_20260520_225829452", (0.33, 0.18, 0.32, 0.56), "interface",
     "MCP3208 &mdash; second angle",
     "Duplicate shot from the same session, kept for the sharper view of the "
     "package marking."),

    ("interface-11-mcp3208-adc-second-unit.jpg",
     "PXL_20260725_153848701", (0.34, 0.36, 0.34, 0.30), "interface",
     "MCP3208 &mdash; second unit",
     "There is more than one of these in the kit; this is the second, shot "
     "during the paper-background inventory pass."),

    # ---------------------------------------------------------------- sensors
    ("sensor-01-mpu9250-9dof-imu.jpg",
     "PXL_20260721_203421140", (0.15, 0.27, 0.74, 0.34), "sensor",
     "MPU-9250 / 6500 / 9255 9-DOF IMU breakout",
     "Three-axis gyro, accelerometer and magnetometer. Speaks both I2C "
     "(<code>SCL</code> / <code>SDA</code>) and SPI (<code>NCS</code>, plus "
     "SDA/SCL doubling as MOSI/SCK), with <code>AD0</code> selecting the I2C "
     "address, <code>INT</code> for data-ready, and <code>EDA</code> / "
     "<code>ECL</code> as an auxiliary master bus. The board silkscreens the "
     "X/Y/Z axis convention, which matters as soon as you fuse this with motor "
     "position."),

    ("sensor-02-mpu9250-imu-in-hand.jpg",
     "PXL_20260721_215853097", (0.14, 0.24, 0.74, 0.56), "sensor",
     "IMU breakout in hand",
     "Scale shot; the QFN sensor and the axis arrows are visible."),

    ("sensor-03-mpu9250-imu-inventory-shot.jpg",
     "PXL_20260725_153837851", (0.30, 0.32, 0.42, 0.42), "sensor",
     "IMU breakout &mdash; inventory shot",
     "The same board photographed flat during the paper-background pass, with "
     "the full pin column legible top to bottom."),

    ("sensor-04-hc-sr04-ultrasonic-rangefinder.jpg",
     "PXL_20260721_203450532", (0.00, 0.36, 1.00, 0.46), "sensor",
     "HC-SR04 ultrasonic rangefinder",
     "Transmit and receive transducers with <code>Vcc</code> / <code>Trig</code> "
     "/ <code>Echo</code> / <code>Gnd</code>. You pulse Trig for 10&nbsp;us and "
     "time how long Echo stays high &mdash; which is pure counter work, so it is "
     "one of the most natural first 'real' FPGA peripherals. Note it is a "
     "5&nbsp;V part: the Echo line needs to come back down through a level "
     "shifter or a divider."),

    ("sensor-05-dht11-temperature-humidity-sensor.jpg",
     "PXL_20260721_203444268", (0.52, 0.36, 0.40, 0.50), "sensor",
     "DHT11 temperature / humidity sensor",
     "Single-wire sensor on a small carrier with its pull-up already fitted. "
     "The protocol is a bit-banged timing sequence, so like the HC-SR04 it is a "
     "timer exercise &mdash; but a fussier one, since it is bidirectional on one "
     "pin."),

    ("sensor-06-ds1307-real-time-clock-module.jpg",
     "PXL_20260721_203429202.MP", (0.10, 0.26, 0.78, 0.42), "sensor",
     "DS1307 real-time clock module",
     "Battery-backed I2C clock/calendar with a coin cell and a square-wave "
     "output (<code>SQW</code>). The SQW pin is the interesting one for FPGA "
     "work: a stable 1&nbsp;Hz reference to check a home-made clock divider "
     "against."),

    ("sensor-07-max7219-dot-matrix-driver-module.jpg",
     "PXL_20260721_203436634", (0.07, 0.16, 0.84, 0.56), "sensor",
     "MAX7219 8x8 dot-matrix module",
     "SPI-ish display driver: <code>DIN</code> / <code>CS</code> / "
     "<code>CLK</code> in on one edge, <code>DOUT</code> out the other so "
     "modules chain. Doing the multiplexing in the FPGA instead is a good "
     "exercise, but this board is the quick way to get a visible output."),

    ("sensor-08-ldr-photoresistor.jpg",
     "PXL_20260725_152915160.MP", (0.36, 0.40, 0.28, 0.32), "sensor",
     "LDR (photoresistor)",
     "Light-dependent resistor. Analog only, so it pairs with the MCP3208 "
     "&mdash; or with a 555 as a light-to-frequency trick if you want to keep "
     "everything digital."),

    ("sensor-09-ntc-thermistor.jpg",
     "PXL_20260725_153009192", (0.33, 0.26, 0.28, 0.34), "sensor",
     "NTC thermistor",
     "Glass-bead negative-temperature-coefficient thermistor. Again analog: "
     "divider plus ADC. Worth keeping near the motor driver &mdash; it is the "
     "cheapest way to notice a power stage getting hot."),

    ("sensor-10-sw520d-tilt-switch-and-s8050-transistor.jpg",
     "PXL_20260721_215520841", (0.12, 0.44, 0.62, 0.38), "sensor",
     "SW-520D tilt switch and S8050 NPN transistor",
     "The SW-520D is a ball tilt switch &mdash; a rolling ball bridges two "
     "contacts depending on orientation. It bounces badly, so it is a good "
     "target for the debounce logic already written for the dock buttons. The "
     "S8050 next to it is a small general-purpose NPN (~500&nbsp;mA), fine for "
     "switching an LED or a relay coil from a logic pin."),

    ("sensor-11-sw18020p-vibration-switch.jpg",
     "PXL_20260725_152947494", (0.34, 0.26, 0.34, 0.36), "sensor",
     "SW-18020P vibration switch",
     "Spring-type vibration/shock switch: the internal spring touches the centre "
     "pin when knocked. Momentary and extremely noisy &mdash; treat it as an "
     "event source through a one-shot, not as a level."),

    ("sensor-12-tilt-switch-transistor-and-buzzer-group.jpg",
     "PXL_20260721_215523025", (0.00, 0.48, 0.98, 0.46), "sensor",
     "Tilt switch, transistor and buzzer together",
     "Group shot of the SW-520D, the S8050 and the magnetic buzzer &mdash; the "
     "three parts of a 'tip it and it beeps' circuit."),

    ("sensor-13-sparkfun-vl53l5cx-tof-imager-qwiic-pair.jpg",
     "PXL_20260721_203413715.MP", (0.281, 0.230, 0.541, 0.259), "sensor",
     "SparkFun VL53L5CX ToF imager on a Qwiic link",
     "The most capable sensor in the pile: a VL53L5CX time-of-flight "
     "<em>imager</em> &mdash; not a single distance reading but an 8x8 grid of "
     "them, up to about 4&nbsp;m. It is taped to card and joined by a Qwiic "
     "(4-pin JST-SH I2C) cable to the small SparkFun breakout on the left. Being "
     "I2C at 3.3&nbsp;V it can talk to the FPGA directly, though the register "
     "interface is heavy enough that it is usually driven from a soft CPU rather "
     "than raw Verilog."),

    ("sensor-14-parts-inventory-overview.jpg",
     "PXL_20260721_203413715.MP", (0.03, 0.02, 0.94, 0.96), "sensor",
     "Whole-kit overview",
     "Everything laid out at once: the Tang dock centre, MAX7219 matrix and "
     "7-segment display, 1602 LCD, HC-SR04, DHT11, DS1307, MPU-9250, the "
     "ULN2003 stepper board in its bag, a water-level sensor, the MCP3208, and "
     "the taped VL53L5CX/Qwiic pair on the cardboard strip."),

    # --------------------------------------------------- discrete semis, misc
    ("discrete-01-2n2222a-npn-transistor.jpg",
     "PXL_20260721_215621726", (0.28, 0.31, 0.36, 0.44), "discrete",
     "2N2222A NPN transistor (TO-92)",
     "General-purpose NPN, ~800&nbsp;mA, ~40&nbsp;V. The default answer to "
     "'this FPGA pin cannot source enough current' &mdash; base resistor from "
     "the pin, load in the collector, flyback diode if the load is a coil."),

    ("discrete-02-2n2222a-npn-on-paper.jpg",
     "PXL_20260725_154046944", (0.38, 0.28, 0.30, 0.44), "discrete",
     "2N2222A &mdash; inventory shot",
     "Same part on the paper background, marking <code>2N2222A</code> legible."),

    ("discrete-03-2n2222a-npn-closeup.jpg",
     "PXL_20260725_154104207", (0.34, 0.20, 0.32, 0.50), "discrete",
     "2N2222A &mdash; closer",
     "Tighter shot showing the flat face and lead order."),

    ("discrete-04-mps2222a-npn-transistor-strip.jpg",
     "PXL_20260725_153059965", (0.30, 0.08, 0.54, 0.86), "discrete",
     "MPS2222A NPN transistors, taped strip",
     "The same 2222 in Motorola/ON's MPS package, still on the tape bandolier "
     "&mdash; a strip of five. Interchangeable with the 2N2222A for this kind of "
     "switching, but check the pinout: TO-92 lead order is not universal."),

    ("discrete-05-mps2222a-npn-markings.jpg",
     "PXL_20260725_153108723", (0.24, 0.02, 0.34, 0.66), "discrete",
     "MPS2222A &mdash; package markings",
     "Close enough to read <code>MPS2222A</code> and the date code."),

    ("discrete-06-mps2907a-pnp-transistor-strip.jpg",
     "PXL_20260725_153134829", (0.26, 0.33, 0.34, 0.64), "discrete",
     "MPS2907A PNP transistors, taped strip",
     "The PNP complement to the 2222. You need these for high-side switching "
     "&mdash; and a pair of 2222/2907 is the basis of a discrete H-bridge if you "
     "would rather build one than use the L293D."),

    ("discrete-07-1n4148-signal-diodes.jpg",
     "PXL_20260725_152751658", (0.03, 0.12, 0.95, 0.60), "discrete",
     "1N4148 small-signal diodes",
     "Glass DO-35 switching diodes: fast, ~200&nbsp;mA, 100&nbsp;V. For logic "
     "steering, clamping and small flyback duty. Not enough for a motor coil "
     "&mdash; that is what the 1N4007s are for."),

    ("discrete-08-1n4007-rectifier-diodes.jpg",
     "PXL_20260725_153150453", (0.12, 0.04, 0.56, 0.92), "discrete",
     "1N4007 rectifier diodes",
     "1&nbsp;A / 1000&nbsp;V DO-41 rectifiers. This is the flyback diode you put "
     "across a relay or motor winding so the inductive kick does not punch back "
     "into the switching transistor. Slow, but the voltage headroom is generous."),

    ("discrete-09-tlc555-timer-ic.jpg",
     "PXL_20260725_153211666", (0.33, 0.30, 0.36, 0.38), "discrete",
     "TLC555CP timer",
     "CMOS version of the 555 &mdash; lower current, works happily at "
     "3.3&nbsp;V, unlike the bipolar NE555. Useful as an independent oscillator "
     "to check FPGA timing against, or as a light-to-frequency front end with "
     "the LDR."),

    ("discrete-10-lm358-dual-op-amp.jpg",
     "PXL_20260725_153225043", (0.25, 0.36, 0.44, 0.32), "discrete",
     "LM358P dual op-amp",
     "Two single-supply op-amps. The motor-relevant use is amplifying the "
     "millivolt drop across a current-sense shunt up to something the MCP3208 "
     "can digitise &mdash; the front end of current control."),

    ("discrete-11-5mm-leds-red-green-white.jpg",
     "PXL_20260725_153331474", (0.18, 0.12, 0.52, 0.74), "discrete",
     "5&nbsp;mm LEDs",
     "Red, green and diffused white 5&nbsp;mm LEDs. In the lessons here the LEDs "
     "mirror the motor PWM, so brightness stands in for motor speed and the "
     "design can be tested with no motor wired at all."),

    ("discrete-12-tactile-pushbutton-6mm.jpg",
     "PXL_20260725_152852832", (0.34, 0.30, 0.30, 0.32), "discrete",
     "6&nbsp;mm tactile pushbutton",
     "Standard four-leg tact switch (the legs are paired, so it is really a "
     "two-terminal part). Bounces for milliseconds on every press, which is "
     "exactly why the motor design has a synchroniser, a debouncer and an edge "
     "detector in front of it."),

    ("discrete-13-spdt-slide-switch.jpg",
     "PXL_20260725_152900911", (0.27, 0.30, 0.36, 0.42), "discrete",
     "SPDT slide switch",
     "Three-pin slide switch for a static mode or direction selection &mdash; a "
     "level rather than an event, so no debounce logic needed beyond a "
     "synchroniser."),

    ("discrete-14-trimmer-potentiometer-10k.jpg",
     "PXL_20260725_152830915", (0.39, 0.32, 0.28, 0.56), "discrete",
     "Trimmer potentiometer, 10&nbsp;k&ohm; (marked 103)",
     "Multi-turn-style trimpot. Marked <code>103</code> = 10 &times; 10^3 = "
     "10&nbsp;k&ohm;. Wire the wiper to an MCP3208 channel and you have a manual "
     "speed/position input for the motor lessons."),

    ("discrete-15-trimmer-potentiometer-1k.jpg",
     "PXL_20260725_152841621", (0.39, 0.28, 0.28, 0.54), "discrete",
     "Trimmer potentiometer, 1&nbsp;k&ohm; (marked 102)",
     "Same body style, <code>102</code> = 1&nbsp;k&ohm;."),

    ("discrete-16-magnetic-buzzer.jpg",
     "PXL_20260721_215533975", (0.22, 0.30, 0.48, 0.36), "discrete",
     "Magnetic buzzer",
     "Sealed buzzer with the 'remove seal after washing' sticker still on. Drive "
     "it with a square wave from the FPGA &mdash; through a transistor, since it "
     "draws more than a pin should supply."),

    ("discrete-17-tdk-piezo-buzzer-element.jpg",
     "PXL_20260725_152821815", (0.39, 0.28, 0.34, 0.42), "discrete",
     "TDK piezo buzzer element",
     "The passive counterpart: a bare piezo element with no drive circuit "
     "inside, so it makes no sound unless you feed it a tone. Higher impedance, "
     "so it can often be driven straight from a pin."),

    ("discrete-18-ceramic-disc-capacitor.jpg",
     "PXL_20260725_154112875", (0.42, 0.16, 0.24, 0.60), "discrete",
     "Ceramic disc capacitor",
     "Single leaded ceramic. The everyday job for these is a decoupling cap "
     "right at each IC's supply pin &mdash; including the level shifters and "
     "the ADC, which misbehave without one."),

    # -------------------------------------------------------------- passives
    ("passive-01-electrolytic-capacitors-100uf-16v.jpg",
     "PXL_20260725_153347343", (0.02, 0.02, 0.96, 0.96), "passive",
     "100&nbsp;uF / 16&nbsp;V electrolytic capacitors",
     "Bulk capacitance for the breadboard rails. Anything that switches a motor "
     "needs this close to the driver, or the supply sags on every PWM edge and "
     "the logic browns out."),

    ("passive-02-electrolytic-capacitors-10uf-50v.jpg",
     "PXL_20260725_153419079", (0.08, 0.02, 0.88, 0.96), "passive",
     "10&nbsp;uF / 50&nbsp;V electrolytics, with ceramics",
     "Smaller electrolytics on the tape alongside yellow <code>103</code> "
     "(10&nbsp;nF) ceramics."),

    ("passive-03-capacitor-assortment-closeup.jpg",
     "PXL_20260725_153421986", (0.04, 0.04, 0.92, 0.92), "passive",
     "Capacitor assortment &mdash; closeup",
     "Mixed strip of 10&nbsp;uF electrolytics and ceramic caps, showing the "
     "date codes and the &minus;40&hellip;+105&nbsp;&deg;C rating."),

    ("passive-04-carbon-film-resistors-bandolier-a.jpg",
     "PXL_20260725_153440267", (0.06, 0.04, 0.86, 0.92), "passive",
     "Carbon-film resistors, bandolier A",
     "Beige-bodied 4-band carbon film, ten on tape, no written value &mdash; "
     "read the bands off the photo or measure them before use. Kept as one of "
     "four unlabelled reels."),

    ("passive-05-carbon-film-resistors-bandolier-b.jpg",
     "PXL_20260725_153447183", (0.04, 0.04, 0.90, 0.92), "passive",
     "Carbon-film resistors, bandolier B",
     "Second unlabelled carbon-film reel, visibly a different colour code from "
     "bandolier A."),

    ("passive-06-carbon-film-resistors-bandolier-c.jpg",
     "PXL_20260725_153453207", (0.06, 0.04, 0.86, 0.92), "passive",
     "Carbon-film resistors, bandolier C",
     "Third unlabelled reel."),

    ("passive-07-carbon-film-resistors-bandolier-d.jpg",
     "PXL_20260725_153458627", (0.02, 0.06, 0.92, 0.90), "passive",
     "Carbon-film resistors, bandolier D",
     "Fourth unlabelled reel."),

    ("passive-08-metal-film-resistors-100-ohm.jpg",
     "PXL_20260725_154157924", (0.06, 0.10, 0.86, 0.82), "passive",
     "Metal-film resistors &mdash; reads as 100&nbsp;&ohm; 1%",
     "Blue-bodied 5-band metal film. Bands read brown-black-black-black-brown, "
     "i.e. 100&nbsp;&ohm; at 1% &mdash; the clearest of the unlabelled reels, "
     "but worth confirming with a meter."),

    ("passive-09-metal-film-resistors-reel-b.jpg",
     "PXL_20260725_154203167", (0.06, 0.06, 0.88, 0.90), "passive",
     "Metal-film resistors, reel B",
     "Blue 5-band metal film, unlabelled. Bands are visible in the photo but not "
     "reliably readable from it &mdash; measure before use."),

    ("passive-10-metal-film-resistors-reel-c.jpg",
     "PXL_20260725_154208214", (0.04, 0.04, 0.90, 0.92), "passive",
     "Metal-film resistors, reel C",
     "Another unlabelled 5-band reel with a distinct code from reel B."),

    ("passive-11-metal-film-resistors-reel-d.jpg",
     "PXL_20260725_154215124", (0.06, 0.04, 0.88, 0.90), "passive",
     "Metal-film resistors, reel D",
     "Fourth unlabelled metal-film reel."),

    ("passive-12-resistors-220-ohm-labelled.jpg",
     "PXL_20260725_154242257", (0.02, 0.08, 0.96, 0.84), "passive",
     "220&nbsp;&ohm; resistors (hand-labelled)",
     "A wide strip written up as <code>220</code> on the tape. 220&nbsp;&ohm; is "
     "the standard LED series resistor at 3.3&ndash;5&nbsp;V, which is why there "
     "are so many."),

    ("passive-13-resistors-1m-labelled.jpg",
     "PXL_20260725_154249785", (0.18, 0.04, 0.62, 0.92), "passive",
     "1&nbsp;M&ohm; resistors (hand-labelled)",
     "Tape marked <code>1M</code>. High-value parts for timing networks and "
     "high-impedance pull-downs."),

    ("passive-14-resistors-5k1-labelled.jpg",
     "PXL_20260725_154257161", (0.12, 0.02, 0.68, 0.94), "passive",
     "5.1&nbsp;k&ohm; resistors (hand-labelled)",
     "Tape marked <code>5K1</code>. Note 5.1&nbsp;k is also the USB-C CC "
     "pull-down value, which is worth remembering around the dock's USB-C port."),

    ("passive-15-resistors-330-ohm-labelled.jpg",
     "PXL_20260725_154305036", (0.10, 0.04, 0.66, 0.92), "passive",
     "330&nbsp;&ohm; resistors (hand-labelled)",
     "Tape marked <code>330</code>. The other common LED series value, and a "
     "sensible base resistor for the small NPNs."),

    # ------------------------------------------------------ power, mcu, tools
    ("power-01-l7805cv-5v-regulator.jpg",
     "PXL_20260725_152616840", (0.08, 0.34, 0.76, 0.32), "power",
     "L7805CV 5&nbsp;V linear regulator",
     "TO-220 5&nbsp;V regulator, 1&nbsp;A. Linear, so it burns the difference as "
     "heat &mdash; fine for logic, wrong for a motor rail."),

    ("power-02-l7805cv-5v-regulator-to220.jpg",
     "PXL_20260725_153247586", (0.16, 0.26, 0.66, 0.26), "power",
     "L7805CV &mdash; inventory shot",
     "The same regulator flat on paper with the part number legible."),

    ("power-03-ld1117-3v3-regulator.jpg",
     "PXL_20260725_153253254", (0.04, 0.40, 0.70, 0.26), "power",
     "LD33V 3.3&nbsp;V regulator (LD1117 family)",
     "TO-220 low-dropout 3.3&nbsp;V regulator &mdash; the rail everything on the "
     "FPGA side runs at. Low dropout matters here: it will hold 3.3&nbsp;V from "
     "a 5&nbsp;V input where a 7805-style part would not."),

    ("power-04-elegoo-breadboard-power-supply.jpg",
     "PXL_20260531_130409085.MP", (0.24, 0.21, 0.56, 0.60), "power",
     "Breadboard power supply module",
     "Straddles the breadboard rails and provides selectable 3.3&nbsp;V or "
     "5&nbsp;V per side from a 6.5&ndash;9&nbsp;V barrel input or USB. This is "
     "what keeps motor current off the FPGA board's regulator &mdash; the right "
     "habit as soon as a servo or an H-bridge is involved."),

    ("power-05-breadboard-power-supply-detail.jpg",
     "PXL_20260531_130413485", (0.03, 0.09, 0.80, 0.84), "power",
     "Breadboard PSU &mdash; jumper detail",
     "Closer view showing the per-rail 3.3&nbsp;V / 5&nbsp;V jumper blocks, the "
     "AMS1117 regulators, the power switch and the USB-A output."),

    ("mcu-01-arduino-mega-2560.jpg",
     "PXL_20260531_130106751", (0.15, 0.06, 0.74, 0.88), "power",
     "Arduino Mega 2560 (Elegoo)",
     "The non-FPGA board in the pile. Useful as a known-good reference "
     "controller &mdash; if a sensor or driver works from the Mega but not from "
     "the FPGA, the problem is in the Verilog, not the wiring. Note it is a "
     "5&nbsp;V part, so anything shared with the Tang has to cross a level "
     "shifter."),

    ("tool-01-wanptek-dps3010u-bench-supply.jpg",
     "PXL_20260725_174841796.MP", (0.10, 0.08, 0.78, 0.78), "power",
     "Wanptek DPS3010U bench power supply",
     "Adjustable 30&nbsp;V / 10&nbsp;A supply with constant-voltage and "
     "constant-current modes, showing 6.00&nbsp;V set. The current limit is the "
     "important feature: dial it down before first power-on and a miswired motor "
     "driver simply refuses to start instead of destroying itself."),

    ("tool-02-klein-mm300-multimeter.jpg",
     "PXL_20260725_175326898", (0.10, 0.10, 0.80, 0.80), "power",
     "Klein Tools MM300 multimeter",
     "Manual-ranging DMM: volts, ohms, up to 10&nbsp;A, continuity and diode "
     "test. The tool for settling the resistor bandoliers above and for "
     "confirming which dock header hole is really <code>G5</code>."),
]

SECTIONS = [
    ("fpga", "FPGA &mdash; Tang Primer 25K",
     "The Gowin GW5A-25 core module and its dock. Everything else in this set "
     "exists to be driven from these pins."),
    ("motor", "Motors &amp; motor controllers",
     "The power stages, from a three-phase BLDC driver down to a DIP H-bridge, "
     "plus the shaft-angle encoder that closes the loop."),
    ("servo", "Servos &amp; the PWM bench rig",
     "Hobby servo and the breadboard setup used to drive it from the dock."),
    ("interface", "Logic glue &mdash; level shifting, expansion, ADC",
     "The parts that let a 3.3&nbsp;V FPGA with no analog input talk to the rest "
     "of the bench."),
    ("sensor", "Sensors &amp; modules",
     "Feedback and I/O: IMU, time-of-flight imager, rangefinder, environmental "
     "sensors and displays."),
    ("discrete", "Discrete semiconductors &amp; switches",
     "Transistors, diodes, small ICs, buttons and buzzers &mdash; the parts that "
     "sit between a logic pin and something that moves."),
    ("passive", "Passives",
     "Capacitors and resistors. Hand-labelled reels have confirmed values; the "
     "rest need a meter."),
    ("power", "Power &amp; test gear",
     "Regulators, supplies, the reference MCU and the two instruments."),
]

CSS = """
:root {
  --bg: #ffffff; --fg: #1a1c1f; --muted: #5c6470; --line: #e3e6ea;
  --card: #f7f8fa; --accent: #b3541e; --code-bg: #eceff3;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #14161a; --fg: #e6e8ec; --muted: #9aa3b0; --line: #272b32;
    --card: #1b1e24; --accent: #e08a4e; --code-bg: #23272e;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg); color: var(--fg);
  font: 16px/1.6 -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
        "Helvetica Neue", Arial, sans-serif;
}
.wrap { max-width: 1180px; margin: 0 auto; padding: 2.5rem 1.25rem 5rem; }
header { border-bottom: 2px solid var(--line); padding-bottom: 1.5rem; margin-bottom: 2rem; }
h1 { font-size: 1.9rem; margin: 0 0 .4rem; letter-spacing: -.02em; }
.sub { color: var(--muted); margin: 0 0 1rem; }
.lede { max-width: 62ch; }
nav { display: flex; flex-wrap: wrap; gap: .5rem; margin-top: 1.25rem; }
nav a {
  font-size: .82rem; text-decoration: none; color: var(--fg);
  border: 1px solid var(--line); border-radius: 999px; padding: .3rem .8rem;
  background: var(--card);
}
nav a:hover { border-color: var(--accent); color: var(--accent); }
section { margin: 3rem 0 0; scroll-margin-top: 1rem; }
h2 {
  font-size: 1.3rem; margin: 0 0 .3rem; padding-bottom: .5rem;
  border-bottom: 1px solid var(--line);
}
.sec-note { color: var(--muted); margin: .5rem 0 1.5rem; max-width: 62ch; font-size: .95rem; }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 1.5rem; }
figure {
  margin: 0; background: var(--card); border: 1px solid var(--line);
  border-radius: 10px; overflow: hidden; display: flex; flex-direction: column;
}
figure img {
  width: 100%; height: auto; display: block; background: #0c0d10;
  aspect-ratio: 4 / 3; object-fit: cover;
}
figcaption { padding: .9rem 1rem 1.1rem; }
figcaption h3 { font-size: 1rem; margin: 0 0 .45rem; line-height: 1.35; }
figcaption p { margin: 0 0 .7rem; font-size: .9rem; color: var(--fg); }
.fname {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: .74rem; color: var(--muted); word-break: break-all;
  border-top: 1px solid var(--line); padding-top: .6rem; margin: 0;
}
code {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: .87em; background: var(--code-bg); padding: .08em .35em; border-radius: 4px;
}
.callout {
  border-left: 3px solid var(--accent); background: var(--card);
  padding: .9rem 1.1rem; margin: 1.5rem 0; border-radius: 0 8px 8px 0;
  max-width: 70ch; font-size: .93rem;
}
footer { margin-top: 4rem; padding-top: 1.5rem; border-top: 1px solid var(--line);
         color: var(--muted); font-size: .85rem; }
a { color: var(--accent); }
"""


def crop_geometry(src_w, src_h, frac):
    fx, fy, fw, fh = frac
    x, y = int(src_w * fx), int(src_h * fy)
    w, h = int(src_w * fw), int(src_h * fh)
    w = min(w, src_w - x)
    h = min(h, src_h - y)
    return f"{w}x{h}+{x}+{y}"


def dimensions(path):
    out = subprocess.run(["identify", "-format", "%w %h", str(path)],
                         capture_output=True, text=True, check=True).stdout
    w, h = out.split()
    return int(w), int(h)


def main():
    IMG.mkdir(parents=True, exist_ok=True)

    for name, stem, frac, *_ in ITEMS:
        src = SRC / f"{stem}.jpg"
        if not src.exists():
            raise SystemExit(f"missing source: {src}")
        w, h = dimensions(src)
        cmd = ["convert", str(src), "-auto-orient"]
        if frac != (0.0, 0.0, 1.0, 1.0):
            cmd += ["-crop", crop_geometry(w, h, frac), "+repage"]
        cmd += ["-resize", "1600x1600>"]

        rects = REDACTIONS.get(name, [])
        if rects:
            # Work out the post-crop, post-resize pixel size so the redaction
            # rectangles land in the right place. -draw takes pixels only.
            cw = int(w * frac[2]) or w
            ch = int(h * frac[3]) or h
            scale = min(1.0, 1600 / max(cw, ch))
            fw_px, fh_px = cw * scale, ch * scale
            cmd += ["-fill", "#3a3a3a"]
            for rx, ry, rw, rh in rects:
                x0, y0 = int(rx * fw_px), int(ry * fh_px)
                x1, y1 = int((rx + rw) * fw_px), int((ry + rh) * fh_px)
                cmd += ["-draw", f"rectangle {x0},{y0} {x1},{y1}"]

        cmd += ["-quality", "86", "-strip", str(IMG / name)]
        subprocess.run(cmd, check=True)
        print(f"  {name}{'  [redacted]' if rects else ''}")

    by_section = {}
    for item in ITEMS:
        by_section.setdefault(item[3], []).append(item)

    parts = []
    for key, heading, note in SECTIONS:
        items = by_section.get(key, [])
        figs = []
        for name, _stem, _frac, _sec, title, body in items:
            figs.append(f"""      <figure>
        <img src="images/{name}" alt="{html.escape(title.replace('&mdash;', '-'), quote=True)}" loading="lazy">
        <figcaption>
          <h3>{title}</h3>
          <p>{body}</p>
          <p class="fname">{name}</p>
        </figcaption>
      </figure>""")
        parts.append(f"""    <section id="{key}">
      <h2>{heading}</h2>
      <p class="sec-note">{note} <em>({len(items)} photo{'s' if len(items) != 1 else ''})</em></p>
      <div class="grid">
{chr(10).join(figs)}
      </div>
    </section>""")

    nav = "\n".join(
        f'      <a href="#{k}">{h.replace("&mdash;", "&ndash;")}</a>'
        for k, h, _ in SECTIONS)

    doc = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tang Primer 25K bench &mdash; labelled parts photos</title>
<style>{CSS}</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>Tang Primer 25K bench &mdash; labelled parts</h1>
    <p class="sub">{len(ITEMS)} photos, cropped to their subject and renamed to say what they are.</p>
    <div class="lede">
      <p>Every image below was pulled from the phone dump, cropped down to the
      part itself, and written out under a descriptive filename. The originals
      are untouched. Ordering follows what the bench is actually for: the FPGA
      first, then the things that move and the electronics that drive them,
      then the supporting parts.</p>
    </div>
    <nav>
{nav}
    </nav>
  </header>

  <div class="callout">
    <strong>One thing worth flagging.</strong> The Verilog in
    <code>motor/src/motor.v</code> is written against a <strong>DRV8833</strong>
    H-bridge (<code>IN1</code>/<code>IN2</code> on pins <code>G5</code>/<code>F5</code>),
    but there is no DRV8833 in these photos. What is here is a
    <strong>DRV8301 three-phase BLDC stage</strong>, an <strong>L293D</strong>
    DIP H-bridge and a <strong>ULN2003</strong> unipolar stepper board. The
    L293D is the closest drop-in for that code &mdash; its two enable pins take
    the PWM and its direction pins replace IN1/IN2 &mdash; while the DRV8301 is
    a different job entirely and needs six PWM inputs, not two.
  </div>

{chr(10).join(parts)}

  <footer>
    <p>Built from 84 phone originals, which are kept outside this repo because
    they carry GPS EXIF. The published copies in <code>photos/images/</code> are
    cropped, stripped of all metadata, and redacted where a delivery slip was in
    frame. Regenerate with
    <code>PHOTO_SRC=&lt;dir&gt; python3 photos/build_gallery.py</code>.</p>
    <p>Part identifications are read off package markings and board silkscreens
    in the photos. Resistor values are only stated where the tape is
    hand-labelled or the bands are unambiguous &mdash; the rest are flagged as
    needing a meter.</p>
  </footer>
</div>
</body>
</html>
"""
    (OUT / "index.html").write_text(doc)
    print(f"\nwrote {OUT/'index.html'} ({len(ITEMS)} entries)")


if __name__ == "__main__":
    main()
