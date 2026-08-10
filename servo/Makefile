# Open-source build for the servo lesson.
#
#   yosys -> nextpnr-himbaechel -> apicula (gowin_pack) -> openFPGALoader
#
# This is an alternative to build.tcl, which drives Gowin's own EDA. Nothing
# here needs a vendor account or a licence. Everything ships in oss-cad-suite:
#
#   https://github.com/YosysHQ/oss-cad-suite-build/releases
#
#   make          build impl/servo.fs
#   make flash    load it into SRAM (volatile -- gone at power-off)
#   make prog     write it to flash (survives power cycling)
#   make sim      run both testbenches
#   make clean

DEVICE := GW5A-LV25MG121NC1/I0
FAMILY := GW5A-25A
BOARD  := tangprimer25k
FREQ   := 50

SRC := src/uart_tx.v src/uart_rx.v src/hx_bus.v src/button.v src/servo.v
CST := src/servo.cst

# GW5A-25A REQUIRES the SSPI pins to be configured as GPIO, and nextpnr and
# gowin_pack cross-check each other on it: set it in one place only and the
# pack step aborts with "conflicting settings". Hence the flag twice.
SSPI := sspi_as_gpio

# The dock's LEDs sit on the DONE and READY config pins, so those have to be
# released to GPIO as well (same reasoning as build.tcl's set_option lines).
PACKFLAGS := --$(SSPI) --done_as_gpio --ready_as_gpio

all: impl/servo.fs

impl/servo.json: $(SRC)
	@mkdir -p impl
	yosys -p "read_verilog $(SRC); synth_gowin -top top -json $@"

impl/servo_pnr.json: impl/servo.json $(CST)
	nextpnr-himbaechel --json $< --write $@ \
	  --device "$(DEVICE)" --vopt cst=$(CST) --vopt $(SSPI) --freq $(FREQ)

impl/servo.fs: impl/servo_pnr.json
	gowin_pack -d $(FAMILY) -o $@ $(PACKFLAGS) $<

# SRAM load: fast, and forgotten at power-off. Use this while iterating --
# it means a bad bitstream is never more than a power cycle away from gone.
flash: impl/servo.fs
	openFPGALoader -b $(BOARD) $<

# Write to onboard flash: survives power cycling.
prog: impl/servo.fs
	openFPGALoader -b $(BOARD) -f $<

sim:
	@mkdir -p impl
	iverilog -g2005 -o impl/servo_tb.vvp tb/servo_tb.v tb/hx_model.v \
	  src/hx_bus.v src/uart_tx.v src/uart_rx.v && vvp impl/servo_tb.vvp
	iverilog -g2005 -o impl/top_tb.vvp tb/top_tb.v tb/hx_model.v \
	  $(SRC) && vvp impl/top_tb.vvp

# --- debug bridge -----------------------------------------------------------
# A separate top module: relays packets between the dock's USB-UART and the
# servo bus so tools/hx.py can interrogate the servo directly.
BSRC := src/uart_tx.v src/uart_rx.v src/bridge.v

bridge: impl/bridge.fs

impl/bridge.json: $(BSRC)
	@mkdir -p impl
	yosys -p "read_verilog $(BSRC); synth_gowin -top top -json $@"

impl/bridge_pnr.json: impl/bridge.json src/bridge.cst
	nextpnr-himbaechel --json $< --write $@ \
	  --device "$(DEVICE)" --vopt cst=src/bridge.cst --vopt $(SSPI) --freq $(FREQ)

impl/bridge.fs: impl/bridge_pnr.json
	gowin_pack -d $(FAMILY) -o $@ $(PACKFLAGS) $<

bridge-flash: impl/bridge.fs
	openFPGALoader -b $(BOARD) $<

clean:
	rm -rf impl *.vvp *.vcd

.PHONY: all flash prog sim clean bridge bridge-flash
