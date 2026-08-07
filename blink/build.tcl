# Gowin EDA build script — run with: gw_sh /path/to/build.tcl
# The IDE is just a GUI over these same Tcl commands.

# The gw_sh wrapper cd's into the Gowin install dir, so anchor all
# relative paths to this script's own location instead.
cd [file dirname [info script]]

# Exact chip on the Tang Primer 25K core board.
# Full ordering code incl. speed grade (C1/I0); "A" is the silicon revision.
set_device GW5A-LV25MG121NC1/I0 -name GW5A-25A

add_file src/blink.v
add_file src/blink.cst
add_file src/blink.sdc

set_option -top_module top

# Dual-purpose configuration pins: after configuration these normally
# still belong to the config engine. Each option below hands one group
# back to the user design as ordinary GPIO. Our design needs:
#   DONE (D7 led), READY (E8 led), CPU/SSPI (E2 clock input!)
# I2C matches Sipeed's reference config for this board.
set_option -use_done_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_i2c_as_gpio 1

set_option -output_base_name blink

# synthesis -> place & route -> bitstream, all stages
run all
