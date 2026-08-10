# Gowin EDA build — run with: gw_sh /path/to/servo/build.tcl
cd [file dirname [info script]]

set_device GW5A-LV25MG121NC1/I0 -name GW5A-25A

add_file src/servo.v
add_file src/hx_bus.v
add_file src/uart_tx.v
add_file src/uart_rx.v
add_file src/servo.cst
add_file src/servo.sdc

set_option -top_module top

# Dual-purpose config pins used by this board (see blink lesson).
set_option -use_done_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_i2c_as_gpio 1

set_option -output_base_name servo

run all
