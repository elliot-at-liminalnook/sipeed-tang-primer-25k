# Timing constraints: tell the tools what "fast enough" means.
# One clock, 50 MHz -> 20 ns period. Place & route must prove that every
# register-to-register path settles within one period, or report failure.
create_clock -name clk_50m -period 20 [get_ports {clk}]
