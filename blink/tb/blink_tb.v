// Testbench for the blinker.
//
// A testbench is NOT hardware and never touches the FPGA. It's a simulation
// script written in the same language: it fabricates a clock, wires up the
// design under test (DUT), watches the outputs, and dumps every signal to a
// waveform file. None of it needs to be synthesizable — delays (#),
// $display, $dumpfile are simulation-only constructs.

`timescale 1ns/1ps   // time unit for '#' delays / time precision

module blink_tb;

    // --- clock generation -------------------------------------------------
    // The real board has a crystal; in simulation we make our own clock:
    // invert every 10 ns -> 20 ns period -> 50 MHz, same as the dock.
    reg clk = 0;
    always #10 clk = ~clk;

    // --- device under test ------------------------------------------------
    // Override the parameter: toggle every 50 cycles instead of 25 million.
    localparam SIM_TOGGLE_LIMIT = 50;

    wire led_done, led_ready;

    top #(.TOGGLE_LIMIT(SIM_TOGGLE_LIMIT)) dut (
        .clk       (clk),
        .led_done  (led_done),
        .led_ready (led_ready)
    );

    // --- self-checking: measure the time between LED toggles ---------------
    time last_toggle = 0;
    always @(led_done) begin
        $display("t=%6t ns  led_done -> %b   (%0d cycles since last toggle)",
                 $time, led_done, ($time - last_toggle) / 20);
        last_toggle = $time;
    end

    // --- run --------------------------------------------------------------
    initial begin
        $dumpfile("blink_tb.vcd");   // waveform output for GTKWave
        $dumpvars(0, blink_tb);      // record everything under this scope

        repeat (8 * SIM_TOGGLE_LIMIT) @(posedge clk);  // ~8 toggles worth
        $finish;
    end

endmodule
