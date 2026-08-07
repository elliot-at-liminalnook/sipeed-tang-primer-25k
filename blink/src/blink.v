// LED blinker for Tang Primer 25K (Gowin GW5A-25)
//
// 50 MHz in, 1 Hz blink out: count 25_000_000 clock edges, toggle, repeat.

module top #(
    // Clock edges per LED toggle. A parameter (not a hardcoded constant) so
    // the simulation testbench can shrink it and not sit through 25M cycles.
    parameter TOGGLE_LIMIT = 25_000_000
)(
    input  wire clk,        // 50 MHz oscillator on the dock (pin E2)
    output wire led_done,   // "DONE" LED on the core board (pin D7)
    output wire led_ready   // "READY" LED on the core board (pin E8)
);

    // $clog2 = ceiling log2, computed at synthesis time: the counter is
    // always exactly wide enough for TOGGLE_LIMIT states, even if the
    // parameter changes. For 25_000_000 this is your 25 bits.
    reg [$clog2(TOGGLE_LIMIT)-1:0] counter   = 0;
    reg                            led_state = 0;

    always @(posedge clk) begin
        if (counter == TOGGLE_LIMIT - 1) begin
            counter   <= 0;
            led_state <= ~led_state;
        end else begin
            counter <= counter + 1'b1;
        end
    end

    assign led_done  = led_state;
    assign led_ready = ~led_state;   // opposite phase: they alternate
endmodule
