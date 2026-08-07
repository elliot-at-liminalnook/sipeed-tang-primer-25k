// Lesson 4: breathing LED via PWM, Tang Primer 25K (GW5A-25)
//
// Two nested timescales:
//   fast:  8-bit counter wraps every 256 clocks -> 195 kHz PWM period
//   slow:  duty cycle sweeps 0..255..0 as a triangle -> ~2 s "breath"

module top #(
    parameter STEP_CYCLES = 196_000   // clocks per brightness step (~3.9 ms)
)(
    input  wire clk,        // 50 MHz (pin E2)
    output wire led_done,   // breathing LED (pin D7)
    output wire led_ready   // anti-phase breathing LED (pin E8)
);

    // ---- fast timescale: the PWM core ---------------------------------
    reg [7:0] pwm_cnt = 0;
    always @(posedge clk)
        pwm_cnt <= pwm_cnt + 1'b1;   // 8 bits wrap 255->0 by themselves:
                                     // the overflow IS the period boundary

    reg [7:0] duty = 0;

    // Combinational compare, not a register: high for exactly `duty`
    // of every 256 clock cycles.
    wire pwm_out = (pwm_cnt < duty);

    // ---- slow timescale: triangle-wave modulator -----------------------
    reg                            dir      = 1'b1;  // 1 = ramping up
    reg [$clog2(STEP_CYCLES)-1:0]  step_cnt = 0;

    always @(posedge clk) begin
        if (step_cnt == STEP_CYCLES - 1) begin
            step_cnt <= 0;
            if (dir) begin
                duty <= duty + 1'b1;
                // Turn around one step EARLY: this cycle's duty is 254,
                // so next cycle duty becomes 255 and dir is already 'down'.
                // Comparing against 255 instead would let duty wrap 255->0:
                // a full-bright to pitch-black glitch every breath.
                if (duty == 8'd254) dir <= 1'b0;
            end else begin
                duty <= duty - 1'b1;
                if (duty == 8'd1) dir <= 1'b1;
            end
        end else begin
            step_cnt <= step_cnt + 1'b1;
        end
    end

    assign led_done  = pwm_out;
    assign led_ready = ~pwm_out;   // complementary duty: breathes in anti-phase
endmodule
