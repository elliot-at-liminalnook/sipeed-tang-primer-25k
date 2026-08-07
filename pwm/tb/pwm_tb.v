// Self-checking testbench for the PWM breather.
//
// New concepts vs Lesson 2:
//  * hierarchical references (dut.pwm_cnt, dut.duty): a testbench may peek
//    at any internal signal of the design — X-ray vision, simulation only.
//  * a checker that verifies the DUT's defining property automatically,
//    instead of a human eyeballing waveforms: in every 256-cycle PWM
//    period, the LED must be high for exactly `duty` cycles (+/-1 when
//    duty happens to change mid-period).

`timescale 1ns/1ps

module pwm_tb;

    reg clk = 0;
    always #10 clk = ~clk;          // 50 MHz

    // Shrink: brightness step every 300 clocks instead of 196,000.
    // One full breath = 510 steps = 153,000 cycles = ~3 ms of sim time.
    localparam STEP = 300;

    wire led_done, led_ready;
    top #(.STEP_CYCLES(STEP)) dut (
        .clk       (clk),
        .led_done  (led_done),
        .led_ready (led_ready)
    );

    // --- checker: per-PWM-period on-time must equal duty ----------------
    integer high_cnt  = 0;
    integer duty_snap = -1;         // -1 = no full period observed yet
    integer errors    = 0;

    always @(posedge clk) begin
        if (dut.pwm_cnt == 8'd0) begin      // period boundary
            if (duty_snap >= 0 &&
                (high_cnt > duty_snap + 1 || high_cnt < duty_snap - 1)) begin
                errors = errors + 1;
                $display("FAIL t=%0t: duty=%0d but LED high %0d/256 cycles",
                         $time, duty_snap, high_cnt);
            end
            duty_snap = dut.duty;
            high_cnt  = 0;
        end
        if (led_done) high_cnt = high_cnt + 1;
    end

    // --- narrate the triangle turnarounds --------------------------------
    always @(dut.dir)
        $display("t=%8t: modulator direction -> %s (duty=%0d)",
                 $time, dut.dir ? "UP" : "DOWN", dut.duty);

    // --- run one full breath ---------------------------------------------
    initial begin
        $dumpfile("pwm_tb.vcd");
        $dumpvars(0, pwm_tb);

        repeat (510 * STEP + 2000) @(posedge clk);

        if (errors == 0)
            $display("PASS: LED on-time tracked duty in every PWM period");
        else
            $display("FAILED: %0d bad periods", errors);
        $finish;
    end

endmodule
