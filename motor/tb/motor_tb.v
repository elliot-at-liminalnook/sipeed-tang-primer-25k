// Testbench: simulates a human finger, chatter and all.
//
// The DUT parameters are shrunk (debounce 100 cycles instead of 1M) and the
// stimulus deliberately bounces the key line like a real contact. The check:
// each bouncy press must advance `level` by EXACTLY one.

`timescale 1ns/1ps

module motor_tb;

    reg clk = 0;
    always #10 clk = ~clk;              // 50 MHz

    localparam PWM_P = 100;             // shrunk PWM period
    localparam DB    = 100;             // shrunk debounce window (cycles)

    reg key = 0;                        // idle low (active-HIGH button, pull-down)
    wire motor_in1, motor_in2, led_done, led_ready;

    top #(.PWM_PERIOD(PWM_P), .DEBOUNCE_CYC(DB)) dut (
        .clk(clk), .key(key),
        .motor_in1(motor_in1), .motor_in2(motor_in2),
        .led_done(led_done), .led_ready(led_ready)
    );

    integer errors = 0;

    // A press with realistic contact chatter on both edges.
    task bouncy_press;
        integer i;
        begin
            // closing chatter: rapid garbage, each glitch << debounce window
            for (i = 0; i < 6; i = i + 1) begin
                key = ~key; #(150 + i * 337);
            end
            key = 1;                       // settled: pressed (active-high)
            #(DB * 20 * 5);                // hold well past debounce window
            // opening chatter
            for (i = 0; i < 6; i = i + 1) begin
                key = ~key; #(130 + i * 411);
            end
            key = 0;                       // settled: released
            #(DB * 20 * 5);
        end
    endtask

    task expect_level(input integer want);
        begin
            if (dut.level !== want) begin
                errors = errors + 1;
                $display("FAIL t=%0t: level=%0d, expected %0d",
                         $time, dut.level, want);
            end else
                $display("ok   t=%8t: level=%0d duty=%0d/%0d",
                         $time, dut.level, dut.duty, PWM_P);
        end
    endtask

    integer p;
    initial begin
        $dumpfile("motor_tb.vcd");
        $dumpvars(0, motor_tb);

        expect_level(0);

        // six presses: levels 1,2,3,4 then wrap to 0, then 1 again
        for (p = 1; p <= 6; p = p + 1) begin
            bouncy_press;
            expect_level(p <= 4 ? p : p - 5);
        end

        if (errors == 0)
            $display("PASS: every bouncy press advanced level exactly once");
        else
            $display("FAILED: %0d checks failed", errors);
        $finish;
    end

endmodule
