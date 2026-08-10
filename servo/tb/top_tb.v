// Top-level test: the whole lesson, button included.
//
// servo_tb.v proves the wire format. This one proves the thing you will
// actually press: five button presses must walk the command sequence and
// land the servo on the positions the design claims.
//
// Timing parameters are shrunk so this runs in milliseconds rather than
// seconds -- the same trick the blink and motor testbenches use.

`timescale 1ns/1ps

module top_tb;

    localparam CPB = 50;        // 50 MHz / 1 Mbaud
    localparam DB  = 100;       // shrunk debounce window
    localparam TMO = 50_000;    // 1 ms response timeout

    reg clk = 0;
    always #10 clk = ~clk;      // 50 MHz

    reg key = 0, key2 = 0;      // active-high, idle low

    wire servo_tx, model_tx;
    wire bus = servo_tx & model_tx;     // single wire, idle high
    wire led_done, led_ready;

    top #(.CLKS_PER_BIT(CPB), .DEBOUNCE_CYC(DB), .RESP_TIMEOUT(TMO)) uut (
        .clk(clk), .key(key), .key2(key2),
        .servo_tx(servo_tx), .servo_rx(bus),
        .led_done(led_done), .led_ready(led_ready));

    wire [15:0] pkt_ok, pkt_bad, model_target;
    wire [15:0] model_mode, model_speed, model_nvs_viol;
    wire        model_locked;

    hx_model #(.BIT_NS(1000), .ID(8'd1), .RESP_DELAY_NS(20_000)) u_model (
        .rx(bus), .tx(model_tx), .enabled(1'b1),
        .pkt_ok(pkt_ok), .pkt_bad(pkt_bad), .target_pos(model_target),
        .ctrl_mode(model_mode), .run_speed(model_speed),
        .nvs_locked(model_locked), .nvs_violations(model_nvs_viol));

    integer errors = 0;

    // Press, hold well past the debounce window, release, settle.
    task press;
        begin
            @(negedge clk); key = 1;
            repeat (DB * 4) @(posedge clk);
            @(negedge clk); key = 0;
            repeat (DB * 4) @(posedge clk);
            wait (uut.mstate == 2'd0);      // macro finished
            repeat (20) @(posedge clk);
        end
    endtask

    // S2 kicks off a FOUR-packet macro, so waiting for the sequencer to
    // return to idle is the only correct completion test -- `busy` goes low
    // between every packet in the sequence.
    task press2;
        begin
            @(negedge clk); key2 = 1;
            repeat (DB * 4) @(posedge clk);
            @(negedge clk); key2 = 0;
            repeat (DB * 4) @(posedge clk);
            wait (uut.mstate == 2'd0);
            repeat (20) @(posedge clk);
        end
    endtask

    task expect_pos(input [15:0] want);
        begin
            if (model_target !== want) begin
                errors = errors + 1;
                $display("FAIL t=%0t: servo target %0d, expected %0d",
                         $time, model_target, want);
            end else
                $display("ok   servo target = %0d", model_target);
        end
    endtask

    task expect_ok;
        begin
            if (led_done !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL t=%0t: led_done low -- exchange did not succeed",
                         $time);
            end
        end
    endtask

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, top_tb);

        repeat (20) @(posedge clk);

        // press 1: PING
        press; expect_ok;
        $display("ok   PING acknowledged");

        // presses 2-4: the three move commands
        press; expect_ok; expect_pos(16'd1024);
        press; expect_ok; expect_pos(16'd2048);
        press; expect_ok; expect_pos(16'd3072);

        // press 5: read position back -- must agree with the last move
        press; expect_ok;
        if (uut.last_pos !== 16'd3072) begin
            errors = errors + 1;
            $display("FAIL: read back %0d, expected 3072", uut.last_pos);
        end else
            $display("ok   read back position = %0d", uut.last_pos);

        // ------------------------------------------------------------------
        // SPIN: four packets on one press
        $display("--- spin on ---");
        press2;
        if (model_mode !== 16'd1) begin
            errors = errors + 1;
            $display("FAIL: control mode %0d, expected 1 (constant speed)", model_mode);
        end else
            $display("ok   servo in constant-speed mode");

        if (model_speed !== 16'h7FFF) begin
            errors = errors + 1;
            $display("FAIL: run speed %0h, expected 7fff", model_speed);
        end else
            $display("ok   run speed = 0x%04h (maximum)", model_speed);

        // The mode write is in NVS. If the design forgot to unlock 0x37, the
        // model would have refused it -- and the design would then look fine
        // in simulation but do nothing on the bench.
        if (model_nvs_viol !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: %0d NVS write(s) refused -- unlock sequence is wrong",
                     model_nvs_viol);
        end else
            $display("ok   no NVS writes refused");

        if (!model_locked) begin
            errors = errors + 1;
            $display("FAIL: NVS left UNLOCKED after spin-on");
        end else
            $display("ok   NVS re-locked afterwards");

        if (uut.spinning !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: design does not think it is spinning");
        end

        $display("--- spin off ---");
        press2;
        if (model_speed !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: run speed %0d after stop, expected 0", model_speed);
        end else
            $display("ok   speed commanded back to 0");

        if (model_mode !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: control mode %0d, expected 0 (position)", model_mode);
        end else
            $display("ok   restored to position-servo mode");

        if (!model_locked) begin
            errors = errors + 1;
            $display("FAIL: NVS left UNLOCKED after spin-off");
        end
        if (uut.spinning !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: design still thinks it is spinning");
        end

        if (pkt_bad !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: servo saw %0d malformed packet(s)", pkt_bad);
        end

        $display("");
        if (errors == 0)
            $display("PASS: %0d packets, button sequence walked end to end", pkt_ok);
        else
            $display("FAILED: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAILED: testbench timed out");
        $finish;
    end

endmodule
