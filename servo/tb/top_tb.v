// End-to-end test for the two-servo dead-man jog UI and packet sequence.
//
// Proves that:
//   * power-up actively commands stop and restores position mode
//   * a short press is stopped again on release
//   * two short presses toggle direction only after the third-click window
//   * three short presses switch between servo IDs 1 and 2
//   * motion begins on the debounced press, without the old 500 ms delay
//   * release commands speed zero and restores position mode
//   * S2 stops an active jog
//   * release during the start macro cannot leave the servo running

`timescale 1ns/1ps

module top_tb;

    localparam CPB   = 50;
    localparam DB    = 10;
    localparam LONG  = 200;
    localparam DCLICK = 800;
    localparam TMO   = 50_000;
    localparam [15:0] SPEED_FWD = 16'h0D48; // documented maximum: 3400
    localparam [15:0] SPEED_REV = 16'h8D48;

    reg clk = 0;
    always #10 clk = ~clk;

    reg key = 0, key2 = 0;
    reg host_uart_start = 0;
    reg [7:0] host_uart_data = 0;
    wire host_rx, host_uart_busy;

    wire servo_tx, model1_tx, model2_tx, host_tx;
    wire bus = servo_tx & model1_tx & model2_tx;
    wire led_done, led_ready;

    top #(
        .CLKS_PER_BIT(CPB),
        .DEBOUNCE_CYC(DB),
        .LONG_PRESS_CYC(LONG),
        .DOUBLE_CLICK_CYC(DCLICK),
        .RESP_TIMEOUT(TMO),
        .HOST_CLKS_PER_BIT(10),
        .POLL_CYC(400),
        .TELEMETRY_CYC(50_000),
        .MS_CYC(50),
        .HOST_DEADMAN_CYC(100_000)
    ) uut (
        .clk(clk), .key(key), .key2(key2), .host_rx(host_rx),
        .servo_tx(servo_tx), .servo_rx(bus),
        .host_tx(host_tx),
        .led_done(led_done), .led_ready(led_ready));

    uart_tx #(.CLKS_PER_BIT(10)) u_host_uart_tx (
        .clk(clk), .rst(uut.rst), .start(host_uart_start),
        .data(host_uart_data), .tx(host_rx), .busy(host_uart_busy));

    wire [15:0] pkt1_ok, pkt1_bad, model1_target;
    wire [15:0] model1_mode, model1_speed, model1_nvs_viol;
    wire        model1_locked;
    wire [15:0] pkt2_ok, pkt2_bad, model2_target;
    wire [15:0] model2_mode, model2_speed, model2_nvs_viol;
    wire        model2_locked;

    // Decode the USB telemetry with the independent receive path. This checks
    // UART timing, framing, CRC, payload length, and byte ordering end to end.
    wire [7:0] host_byte;
    wire host_byte_valid;
    uart_rx #(.CLKS_PER_BIT(10)) u_host_uart_rx (
        .clk(clk), .rst(uut.rst), .rx(host_tx), .enable(1'b1),
        .data(host_byte), .valid(host_byte_valid));

    wire host_frame_valid;
    wire [7:0] host_seq, host_op, host_plen, host_pay_data;
    wire [5:0] host_pay_addr;
    wire host_pay_we;
    wire [15:0] host_crc_errors, host_frame_timeouts;
    link_rx #(.IDLE_TIMEOUT(5000)) u_host_frame_rx (
        .clk(clk), .rst(uut.rst),
        .rx_data(host_byte), .rx_valid(host_byte_valid),
        .frame_valid(host_frame_valid), .seq(host_seq),
        .op(host_op), .plen(host_plen),
        .crc_errors(host_crc_errors), .timeouts(host_frame_timeouts),
        .pay_data(host_pay_data), .pay_addr(host_pay_addr),
        .pay_we(host_pay_we));

    reg [7:0] host_payload [0:63];
    integer telemetry_frames = 0;
    always @(posedge clk) begin
        if (host_pay_we)
            host_payload[host_pay_addr] <= host_pay_data;
        if (host_frame_valid && host_op == 8'h81 && host_plen == 8'd58)
            telemetry_frames <= telemetry_frames + 1;
    end

    hx_model #(.BIT_NS(1000), .ID(8'd1), .RESP_DELAY_NS(20_000)) u_model1 (
        .rx(bus), .tx(model1_tx), .enabled(1'b1),
        .pkt_ok(pkt1_ok), .pkt_bad(pkt1_bad), .target_pos(model1_target),
        .ctrl_mode(model1_mode), .run_speed(model1_speed),
        .nvs_locked(model1_locked), .nvs_violations(model1_nvs_viol));

    hx_model #(.BIT_NS(1000), .ID(8'd2), .RESP_DELAY_NS(20_000)) u_model2 (
        .rx(bus), .tx(model2_tx), .enabled(1'b1),
        .pkt_ok(pkt2_ok), .pkt_bad(pkt2_bad), .target_pos(model2_target),
        .ctrl_mode(model2_mode), .run_speed(model2_speed),
        .nvs_locked(model2_locked), .nvs_violations(model2_nvs_viol));

    integer errors = 0;

    task wait_idle;
        begin
            wait (uut.mstate == 2'd0 && !uut.stop_pending && !uut.busy);
            repeat (20) @(posedge clk);
        end
    endtask

    task short_click;
        begin
            @(negedge clk); key = 1'b1;
            repeat (DB * 3) @(posedge clk);
            @(negedge clk); key = 1'b0;
            repeat (DB * 3) @(posedge clk);
        end
    endtask

    function [7:0] host_crc8;
        input [7:0] crc_in;
        input [7:0] data;
        integer i;
        reg [7:0] c;
        begin
            c = crc_in ^ data;
            for (i = 0; i < 8; i = i + 1)
                c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
            host_crc8 = c;
        end
    endfunction

    task send_host_byte;
        input [7:0] value;
        begin
            wait (!host_uart_busy);
            @(negedge clk);
            host_uart_data = value;
            host_uart_start = 1'b1;
            @(negedge clk);
            host_uart_start = 1'b0;
            wait (host_uart_busy);
            wait (!host_uart_busy);
        end
    endtask

    task host_control;
        input [7:0] motor_id;
        input [7:0] flags;
        reg [7:0] c;
        begin
            c = host_crc8(0, 8'd2);
            c = host_crc8(c, 8'h44);
            c = host_crc8(c, 8'h10);
            c = host_crc8(c, motor_id);
            c = host_crc8(c, flags);
            send_host_byte(8'hAA); send_host_byte(8'h55); send_host_byte(8'd2);
            send_host_byte(8'h44); send_host_byte(8'h10);
            send_host_byte(motor_id); send_host_byte(flags); send_host_byte(c);
        end
    endtask

    task double_click;
        begin
            short_click;
            repeat (DCLICK / 8) @(posedge clk);
            short_click;
            repeat (DCLICK + 30) @(posedge clk);
        end
    endtask

    task triple_click;
        begin
            short_click;
            repeat (DCLICK / 8) @(posedge clk);
            short_click;
            repeat (DCLICK / 8) @(posedge clk);
            short_click;
            repeat (30) @(posedge clk);
        end
    endtask

    task press_and_hold;
        begin
            @(negedge clk); key = 1'b1;
            repeat (DB * 3) @(posedge clk);
        end
    endtask

    task release_hold;
        begin
            @(negedge clk); key = 1'b0;
            repeat (DB * 3) @(posedge clk);
            wait_idle;
        end
    endtask

    task press_emergency_stop;
        begin
            @(negedge clk); key2 = 1'b1;
            repeat (DB * 3) @(posedge clk);
            @(negedge clk); key2 = 1'b0;
            repeat (DB * 3) @(posedge clk);
            wait_idle;
        end
    endtask

    task expect_stopped;
        begin
            if (model1_speed !== 16'd0 || model2_speed !== 16'd0) begin
                errors = errors + 1;
                $display("FAIL t=%0t: speed1=%04h speed2=%04h while expected stopped",
                         $time, model1_speed, model2_speed);
            end
            if (model1_mode !== 16'd0 || model2_mode !== 16'd0) begin
                errors = errors + 1;
                $display("FAIL t=%0t: mode1=%0d mode2=%0d, expected position mode",
                         $time, model1_mode, model2_mode);
            end
            if (uut.moving !== 1'b0) begin
                errors = errors + 1;
                $display("FAIL t=%0t: FPGA still reports moving", $time);
            end
        end
    endtask

    task expect_moving(input [7:0] want_id, input [15:0] want_speed);
        begin
            wait (uut.moving == 1'b1);
            repeat (20) @(posedge clk);
            if ((want_id == 1 && model1_mode !== 16'd1) ||
                (want_id == 2 && model2_mode !== 16'd1)) begin
                errors = errors + 1;
                $display("FAIL t=%0t: selected ID %0d did not enter speed mode",
                         $time, want_id);
            end
            if ((want_id == 1 && model1_speed !== want_speed) ||
                (want_id == 2 && model2_speed !== want_speed)) begin
                errors = errors + 1;
                $display("FAIL t=%0t: selected ID %0d speed mismatch",
                         $time, want_id);
            end
            if ((want_id == 1 && model2_speed !== 0) ||
                (want_id == 2 && model1_speed !== 0)) begin
                errors = errors + 1;
                $display("FAIL t=%0t: unselected servo moved", $time);
            end
        end
    endtask

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, top_tb);

        repeat (20) @(posedge clk);

        // The bitstream must first clean up both possible persistent modes.
        wait_idle;
        expect_stopped;
        if (!model1_locked || !model2_locked) begin
            errors = errors + 1;
            $display("FAIL: power-up cleanup left an NVS lock open");
        end else
            $display("ok   power-up cleanup stopped and locked both servo IDs");

        // A lone short press releases back into STOP without changing direction.
        short_click;
        repeat (DCLICK + 30) @(posedge clk);
        expect_stopped;
        if (uut.reverse_dir !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: one click changed direction");
        end else
            $display("ok   one short press stopped without changing direction");

        // A double-click selects reverse after its two press/release cycles.
        double_click;
        expect_stopped;
        if (uut.reverse_dir !== 1'b1 || led_done !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: double-click did not select/indicate reverse");
        end else
            $display("ok   double-click selected reverse");

        // A press starts reverse jog before the old long-press threshold;
        // releasing must still stop and restore position mode.
        press_and_hold;
        if (uut.hold_fired !== 1'b0 ||
            (uut.mstate == 2'd0 && !uut.start_pending)) begin
            errors = errors + 1;
            $display("FAIL: press did not arm jog before long-press threshold");
        end else
            $display("ok   press armed reverse jog immediately after debounce");
        expect_moving(8'd1, SPEED_REV);
        release_hold;
        expect_stopped;
        $display("ok   release stopped reverse jog and restored position mode");

        // Toggle back to forward, then prove the opposite signed speed.
        double_click;
        if (uut.reverse_dir !== 1'b0 || led_done !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: second double-click did not select forward");
        end
        press_and_hold;
        expect_moving(8'd1, SPEED_FWD);
        $display("ok   forward jog uses the opposite sign");

        // S2 must stop even while S1 remains physically held.
        press_emergency_stop;
        expect_stopped;
        $display("ok   S2 stopped motion while S1 remained held");
        release_hold;
        expect_stopped;

        // Release as soon as the start macro begins. It may finish the packet
        // already in flight, but it must never reach a lasting run command.
        @(negedge clk); key = 1'b1;
        wait (uut.jog_start);
        wait (uut.macro == 1'b0 && uut.mstate != 2'd0);
        @(negedge clk); key = 1'b0;
        repeat (DB * 3) @(posedge clk);
        wait_idle;
        expect_stopped;
        $display("ok   release during startup fell through to safe cleanup");

        // A triple-click must switch motors without also triggering the
        // double-click direction action. Every click still obeys dead-man
        // release, and the newly selected motor receives a STOP cleanup.
        triple_click;
        wait_idle;
        expect_stopped;
        if (uut.selected_servo_id !== 8'd2 || uut.reverse_dir !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: triple-click did not select ID 2 cleanly");
        end else
            $display("ok   triple-click selected ID 2 without reversing");

        press_and_hold;
        expect_moving(8'd2, SPEED_FWD);
        release_hold;
        expect_stopped;
        $display("ok   ID 2 jogged while ID 1 remained stopped");

        // The dashboard uses the same dead-man semantics over a CRC-protected
        // receive UART. Selection is sent stopped, then RUN must be refreshed.
        host_control(8'd1, 8'b00000000);
        wait_idle;
        host_control(8'd1, 8'b00000001);
        expect_moving(8'd1, SPEED_FWD);
        host_control(8'd1, 8'b00000000);
        wait_idle;
        expect_stopped;
        $display("ok   dashboard hold/release controlled ID 1 safely");

        host_control(8'd2, 8'b00000010);
        wait_idle;
        host_control(8'd2, 8'b00000011);
        expect_moving(8'd2, SPEED_REV);
        wait (!uut.host_run);
        wait_idle;
        expect_stopped;
        if (uut.host_crc_errors !== 0 || uut.host_frame_timeouts !== 0) begin
            errors = errors + 1;
            $display("FAIL: dashboard receiver reported link errors");
        end else
            $display("ok   dashboard link loss stopped reverse ID 2 jog");

        // The low-priority poller must coexist with the button macros and
        // eventually populate every advertised diagnostic register.
        wait (uut.diag_valid[20:0] == 21'h1FFFFF);
        if (uut.voltage_mv !== 16'd11100 || uut.temperature_c !== 8'd32 ||
            uut.current_ma !== 16'd321 || uut.current_limit_ma !== 16'd3000) begin
            errors = errors + 1;
            $display("FAIL: diagnostic cache values do not match model");
        end else
            $display("ok   all 21 diagnostic registers populated while controls remained active");

        // Drain a possibly in-flight snapshot, then inspect one captured
        // after the cache became complete.
        @(posedge clk);
        wait (host_frame_valid);
        @(posedge clk);
        wait (!host_frame_valid);
        wait (host_frame_valid);
        repeat (5) @(posedge clk);
        if (telemetry_frames == 0 || host_payload[0] !== 8'd1 ||
            {host_payload[11], host_payload[10], host_payload[9], host_payload[8]}
                !== 32'h001FFFFF ||
            {host_payload[47], host_payload[46]} !== 16'd11100 ||
            {host_payload[49], host_payload[48]} !== 16'd321 ||
            host_payload[55] !== 8'd2) begin
            errors = errors + 1;
            $display("FAIL: CRC-valid host telemetry payload was missing or incorrect");
        end else
            $display("ok   CRC-valid USB telemetry snapshot has correct schema and values");

        if (host_crc_errors !== 0 || host_frame_timeouts !== 0) begin
            errors = errors + 1;
            $display("FAIL: host telemetry decoder reported framing errors");
        end

        if (uut.bad_replies !== 0 || uut.timeout_replies !== 0 ||
            uut.servo_error_replies !== 0) begin
            errors = errors + 1;
            $display("FAIL: telemetry polling introduced bus errors");
        end

        if (model1_nvs_viol !== 16'd0 || model2_nvs_viol !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: NVS write(s) refused: ID1=%0d ID2=%0d",
                     model1_nvs_viol, model2_nvs_viol);
        end
        if (!model1_locked || !model2_locked) begin
            errors = errors + 1;
            $display("FAIL: NVS left unlocked at end of test");
        end
        if (pkt1_bad !== 16'd0 || pkt2_bad !== 16'd0) begin
            errors = errors + 1;
            $display("FAIL: malformed packets ID1=%0d ID2=%0d", pkt1_bad, pkt2_bad);
        end

        $display("");
        if (errors == 0)
            $display("PASS: two-servo dead-man sequence (%0d observed packets)", pkt1_ok);
        else
            $display("FAILED: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("FAILED: testbench timed out");
        $finish;
    end

endmodule
