// Testbench: proves the wire format before any real servo sees it.
//
// The bus is modelled as a wired-AND of every driver, which is what a
// single-wire half-duplex bus with idle-high push-pull drivers behaves like
// when only one party talks at a time. Crucially this means the DUT's own
// transmission lands back on its own receive pin -- so the echo defence is
// genuinely exercised here, not assumed.
//
// Checks, in order:
//   1. uart_tx -> uart_rx round trip (the M1 gate, in simulation)
//   2. PING / MOVE / READ produce byte-exact frames from the plan
//   3. a MOVE actually lands in the model's register file
//   4. a READ brings the position back
//   5. with the model silent, the DUT TIMES OUT rather than believing its
//      own echo  <-- the one that catches the vendor SDK's bug
//   6. a deliberately corrupted frame is rejected by the model

`timescale 1ns/1ps

module servo_tb;

    localparam CPB     = 50;         // 50 MHz / 1 Mbaud
    localparam TMO     = 50_000;     // 1 ms response timeout
    localparam BIT_NS  = 1000;

    reg clk = 0;
    always #10 clk = ~clk;           // 50 MHz
    reg rst = 1;

    integer errors = 0;

    task fail(input [1023:0] why);
        begin
            errors = errors + 1;
            $display("FAIL t=%0t: %0s", $time, why);
        end
    endtask

    // =====================================================================
    // 1. UART round trip, standalone
    // =====================================================================
    reg        lb_start = 0;
    reg  [7:0] lb_data  = 0;
    wire       lb_line, lb_busy;
    wire [7:0] lb_out;
    wire       lb_valid;

    uart_tx #(.CLKS_PER_BIT(CPB)) lb_tx (
        .clk(clk), .rst(rst), .start(lb_start), .data(lb_data),
        .tx(lb_line), .busy(lb_busy));

    uart_rx #(.CLKS_PER_BIT(CPB)) lb_rx (
        .clk(clk), .rst(rst), .rx(lb_line), .enable(1'b1),
        .data(lb_out), .valid(lb_valid));

    // Two subtleties, both of which bit during bring-up:
    //
    //  - Wait on the `valid` EVENT, not on a sticky flag written non-blocking
    //    by an always block and blocking by a task; that is a race.
    //
    //  - Wait for `!busy` before starting the next byte. The RECEIVER reports
    //    a byte from the middle of the stop bit, which is half a bit-time
    //    BEFORE the transmitter is finished with it. Treating `valid` as
    //    "ready to send the next one" asserts `start` while the transmitter
    //    is still busy, where it is silently ignored. hx_bus gates every byte
    //    on !tx_busy for exactly this reason.
    task loopback(input [7:0] v);
        begin
            wait (!lb_busy);
            @(negedge clk); lb_data = v; lb_start = 1;
            @(negedge clk); lb_start = 0;
            @(posedge lb_valid);
            if (lb_out !== v) begin
                errors = errors + 1;
                $display("FAIL t=%0t: loopback sent %02h got %02h", $time, v, lb_out);
            end else
                $display("ok   loopback %02h", v);
        end
    endtask

    // =====================================================================
    // The bus
    // =====================================================================
    wire dut_tx, model_tx;
    reg  tb_tx = 1'b1;                       // for injecting corrupt frames
    wire bus = dut_tx & model_tx & tb_tx;    // idle high; any driver pulls low

    reg         send = 0;
    reg  [7:0]  id = 8'd1, instr = 0;
    reg  [63:0] params = 0;
    reg  [3:0]  nparam = 0;
    reg         expect_reply = 1;

    wire        busy, resp_valid, resp_bad, resp_timeout;
    wire [7:0]  err_byte;
    wire [63:0] resp_params;
    wire [3:0]  resp_nparam;

    hx_bus #(.CLKS_PER_BIT(CPB), .RESP_TIMEOUT(TMO)) dut (
        .clk(clk), .rst(rst),
        .send(send), .id(id), .instr(instr), .params(params),
        .nparam(nparam), .expect_reply(expect_reply), .busy(busy),
        .resp_valid(resp_valid), .resp_bad(resp_bad),
        .resp_timeout(resp_timeout), .err_byte(err_byte),
        .resp_params(resp_params), .resp_nparam(resp_nparam),
        .tx_pin(dut_tx), .rx_pin(bus));

    reg         model_en = 1;
    wire [15:0] pkt_ok, pkt_bad, model_target;

    hx_model #(.BIT_NS(BIT_NS), .ID(8'd1), .RESP_DELAY_NS(20_000)) u_model (
        .rx(bus), .tx(model_tx), .enabled(model_en),
        .pkt_ok(pkt_ok), .pkt_bad(pkt_bad), .target_pos(model_target));

    // --- sniffer: decode every byte that appears on the wire -------------
    reg [7:0] snif [0:31];
    integer   sn = 0;
    reg [7:0] sb;
    integer   si;

    initial begin
        forever begin
            @(negedge bus);
            #(BIT_NS + BIT_NS/2);
            for (si = 0; si < 8; si = si + 1) begin
                sb[si] = bus;
                #(BIT_NS);
            end
            if (sn < 32) snif[sn] = sb;
            sn = sn + 1;
        end
    end

    task chk(input integer idx, input [7:0] want);
        begin
            if (snif[idx] !== want) begin
                errors = errors + 1;
                $display("FAIL: tx byte %0d = %02h, expected %02h", idx, snif[idx], want);
            end
        end
    endtask

    // --- result flags ----------------------------------------------------
    reg clr = 0;
    reg got_valid = 0, got_bad = 0, got_tmo = 0;
    always @(posedge clk) begin
        if (clr) begin
            got_valid <= 0; got_bad <= 0; got_tmo <= 0;
        end else begin
            if (resp_valid)   got_valid <= 1;
            if (resp_bad)     got_bad   <= 1;
            if (resp_timeout) got_tmo   <= 1;
        end
    end

    // Stimulus is driven on the FALLING edge throughout.
    //
    // Driving a DUT input with a blocking assignment from a process that
    // resumed on `posedge clk` is a race: the scheduler does not define
    // whether this block or the DUT's always block runs first, so the DUT
    // may sample the new value or the old one. Driving on the opposite edge
    // makes the value unambiguous at every sampling edge.
    task do_cmd;
        begin
            @(negedge clk); clr = 1; sn = 0;
            @(negedge clk); clr = 0;
            @(negedge clk); send = 1;
            @(negedge clk); send = 0;
            wait (busy);
            wait (!busy);
            repeat (10) @(posedge clk);
        end
    endtask

    // =====================================================================
    // Stimulus
    // =====================================================================
    initial begin
        $dumpfile("servo_tb.vcd");
        $dumpvars(0, servo_tb);

        repeat (4) @(posedge clk);
        rst = 0;
        repeat (4) @(posedge clk);

        $display("--- 1. UART round trip ---");
        loopback(8'hA5);
        loopback(8'hFF);
        loopback(8'h00);
        loopback(8'h3C);

        // ------------------------------------------------------------------
        $display("--- 2. PING ---");
        id = 8'd1; instr = 8'h01; nparam = 4'd0; params = 64'd0; expect_reply = 1;
        do_cmd;
        // FF FF 01 02 01 FB
        chk(0, 8'hFF); chk(1, 8'hFF); chk(2, 8'h01);
        chk(3, 8'h02); chk(4, 8'h01); chk(5, 8'hFB);
        if (!got_valid)          fail("PING: no valid response");
        if (err_byte !== 8'h00)  fail("PING: error byte not zero");
        if (got_tmo)             fail("PING: unexpected timeout");
        $display("ok   PING framed correctly and answered");

        // ------------------------------------------------------------------
        $display("--- 3. MOVE to 2048 @ 500 steps/s ---");
        instr  = 8'h03;
        params = {8'h00, 8'h01, 8'hF4, 8'h00, 8'h00, 8'h08, 8'h00, 8'h2A};
        nparam = 4'd7;
        do_cmd;
        // FF FF 01 09 03 2A 00 08 00 00 F4 01 CB
        chk(0, 8'hFF); chk(1, 8'hFF); chk(2, 8'h01); chk(3, 8'h09);
        chk(4, 8'h03); chk(5, 8'h2A); chk(6, 8'h00); chk(7, 8'h08);
        chk(8, 8'h00); chk(9, 8'h00); chk(10,8'hF4); chk(11,8'h01);
        chk(12,8'hCB);
        if (!got_valid)               fail("MOVE: no valid response");
        if (model_target !== 16'd2048) fail("MOVE: model target position wrong");
        $display("ok   MOVE framed correctly, model target = %0d", model_target);

        // ------------------------------------------------------------------
        $display("--- 4. READ current position ---");
        instr  = 8'h02;
        params = {48'd0, 8'h02, 8'h38};
        nparam = 4'd2;
        do_cmd;
        // FF FF 01 04 02 38 02 BE
        chk(0, 8'hFF); chk(1, 8'hFF); chk(2, 8'h01); chk(3, 8'h04);
        chk(4, 8'h02); chk(5, 8'h38); chk(6, 8'h02); chk(7, 8'hBE);
        if (!got_valid)                fail("READ: no valid response");
        if (resp_nparam !== 4'd2)      fail("READ: wrong parameter count");
        if (resp_params[15:0] !== 16'd2048)
            fail("READ: position did not round trip");
        $display("ok   READ returned position = %0d", resp_params[15:0]);

        // ------------------------------------------------------------------
        $display("--- 5. echo rejection (model silent) ---");
        model_en = 0;
        instr = 8'h01; nparam = 4'd0; params = 64'd0;
        do_cmd;
        if (got_valid)
            fail("ECHO: valid reply reported with the servo silent -- the receiver is reading its own transmission");
        if (!got_tmo)
            fail("ECHO: expected a timeout");
        $display("ok   silent bus produced a timeout, not a phantom reply");
        model_en = 1;

        // ------------------------------------------------------------------
        $display("--- 6. corrupted frame is rejected ---");
        begin : corrupt
            integer prev_ok, prev_bad, m, q;
            reg [7:0] frame [0:5];
            prev_ok  = pkt_ok;
            prev_bad = pkt_bad;
            // A PING with the checksum deliberately wrong (FB -> FC).
            frame[0]=8'hFF; frame[1]=8'hFF; frame[2]=8'h01;
            frame[3]=8'h02; frame[4]=8'h01; frame[5]=8'hFC;
            for (m = 0; m < 6; m = m + 1) begin
                tb_tx = 0; #(BIT_NS);
                for (q = 0; q < 8; q = q + 1) begin
                    tb_tx = frame[m][q]; #(BIT_NS);
                end
                tb_tx = 1; #(BIT_NS);
            end
            #(BIT_NS * 4);
            if (pkt_bad !== prev_bad + 1)
                fail("CORRUPT: model did not flag the bad checksum");
            else if (pkt_ok !== prev_ok)
                fail("CORRUPT: model counted a bad frame as good");
            else
                $display("ok   model rejected the corrupted frame");
        end

        // ------------------------------------------------------------------
        repeat (50) @(posedge clk);
        $display("");
        if (errors == 0)
            $display("PASS: framing, checksums, round trip, echo rejection");
        else
            $display("FAILED: %0d check(s) failed", errors);
        $finish;
    end

    // Watchdog: never let a hung handshake masquerade as a passing run.
    initial begin
        #20_000_000;
        $display("FAILED: testbench timed out");
        $finish;
    end

endmodule
