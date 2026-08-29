// Link layer round trip: link_tx -> uart_tx -> wire -> uart_rx -> link_rx.
//
// The whole path is exercised rather than the parsers in isolation, because
// the interesting bugs live at the seams -- byte handshakes, the payload
// fetch latency, and CRC accumulation order.
//
// Checks:
//   1. a frame with a payload survives the round trip intact
//   2. a zero-payload frame (KEEPALIVE) works -- the plen == 0 corner
//   3. a corrupted byte is REJECTED, not partially applied
//   4. garbage before a frame does not prevent the frame being found
//   5. a truncated frame followed by a good one recovers

`timescale 1ns/1ps

module link_tb;

    localparam CPB = 25;             // 50 MHz / 2 Mbaud -- exact

    reg clk = 0;
    always #10 clk = ~clk;           // 50 MHz
    reg rst = 1;

    integer errors = 0;

    // ---- transmit side -------------------------------------------------
    reg        send = 0;
    reg [7:0]  op = 0, plen = 0, seq = 0;
    wire       ltx_busy;
    wire [5:0] tx_pay_addr;
    reg  [7:0] tx_pay [0:63];
    reg  [7:0] tx_pay_data;
    always @(posedge clk) tx_pay_data <= tx_pay[tx_pay_addr];

    wire [7:0] utx_data;
    wire       utx_start, utx_busy;

    link_tx u_ltx (
        .clk(clk), .rst(rst), .send(send), .op(op), .plen(plen), .seq(seq),
        .busy(ltx_busy), .pay_addr(tx_pay_addr), .pay_data(tx_pay_data),
        .tx_data(utx_data), .tx_start(utx_start), .tx_busy(utx_busy));

    wire line;
    uart_tx #(.CLKS_PER_BIT(CPB)) u_utx (
        .clk(clk), .rst(rst), .start(utx_start), .data(utx_data),
        .tx(line), .busy(utx_busy));

    // A second transmitter so the testbench can inject deliberately
    // malformed bytes onto the same wire.
    reg       inj_start = 0;
    reg [7:0] inj_data = 0;
    wire      inj_line, inj_busy;
    uart_tx #(.CLKS_PER_BIT(CPB)) u_inj (
        .clk(clk), .rst(rst), .start(inj_start), .data(inj_data),
        .tx(inj_line), .busy(inj_busy));

    wire wire_in = line & inj_line;      // idle high, either may drive low

    // ---- receive side --------------------------------------------------
    wire [7:0] urx_data;
    wire       urx_valid;

    uart_rx #(.CLKS_PER_BIT(CPB)) u_urx (
        .clk(clk), .rst(rst), .rx(wire_in), .enable(1'b1),
        .data(urx_data), .valid(urx_valid));

    wire        fvalid;
    wire [7:0]  rseq, rop, rplen;
    wire [15:0] crc_errors, timeouts;
    wire [7:0]  rx_pay_data;
    wire [5:0]  rx_pay_addr;
    wire        rx_pay_we;

    localparam LINK_TMO = 2000;      // shrunk for simulation

    link_rx #(.IDLE_TIMEOUT(LINK_TMO)) u_lrx (
        .clk(clk), .rst(rst), .rx_data(urx_data), .rx_valid(urx_valid),
        .frame_valid(fvalid), .seq(rseq), .op(rop), .plen(rplen),
        .crc_errors(crc_errors), .timeouts(timeouts),
        .pay_data(rx_pay_data), .pay_addr(rx_pay_addr), .pay_we(rx_pay_we));

    reg [7:0] rx_pay [0:63];
    always @(posedge clk) if (rx_pay_we) rx_pay[rx_pay_addr] <= rx_pay_data;

    reg got = 0;
    always @(posedge clk) if (fvalid) got <= 1;

    // ---- helpers -------------------------------------------------------
    task send_frame(input [7:0] o, input [7:0] s, input [7:0] n);
        begin
            @(negedge clk); op = o; seq = s; plen = n; send = 1;
            @(negedge clk); send = 0;
            wait (ltx_busy);
            wait (!ltx_busy);
            repeat (CPB * 15) @(posedge clk);   // let the last byte drain
        end
    endtask

    task inject(input [7:0] v);
        begin
            wait (!inj_busy);
            @(negedge clk); inj_data = v; inj_start = 1;
            @(negedge clk); inj_start = 0;
            wait (inj_busy); wait (!inj_busy);
        end
    endtask

    integer i;
    task check_payload(input [7:0] n);
        begin
            for (i = 0; i < n; i = i + 1)
                if (rx_pay[i] !== tx_pay[i]) begin
                    errors = errors + 1;
                    $display("FAIL: payload[%0d] = %02h, expected %02h",
                             i, rx_pay[i], tx_pay[i]);
                end
        end
    endtask

    initial begin
        $dumpfile("link_tb.vcd");
        $dumpvars(0, link_tb);

        for (i = 0; i < 64; i = i + 1) tx_pay[i] = 8'h00;

        repeat (4) @(posedge clk); rst = 0; repeat (4) @(posedge clk);

        // --- 1. frame with payload ---------------------------------------
        $display("--- 1. round trip with payload ---");
        tx_pay[0]=8'h01; tx_pay[1]=8'hD0; tx_pay[2]=8'h07;   // id 1, pos 2000
        tx_pay[3]=8'h02; tx_pay[4]=8'hB8; tx_pay[5]=8'h0B;   // id 2, pos 3000
        got = 0;
        send_frame(8'h01, 8'h2A, 8'd6);
        if (!got)                    begin errors=errors+1; $display("FAIL: no frame"); end
        else if (rop  !== 8'h01)     begin errors=errors+1; $display("FAIL: op %02h", rop); end
        else if (rseq !== 8'h2A)     begin errors=errors+1; $display("FAIL: seq %02h", rseq); end
        else if (rplen!== 8'd6)      begin errors=errors+1; $display("FAIL: len %0d", rplen); end
        else begin check_payload(8'd6); $display("ok   op=%02h seq=%02h len=%0d payload intact",
                                                 rop, rseq, rplen); end

        // --- 2. zero-payload frame ---------------------------------------
        $display("--- 2. zero-payload frame (KEEPALIVE) ---");
        got = 0;
        send_frame(8'h7F, 8'h2B, 8'd0);
        if (!got || rop !== 8'h7F || rplen !== 8'd0) begin
            errors = errors + 1; $display("FAIL: keepalive not received cleanly");
        end else $display("ok   zero-length payload handled");

        // --- 3. corrupted frame must be rejected -------------------------
        $display("--- 3. corrupted frame ---");
        begin : corrupt
            integer before_err;
            before_err = crc_errors;
            got = 0;
            // A well-formed header and payload, but a wrong CRC byte.
            inject(8'hAA); inject(8'h55); inject(8'h02); inject(8'h30);
            inject(8'h01); inject(8'h11); inject(8'h22); inject(8'hFF);
            repeat (CPB * 20) @(posedge clk);
            if (got) begin
                errors = errors + 1;
                $display("FAIL: corrupted frame was ACCEPTED");
            end else if (crc_errors !== before_err + 1) begin
                errors = errors + 1;
                $display("FAIL: crc_errors did not increment (%0d -> %0d)",
                         before_err, crc_errors);
            end else
                $display("ok   corrupted frame rejected, crc_errors = %0d", crc_errors);
        end

        // --- 4. leading garbage ------------------------------------------
        $display("--- 4. garbage before a good frame ---");
        inject(8'h00); inject(8'hFF); inject(8'hAA); inject(8'h13);
        tx_pay[0]=8'hDE; tx_pay[1]=8'hAD;
        got = 0;
        send_frame(8'h02, 8'h44, 8'd2);
        if (!got || rop !== 8'h02 || rseq !== 8'h44) begin
            errors = errors + 1; $display("FAIL: did not resync after garbage");
        end else begin check_payload(8'd2); $display("ok   resynchronised after garbage"); end

        // --- 5. truncated frame then a good one --------------------------
        $display("--- 5. truncated frame, then a good one ---");
        inject(8'hAA); inject(8'h55); inject(8'h08); inject(8'h01);
        inject(8'h01);                       // claims 8 payload bytes, sends none
        repeat (LINK_TMO + 500) @(posedge clk);   // let the parser time out
        if (timeouts !== 16'd1) begin
            errors = errors + 1;
            $display("FAIL: parser did not register a mid-frame timeout");
        end else
            $display("ok   truncated frame timed out and resynchronised");
        tx_pay[0]=8'hBE; tx_pay[1]=8'hEF;
        got = 0;
        send_frame(8'h03, 8'h55, 8'd2);
        if (!got || rop !== 8'h03) begin
            errors = errors + 1;
            $display("FAIL: did not recover after a truncated frame");
        end else begin check_payload(8'd2); $display("ok   recovered after truncation"); end

        $display("");
        if (errors == 0) $display("PASS: link framing, CRC, and resynchronisation");
        else             $display("FAILED: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAILED: testbench timed out");
        $finish;
    end

endmodule
