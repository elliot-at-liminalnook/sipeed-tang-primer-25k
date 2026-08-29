// Behavioural HX-30HM, for simulation only.
//
// With no scope and no logic analyser on the bench, this model is the
// instrument: it parses what the FPGA actually transmits, REJECTS bad
// checksums rather than tolerating them, and answers with correctly framed
// replies. Every packet is proven here before a real servo is asked to
// interpret it.
//
// Written behaviourally (bit-banged with delays, not a clocked FSM) on
// purpose -- it is a measuring device, and it should be obvious at a glance
// that it is not part of the design under test.
//
// `enabled` exists for one specific test: switch the model off and the DUT
// must time out. If it instead reports a valid reply, it is reading its own
// echo -- exactly the failure in Hiwonder's shipped SDK.

`timescale 1ns/1ps

module hx_model #(
    parameter BIT_NS        = 1000,     // 1 Mbaud
    parameter [7:0] ID      = 8'd1,
    parameter RESP_DELAY_NS = 20_000    // servo turnaround (real part: >= 1 ms)
)(
    input  wire        rx,              // bus -> model
    output reg         tx = 1'b1,       // model -> bus (wired-AND at the top)
    input  wire        enabled,
    output reg [15:0]  pkt_ok  = 16'd0, // well-formed packets received
    output reg [15:0]  pkt_bad = 16'd0, // checksum failures
    output reg [15:0]  target_pos = 16'd0,
    output reg [15:0]  ctrl_mode  = 16'd0,  // register 0x21
    output reg [15:0]  run_speed  = 16'd0,  // register 0x2E
    output reg         nvs_locked = 1'b1,   // register 0x37
    output reg [15:0]  nvs_violations = 16'd0
);

    // Addresses below this are the NVS/ROM area and are write-protected.
    localparam [7:0] NVS_LIMIT = 8'h28;

    // Register file. Only the addresses this lesson touches are meaningful.
    reg [7:0] mem [0:255];

    reg [7:0]  pid, plen, pinstr, pcs, acc;
    reg [7:0]  pbuf [0:15];
    integer    n, i;

    reg [7:0]  rbuf [0:15];
    integer    rn;

    integer    k;
    initial begin
        for (k = 0; k < 256; k = k + 1) mem[k] = 8'h00;
        mem[8'h05] = ID;            // ID
        mem[8'h06] = 8'd0;          // baud index 0 = 1 Mbaud
        mem[8'h08] = 8'd1;          // reply to reads and writes
        mem[8'h0D] = 8'd70;         // 70 C protection threshold
        mem[8'h13] = 8'd100;        // maximum torque, percent
        mem[8'h14] = 8'h1F;         // protection sources enabled
        mem[8'h1F] = 8'hB8;         // 3000 mA current limit
        mem[8'h20] = 8'h0B;
        mem[8'h38] = 8'h00;         // current position low
        mem[8'h39] = 8'h00;         // current position high
        mem[8'h3A] = 8'd23;         // current speed
        mem[8'h3C] = 8'd45;         // current load
        mem[8'h3E] = 8'h5C;         // 11100 mV (special-cased below)
        mem[8'h3F] = 8'd32;         // temperature, degrees C
        mem[8'h41] = 8'd0;          // no protection fault
        mem[8'h42] = 8'd0;          // stopped
        mem[8'h45] = 8'h41;         // 321 mA
        mem[8'h46] = 8'h01;
    end

    // --- line level ------------------------------------------------------
    task get_byte(output [7:0] b);
        integer j;
        begin
            @(negedge rx);                  // start bit
            #(BIT_NS + BIT_NS/2);           // centre of bit 0
            for (j = 0; j < 8; j = j + 1) begin
                b[j] = rx;                  // LSB first
                #(BIT_NS);
            end
            // now sitting in the stop bit
        end
    endtask

    task put_byte(input [7:0] b);
        integer j;
        begin
            tx = 1'b0; #(BIT_NS);           // start
            for (j = 0; j < 8; j = j + 1) begin
                tx = b[j]; #(BIT_NS);
            end
            tx = 1'b1; #(BIT_NS);           // stop
        end
    endtask

    // --- reply -----------------------------------------------------------
    // rbuf[0..rn-1] holds the parameter bytes; err is the status byte.
    task send_reply(input [7:0] err);
        integer j;
        reg [7:0] cs;
        begin
            #(RESP_DELAY_NS);
            cs = ID + (rn + 2) + err;
            for (j = 0; j < rn; j = j + 1) cs = cs + rbuf[j];
            put_byte(8'hFF);
            put_byte(8'hFF);
            put_byte(ID);
            put_byte(rn + 2);
            put_byte(err);
            for (j = 0; j < rn; j = j + 1) put_byte(rbuf[j]);
            put_byte(~cs);
        end
    endtask

    // --- main loop -------------------------------------------------------
    reg [7:0] b0, b1;
    reg [7:0] addr;
    integer   len;

    initial begin
        forever begin
            get_byte(b0);
            if (b0 == 8'hFF) begin
                get_byte(b1);
                if (b1 == 8'hFF) begin
                    get_byte(pid);
                    get_byte(plen);
                    get_byte(pinstr);
                    n   = plen - 2;
                    acc = pid + plen + pinstr;
                    for (i = 0; i < n; i = i + 1) begin
                        get_byte(pbuf[i]);
                        acc = acc + pbuf[i];
                    end
                    get_byte(pcs);

                    if (pcs !== ((~acc) & 8'hFF)) begin
                        pkt_bad = pkt_bad + 1;
                        $display("  [model] BAD CHECKSUM t=%0t: got %02h, expected %02h",
                                 $time, pcs, (~acc) & 8'hFF);
                    end else begin
                        pkt_ok = pkt_ok + 1;

                        if (pid == ID || pid == 8'hFE) begin
                            rn = 0;
                            case (pinstr)
                                8'h01: begin                   // PING
                                    $display("  [model] PING id=%0d", pid);
                                end

                                8'h02: begin                   // READ
                                    addr = pbuf[0];
                                    len  = pbuf[1];
                                    // The real device exposes voltage as a
                                    // two-byte virtual register at 0x3E while
                                    // also exposing temperature at 0x3F. It is
                                    // therefore not a flat byte-addressed pair.
                                    if (addr == 8'h3E && len == 2) begin
                                        rbuf[0] = 8'h5C; // 11100 mV
                                        rbuf[1] = 8'h2B;
                                    end else begin
                                        for (i = 0; i < len; i = i + 1)
                                            rbuf[i] = mem[(addr + i) & 8'hFF];
                                    end
                                    rn = len;
                                    $display("  [model] READ  addr=%02h len=%0d", addr, len);
                                end

                                8'h03: begin                   // WRITE
                                    addr = pbuf[0];

                                    // Enforce the write-protect lock. A real
                                    // servo silently refuses NVS writes while
                                    // 0x37 is 1; if we tolerated them here,
                                    // a design that forgot to unlock would
                                    // pass in simulation and fail on the
                                    // bench, which is the worst outcome.
                                    if (addr < NVS_LIMIT && mem[8'h37] != 8'h00) begin
                                        nvs_violations = nvs_violations + 1;
                                        $display("  [model] NVS WRITE REFUSED addr=%02h -- lock 0x37 is still set",
                                                 addr);
                                    end else begin
                                        for (i = 1; i < n; i = i + 1)
                                            mem[(addr + i - 1) & 8'hFF] = pbuf[i];

                                        // The atomic motion block. A real servo
                                        // ramps; this one teleports, which is all
                                        // the protocol test needs.
                                        if (addr == 8'h2A) begin
                                            target_pos = {pbuf[2], pbuf[1]};
                                            mem[8'h38] = pbuf[1];
                                            mem[8'h39] = pbuf[2];
                                            $display("  [model] MOVE  target=%0d speed=%0d",
                                                     {pbuf[2], pbuf[1]}, {pbuf[6], pbuf[5]});
                                        end else
                                            $display("  [model] WRITE addr=%02h n=%0d", addr, n - 1);
                                    end

                                    // Mirror the registers the tests care about.
                                    ctrl_mode  = {mem[8'h22], mem[8'h21]};
                                    run_speed  = {mem[8'h2F], mem[8'h2E]};
                                    nvs_locked = (mem[8'h37] != 8'h00);
                                end

                                default:
                                    $display("  [model] unhandled instruction %02h", pinstr);
                            endcase

                            // Broadcast is never answered, and a disabled
                            // model is silent by construction.
                            if (pid != 8'hFE && enabled)
                                send_reply(8'h00);
                        end
                    end
                end
            end
        end
    end

endmodule
