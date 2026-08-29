// Host/ESP32 -> FPGA link frame parser.
//
//   AA 55 | LEN | SEQ | OP | payload[LEN] | CRC8
//
// CRC covers LEN, SEQ, OP and the payload -- not the sync bytes, which carry
// no information and would only weaken the check.
//
// The sync word is AA 55 rather than the servo bus's FF FF on purpose: if the
// two links ever get cross-wired, the failure is total and obvious instead of
// partial and confusing. AA 55 is also a poor imitation of line noise, since
// it alternates.
//
// SEQ is passed up rather than acted on here. UDP gives no ordering or
// delivery guarantee, so something upstairs has to decide whether a frame is
// stale -- but that is policy, and this module is plumbing.
//
// A frame that fails CRC is counted and dropped. It is never partially
// applied: `valid` pulses only after the checksum has been verified, so a
// corrupted position command cannot reach a servo.

module link_rx #(
    parameter MAX_PAYLOAD  = 64,
    // Mid-frame silence after which we give up and hunt for a new header.
    // 1 ms at 50 MHz. Without this a truncated frame parks the parser in
    // S_PAY forever, and it then consumes the NEXT good frame as payload --
    // one lost byte would desynchronise the link permanently.
    parameter IDLE_TIMEOUT = 50_000
)(
    input  wire       clk,
    input  wire       rst,

    // byte stream from a uart_rx
    input  wire [7:0] rx_data,
    input  wire       rx_valid,

    // one-cycle pulse, CRC already checked
    output reg        frame_valid,
    output reg [7:0]  seq,
    output reg [7:0]  op,
    output reg [7:0]  plen,
    output reg [15:0] crc_errors,     // saturating; a health signal for the host
    output reg [15:0] timeouts,       // frames abandoned mid-way

    // payload byte fan-out, written as bytes arrive
    output reg [7:0]  pay_data,
    output reg [5:0]  pay_addr,
    output reg        pay_we
);

    `include "crc8.v"

    localparam S_A5   = 3'd0,   // hunt for AA
               S_55   = 3'd1,
               S_LEN  = 3'd2,
               S_SEQ  = 3'd3,
               S_OP   = 3'd4,
               S_PAY  = 3'd5,
               S_CRC  = 3'd6;

    reg [2:0] state = S_A5;
    reg [7:0] crc;
    reg [6:0] cnt;
    reg [$clog2(IDLE_TIMEOUT+1)-1:0] idle_cnt = 0;

    always @(posedge clk) begin
        frame_valid <= 1'b0;
        pay_we      <= 1'b0;

        if (rst) begin
            state      <= S_A5;
            crc_errors <= 16'd0;
            timeouts   <= 16'd0;
            idle_cnt   <= 0;
        end else if (rx_valid) begin
            idle_cnt <= 0;
            case (state)
                S_A5: if (rx_data == 8'hAA) state <= S_55;

                // A second AA is still a plausible frame start, so stay put
                // rather than falling back to S_A5 and losing it.
                S_55: if (rx_data == 8'h55)      state <= S_LEN;
                      else if (rx_data == 8'hAA) state <= S_55;
                      else                       state <= S_A5;

                S_LEN: begin
                    plen  <= rx_data;
                    crc   <= crc8_byte(8'h00, rx_data);
                    cnt   <= 7'd0;
                    // Reject an overlong frame here rather than overrunning
                    // the payload buffer partway through.
                    state <= (rx_data > MAX_PAYLOAD) ? S_A5 : S_SEQ;
                end

                S_SEQ: begin
                    seq   <= rx_data;
                    crc   <= crc8_byte(crc, rx_data);
                    state <= S_OP;
                end

                S_OP: begin
                    op    <= rx_data;
                    crc   <= crc8_byte(crc, rx_data);
                    state <= (plen == 8'd0) ? S_CRC : S_PAY;
                end

                S_PAY: begin
                    pay_data <= rx_data;
                    pay_addr <= cnt[5:0];
                    pay_we   <= 1'b1;
                    crc      <= crc8_byte(crc, rx_data);
                    cnt      <= cnt + 7'd1;
                    if (cnt + 7'd1 == plen[6:0]) state <= S_CRC;
                end

                S_CRC: begin
                    if (rx_data == crc)
                        frame_valid <= 1'b1;
                    else if (crc_errors != 16'hFFFF)
                        crc_errors <= crc_errors + 16'd1;
                    state <= S_A5;
                end

                default: state <= S_A5;
            endcase

        // Silence part-way through a frame: abandon it and resynchronise.
        end else if (state != S_A5) begin
            if (idle_cnt == IDLE_TIMEOUT - 1) begin
                state    <= S_A5;
                idle_cnt <= 0;
                if (timeouts != 16'hFFFF) timeouts <= timeouts + 16'd1;
            end else
                idle_cnt <= idle_cnt + 1'b1;
        end
    end

endmodule
