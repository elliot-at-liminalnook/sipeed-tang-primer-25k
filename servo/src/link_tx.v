// FPGA -> host/ESP32 link frame builder. Mirror of link_rx.
//
//   AA 55 | LEN | SEQ | OP | payload[LEN] | CRC8
//
// Used for telemetry: with a policy in the loop the observation vector is on
// the critical path every tick, so this is not a debug afterthought.
//
// The payload is read from an external buffer through pay_addr/pay_data with
// one cycle of latency, which is free here -- each byte takes ten bit-times
// to leave the UART, so there is no shortage of cycles to fetch the next one.

module link_tx (
    input  wire       clk,
    input  wire       rst,

    input  wire       send,          // one-cycle pulse
    input  wire [7:0] op,
    input  wire [7:0] plen,
    input  wire [7:0] seq,
    output reg        busy,

    // payload fetch (registered: address out, data back next cycle)
    output reg [5:0]  pay_addr,
    input  wire [7:0] pay_data,

    // to a uart_tx
    output reg [7:0]  tx_data,
    output reg        tx_start,
    input  wire       tx_busy
);

    `include "crc8.v"

    localparam S_IDLE = 4'd0,
               S_AA   = 4'd1,
               S_55   = 4'd2,
               S_LEN  = 4'd3,
               S_SEQ  = 4'd4,
               S_OP   = 4'd5,
               S_ADDR = 4'd6,
               S_PAY  = 4'd7,
               S_CRC  = 4'd8;

    reg [3:0] state = S_IDLE;
    reg [7:0] crc;
    reg [6:0] idx;
    reg [7:0] hold_op, hold_len, hold_seq;

    always @(posedge clk) begin
        tx_start <= 1'b0;

        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
        end else case (state)

            S_IDLE: begin
                busy <= 1'b0;
                if (send) begin
                    hold_op  <= op;
                    hold_len <= plen;
                    hold_seq <= seq;
                    crc      <= 8'h00;
                    idx      <= 7'd0;
                    busy     <= 1'b1;
                    state    <= S_AA;
                end
            end

            // Sync bytes are outside the CRC -- they carry no information.
            S_AA: if (!tx_busy) begin
                tx_data <= 8'hAA; tx_start <= 1'b1; state <= S_55;
            end

            S_55: if (!tx_busy) begin
                tx_data <= 8'h55; tx_start <= 1'b1; state <= S_LEN;
            end

            S_LEN: if (!tx_busy) begin
                tx_data  <= hold_len;
                tx_start <= 1'b1;
                crc      <= crc8_byte(8'h00, hold_len);
                state    <= S_SEQ;
            end

            S_SEQ: if (!tx_busy) begin
                tx_data  <= hold_seq;
                tx_start <= 1'b1;
                crc      <= crc8_byte(crc, hold_seq);
                state    <= S_OP;
            end

            S_OP: if (!tx_busy) begin
                tx_data  <= hold_op;
                tx_start <= 1'b1;
                crc      <= crc8_byte(crc, hold_op);
                pay_addr <= 6'd0;
                state    <= (hold_len == 8'd0) ? S_CRC : S_ADDR;
            end

            // One cycle for the payload buffer to present pay_data.
            S_ADDR: state <= S_PAY;

            S_PAY: if (!tx_busy) begin
                tx_data  <= pay_data;
                tx_start <= 1'b1;
                crc      <= crc8_byte(crc, pay_data);
                idx      <= idx + 7'd1;
                pay_addr <= idx[5:0] + 6'd1;
                state    <= (idx + 7'd1 == hold_len[6:0]) ? S_CRC : S_ADDR;
            end

            S_CRC: if (!tx_busy) begin
                tx_data  <= crc;
                tx_start <= 1'b1;
                state    <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end

endmodule
