// 8N1 UART transmitter.
//
// CLKS_PER_BIT is the whole story: hold each bit on the wire for that many
// clock edges. At 50 MHz and 1 Mbaud that is exactly 50 — no fractional
// accumulator, no accumulated phase error. The HX-30HM's fastest mode is
// also the one this board hits most precisely.
//
// `busy` is deliberately combinational in `start`, so a caller can pulse
// start for one cycle and then simply wait for `busy` to fall. Without
// that, the caller has to guess how long it takes for busy to rise and
// races the transmitter on the very first byte.
//
// CONTRACT: `start` is latched ONLY in S_IDLE. Asserting it while `busy`
// is high does nothing at all -- the byte is silently dropped, not queued.
// Callers must gate on !busy. Note in particular that a receiver's `valid`
// is NOT a safe "ready for the next byte" signal: it fires from the middle
// of the stop bit, half a bit-time before the transmitter is done.

module uart_tx #(
    parameter CLKS_PER_BIT = 50
)(
    input  wire       clk,
    input  wire       rst,        // synchronous, active high
    input  wire       start,      // one-cycle pulse: latch `data` and send
    input  wire [7:0] data,
    output reg        tx,         // idles HIGH — this is what releases the bus
    output wire       busy
);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]                    state = S_IDLE;
    reg [$clog2(CLKS_PER_BIT)-1:0] tick = 0;
    reg [2:0]                    bit_idx = 0;
    reg [7:0]                    shifter = 8'h00;

    // Combinational in `start` — see header comment.
    assign busy = (state != S_IDLE) | start;

    wire tick_done = (tick == CLKS_PER_BIT - 1);

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            tx      <= 1'b1;
            tick    <= 0;
            bit_idx <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx   <= 1'b1;          // release the bus
                    tick <= 0;
                    if (start) begin
                        shifter <= data;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;            // start bit
                    if (tick_done) begin
                        tick    <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else
                        tick <= tick + 1'b1;
                end

                S_DATA: begin
                    tx <= shifter[0];      // LSB first
                    if (tick_done) begin
                        tick    <= 0;
                        shifter <= {1'b0, shifter[7:1]};
                        if (bit_idx == 3'd7)
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 1'b1;
                    end else
                        tick <= tick + 1'b1;
                end

                S_STOP: begin
                    tx <= 1'b1;            // stop bit
                    if (tick_done) begin
                        tick  <= 0;
                        state <= S_IDLE;
                    end else
                        tick <= tick + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
