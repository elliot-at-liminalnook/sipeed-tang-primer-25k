// 8N1 UART receiver with a mid-bit sampling point and a gate.
//
// Two things here that the transmitter does not need:
//
//  1. A SYNCHRONISER. `rx` arrives from outside this clock domain; it can
//     change at any moment relative to our edges. Same two-flip-flop cure
//     as the button in the motor lesson, same reason.
//
//  2. The GATE. On a single-wire half-duplex bus our own transmission is
//     echoed straight back onto this pin. Holding `enable` low across our
//     own burst is what stops the parser latching onto it. Hiwonder's own
//     SDK omits this and, as a result, its ping() succeeds against its own
//     echo with no servo attached.
//
// Sampling is at CLKS_PER_BIT/2 into each bit — the point furthest from
// both edges, so it is the most tolerant of baud mismatch and slew.

module uart_rx #(
    parameter CLKS_PER_BIT = 50
)(
    input  wire       clk,
    input  wire       rst,        // synchronous, active high
    input  wire       rx,         // raw pin, asynchronous
    input  wire       enable,     // 0 = ignore the line entirely (echo gate)
    output reg  [7:0] data,
    output reg        valid       // one-cycle pulse when `data` is good
);

    localparam HALF_BIT = CLKS_PER_BIT / 2;

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    // --- synchroniser -----------------------------------------------------
    reg [1:0] sync = 2'b11;         // idle high, so reset does not look like a start bit
    always @(posedge clk)
        if (rst) sync <= 2'b11;
        else     sync <= {sync[0], rx};

    wire rx_s = sync[1];

    reg [1:0]                      state = S_IDLE;
    reg [$clog2(CLKS_PER_BIT)-1:0] tick  = 0;
    reg [2:0]                      bit_idx = 0;
    reg [7:0]                      shifter = 8'h00;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            valid <= 1'b0;
            tick  <= 0;
            data  <= 8'h00;
        end else if (!enable) begin
            // Gated: abandon anything in flight and emit nothing. This is the
            // whole echo defence, and it is one line because in hardware we
            // control the receiver directly instead of flushing a buffer.
            state <= S_IDLE;
            valid <= 1'b0;
            tick  <= 0;
        end else begin
            valid <= 1'b0;              // default: single-cycle pulse

            case (state)
                S_IDLE: begin
                    tick <= 0;
                    if (rx_s == 1'b0)   // falling edge = start bit
                        state <= S_START;
                end

                // Wait to the MIDDLE of the start bit and re-check. A glitch
                // shorter than half a bit is rejected here rather than
                // shifting the whole byte out of alignment.
                S_START: begin
                    if (tick == HALF_BIT - 1) begin
                        tick <= 0;
                        if (rx_s == 1'b0) begin
                            bit_idx <= 0;
                            state   <= S_DATA;
                        end else
                            state <= S_IDLE;   // false start
                    end else
                        tick <= tick + 1'b1;
                end

                S_DATA: begin
                    if (tick == CLKS_PER_BIT - 1) begin
                        tick    <= 0;
                        shifter <= {rx_s, shifter[7:1]};   // LSB first
                        if (bit_idx == 3'd7)
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 1'b1;
                    end else
                        tick <= tick + 1'b1;
                end

                S_STOP: begin
                    if (tick == CLKS_PER_BIT - 1) begin
                        tick  <= 0;
                        state <= S_IDLE;
                        // A framing error (stop bit low) drops the byte. The
                        // packet layer's checksum would catch it anyway, but
                        // failing early keeps the parser from resynchronising
                        // mid-packet.
                        if (rx_s == 1'b1) begin
                            data  <= shifter;
                            valid <= 1'b1;
                        end
                    end else
                        tick <= tick + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
