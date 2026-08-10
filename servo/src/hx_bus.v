// HX-30HM packet layer: frame out, frame in, and who owns the wire.
//
// Frame (both directions):
//
//   FF FF | ID | Length | Instr/Error | P1..PN | CheckSum
//
//   Length   = N + 2                         (N = parameter byte count)
//   CheckSum = ~(ID + Length + Instr + P1..PN) & 0xFF     -- headers excluded
//
// The checksum is accumulated as bytes go out, so there is no second pass
// over a buffer and no separate adder tree — one 8-bit register.
//
// BUS OWNERSHIP is the part worth reading carefully. The servo's DATA line
// is a single wire carrying both directions, so everything this module
// transmits is echoed back onto its own receive pin. The defence is that
// the receiver is only enabled in S_LISTEN, which is entered after the
// transmitter has fully drained AND a guard interval has passed. The echo
// is therefore not filtered out downstream, it is never sampled at all.
//
// Getting this wrong is not hypothetical: Hiwonder's own SDK syncs on the
// first FF FF after transmitting, which is the echo, and its PING parses
// that echo as a valid reply (checksum and all) with no servo attached.

module hx_bus #(
    parameter CLKS_PER_BIT = 50,        // 50 MHz / 1 Mbaud, exact
    parameter GUARD_CYCLES = 100,       // ~2 bit times after TX drains
    parameter RESP_TIMEOUT = 250_000,   // 5 ms; protocol asks for >= 1 ms
    parameter IFG_CYCLES   = 500        // 10 us of idle between exchanges
)(
    input  wire        clk,
    input  wire        rst,

    // --- command ---------------------------------------------------------
    input  wire        send,            // one-cycle pulse
    input  wire [7:0]  id,
    input  wire [7:0]  instr,
    input  wire [63:0] params,          // P1 in [7:0], P2 in [15:8], ...
    input  wire [3:0]  nparam,          // 0..8
    input  wire        expect_reply,    // 0 for broadcast (ID 0xFE)
    output reg         busy,

    // --- response (each output is a one-cycle pulse) ---------------------
    output reg         resp_valid,      // well-formed, checksum good
    output reg         resp_bad,        // checksum or length wrong
    output reg         resp_timeout,    // nothing arrived in time
    output reg  [7:0]  err_byte,        // servo status byte from the reply
    output reg  [63:0] resp_params,
    output reg  [3:0]  resp_nparam,

    // --- pins ------------------------------------------------------------
    output wire        tx_pin,
    input  wire        rx_pin
);

    // ---------------------------------------------------------------------
    // UART primitives
    // ---------------------------------------------------------------------
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst),
        .start(tx_start), .data(tx_data),
        .tx(tx_pin), .busy(tx_busy)
    );

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       listening;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst),
        .rx(rx_pin), .enable(listening),
        .data(rx_data), .valid(rx_valid)
    );

    // ---------------------------------------------------------------------
    // Transmit / sequencing FSM
    // ---------------------------------------------------------------------
    localparam S_IDLE   = 4'd0,
               S_H1     = 4'd1,
               S_H2     = 4'd2,
               S_ID     = 4'd3,
               S_LEN    = 4'd4,
               S_INSTR  = 4'd5,
               S_PARAM  = 4'd6,
               S_CS     = 4'd7,
               S_DRAIN  = 4'd8,
               S_GUARD  = 4'd9,
               S_LISTEN = 4'd10,
               S_IFG    = 4'd11;

    reg [3:0]  state = S_IDLE;
    reg [7:0]  csum;                    // running sum; inverted at the end
    reg [3:0]  pidx;
    reg [7:0]  hold_id, hold_instr;
    reg [63:0] hold_params;
    reg [3:0]  hold_nparam;
    reg        hold_expect;

    reg [$clog2(GUARD_CYCLES+1)-1:0] guard;
    reg [$clog2(RESP_TIMEOUT+1)-1:0] tmo;
    reg [$clog2(IFG_CYCLES+1)-1:0]   ifg;

    assign listening = (state == S_LISTEN);

    // The receive parser (below) reports completion through this.
    wire parse_done;

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            tx_start     <= 1'b0;
            resp_timeout <= 1'b0;
        end else begin
            tx_start     <= 1'b0;       // defaults: single-cycle pulses
            resp_timeout <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (send) begin
                        hold_id     <= id;
                        hold_instr  <= instr;
                        hold_params <= params;
                        hold_nparam <= nparam;
                        hold_expect <= expect_reply;
                        csum        <= 8'h00;
                        pidx        <= 4'd0;
                        busy        <= 1'b1;
                        state       <= S_H1;
                    end
                end

                // The two header bytes are NOT part of the checksum.
                S_H1: if (!tx_busy) begin
                    tx_data  <= 8'hFF;
                    tx_start <= 1'b1;
                    state    <= S_H2;
                end

                S_H2: if (!tx_busy) begin
                    tx_data  <= 8'hFF;
                    tx_start <= 1'b1;
                    state    <= S_ID;
                end

                S_ID: if (!tx_busy) begin
                    tx_data  <= hold_id;
                    tx_start <= 1'b1;
                    csum     <= csum + hold_id;
                    state    <= S_LEN;
                end

                S_LEN: if (!tx_busy) begin
                    tx_data  <= {4'd0, hold_nparam} + 8'd2;   // Length = N + 2
                    tx_start <= 1'b1;
                    csum     <= csum + {4'd0, hold_nparam} + 8'd2;
                    state    <= S_INSTR;
                end

                S_INSTR: if (!tx_busy) begin
                    tx_data  <= hold_instr;
                    tx_start <= 1'b1;
                    csum     <= csum + hold_instr;
                    state    <= (hold_nparam == 4'd0) ? S_CS : S_PARAM;
                end

                S_PARAM: if (!tx_busy) begin
                    tx_data  <= hold_params[pidx*8 +: 8];
                    tx_start <= 1'b1;
                    csum     <= csum + hold_params[pidx*8 +: 8];
                    pidx     <= pidx + 1'b1;
                    if (pidx == hold_nparam - 1'b1)
                        state <= S_CS;
                end

                S_CS: if (!tx_busy) begin
                    tx_data  <= ~csum;
                    tx_start <= 1'b1;
                    state    <= S_DRAIN;
                end

                // Let the final stop bit actually leave the pin before
                // considering the bus released.
                S_DRAIN: if (!tx_busy) begin
                    guard <= GUARD_CYCLES;
                    state <= S_GUARD;
                end

                S_GUARD: begin
                    if (guard == 0) begin
                        if (hold_expect) begin
                            tmo   <= RESP_TIMEOUT;
                            state <= S_LISTEN;
                        end else begin
                            ifg   <= IFG_CYCLES;   // broadcast: never answered
                            state <= S_IFG;
                        end
                    end else
                        guard <= guard - 1'b1;
                end

                S_LISTEN: begin
                    if (parse_done) begin
                        ifg   <= IFG_CYCLES;
                        state <= S_IFG;
                    end else if (tmo == 0) begin
                        resp_timeout <= 1'b1;
                        ifg          <= IFG_CYCLES;
                        state        <= S_IFG;
                    end else
                        tmo <= tmo - 1'b1;
                end

                // Inter-frame gap. We consider a reply complete at the MIDDLE
                // of its final stop bit, which is half a bit-time before the
                // servo has actually finished driving it -- and the servo then
                // needs to re-arm its own receiver. Transmitting immediately
                // means the next packet's start bit lands while the far end is
                // not listening, and it vanishes. Staying busy here is what
                // makes back-to-back commands reliable.
                S_IFG: begin
                    if (ifg == 0) begin
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end else
                        ifg <= ifg - 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Receive parser
    // ---------------------------------------------------------------------
    localparam R_H1    = 3'd0,
               R_H2    = 3'd1,
               R_ID    = 3'd2,
               R_LEN   = 3'd3,
               R_ERR   = 3'd4,
               R_PARAM = 3'd5,
               R_CS    = 3'd6;

    reg [2:0] rstate = R_H1;
    reg [7:0] racc;                     // checksum accumulator
    reg [7:0] rlen;
    reg [3:0] rpidx;
    reg [3:0] rn;                       // parameter count = rlen - 2
    reg       rlen_bad;                 // reply claims more params than we can hold

    reg       done_pulse;
    assign parse_done = done_pulse;

    always @(posedge clk) begin
        if (rst) begin
            rstate     <= R_H1;
            done_pulse <= 1'b0;
            resp_valid <= 1'b0;
            resp_bad   <= 1'b0;
        end else begin
            done_pulse <= 1'b0;
            resp_valid <= 1'b0;
            resp_bad   <= 1'b0;

            if (!listening) begin
                rstate <= R_H1;         // re-arm for the next exchange
            end else if (rx_valid) begin
                case (rstate)
                    // Hunt for FF FF. Staying in R_H2 on a second FF matters:
                    // in FF FF FF the last two bytes are still a valid header.
                    R_H1: if (rx_data == 8'hFF) rstate <= R_H2;

                    R_H2: if (rx_data == 8'hFF) rstate <= R_ID;
                          else                  rstate <= R_H1;

                    R_ID: begin
                        racc   <= rx_data;
                        rstate <= R_LEN;
                    end

                    R_LEN: begin
                        racc     <= racc + rx_data;
                        rlen     <= rx_data;
                        rlen_bad <= (rx_data < 8'd2) || (rx_data > 8'd10);
                        rstate   <= R_ERR;
                    end

                    R_ERR: begin
                        racc        <= racc + rx_data;
                        err_byte    <= rx_data;
                        rn          <= rlen[3:0] - 4'd2;
                        resp_nparam <= rlen[3:0] - 4'd2;
                        rpidx       <= 4'd0;
                        rstate      <= (rlen == 8'd2) ? R_CS : R_PARAM;
                    end

                    R_PARAM: begin
                        racc  <= racc + rx_data;
                        if (!rlen_bad)
                            resp_params[rpidx*8 +: 8] <= rx_data;
                        rpidx <= rpidx + 1'b1;
                        if (rpidx == rn - 1'b1)
                            rstate <= R_CS;
                    end

                    R_CS: begin
                        done_pulse <= 1'b1;
                        if ((rx_data == ~racc) && !rlen_bad)
                            resp_valid <= 1'b1;
                        else
                            resp_bad <= 1'b1;
                        rstate <= R_H1;
                    end

                    default: rstate <= R_H1;
                endcase
            end
        end
    end

endmodule
