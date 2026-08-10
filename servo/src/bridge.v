// Transparent USB-UART <-> servo-bus bridge. A debugging instrument.
//
// The dock's FT2232 channel B is a UART on B3/C3 (verified against Sipeed's
// own UART/simple_uart example). This design relays packets between that
// port at 115200 and the servo's single-wire bus at 1 Mbaud, so a script on
// the Mac can talk to the servo directly -- read registers back, try a
// command, see the actual error byte.
//
// Without this, every question about the servo has to be answered through
// one LED. With it, you get the bytes.
//
// Two things the bridge must do beyond copying bytes:
//
//  1. STORE AND FORWARD, not byte-by-byte. At 115200 the host's bytes are
//     ~87 us apart; blindly re-emitting them at 1 Mbaud would stretch a
//     packet across 1.1 ms of bus time with 77 us gaps mid-frame, and the
//     servo's receiver would time out partway through. So the bridge parses
//     the length field, collects the whole packet, and then sends it as one
//     contiguous burst.
//
//  2. GATE THE ECHO, exactly as hx_bus does -- the servo bus is one wire and
//     we hear ourselves on it.
//
// The reverse path needs a FIFO for the opposite reason: replies arrive at
// 1 Mbaud (10 us/byte) and leave at 115200 (87 us/byte).

module top #(
    parameter HOST_CLKS  = 434,      // 50 MHz / 115200 (0.007% error)
    parameter SERVO_CLKS = 50,       // 50 MHz / 1 Mbaud, exact
    parameter GUARD      = 100
)(
    input  wire clk,
    input  wire host_rx,             // B3  <- USB-UART
    output wire host_tx,             // C3  -> USB-UART
    output wire servo_tx,            // G5  -> 1k -> SIG
    input  wire servo_rx,            // F5  <- 1k <- SIG
    output wire led_done,
    output wire led_ready
);

    reg [3:0] por = 4'd0;
    wire      rst = (por != 4'hF);
    always @(posedge clk)
        if (por != 4'hF) por <= por + 1'b1;

    // ---------------------------------------------------------------- host in
    wire [7:0] h_data;
    wire       h_valid;

    uart_rx #(.CLKS_PER_BIT(HOST_CLKS)) u_hrx (
        .clk(clk), .rst(rst), .rx(host_rx), .enable(1'b1),
        .data(h_data), .valid(h_valid));

    // Collect a whole packet: FF FF ID LEN, then LEN more bytes.
    // Total length is therefore LEN + 4.
    localparam C_H1 = 2'd0, C_H2 = 2'd1, C_BODY = 2'd2, C_SEND = 2'd3;

    reg [1:0] cstate = C_H1;
    reg [7:0] pbuf [0:63];
    reg [6:0] pcnt = 0;              // bytes collected
    reg [6:0] ptot = 0;              // total expected
    reg [6:0] pidx = 0;              // transmit index

    reg [7:0] s_tx_data;
    reg       s_tx_start;
    wire      s_tx_busy;

    uart_tx #(.CLKS_PER_BIT(SERVO_CLKS)) u_stx (
        .clk(clk), .rst(rst), .start(s_tx_start), .data(s_tx_data),
        .tx(servo_tx), .busy(s_tx_busy));

    reg [$clog2(GUARD+1)-1:0] guard = 0;
    wire listening = (cstate != C_SEND) && !s_tx_busy && (guard == 0);

    always @(posedge clk) begin
        s_tx_start <= 1'b0;

        if (rst) begin
            cstate <= C_H1;
            pcnt   <= 0;
            guard  <= 0;
        end else begin
            if (s_tx_busy)      guard <= GUARD;
            else if (guard != 0) guard <= guard - 1'b1;

            case (cstate)
                C_H1: if (h_valid && h_data == 8'hFF) begin
                    pbuf[0] <= 8'hFF; pcnt <= 7'd1; cstate <= C_H2;
                end

                C_H2: if (h_valid) begin
                    if (h_data == 8'hFF) begin
                        pbuf[1] <= 8'hFF; pcnt <= 7'd2; cstate <= C_BODY;
                    end else
                        cstate <= C_H1;
                end

                C_BODY: if (h_valid) begin
                    pbuf[pcnt] <= h_data;
                    // The length byte is the 4th, and fixes the total.
                    if (pcnt == 7'd3)
                        ptot <= h_data + 7'd4;
                    if (pcnt >= 7'd3 && (pcnt + 7'd1) >= ptot && pcnt != 7'd3) begin
                        pidx   <= 0;
                        cstate <= C_SEND;
                    end else if (pcnt == 7'd63)
                        cstate <= C_H1;              // overlong: resynchronise
                    else
                        pcnt <= pcnt + 7'd1;
                end

                // Blast the collected packet back to back at 1 Mbaud.
                C_SEND: if (!s_tx_busy) begin
                    s_tx_data  <= pbuf[pidx];
                    s_tx_start <= 1'b1;
                    if (pidx + 7'd1 == ptot) begin
                        pcnt   <= 0;
                        cstate <= C_H1;
                    end else
                        pidx <= pidx + 7'd1;
                end

                default: cstate <= C_H1;
            endcase
        end
    end

    // --------------------------------------------------------------- servo in
    wire [7:0] s_data;
    wire       s_valid;

    uart_rx #(.CLKS_PER_BIT(SERVO_CLKS)) u_srx (
        .clk(clk), .rst(rst), .rx(servo_rx), .enable(listening),
        .data(s_data), .valid(s_valid));

    // 256-byte circular FIFO absorbs the 1 Mbaud -> 115200 rate step.
    reg [7:0] fifo [0:255];
    reg [7:0] wptr = 0, rptr = 0;
    wire fifo_empty = (wptr == rptr);

    reg [7:0] h_tx_data;
    reg       h_tx_start;
    wire      h_tx_busy;

    uart_tx #(.CLKS_PER_BIT(HOST_CLKS)) u_htx (
        .clk(clk), .rst(rst), .start(h_tx_start), .data(h_tx_data),
        .tx(host_tx), .busy(h_tx_busy));

    always @(posedge clk) begin
        h_tx_start <= 1'b0;
        if (rst) begin
            wptr <= 0;
            rptr <= 0;
        end else begin
            if (s_valid) begin
                fifo[wptr] <= s_data;
                wptr       <= wptr + 8'd1;
            end
            if (!fifo_empty && !h_tx_busy && !h_tx_start) begin
                h_tx_data  <= fifo[rptr];
                h_tx_start <= 1'b1;
                rptr       <= rptr + 8'd1;
            end
        end
    end

    // led_done flickers on servo traffic; led_ready is the heartbeat.
    reg [19:0] act = 0;
    always @(posedge clk)
        if (s_valid)      act <= 20'hFFFFF;
        else if (act != 0) act <= act - 1'b1;
    assign led_done = (act != 0);

    reg [24:0] hb = 0;
    always @(posedge clk) hb <= hb + 1'b1;
    assign led_ready = hb[24];

endmodule
