// FPGA-side endpoint of the host link. This is the piece the ESP32 talks to.
//
//   host UART <-> link_rx / link_tx <-> targets + control tick + deadman
//
// What lives here and why: the control tick, the deadman and (later) the
// safety clamps are all things that must hold regardless of what the policy
// or the network does. The FPGA is the only layer in the chain that cannot
// be taken out by a garbage collector, a WiFi dropout or a reboot, so it is
// where the promises belong.
//
// Telemetry is emitted on a fixed tick rather than in reply to each command.
// With a policy in the loop the observation is the network's INPUT, so it
// needs to arrive at a constant rate with constant phase -- a policy trained
// on a fixed timestep degrades in confusing ways when the timestep wobbles.
//
// The servo bus is not wired in yet: OBS currently reports the latched
// targets, which makes the whole host path testable end to end before any
// hardware exists. Swapping in real reads from hx_bus changes only where
// obs_pos comes from.

module link_core #(
    parameter CLKS_PER_BIT = 25,          // 50 MHz / 2 Mbaud, exact
    parameter TICK_DIV     = 1_000_000,   // 50 Hz at 50 MHz
    parameter DEADMAN_CYC  = 10_000_000,  // 200 ms of silence
    parameter LINK_TIMEOUT = 50_000       // 1 ms mid-frame stall
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        host_rx,
    output wire        host_tx,

    // observable state (exposed for simulation and for LEDs)
    output reg [15:0]  target0,
    output reg [15:0]  target1,
    output reg [15:0]  target2,
    output reg         deadman,           // 1 = link considered dead
    output reg [31:0]  tick_count,
    output wire [15:0] crc_errors,
    output wire [15:0] timeouts,
    output reg [7:0]   last_seq
);

    localparam OP_SET_TARGETS = 8'h01,
               OP_STOP        = 8'h04,
               OP_KEEPALIVE   = 8'h7F,
               OP_OBS         = 8'h81;

    // ---------------------------------------------------------------- uarts
    wire [7:0] urx_data;
    wire       urx_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rx(host_rx), .enable(1'b1),
        .data(urx_data), .valid(urx_valid));

    wire [7:0] utx_data;
    wire       utx_start, utx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst), .start(utx_start), .data(utx_data),
        .tx(host_tx), .busy(utx_busy));

    // ------------------------------------------------------------- link in
    wire       frame_valid;
    wire [7:0] rseq, rop, rplen;
    wire [7:0] rpay_data;
    wire [5:0] rpay_addr;
    wire       rpay_we;

    link_rx #(.IDLE_TIMEOUT(LINK_TIMEOUT)) u_lrx (
        .clk(clk), .rst(rst),
        .rx_data(urx_data), .rx_valid(urx_valid),
        .frame_valid(frame_valid), .seq(rseq), .op(rop), .plen(rplen),
        .crc_errors(crc_errors), .timeouts(timeouts),
        .pay_data(rpay_data), .pay_addr(rpay_addr), .pay_we(rpay_we));

    // Payload lands here as it arrives; only acted on once the CRC passes.
    //
    // ram_style="registers": left to itself yosys infers LUT-RAM (RAM16SDP4)
    // and place-and-route then fails -- the GW5A has few of those BELs and
    // two 64-byte buffers exhaust them. These are shallow and read
    // asynchronously by the framer, so flip-flops are the right fit anyway.
    (* ram_style = "registers" *)
    reg [7:0] rxbuf [0:63];
    always @(posedge clk)
        if (rpay_we) rxbuf[rpay_addr] <= rpay_data;

    // ------------------------------------------------------- command apply
    // Payload layout for SET_TARGETS: n, then (id, pos_lo, pos_hi) x n.
    reg [$clog2(DEADMAN_CYC+1)-1:0] dm_cnt;
    always @(posedge clk) begin
        if (rst) begin
            target0 <= 16'd2048;      // mid-travel is the safe idle pose
            target1 <= 16'd2048;
            target2 <= 16'd2048;
            deadman <= 1'b0;
            dm_cnt  <= 0;
            last_seq<= 8'd0;
        end else begin
            // Any valid frame is proof of life, including a bare keepalive.
            if (frame_valid) begin
                dm_cnt   <= 0;
                deadman  <= 1'b0;
                last_seq <= rseq;

                case (rop)
                    // Check the frame is long enough for the count it
                    // claims before trusting any of it. A CRC-valid frame can
                    // still be semantically malformed.
                    OP_SET_TARGETS: if (rplen >= (8'd1 + 8'd3 * rxbuf[0])) begin
                        // rxbuf[0] = count; entries follow, 3 bytes each.
                        if (rxbuf[0] >= 8'd1)
                            target0 <= {rxbuf[3],  rxbuf[2]};
                        if (rxbuf[0] >= 8'd2)
                            target1 <= {rxbuf[6],  rxbuf[5]};
                        if (rxbuf[0] >= 8'd3)
                            target2 <= {rxbuf[9],  rxbuf[8]};
                    end

                    OP_STOP: begin
                        // Deliberately does NOT zero the targets: on an
                        // articulated machine, commanding zero is a fall.
                        // Holding the current pose is the safe stop.
                        deadman <= 1'b1;
                    end

                    // A keepalive carries nothing; its whole purpose is the
                    // deadman reset above. Listed explicitly so the intent is
                    // not mistaken for an omission.
                    OP_KEEPALIVE: ;

                    default: ;   // unknown opcodes: proof of life only
                endcase

            end else if (dm_cnt == DEADMAN_CYC - 1) begin
                deadman <= 1'b1;          // targets are held, not released
            end else
                dm_cnt <= dm_cnt + 1'b1;
        end
    end

    // ------------------------------------------------------- control tick
    reg [$clog2(TICK_DIV+1)-1:0] tick_cnt;
    reg                          tick;

    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst) begin
            tick_cnt   <= 0;
            tick_count <= 32'd0;
        end else if (tick_cnt == TICK_DIV - 1) begin
            tick_cnt   <= 0;
            tick       <= 1'b1;
            tick_count <= tick_count + 32'd1;
        end else
            tick_cnt <= tick_cnt + 1'b1;
    end

    // ------------------------------------------------------------ link out
    // OBS payload: tick32 (LE) then three positions (LE). Ten bytes.
    (* ram_style = "registers" *)
    reg [7:0] txbuf [0:63];
    wire [5:0] tpay_addr;
    reg  [7:0] tpay_data;
    always @(posedge clk) tpay_data <= txbuf[tpay_addr];

    reg       ltx_send;
    reg [7:0] tx_seq;
    wire      ltx_busy;

    link_tx u_ltx (
        .clk(clk), .rst(rst),
        .send(ltx_send), .op(OP_OBS), .plen(8'd10), .seq(tx_seq),
        .busy(ltx_busy),
        .pay_addr(tpay_addr), .pay_data(tpay_data),
        .tx_data(utx_data), .tx_start(utx_start), .tx_busy(utx_busy));

    always @(posedge clk) begin
        ltx_send <= 1'b0;
        if (rst) begin
            tx_seq <= 8'd0;
        end else if (tick && !ltx_busy) begin
            txbuf[0] <= tick_count[7:0];
            txbuf[1] <= tick_count[15:8];
            txbuf[2] <= tick_count[23:16];
            txbuf[3] <= tick_count[31:24];
            txbuf[4] <= target0[7:0];
            txbuf[5] <= target0[15:8];
            txbuf[6] <= target1[7:0];
            txbuf[7] <= target1[15:8];
            txbuf[8] <= target2[7:0];
            txbuf[9] <= target2[15:8];
            tx_seq   <= tx_seq + 8'd1;
            ltx_send <= 1'b1;
        end
    end

endmodule
