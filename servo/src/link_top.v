// Synthesizable top for the host link, tethered phase.
//
// host_rx/host_tx sit on the dock's FT2232 UART (B3/C3), so the Mac drives
// the FPGA directly with no radio in the path. The ESP32 speaks the same
// framing on the same pins later -- develop tethered, deploy wireless, one
// variable at a time.
//
// The servo bus is not connected here yet. link_core reports its latched
// targets back as telemetry, which makes the whole host path -- framing,
// CRC, control tick, deadman -- testable on real silicon before any servo
// is involved. That ordering means a failure at the next step can only be
// the servo bus.

module top #(
    parameter CLKS_PER_BIT = 25,          // 50 MHz / 2 Mbaud, exact
    parameter TICK_DIV     = 1_000_000,   // 50 Hz control tick
    parameter DEADMAN_CYC  = 10_000_000   // 200 ms of silence
)(
    input  wire clk,          // 50 MHz (E2)
    input  wire host_rx,      // B3  <- FT2232 / ESP32
    output wire host_tx,      // C3  -> FT2232 / ESP32
    output wire led_done,     // link alive
    output wire led_ready     // heartbeat; fast while the deadman is tripped
);

    reg [3:0] por = 4'd0;
    wire      rst = (por != 4'hF);
    always @(posedge clk)
        if (por != 4'hF) por <= por + 1'b1;

    wire [15:0] t0, t1, t2;
    wire        deadman;
    wire [31:0] tick_count;
    wire [15:0] crc_errors, timeouts;
    wire [7:0]  last_seq;

    link_core #(
        .CLKS_PER_BIT(CLKS_PER_BIT),
        .TICK_DIV(TICK_DIV),
        .DEADMAN_CYC(DEADMAN_CYC)
    ) u_core (
        .clk(clk), .rst(rst),
        .host_rx(host_rx), .host_tx(host_tx),
        .target0(t0), .target1(t1), .target2(t2),
        .deadman(deadman), .tick_count(tick_count),
        .crc_errors(crc_errors), .timeouts(timeouts),
        .last_seq(last_seq)
    );

    // led_done: lit while the link is considered alive. This is the signal
    // you actually want on a bench -- "is anything talking to me" answered
    // without a terminal.
    assign led_done = ~deadman;

    // Heartbeat runs 4x faster once the deadman has tripped, so a dead link
    // is distinguishable from a dead bitstream across the room.
    reg [24:0] hb = 25'd0;
    always @(posedge clk) hb <= hb + 1'b1;
    assign led_ready = deadman ? hb[22] : hb[24];

endmodule
