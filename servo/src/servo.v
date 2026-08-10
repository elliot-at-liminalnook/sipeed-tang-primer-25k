// Lesson 6: talking to an HX-30HM bus servo, Tang Primer 25K (GW5A-25)
//
// S1 (K6) steps through position commands:
//   0  PING                      is anyone there?
//   1  move to 1024  (~90 deg)
//   2  move to 2048  (~180 deg)
//   3  move to 3072  (~270 deg)
//   4  read position (0x38)      what does the encoder actually say?
//
// S2 toggles SPIN -- continuous rotation at the servo's maximum speed.
//
// led_done  = the last exchange returned a valid packet with error byte 0
// led_ready = heartbeat; SLOW when idle, FAST while spinning
//
// -----------------------------------------------------------------------
// Why spin needs four packets, not one
//
// Continuous rotation means leaving position-servo mode for CONSTANT-SPEED
// mode, which is control register 0x21. That register lives in the NVS/ROM
// area, and the servo refuses NVS writes unless the write-protect lock at
// 0x37 has first been cleared to 0. So:
//
//   SPIN ON   0x37 <- 0      unlock NVS
//             0x21 <- 1      constant-speed mode
//             0x37 <- 1      re-lock, so a stray write cannot corrupt NVS
//             0x2E <- 32767  maximum speed (plain SRAM, no unlock needed)
//
//   SPIN OFF  0x2E <- 0      stop FIRST, while still in speed mode
//             0x37 <- 0
//             0x21 <- 0      back to position-servo mode
//             0x37 <- 1
//
// Order matters on the way out: stopping before changing mode means the
// servo is never sitting at full speed while its control mode changes
// underneath it.
//
// 0x21 is NVS, so it PERSISTS across power cycles. That is why spin-off
// restores position mode rather than relying on a power cycle to do it --
// otherwise the servo would silently come back up in speed mode and ignore
// every position command in the S1 sequence.
//
// The macro sequencer aborts if any step fails, so a servo that stops
// answering cannot leave us blindly writing the rest of a sequence.

module top #(
    parameter CLKS_PER_BIT = 50,         // 50 MHz / 1 Mbaud, exact
    parameter DEBOUNCE_CYC = 1_000_000,  // 20 ms of required stability
    parameter RESP_TIMEOUT = 250_000     // 5 ms
)(
    input  wire clk,          // 50 MHz (E2)
    input  wire key,          // dock button S1 (H11) -- position sequence
    input  wire key2,         // dock button S2 (H10) -- toggle spin
    output wire servo_tx,     // -> 1k -> servo SIG
    input  wire servo_rx,     // <- 1k <- servo SIG
    output wire led_done,     // last exchange OK
    output wire led_ready     // heartbeat (fast while spinning)
);

    // --- power-on reset ---------------------------------------------------
    reg [3:0] por = 4'd0;
    wire      rst = (por != 4'hF);
    always @(posedge clk)
        if (por != 4'hF) por <= por + 1'b1;

    // --- buttons ----------------------------------------------------------
    wire key1_pressed, key2_pressed;
    wire key1_state, key2_state;

    button #(.DEBOUNCE_CYC(DEBOUNCE_CYC)) u_b1 (
        .clk(clk), .rst(rst), .raw(key),
        .state(key1_state), .pressed(key1_pressed));

    button #(.DEBOUNCE_CYC(DEBOUNCE_CYC)) u_b2 (
        .clk(clk), .rst(rst), .raw(key2),
        .state(key2_state), .pressed(key2_pressed));

    // --- bus signals ------------------------------------------------------
    // Declared ahead of first use: a wire used before its declaration
    // elaborates or not depending on source-file order.
    wire        busy;
    wire        resp_valid, resp_bad, resp_timeout;
    wire [7:0]  err_byte;
    wire [63:0] resp_params;
    wire [3:0]  resp_nparam;

    // --- registers and instructions ---------------------------------------
    localparam SERVO_ID = 8'd1;          // servos ship as ID 1

    localparam INSTR_PING  = 8'h01,
               INSTR_READ  = 8'h02,
               INSTR_WRITE = 8'h03;

    localparam REG_CTRL_MODE   = 8'h21,  // NVS: 0 = position, 1 = constant speed
               REG_TARGET_POS  = 8'h2A,  // atomic 6-byte block: pos/time/speed
               REG_RUN_SPEED   = 8'h2E,
               REG_NVS_LOCK    = 8'h37,  // 0 = unlocked, 1 = locked
               REG_CURRENT_POS = 8'h38;

    localparam [15:0] MOVE_SPEED = 16'd500;    // steps/s for position moves

    // Speed is SIGN-MAGNITUDE with BIT15 as direction, so the largest
    // forward value is 0x7FFF. This is a request, not a promise -- and the
    // servo clamps it hard. Measured on the real part at 11.1 V:
    //
    //     commanded    500 -> 450 steps/s
    //     commanded   3000 -> 2800 steps/s
    //     commanded  32767 -> 2850 steps/s      <- the ceiling
    //
    // Open-loop PWM mode at full duty reaches the same 2850, so ~2850 is a
    // MECHANICAL limit, not something the speed loop is holding back. That
    // is 2850/4096 rev/s, about 42 RPM: this is a 30 kg.cm geared servo, and
    // gearing that hard costs top speed.
    //
    // Worth knowing when it looks broken: spinning with no load draws only
    // ~0.1 A total, most of it the electronics. A low current reading is not
    // evidence that it is stalled.
    localparam [15:0] MAX_SPEED = 16'h7FFF;

    // --- macro sequencer --------------------------------------------------
    localparam MAC_SEQ      = 2'd0,      // one packet, selected by `cmd`
               MAC_SPIN_ON  = 2'd1,      // four packets
               MAC_SPIN_OFF = 2'd2;

    localparam ST_IDLE  = 2'd0,
               ST_START = 2'd1,
               ST_ARM   = 2'd2,
               ST_WAIT  = 2'd3;

    reg [1:0] mstate   = ST_IDLE;
    reg [1:0] macro    = MAC_SEQ;
    reg [1:0] step     = 2'd0;
    reg       spinning = 1'b0;
    reg       step_ok  = 1'b0;
    reg       send     = 1'b0;
    reg [2:0] cmd      = 3'd0;

    wire [1:0] last_step = (macro == MAC_SEQ) ? 2'd0 : 2'd3;

    always @(posedge clk) begin
        send <= 1'b0;                      // one-cycle pulse

        if (rst) begin
            mstate   <= ST_IDLE;
            macro    <= MAC_SEQ;
            step     <= 2'd0;
            cmd      <= 3'd0;
            spinning <= 1'b0;
        end else case (mstate)

            ST_IDLE: begin
                if (key1_pressed && !busy) begin
                    macro  <= MAC_SEQ;
                    step   <= 2'd0;
                    mstate <= ST_START;
                end else if (key2_pressed && !busy) begin
                    macro  <= spinning ? MAC_SPIN_OFF : MAC_SPIN_ON;
                    step   <= 2'd0;
                    mstate <= ST_START;
                end
            end

            ST_START: begin
                send    <= 1'b1;
                step_ok <= 1'b0;
                mstate  <= ST_ARM;
            end

            // Wait for the bus to actually pick the command up before
            // watching for it to finish, or we would see the idle `busy`
            // and call the exchange complete immediately.
            ST_ARM: if (busy) mstate <= ST_WAIT;

            ST_WAIT: begin
                if (resp_valid)
                    step_ok <= (err_byte == 8'h00);
                if (!busy) begin
                    if (!step_ok)
                        mstate <= ST_IDLE;               // abort the macro
                    else if (step == last_step) begin
                        case (macro)
                            MAC_SPIN_ON:  spinning <= 1'b1;
                            MAC_SPIN_OFF: spinning <= 1'b0;
                            default:      cmd <= (cmd == 3'd4) ? 3'd0
                                                               : cmd + 1'b1;
                        endcase
                        mstate <= ST_IDLE;
                    end else begin
                        step   <= step + 1'b1;
                        mstate <= ST_START;
                    end
                end
            end

            default: mstate <= ST_IDLE;
        endcase
    end

    // --- position target for the three move commands ----------------------
    reg [15:0] target;
    always @(*) begin
        case (cmd)
            3'd1:    target = 16'd1024;
            3'd2:    target = 16'd2048;
            default: target = 16'd3072;
        endcase
    end

    // --- packet decode ----------------------------------------------------
    // Parameters pack P1 into [7:0], P2 into [15:8], ... as hx_bus expects.
    reg [7:0]  instr;
    reg [63:0] params;
    reg [3:0]  nparam;

    always @(*) begin
        case (macro)

            MAC_SPIN_ON: case (step)
                2'd0: begin                                  // unlock NVS
                    instr  = INSTR_WRITE;
                    params = {48'd0, 8'h00, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                2'd1: begin                                  // constant-speed
                    instr  = INSTR_WRITE;
                    params = {40'd0, 8'h00, 8'h01, REG_CTRL_MODE};
                    nparam = 4'd3;
                end
                2'd2: begin                                  // re-lock NVS
                    instr  = INSTR_WRITE;
                    params = {48'd0, 8'h01, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                default: begin                               // go
                    instr  = INSTR_WRITE;
                    params = {40'd0, MAX_SPEED[15:8], MAX_SPEED[7:0],
                              REG_RUN_SPEED};
                    nparam = 4'd3;
                end
            endcase

            MAC_SPIN_OFF: case (step)
                2'd0: begin                                  // stop first
                    instr  = INSTR_WRITE;
                    params = {40'd0, 8'h00, 8'h00, REG_RUN_SPEED};
                    nparam = 4'd3;
                end
                2'd1: begin                                  // unlock NVS
                    instr  = INSTR_WRITE;
                    params = {48'd0, 8'h00, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                2'd2: begin                                  // position mode
                    instr  = INSTR_WRITE;
                    params = {40'd0, 8'h00, 8'h00, REG_CTRL_MODE};
                    nparam = 4'd3;
                end
                default: begin                               // re-lock NVS
                    instr  = INSTR_WRITE;
                    params = {48'd0, 8'h01, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
            endcase

            default: case (cmd)                              // MAC_SEQ
                3'd0: begin                                  // PING
                    instr  = INSTR_PING;
                    params = 64'd0;
                    nparam = 4'd0;
                end
                3'd4: begin                                  // READ position
                    instr  = INSTR_READ;
                    params = {48'd0, 8'd2, REG_CURRENT_POS};
                    nparam = 4'd2;
                end
                default: begin                               // atomic move
                    instr  = INSTR_WRITE;
                    params = {8'h00,
                              MOVE_SPEED[15:8], MOVE_SPEED[7:0],
                              16'h0000,
                              target[15:8], target[7:0],
                              REG_TARGET_POS};
                    nparam = 4'd7;
                end
            endcase
        endcase
    end

    // --- bus --------------------------------------------------------------
    hx_bus #(
        .CLKS_PER_BIT(CLKS_PER_BIT),
        .RESP_TIMEOUT(RESP_TIMEOUT)
    ) u_bus (
        .clk(clk), .rst(rst),
        .send(send),
        .id(SERVO_ID),
        .instr(instr),
        .params(params),
        .nparam(nparam),
        .expect_reply(1'b1),          // never broadcasting in this lesson
        .busy(busy),
        .resp_valid(resp_valid),
        .resp_bad(resp_bad),
        .resp_timeout(resp_timeout),
        .err_byte(err_byte),
        .resp_params(resp_params),
        .resp_nparam(resp_nparam),
        .tx_pin(servo_tx),
        .rx_pin(servo_rx)
    );

    // --- last known position ---------------------------------------------
    reg [15:0] last_pos = 16'd0;
    always @(posedge clk)
        if (resp_valid && resp_nparam == 4'd2)
            last_pos <= {resp_params[15:8], resp_params[7:0]};

    // --- status -----------------------------------------------------------
    reg ok = 1'b0;
    always @(posedge clk)
        if (rst)                           ok <= 1'b0;
        else if (send)                     ok <= 1'b0;
        else if (resp_valid)               ok <= (err_byte == 8'h00);
        else if (resp_bad || resp_timeout) ok <= 1'b0;

    assign led_done = ok;

    // Heartbeat doubles as a spin indicator: 4x faster while the servo is
    // in constant-speed mode, so "is it still spinning" is answerable from
    // across the bench.
    reg [24:0] hb = 25'd0;
    always @(posedge clk)
        hb <= hb + 1'b1;
    assign led_ready = spinning ? hb[22] : hb[24];

endmodule
