// Dead-man jog control for two daisy-chained HX-30HM bus servos.
//
// S1 is the only motion control:
//   press               jog immediately at maximum rated speed
//   release             stop immediately and restore position-servo mode
//   short double-click  toggle direction after the click window expires
//   short triple-click  select the other servo (ID 1 <-> ID 2)
//
// S2 is an emergency stop. It never starts motion.
//
// led_done  = selected direction (off = direction A, on = direction B)
// led_ready = heartbeat; SLOW when stopped, FAST while jogging
//
// The servo's constant-speed mode lives in NVS and therefore survives power
// cycles. To make the safe state explicit, power-up, release, emergency stop,
// and every failed start all run the same four-packet STOP macro:
//
//   0x2E <- 0      stop first, while still in speed mode
//   0x37 <- 0      unlock NVS
//   0x21 <- 0      restore position-servo mode
//   0x37 <- 1      re-lock NVS
//
// Starting a jog is also four packets:
//
//   0x37 <- 0      unlock NVS
//   0x21 <- 1      select constant-speed mode
//   0x37 <- 1      re-lock NVS
//   0x2E <- speed  start after the debounced button press
//
// If the button is released while that sequence is in flight, the sequencer
// finishes at most the current packet and switches directly to STOP. The STOP
// macro attempts every cleanup packet even if a reply is lost.

module top #(
    parameter CLKS_PER_BIT     = 50,          // 50 MHz / 1 Mbaud, exact
    parameter HOST_CLKS_PER_BIT= 434,         // 50 MHz / 115200 baud
    parameter DEBOUNCE_CYC     = 1_000_000,   // 20 ms
    parameter LONG_PRESS_CYC   = 25_000_000,  // 500 ms
    parameter DOUBLE_CLICK_CYC = 20_000_000,  // 400 ms, release-to-release
    parameter RESP_TIMEOUT     = 250_000,     // 5 ms
    parameter POLL_CYC         = 250_000,     // one register read every 5 ms
    parameter TELEMETRY_CYC    = 5_000_000,   // host snapshot every 100 ms
    parameter MS_CYC           = 50_000,      // uptime/hold clock, 1 ms
    parameter HOST_DEADMAN_CYC = 15_000_000   // 300 ms without host refresh
)(
    input  wire clk,
    input  wire key,          // dock S1: jog / double direction / triple servo
    input  wire key2,         // dock S2: emergency stop only
    output wire servo_tx,     // G5 -> 1k -> servo SIG
    input  wire servo_rx,     // F5 <- 1k <- servo SIG
    input  wire host_rx,      // B3 <- dock FT2232 channel-B USB UART
    output wire host_tx,      // C3 -> dock FT2232 channel-B USB UART
    output wire led_done,
    output wire led_ready
);
    `include "crc8.v"

    // --- power-on reset ---------------------------------------------------
    reg [3:0] por = 4'd0;
    wire      rst = (por != 4'hF);
    always @(posedge clk)
        if (por != 4'hF) por <= por + 1'b1;

    // --- buttons ----------------------------------------------------------
    wire key1_pressed, key1_released, key1_state;
    wire key2_pressed, key2_state;

    button #(.DEBOUNCE_CYC(DEBOUNCE_CYC)) u_b1 (
        .clk(clk), .rst(rst), .raw(key),
        .state(key1_state), .pressed(key1_pressed),
        .released(key1_released));

    button #(.DEBOUNCE_CYC(DEBOUNCE_CYC)) u_b2 (
        .clk(clk), .rst(rst), .raw(key2),
        .state(key2_state), .pressed(key2_pressed), .released());

    // --- S1 gesture decoder ----------------------------------------------
    localparam HOLD_W  = $clog2(LONG_PRESS_CYC + 1);
    localparam CLICK_W = $clog2(DOUBLE_CLICK_CYC + 1);

    reg [HOLD_W-1:0]  hold_count = 0;
    reg [CLICK_W-1:0] click_timer = 0;
    reg [1:0] click_tally = 0;
    reg hold_fired    = 1'b0;
    reg reverse_dir   = 1'b0;
    reg selected_motor= 1'b0; // 0 = ID 1, 1 = ID 2
    reg motor_changed = 1'b0;
    reg jog_start     = 1'b0;
    reg jog_stop      = 1'b0;
    wire click_pending = (click_tally != 0);

    // --- CRC-protected dashboard control --------------------------------
    // OP 0x10, payload: selected ID (1/2), flags (run/reverse/e-stop).
    // The page must refresh a held RUN at least every 300 ms. A lost browser,
    // USB cable, server, malformed packet, or stale pointer therefore turns
    // into the same STOP cleanup as releasing S1.
    localparam [7:0] OP_HOST_CONTROL = 8'h10;
    wire [7:0] host_rx_data;
    wire host_rx_valid;
    uart_rx #(.CLKS_PER_BIT(HOST_CLKS_PER_BIT)) u_host_uart_rx (
        .clk(clk), .rst(rst), .rx(host_rx), .enable(1'b1),
        .data(host_rx_data), .valid(host_rx_valid));

    /* verilator lint_off UNUSEDSIGNAL */
    wire host_frame_valid, host_pay_we;
    wire [7:0] host_seq, host_op, host_plen, host_pay_data;
    wire [5:0] host_pay_addr;
    wire [15:0] host_crc_errors, host_frame_timeouts;
    link_rx #(.IDLE_TIMEOUT(50_000)) u_host_link_rx (
        .clk(clk), .rst(rst), .rx_data(host_rx_data),
        .rx_valid(host_rx_valid), .frame_valid(host_frame_valid),
        .seq(host_seq), .op(host_op), .plen(host_plen),
        .crc_errors(host_crc_errors), .timeouts(host_frame_timeouts),
        .pay_data(host_pay_data), .pay_addr(host_pay_addr),
        .pay_we(host_pay_we));

    reg [7:0] host_payload0 = 0, host_payload1 = 0;
    /* verilator lint_on UNUSEDSIGNAL */
    always @(posedge clk) begin
        if (host_pay_we && host_pay_addr == 0) host_payload0 <= host_pay_data;
        if (host_pay_we && host_pay_addr == 1) host_payload1 <= host_pay_data;
    end

    reg host_run = 1'b0;
    reg host_select_valid = 1'b0;
    reg host_select_motor = 1'b0;
    reg host_select_reverse = 1'b0;
    reg host_jog_start = 1'b0, host_jog_stop = 1'b0, host_estop = 1'b0;
    reg [$clog2(HOST_DEADMAN_CYC+1)-1:0] host_deadman = 0;

    always @(posedge clk) begin
        host_select_valid <= 1'b0;
        host_jog_start <= 1'b0;
        host_jog_stop <= 1'b0;
        host_estop <= 1'b0;

        if (rst) begin
            host_run <= 1'b0;
            host_deadman <= 0;
        end else begin
            if (host_run) begin
                if (host_deadman == HOST_DEADMAN_CYC - 1) begin
                    host_run <= 1'b0;
                    host_deadman <= 0;
                    host_jog_stop <= 1'b1;
                end else
                    host_deadman <= host_deadman + 1'b1;
            end else
                host_deadman <= 0;

            if (host_frame_valid && host_op == OP_HOST_CONTROL &&
                host_plen == 8'd2 &&
                (host_payload0 == 8'd1 || host_payload0 == 8'd2)) begin
                host_deadman <= 0;

                if (host_payload1[2]) begin
                    host_run <= 1'b0;
                    host_estop <= 1'b1;
                    host_jog_stop <= 1'b1;
                end else begin
                    host_select_valid <= 1'b1;
                    host_select_motor <= (host_payload0 == 8'd2);
                    host_select_reverse <= host_payload1[1];
                    if (host_payload1[0]) begin
                    // If selection changed, stop first. The next 100 ms web
                    // refresh starts the newly selected motor after cleanup.
                        if ((selected_motor != (host_payload0 == 8'd2)) ||
                            (reverse_dir != host_payload1[1])) begin
                            host_run <= 1'b0;
                            host_jog_stop <= 1'b1;
                        end else begin
                            if (!host_run) host_jog_start <= 1'b1;
                            host_run <= 1'b1;
                        end
                    end else begin
                        if (host_run) host_jog_stop <= 1'b1;
                        host_run <= 1'b0;
                    end
                end
            end

            // The physical e-stop always wins over a simultaneous host frame.
            if (key2_pressed || key2_state) begin
                host_run <= 1'b0;
                host_jog_stop <= 1'b1;
            end
        end
    end

    wire effective_hold = key1_state | host_run;
    wire any_jog_start = jog_start | host_jog_start;
    wire any_jog_stop = jog_stop | host_jog_stop;
    wire any_estop = key2_pressed | host_estop;

    // Human-readable timing is included in every telemetry snapshot. This is
    // deliberately independent of the gesture counter so the dashboard can
    // show how long S1 has actually been held without knowing the FPGA clock.
    reg [$clog2(MS_CYC+1)-1:0] ms_div = 0;
    reg [31:0]                  uptime_ms = 0;
    reg [15:0]                  hold_ms = 0;

    always @(posedge clk) begin
        if (rst) begin
            ms_div    <= 0;
            uptime_ms <= 0;
            hold_ms   <= 0;
        end else begin
            if (ms_div == MS_CYC - 1) begin
                ms_div    <= 0;
                uptime_ms <= uptime_ms + 1'b1;
                if (effective_hold && hold_ms != 16'hFFFF)
                    hold_ms <= hold_ms + 1'b1;
                else if (!effective_hold)
                    hold_ms <= 0;
            end else
                ms_div <= ms_div + 1'b1;
        end
    end

    always @(posedge clk) begin
        jog_start <= 1'b0;
        jog_stop  <= 1'b0;
        motor_changed <= 1'b0;

        if (rst) begin
            hold_count    <= 0;
            click_timer   <= 0;
            click_tally   <= 0;
            hold_fired    <= 1'b0;
            reverse_dir   <= 1'b0;
            selected_motor<= 1'b0;
        end else begin
            if (host_select_valid) begin
                if (selected_motor != host_select_motor)
                    motor_changed <= 1'b1;
                selected_motor <= host_select_motor;
                reverse_dir <= host_select_reverse;
                click_tally <= 0;
                click_timer <= 0;
            end

            // A double-click cannot be committed until the third-click window
            // expires; otherwise the first two clicks of a triple-click would
            // also reverse direction. A single click expires with no gesture.
            if (click_tally != 0 && !key1_released) begin
                if (click_timer == 0) begin
                    if (click_tally == 2)
                        reverse_dir <= ~reverse_dir;
                    click_tally <= 0;
                end else begin
                    click_timer <= click_timer - 1'b1;
                end
            end

            if (key1_pressed) begin
                hold_count <= 0;
                hold_fired <= 1'b0;
                jog_start  <= 1'b1;
            end else if (key1_state && !hold_fired) begin
                if (hold_count == LONG_PRESS_CYC - 1) begin
                    hold_fired  <= 1'b1;
                    click_tally <= 0; // a hold cannot complete a click gesture
                    click_timer <= 0;
                end else begin
                    hold_count <= hold_count + 1'b1;
                end
            end

            if (key1_released) begin
                // Every release stops, including the releases that also
                // participate in the double-click direction gesture.
                jog_stop  <= 1'b1;
                hold_count <= 0;
                hold_fired <= 1'b0;

                if (hold_fired) begin
                    click_tally <= 0;
                    click_timer <= 0;
                end else if (click_tally == 2) begin
                    // The third release selects the other servo immediately.
                    selected_motor <= ~selected_motor;
                    motor_changed  <= 1'b1;
                    click_tally    <= 0;
                    click_timer    <= 0;
                end else begin
                    click_tally <= click_tally + 1'b1;
                    click_timer <= DOUBLE_CLICK_CYC;
                end
            end
        end
    end

    // --- bus signals ------------------------------------------------------
    wire        busy;
    wire        resp_valid;
    wire        resp_bad;
    wire        resp_timeout;
    wire [7:0]  err_byte;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [63:0] resp_params; // HX parser supports eight bytes; these reads use at most two
    /* verilator lint_on UNUSEDSIGNAL */

    // --- HX-30HM registers and instructions ------------------------------
    localparam [7:0] SERVO_ID_1 = 8'd1,
                     SERVO_ID_2 = 8'd2;
    localparam [7:0] INSTR_READ  = 8'h02,
                     INSTR_WRITE = 8'h03;
    localparam [7:0] REG_CTRL_MODE = 8'h21,
                     REG_RUN_SPEED = 8'h2E,
                     REG_NVS_LOCK  = 8'h37;

    // HX-30HM manual maximum: 3400 steps/s (about 299 degrees/s).
    // Direction is sign-magnitude: bit 15 is direction, bits 14:0 magnitude.
    localparam [15:0] JOG_SPEED = 16'd3400;
    wire [7:0] selected_servo_id = selected_motor ? SERVO_ID_2 : SERVO_ID_1;
    wire [15:0] requested_speed = reverse_dir
                                ? (16'h8000 | JOG_SPEED)
                                : JOG_SPEED;

    // --- fail-safe macro sequencer ---------------------------------------
    localparam MAC_START = 1'b0,
               MAC_STOP  = 1'b1;

    localparam ST_IDLE  = 2'd0,
               ST_START = 2'd1,
               ST_ARM   = 2'd2,
               ST_WAIT  = 2'd3;

    reg [1:0] mstate = ST_IDLE;
    reg       macro  = MAC_STOP;
    reg [1:0] step   = 2'd0;
    reg       moving = 1'b0;
    reg       step_ok = 1'b0;
    reg       send = 1'b0;
    reg       is_poll = 1'b0;
    reg       start_pending = 1'b0;
    reg [7:0] command_id = SERVO_ID_1;
    reg       boot_cleanup_active = 1'b1;
    reg       boot_cleanup_second = 1'b0;

    // --- diagnostic register cache --------------------------------------
    // One low-priority read is issued every POLL_CYC. A complete sweep is
    // about 105 ms with the defaults. Any motion or stop macro wins bus
    // arbitration; a stop already requested waits only for the single HX
    // packet currently in flight (bounded by RESP_TIMEOUT).
    localparam [4:0] POLL_LAST = 5'd20;
    reg [4:0] poll_index = 0;
    reg [$clog2(POLL_CYC+1)-1:0] poll_count = 0;
    reg poll_due = 1'b0;
    reg [7:0] poll_addr;
    reg [7:0] poll_len;

    reg [31:0] diag_valid = 0;
    reg [15:0] good_replies = 0;
    reg [15:0] bad_replies = 0;
    reg [15:0] timeout_replies = 0;
    reg [15:0] servo_error_replies = 0;
    reg [15:0] emergency_stops = 0;
    reg [7:0]  last_err_byte = 0;

    reg [7:0]  fw_major = 0, fw_minor = 0, diag_id = 0, baud_idx = 0;
    reg [7:0]  resp_level = 0, ctrl_mode = 0, nvs_lock = 0;
    reg [7:0]  temp_limit = 0, max_torque = 0, protect_ctrl = 0;
    reg [7:0]  temperature_c = 0, status_bits = 0, servo_moving = 0;
    reg [15:0] current_limit_ma = 0, target_pos = 0, run_speed = 0;
    reg [15:0] current_pos = 0, current_speed = 0, current_load = 0;
    reg [15:0] voltage_mv = 0, current_ma = 0;

    // Set at reset so loading this bitstream actively cleans up any speed
    // mode or non-zero run-speed left by an earlier FPGA image.
    reg stop_pending = 1'b1;

    always @(posedge clk) begin
        send <= 1'b0;

        if (rst) begin
            mstate       <= ST_IDLE;
            macro        <= MAC_STOP;
            step         <= 2'd0;
            moving       <= 1'b0;
            step_ok      <= 1'b0;
            stop_pending <= 1'b1;
            is_poll      <= 1'b0;
            start_pending<= 1'b0;
            command_id   <= SERVO_ID_1;
            boot_cleanup_active <= 1'b1;
            boot_cleanup_second <= 1'b0;
            poll_index   <= 0;
            poll_count   <= 0;
            poll_due     <= 1'b0;
            diag_valid   <= 0;
            good_replies <= 0;
            bad_replies  <= 0;
            timeout_replies <= 0;
            servo_error_replies <= 0;
            emergency_stops <= 0;
            last_err_byte <= 0;
        end else begin
            if (!poll_due) begin
                if (poll_count == POLL_CYC - 1) begin
                    poll_count <= 0;
                    poll_due   <= 1'b1;
                end else
                    poll_count <= poll_count + 1'b1;
            end

            // Bus health covers both control writes and diagnostic reads.
            if (resp_valid) begin
                if (good_replies != 16'hFFFF)
                    good_replies <= good_replies + 1'b1;
                last_err_byte <= err_byte;
                if (err_byte != 0 && servo_error_replies != 16'hFFFF)
                    servo_error_replies <= servo_error_replies + 1'b1;

                if (is_poll && err_byte == 0) begin
                    diag_valid[poll_index] <= 1'b1;
                    case (poll_index)
                        5'd0:  fw_major        <= resp_params[7:0];
                        5'd1:  fw_minor        <= resp_params[7:0];
                        5'd2:  diag_id         <= resp_params[7:0];
                        5'd3:  baud_idx        <= resp_params[7:0];
                        5'd4:  ctrl_mode       <= resp_params[7:0];
                        5'd5:  target_pos      <= resp_params[15:0];
                        5'd6:  run_speed       <= resp_params[15:0];
                        5'd7:  nvs_lock        <= resp_params[7:0];
                        5'd8:  current_pos     <= resp_params[15:0];
                        5'd9:  current_speed   <= resp_params[15:0];
                        5'd10: current_load    <= resp_params[15:0];
                        5'd11: voltage_mv      <= resp_params[15:0];
                        5'd12: temperature_c   <= resp_params[7:0];
                        5'd13: status_bits     <= resp_params[7:0];
                        5'd14: servo_moving    <= resp_params[7:0];
                        5'd15: current_ma      <= resp_params[15:0];
                        5'd16: resp_level      <= resp_params[7:0];
                        5'd17: temp_limit      <= resp_params[7:0];
                        5'd18: max_torque      <= resp_params[7:0];
                        5'd19: protect_ctrl    <= resp_params[7:0];
                        5'd20: current_limit_ma<= resp_params[15:0];
                        default: ;
                    endcase
                end
            end
            if (resp_bad && bad_replies != 16'hFFFF)
                bad_replies <= bad_replies + 1'b1;
            if (resp_timeout && timeout_replies != 16'hFFFF)
                timeout_replies <= timeout_replies + 1'b1;
            if (any_estop && emergency_stops != 16'hFFFF)
                emergency_stops <= emergency_stops + 1'b1;

            // Release, motor selection, and S2 have priority over motion.
            // Selecting a motor first stops the old target (via release), then
            // runs a fresh STOP cleanup against the newly selected target.
            if (motor_changed) begin
                stop_pending <= 1'b1;
                start_pending <= 1'b0;
                poll_index <= 0;
                poll_due <= 1'b0;
                diag_valid <= 0;
            end else if (any_jog_stop || any_estop) begin
                stop_pending <= 1'b1;
                start_pending <= 1'b0;
            end else if (any_jog_start && effective_hold)
                start_pending <= 1'b1;
            else if (!effective_hold)
                start_pending <= 1'b0;

            case (mstate)
                ST_IDLE: begin
                    if (stop_pending || any_jog_stop || any_estop) begin
                        is_poll     <= 1'b0;
                        // A fresh press can arrive while cleanup from the
                        // preceding click is still finishing. Preserve it
                        // across that cleanup, but never across a release or
                        // emergency stop.
                        start_pending <= (start_pending || any_jog_start) &&
                                         effective_hold && !any_jog_stop &&
                                         !any_estop && !key2_state;
                        macro        <= MAC_STOP;
                        step         <= 2'd0;
                        moving       <= 1'b0;
                        stop_pending <= 1'b0;
                        command_id   <= (boot_cleanup_active &&
                                         boot_cleanup_second)
                                        ? SERVO_ID_2 : selected_servo_id;
                        mstate       <= ST_START;
                    end else if ((start_pending || any_jog_start) && effective_hold) begin
                        is_poll <= 1'b0;
                        start_pending <= 1'b0;
                        command_id <= selected_servo_id;
                        macro  <= MAC_START;
                        step   <= 2'd0;
                        mstate <= ST_START;
                    end else if (poll_due) begin
                        is_poll   <= 1'b1;
                        poll_due <= 1'b0;
                        command_id <= selected_servo_id;
                        mstate   <= ST_START;
                    end
                end

                ST_START: begin
                    send    <= 1'b1;
                    step_ok <= 1'b0;
                    mstate  <= ST_ARM;
                end

                ST_ARM: begin
                    if (busy)
                        mstate <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (resp_valid && !is_poll)
                        step_ok <= (err_byte == 8'h00);

                    if (!busy) begin
                        if (is_poll) begin
                            poll_index <= (poll_index == POLL_LAST)
                                        ? 5'd0 : poll_index + 1'b1;
                            is_poll <= 1'b0;
                            mstate  <= ST_IDLE;
                        end else if (macro == MAC_START) begin
                            // Never send (or leave active) a speed command
                            // once the user's hand has left S1.
                            if (stop_pending || any_jog_stop || any_estop ||
                                !effective_hold || !step_ok) begin
                                macro        <= MAC_STOP;
                                step         <= 2'd0;
                                moving       <= 1'b0;
                                stop_pending <= 1'b0;
                                start_pending<= 1'b0;
                                mstate       <= ST_START;
                            end else if (step == 2'd3) begin
                                moving <= 1'b1;
                                mstate <= ST_IDLE;
                            end else begin
                                step   <= step + 1'b1;
                                mstate <= ST_START;
                            end
                        end else begin
                            // STOP is cleanup: attempt every step even if a
                            // reply is missing or reports an error.
                            if (step == 2'd3) begin
                                moving <= 1'b0;
                                mstate <= ST_IDLE;
                                // At boot, explicitly clean both IDs. This
                                // handles either servo having retained speed
                                // mode in NVS from a previous session.
                                if (boot_cleanup_active) begin
                                    if (!boot_cleanup_second) begin
                                        boot_cleanup_second <= 1'b1;
                                        stop_pending <= 1'b1;
                                    end else begin
                                        boot_cleanup_active <= 1'b0;
                                        boot_cleanup_second <= 1'b0;
                                    end
                                end
                            end else begin
                                step   <= step + 1'b1;
                                mstate <= ST_START;
                            end
                        end
                    end
                end

                default: mstate <= ST_IDLE;
            endcase
        end
    end

    // --- packet selection ------------------------------------------------
    // Parameters pack P1 into [7:0], P2 into [15:8], ... as hx_bus expects.
    reg [7:0]  instr;
    reg [63:0] params;
    reg [3:0]  nparam;
    always @(*) begin
        poll_addr = 8'h00;
        poll_len  = 8'd1;
        case (poll_index)
            5'd0:  poll_addr = 8'h00;                         // fw major
            5'd1:  poll_addr = 8'h01;                         // fw minor
            5'd2:  poll_addr = 8'h05;                         // ID
            5'd3:  poll_addr = 8'h06;                         // baud index
            5'd4:  poll_addr = 8'h21;                         // control mode
            5'd5:  begin poll_addr = 8'h2A; poll_len = 2; end // target position
            5'd6:  begin poll_addr = 8'h2E; poll_len = 2; end // run speed
            5'd7:  poll_addr = 8'h37;                         // NVS lock
            5'd8:  begin poll_addr = 8'h38; poll_len = 2; end // current position
            5'd9:  begin poll_addr = 8'h3A; poll_len = 2; end // current speed
            5'd10: begin poll_addr = 8'h3C; poll_len = 2; end // current load
            5'd11: begin poll_addr = 8'h3E; poll_len = 2; end // voltage, mV
            5'd12: poll_addr = 8'h3F;                         // temperature
            5'd13: poll_addr = 8'h41;                         // status bits
            5'd14: poll_addr = 8'h42;                         // moving flag
            5'd15: begin poll_addr = 8'h45; poll_len = 2; end // current, mA
            5'd16: poll_addr = 8'h08;                         // response level
            5'd17: poll_addr = 8'h0D;                         // temperature limit
            5'd18: poll_addr = 8'h13;                         // maximum torque
            5'd19: poll_addr = 8'h14;                         // protection config
            5'd20: begin poll_addr = 8'h1F; poll_len = 2; end // current limit, mA
            default: ;
        endcase
    end

    always @(*) begin
        instr  = INSTR_WRITE;
        params = 64'd0;
        nparam = 4'd0;

        if (is_poll) begin
            instr  = INSTR_READ;
            params = {48'd0, poll_len, poll_addr};
            nparam = 4'd2;
        end else if (macro == MAC_START) begin
            case (step)
                2'd0: begin                                  // unlock NVS
                    params = {48'd0, 8'h00, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                2'd1: begin                                  // speed mode
                    params = {40'd0, 8'h00, 8'h01, REG_CTRL_MODE};
                    nparam = 4'd3;
                end
                2'd2: begin                                  // re-lock NVS
                    params = {48'd0, 8'h01, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                default: begin                               // jog
                    params = {40'd0,
                              requested_speed[15:8], requested_speed[7:0],
                              REG_RUN_SPEED};
                    nparam = 4'd3;
                end
            endcase
        end else begin
            case (step)
                2'd0: begin                                  // stop first
                    params = {40'd0, 8'h00, 8'h00, REG_RUN_SPEED};
                    nparam = 4'd3;
                end
                2'd1: begin                                  // unlock NVS
                    params = {48'd0, 8'h00, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
                2'd2: begin                                  // position mode
                    params = {40'd0, 8'h00, 8'h00, REG_CTRL_MODE};
                    nparam = 4'd3;
                end
                default: begin                               // re-lock NVS
                    params = {48'd0, 8'h01, REG_NVS_LOCK};
                    nparam = 4'd2;
                end
            endcase
        end
    end

    // --- bus --------------------------------------------------------------
    hx_bus #(
        .CLKS_PER_BIT(CLKS_PER_BIT),
        .RESP_TIMEOUT(RESP_TIMEOUT)
    ) u_bus (
        .clk(clk), .rst(rst),
        .send(send),
        .id(command_id),
        .instr(instr),
        .params(params),
        .nparam(nparam),
        .expect_reply(1'b1),
        .busy(busy),
        .resp_valid(resp_valid),
        .resp_bad(resp_bad),
        .resp_timeout(resp_timeout),
        .err_byte(err_byte),
        .resp_params(resp_params),
        .resp_nparam(),
        .tx_pin(servo_tx),
        .rx_pin(servo_rx)
    );

    assign led_done = reverse_dir;

    reg [24:0] hb = 25'd0;
    always @(posedge clk)
        hb <= hb + 1'b1;
    assign led_ready = moving ? hb[22] : hb[24];

    // --- read-only USB telemetry -----------------------------------------
    // The computer is never in the safety/control path. This UART only emits
    // snapshots; disconnecting it cannot start, sustain, or stop the motor.
    wire [7:0] control_flags = {
        busy, stop_pending, click_pending, moving,
        hold_fired, reverse_dir, key2_state, effective_hold
    };
    wire [7:0] controller_state = {
        1'b0, is_poll, step, macro, mstate
    };

    // A deliberately simple 100-slot telemetry scheduler. Each 1 ms slot
    // either launches one UART byte or idles; 64 frame bytes + 36 idle slots
    // produce a 10 Hz stream with ample spacing between UART bytes.
    localparam [7:0] HOST_PAYLOAD_LEN = 8'd58;
    localparam integer TELEMETRY_SLOT_CYC = TELEMETRY_CYC / 100;
    reg [$clog2(TELEMETRY_SLOT_CYC+1)-1:0] telemetry_pace = 0;
    reg [6:0] telemetry_slot = 7'd99;
    reg [7:0] telemetry_sequence = 0;
    reg [7:0] telemetry_crc = 0;
    reg [463:0] telemetry_payload = 0;
    reg [7:0] telemetry_uart_data = 0;
    reg telemetry_uart_start = 0;
    wire telemetry_uart_busy;
    reg [7:0] telemetry_frame_byte;

    // The FT2232 channel can remain dormant after FPGA reconfiguration until
    // it sees a complete byte. Emit an out-of-frame A8 wake marker immediately
    // before each AA55 packet; the host parser safely ignores it while this
    // also provides a simple scope/terminal heartbeat.
    reg [22:0] host_wake_count = 0;
    reg host_wake_start = 0;
    always @(posedge clk) begin
        host_wake_start <= 1'b0;
        if (rst) host_wake_count <= 0;
        else if (host_wake_count == TELEMETRY_CYC - 1) begin
            host_wake_count <= 0;
            if (!telemetry_uart_busy) host_wake_start <= 1'b1;
        end else host_wake_count <= host_wake_count + 1'b1;
    end

    always @* begin
        case (telemetry_slot)
            7'd0:  telemetry_frame_byte = 8'hAA;
            7'd1:  telemetry_frame_byte = 8'h55;
            7'd2:  telemetry_frame_byte = HOST_PAYLOAD_LEN;
            7'd3:  telemetry_frame_byte = telemetry_sequence;
            7'd4:  telemetry_frame_byte = 8'h81;
            7'd63: telemetry_frame_byte = telemetry_crc;
            default: telemetry_frame_byte = telemetry_payload[7:0];
        endcase
    end

    uart_tx #(.CLKS_PER_BIT(HOST_CLKS_PER_BIT)) u_host_uart_tx (
        .clk(clk), .rst(rst),
        .start(host_wake_start | telemetry_uart_start),
        .data(telemetry_uart_start ? telemetry_uart_data : 8'hA8),
        .tx(host_tx),
        .busy(telemetry_uart_busy));

    always @(posedge clk) begin
        telemetry_uart_start <= 1'b0;
        if (rst) begin
            telemetry_pace <= 0;
            telemetry_slot <= 7'd99;
            telemetry_sequence <= 0;
            telemetry_crc <= 0;
            telemetry_uart_data <= 0;
        end else if (telemetry_pace == TELEMETRY_SLOT_CYC - 1) begin
            telemetry_pace <= 0;
            if (telemetry_slot == 7'd99) begin
                telemetry_slot <= 0;
                telemetry_sequence <= telemetry_sequence + 1'b1;
                telemetry_crc <= 0;
                telemetry_payload <= {
                    emergency_stops[15:8], emergency_stops[7:0],
                    selected_servo_id, {3'd0, poll_index},
                    hold_ms[15:8], hold_ms[7:0],
                    (moving ? requested_speed[15:8] : 8'd0),
                    (moving ? requested_speed[7:0] : 8'd0),
                    current_ma[15:8], current_ma[7:0],
                    voltage_mv[15:8], voltage_mv[7:0],
                    current_load[15:8], current_load[7:0],
                    current_speed[15:8], current_speed[7:0],
                    current_pos[15:8], current_pos[7:0],
                    run_speed[15:8], run_speed[7:0],
                    target_pos[15:8], target_pos[7:0],
                    current_limit_ma[15:8], current_limit_ma[7:0],
                    8'd0, servo_moving, status_bits, temperature_c,
                    protect_ctrl, max_torque, temp_limit, nvs_lock,
                    ctrl_mode, resp_level, baud_idx, diag_id,
                    fw_minor, fw_major,
                    servo_error_replies[15:8], servo_error_replies[7:0],
                    timeout_replies[15:8], timeout_replies[7:0],
                    bad_replies[15:8], bad_replies[7:0],
                    good_replies[15:8], good_replies[7:0],
                    diag_valid[31:24], diag_valid[23:16],
                    diag_valid[15:8], diag_valid[7:0],
                    uptime_ms[31:24], uptime_ms[23:16],
                    uptime_ms[15:8], uptime_ms[7:0],
                    last_err_byte, controller_state, control_flags, 8'd1
                };
            end else begin
                telemetry_slot <= telemetry_slot + 1'b1;
                if (telemetry_slot <= 7'd63 && !telemetry_uart_busy) begin
                    telemetry_uart_data <= telemetry_frame_byte;
                    telemetry_uart_start <= 1'b1;
                    if (telemetry_slot == 7'd2)
                        telemetry_crc <= crc8_byte(0, telemetry_frame_byte);
                    else if (telemetry_slot >= 7'd3 && telemetry_slot <= 7'd62)
                        telemetry_crc <= crc8_byte(telemetry_crc, telemetry_frame_byte);
                    if (telemetry_slot >= 7'd5 && telemetry_slot <= 7'd62)
                        telemetry_payload <= {8'd0, telemetry_payload[463:8]};
                end
            end
        end else
            telemetry_pace <= telemetry_pace + 1'b1;
    end

endmodule
