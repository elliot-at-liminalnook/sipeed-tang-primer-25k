// Lesson 5: DC motor speed controller, Tang Primer 25K (GW5A-25)
//
// Button K6 cycles speed: 0% -> 25% -> 50% -> 75% -> 100% -> 0% ...
// Output: 20 kHz PWM on motor_in1, for a DRV8833 H-bridge (IN1/IN2).
// The LEDs mirror the motor PWM, so speed is visible with no motor wired.
//
// New concepts here:
//  1. synchronizer  - the button is an ASYNC input; two flip-flops tame it
//  2. debouncer     - mechanical contacts bounce for ~ms; require stability
//  3. edge detector - act once per press, not every cycle it's held

module top #(
    parameter PWM_PERIOD   = 2500,      // 50 MHz / 2500 = 20 kHz PWM
    parameter DEBOUNCE_CYC = 1_000_000  // 20 ms of required stability
)(
    input  wire clk,          // 50 MHz (E2)
    input  wire key,          // dock button S1 (H11), ACTIVE-HIGH (1 = pressed),
                              // internal pull-down enabled in the .cst
    output wire motor_in1,    // -> DRV8833 IN1: the PWM
    output wire motor_in2,    // -> DRV8833 IN2: 0 = forward
    output wire led_done,     // mirrors PWM: LED brightness = motor speed
    output wire led_ready
);

    // --- 1. synchronizer -------------------------------------------------
    // The button edge can land at ANY moment relative to our clock,
    // violating flip-flop setup time -> metastability. Two cascaded FFs
    // give a random value a full cycle to resolve before the design sees it.
    reg [1:0] key_sync = 2'b00;        // idle = 0 (pull-down, active-high)
    always @(posedge clk)
        key_sync <= {key_sync[0], key};

    // --- 2. debouncer ------------------------------------------------------
    // Accept a new button state only after it has held steady for
    // DEBOUNCE_CYC cycles; contact chatter keeps resetting the timer.
    reg key_state = 1'b0;               // debounced state, idle = 0
    reg [$clog2(DEBOUNCE_CYC)-1:0] db_cnt = 0;

    always @(posedge clk) begin
        if (key_sync[1] == key_state)
            db_cnt <= 0;
        else if (db_cnt == DEBOUNCE_CYC - 1) begin
            key_state <= key_sync[1];
            db_cnt    <= 0;
        end else
            db_cnt <= db_cnt + 1'b1;
    end

    // --- 3. edge detector ---------------------------------------------------
    // One clean pulse on the 0->1 transition (press), no matter how long
    // the button is held.
    reg key_state_d = 1'b0;
    always @(posedge clk)
        key_state_d <= key_state;

    wire key_pressed = ~key_state_d & key_state;

    // --- speed level: 0..4, advance once per press ---------------------------
    reg [2:0] level = 0;
    always @(posedge clk)
        if (key_pressed)
            level <= (level == 3'd4) ? 3'd0 : level + 1'b1;

    // level -> duty (combinational lookup; /4 of a constant is free)
    reg [$clog2(PWM_PERIOD+1)-1:0] duty;
    always @(*) begin
        case (level)
            3'd0:    duty = 0;
            3'd1:    duty = PWM_PERIOD / 4;
            3'd2:    duty = PWM_PERIOD / 2;
            3'd3:    duty = (3 * PWM_PERIOD) / 4;
            default: duty = PWM_PERIOD;          // 100%: always on
        endcase
    end

    // --- 20 kHz PWM core ---------------------------------------------------
    reg [$clog2(PWM_PERIOD)-1:0] pwm_cnt = 0;
    always @(posedge clk)
        pwm_cnt <= (pwm_cnt == PWM_PERIOD - 1) ? 0 : pwm_cnt + 1'b1;

    wire pwm_out = (pwm_cnt < duty);

    // --- outputs -------------------------------------------------------------
    // DRV8833 truth table: IN1=PWM, IN2=0 -> forward, coasting between pulses.
    // Swapping which pin gets the PWM reverses the motor (a later exercise).
    assign motor_in1 = pwm_out;
    assign motor_in2 = 1'b0;

    assign led_done  = pwm_out;   // brightness tracks speed

    // Heartbeat: proves the design is alive even at speed level 0.
    // Debugging lesson learned the hard way — always give yourself a
    // sign of life that doesn't depend on the feature under test.
    reg [24:0] hb_cnt = 0;
    always @(posedge clk)
        hb_cnt <= hb_cnt + 1'b1;
    assign led_ready = hb_cnt[24];   // ~1.5 Hz blink
endmodule
