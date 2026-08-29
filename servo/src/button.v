// Synchronise, debounce, and edge-detect one mechanical button.
//
// Lifted out of motor.v now that there are two buttons to condition. The
// three stages, and why each is needed:
//
//  1. SYNCHRONISER  - the contact closes at a moment unrelated to our clock,
//     which violates flip-flop setup time and can leave a register
//     metastable. Two cascaded flops give a half-resolved value a full cycle
//     to settle before anything downstream looks at it.
//
//  2. DEBOUNCER     - metal contacts chatter for milliseconds. Accept a new
//     level only once it has held steady for DEBOUNCE_CYC cycles; any
//     chatter restarts the timer.
//
//  3. EDGE DETECTOR - `pressed` and `released` are one cycle long on the
//     corresponding transitions, however long the button is held down.
//
// Buttons on this dock are ACTIVE HIGH with an internal pull-down, so idle
// is 0 and the reset state matches.

module button #(
    parameter DEBOUNCE_CYC = 1_000_000       // 20 ms at 50 MHz
)(
    input  wire clk,
    input  wire rst,
    input  wire raw,          // straight off the pin
    output reg  state,        // debounced level
    output wire pressed,      // one-cycle pulse on press
    output wire released      // one-cycle pulse on release
);

    reg [1:0] sync = 2'b00;
    always @(posedge clk)
        if (rst) sync <= 2'b00;
        else     sync <= {sync[0], raw};

    reg [$clog2(DEBOUNCE_CYC)-1:0] cnt = 0;
    always @(posedge clk) begin
        if (rst) begin
            state <= 1'b0;
            cnt   <= 0;
        end else if (sync[1] == state)
            cnt <= 0;
        else if (cnt == DEBOUNCE_CYC - 1) begin
            state <= sync[1];
            cnt   <= 0;
        end else
            cnt <= cnt + 1'b1;
    end

    reg state_d = 1'b0;
    always @(posedge clk)
        if (rst) state_d <= 1'b0;
        else     state_d <= state;

    assign pressed = ~state_d & state;
    assign released = state_d & ~state;

endmodule
