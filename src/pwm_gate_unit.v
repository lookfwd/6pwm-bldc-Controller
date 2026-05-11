// PWM Gate Unit — variant-selecting wrapper around the
// pwm_phase_correct_* modules. Owns:
//   1. Saturating duty±DT_OFFSET arithmetic (slow-domain combinational).
//   2. ctrl_state → duty-value encoding (OPEN / RUNNING / BRAKE).
//   3. Instantiation of the selected pwm_phase_correct_<variant> module
//      based on the `define VARIANT_* passed at synthesis time.
//
// The slow-domain cmd_parser holds ctrl_state at OPEN for one full PWM
// period after every state change (shoot-through guard). During that
// OPEN window, the gate unit emits the OPEN encoding to the fast-domain
// PWM module:
//
//       gate_high = (counter <  duty - dt/2)
//       gate_low  = (counter >= duty + dt/2)
//
//     OPEN:     duty_*_minus_dt_half = 0,    duty_*_plus_dt_half = 2047
//               → gate_high always 0; gate_low fires only at the 2-cycle
//                 peak of the triangle. Essentially OPEN.
//     BRAKE:    duty_*_minus_dt_half = 0,    duty_*_plus_dt_half = 0
//               → gate_high = 0; gate_low = 1 throughout. BRAKE.
//     RUNNING:  duty_*_minus_dt_half = sat(duty - dt/2)
//               duty_*_plus_dt_half  = sat(duty + dt/2)
//
// At 82.5 MHz with the 11-bit counter, DEAD_TIME = 50 fast cycles ≈
// 606 ns. Tunable via the DEAD_TIME parameter.

module pwm_gate_unit #(
    parameter [10:0] DEAD_TIME = 11'd50
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  ctrl_state,                // 0=OPEN, 1=RUNNING, 2=BRAKE
    input  wire [10:0] duty_u, duty_v, duty_w,    // from spwm_tdm
    output wire        sync,
    output wire        gate_uh, gate_ul,
    output wire        gate_vh, gate_vl,
    output wire        gate_wh, gate_wl
);

    localparam [10:0] DT_OFFSET = DEAD_TIME >> 1;  // dt/2

    // Saturating duty ± DT_OFFSET (slow-domain combinational).
    // The high-side compare uses a 12-bit operand to avoid 11-bit overflow.
    wire [10:0] dt_lim_low  = DT_OFFSET;
    wire [11:0] dt_lim_high = 12'd2047 - {1'b0, DT_OFFSET};

    wire [10:0] run_u_minus = (duty_u >= dt_lim_low)
                              ? duty_u - DT_OFFSET : 11'd0;
    wire [10:0] run_u_plus  = ({1'b0, duty_u} <= dt_lim_high)
                              ? duty_u + DT_OFFSET : 11'd2047;

    wire [10:0] run_v_minus = (duty_v >= dt_lim_low)
                              ? duty_v - DT_OFFSET : 11'd0;
    wire [10:0] run_v_plus  = ({1'b0, duty_v} <= dt_lim_high)
                              ? duty_v + DT_OFFSET : 11'd2047;

    wire [10:0] run_w_minus = (duty_w >= dt_lim_low)
                              ? duty_w - DT_OFFSET : 11'd0;
    wire [10:0] run_w_plus  = ({1'b0, duty_w} <= dt_lim_high)
                              ? duty_w + DT_OFFSET : 11'd2047;

    // ctrl_state → duty-value encoding.
    // RUNNING uses computed thresholds; BRAKE = (0, 0); OPEN = (0, 2047).
    wire is_running = (ctrl_state == 2'd1);
    wire is_brake   = (ctrl_state == 2'd2);

    wire [10:0] eff_u_minus = is_running ? run_u_minus : 11'd0;
    wire [10:0] eff_v_minus = is_running ? run_v_minus : 11'd0;
    wire [10:0] eff_w_minus = is_running ? run_w_minus : 11'd0;

    wire [10:0] eff_u_plus  = is_running ? run_u_plus
                            : is_brake   ? 11'd0
                            :              11'd2047;
    wire [10:0] eff_v_plus  = is_running ? run_v_plus
                            : is_brake   ? 11'd0
                            :              11'd2047;
    wire [10:0] eff_w_plus  = is_running ? run_w_plus
                            : is_brake   ? 11'd0
                            :              11'd2047;

`ifdef VARIANT_PIPE
    pwm_phase_correct_pipelined u_pwm (
        .clk (clk), .rst_n (rst_n),
        .duty_u_minus_dt_half (eff_u_minus), .duty_u_plus_dt_half (eff_u_plus),
        .duty_v_minus_dt_half (eff_v_minus), .duty_v_plus_dt_half (eff_v_plus),
        .duty_w_minus_dt_half (eff_w_minus), .duty_w_plus_dt_half (eff_w_plus),
        .sync (sync),
        .gate_uh (gate_uh), .gate_ul (gate_ul),
        .gate_vh (gate_vh), .gate_vl (gate_vl),
        .gate_wh (gate_wh), .gate_wl (gate_wl)
    );
`elsif VARIANT_TWIN
    pwm_phase_correct_twin u_pwm (
        .clk (clk), .rst_n (rst_n),
        .duty_u_minus_dt_half (eff_u_minus), .duty_u_plus_dt_half (eff_u_plus),
        .duty_v_minus_dt_half (eff_v_minus), .duty_v_plus_dt_half (eff_v_plus),
        .duty_w_minus_dt_half (eff_w_minus), .duty_w_plus_dt_half (eff_w_plus),
        .sync (sync),
        .gate_uh (gate_uh), .gate_ul (gate_ul),
        .gate_vh (gate_vh), .gate_vl (gate_vl),
        .gate_wh (gate_wh), .gate_wl (gate_wl)
    );
`elsif VARIANT_BRAMS
    pwm_phase_correct_brams u_pwm (
        .clk (clk), .rst_n (rst_n),
        .duty_u_minus_dt_half (eff_u_minus), .duty_u_plus_dt_half (eff_u_plus),
        .duty_v_minus_dt_half (eff_v_minus), .duty_v_plus_dt_half (eff_v_plus),
        .duty_w_minus_dt_half (eff_w_minus), .duty_w_plus_dt_half (eff_w_plus),
        .sync (sync),
        .gate_uh (gate_uh), .gate_ul (gate_ul),
        .gate_vh (gate_vh), .gate_vl (gate_vl),
        .gate_wh (gate_wh), .gate_wl (gate_wl)
    );
`else
    // No variant define passed — synthesis will fail with unconnected
    // module-output wires. Build with one of:
    //   yosys -D VARIANT_PIPE   ...
    //   yosys -D VARIANT_TWIN   ...
    //   yosys -D VARIANT_BRAMS  ...
`endif

endmodule
