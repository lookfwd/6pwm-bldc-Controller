// Top-Level Module — iCE40 Three-Phase SPWM Motor Controller
//
// Two synchronously-related clock domains, 4:1 ratio:
//   clk_fast (~82.5 MHz): pwm_phase_correct_<variant>, gate outputs
//   clk_slow (~20.6 MHz): uart_rx, cmd_parser, sine_lut, spwm_tdm,
//                          pwm_gate_unit (saturating arith + state encoding),
//                          heartbeat
//
// 11-bit phase-correct counter at 82.5 MHz → PWM frequency ≈ 20.14 kHz
// (above audible threshold; period 4096 fast cycles ≈ 49.65 µs).
//
// CDC handling:
//   pwm_sync (fast→slow): stretched to 4 fast cycles so the slow domain
//   captures it exactly once per PWM period.
//
//   duty_u/v/w threshold bundle (slow→fast): stable across the entire
//   PWM period (~4096 fast cycles) since duty values change only at
//   sync; synchronously-related 4:1 clocks rule out metastability, so
//   direct sampling by the fast domain is safe.
//
// Variant selection: pass exactly one of these defines at synthesis time
// to choose the counter-generation flavor inside pwm_phase_correct:
//   VARIANT_PIPE   — addr counter + registered triangle mux
//   VARIANT_BRAMS  — BRAM-materialized triangle table
// The Makefile exposes these as `make top_pipe` and `make top_brams`.

module top (
    input  wire       clk_12m,
    input  wire       uart_rx,
    output wire       uart_tx,         // debug telemetry back to host
    output wire [5:0] gate,            // {UH, UL, VH, VL, WH, WL}
    output wire       led_heartbeat
);

    // ---- Fast Clock (PLL: 12 MHz → 82.5 MHz) ----
    wire clk_fast, pll_locked;

    pll u_pll (
        .clk_12m  (clk_12m),
        .clk_fast (clk_fast),
        .locked   (pll_locked)
    );

    wire rst_n = pll_locked;

    // ---- Slow Clock: clk_fast / 4 (~20.625 MHz) via fabric divider + global buffer ----
    reg [1:0] gear_cnt;
    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) gear_cnt <= 2'd0;
        else        gear_cnt <= gear_cnt + 2'd1;
    end

    wire clk_slow;
    SB_GB u_gb_slow (
        .USER_SIGNAL_TO_GLOBAL_BUFFER (gear_cnt[1]),
        .GLOBAL_BUFFER_OUTPUT          (clk_slow)
    );

    // ---- pwm_sync stretched to 4 fast cycles for slow-domain capture ----
    // The pwm_phase_correct variants emit a 1-cycle sync; widening it
    // to one full slow period guarantees exactly one slow-edge capture
    // per PWM period.
    wire pwm_sync_fast;
    reg  [2:0] pwm_sync_pipe;
    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) pwm_sync_pipe <= 3'd0;
        else        pwm_sync_pipe <= {pwm_sync_pipe[1:0], pwm_sync_fast};
    end
    wire pwm_sync_wide =
        pwm_sync_fast | pwm_sync_pipe[0] | pwm_sync_pipe[1] | pwm_sync_pipe[2];

    // ---- UART Receiver (slow domain) ----
    // CLK_DIV at 20.625 MHz: 20.625e6 / 115200 ≈ 179.0. Use 176 so
    // SAMPLE_DIV = 176/16 = 11 gives bit period 176 cycles ≈ 8.53 µs vs
    // nominal 8.68 µs — error -1.7%, well inside UART tolerance.
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_DIV(176)) u_uart (
        .clk   (clk_slow),
        .rst_n (rst_n),
        .rx    (uart_rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    // ---- Command Parser & Shadow Registers (slow domain) ----
    wire [31:0] phase_inc;
    wire [7:0]  amplitude;

    cmd_parser u_cmd (
        .clk       (clk_slow),
        .rst_n     (rst_n),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .pwm_sync  (pwm_sync_wide),
        .phase_inc (phase_inc),
        .amplitude (amplitude)
    );

    // ---- TDM State Machine (slow domain; sine_lut is instantiated inside) ----
    // duty_minus = 11-bit (gate_high never-fires at 0)
    // duty_plus  = 12-bit (gate_low never-fires at 2048; one wider so the
    //              comparison against 11-bit counter cleanly evaluates false)
    wire [10:0] u_minus, v_minus, w_minus;
    wire [11:0] u_plus,  v_plus,  w_plus;

    spwm_tdm #(.HALF_DEAD_TIME(11'd25)) u_tdm (
        .clk       (clk_slow),
        .rst_n     (rst_n),
        .pwm_sync  (pwm_sync_wide),
        .phase_inc (phase_inc),
        .amplitude (amplitude),
        .u_minus   (u_minus),
        .u_plus    (u_plus),
        .v_minus   (v_minus),
        .v_plus    (v_plus),
        .w_minus   (w_minus),
        .w_plus    (w_plus)
    );
	
	wire gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;

    // The VARIANT_* define passed at synthesis selects the counter-
    // generation flavor inside pwm_phase_correct.
    pwm_phase_correct u_pwm (
        .clk (clk_fast), .rst_n (rst_n),
        .duty_u_minus_dt_half (u_minus), .duty_u_plus_dt_half (u_plus),
        .duty_v_minus_dt_half (v_minus), .duty_v_plus_dt_half (v_plus),
        .duty_w_minus_dt_half (w_minus), .duty_w_plus_dt_half (w_plus),
        .sync (pwm_sync_fast),
        .gate_uh (gate_uh), .gate_ul (gate_ul),
        .gate_vh (gate_vh), .gate_vl (gate_vl),
        .gate_wh (gate_wh), .gate_wl (gate_wl)
    );

    assign gate = {gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl};

    // ---- UART TX (slow domain) ----
    // Kept instantiated so the uart_tx pin stays driven (idles high) and
    // there's a transmit path ready to wire up later. start is hard-tied
    // to 0, so no bytes ever go out.
    uart_tx #(.CLK_DIV(176)) u_uart_tx (
        .clk   (clk_slow),
        .rst_n (rst_n),
        .data  (8'h00),
        .start (1'b0),
        .ready (),
        .tx    (uart_tx)
    );

    // ---- Heartbeat LED (slow domain — ~2.46 Hz blink at 20.625 MHz) ----
    reg [22:0] hb_cnt;
    always @(posedge clk_slow or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 23'd0;
        else        hb_cnt <= hb_cnt + 23'd1;
    end
    assign led_heartbeat = hb_cnt[22];

endmodule
