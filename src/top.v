// Top-Level Module — iCE40 Three-Phase SPWM Motor Controller
//
// Two synchronously-related clock domains, 4:1 ratio:
//   clk_fast (~50.25 MHz): pwm_phase_correct, deadtime ×3, gate outputs
//   clk_slow (~12.56 MHz): uart_rx, cmd_parser, sine_lut, spwm_tdm, heartbeat
//
// 10-bit phase-correct counter at 50.25 MHz → PWM frequency ≈ 24.5 kHz
// (above audible threshold, comfortable margin for timing closure).
//
// CDC handling:
//   pwm_sync (fast→slow): stretched to 4 fast cycles so the slow domain
//   captures it exactly once per PWM period.
//
//   duty_u/v/w, ctrl_state (slow→fast): stable across the entire PWM
//   period (~4080 fast cycles); synchronously-related clocks rule out
//   metastability, so direct sampling is safe.

module top (
    input  wire       clk_12m,
    input  wire       uart_rx,
    output wire [5:0] gate,           // {UH, UL, VH, VL, WH, WL}
    output wire       led_heartbeat
);

    // ---- Fast Clock (PLL: 12 MHz → 50.25 MHz) ----
    wire clk_fast, pll_locked;

    pll u_pll (
        .clk_12m (clk_12m),
        .clk_50m (clk_fast),
        .locked  (pll_locked)
    );

    wire rst_n = pll_locked;

    // ---- Slow Clock: clk_fast / 4 (~12.5625 MHz) via fabric divider + global buffer ----
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
    // The 1-cycle pulse from pwm_phase_correct is too short for clk_slow to
    // sample reliably; widening it to one full slow period guarantees exactly
    // one slow-edge capture per PWM period.
    wire pwm_sync_fast;
    reg  [2:0] pwm_sync_pipe;
    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) pwm_sync_pipe <= 3'd0;
        else        pwm_sync_pipe <= {pwm_sync_pipe[1:0], pwm_sync_fast};
    end
    wire pwm_sync_wide =
        pwm_sync_fast | pwm_sync_pipe[0] | pwm_sync_pipe[1] | pwm_sync_pipe[2];

    // ---- PWM Counter (fast domain) ----
    wire [9:0] pwm_counter;
    wire       pwm_dir;

    pwm_phase_correct u_pwm_cnt (
        .clk       (clk_fast),
        .rst_n     (rst_n),
        .enable    (1'b1),
        .counter   (pwm_counter),
        .sync      (pwm_sync_fast),
        .direction (pwm_dir)
    );

    // ---- UART Receiver (slow domain) ----
    // CLK_DIV at 12.5625 MHz: 12.5625e6 / 115200 ≈ 109.05. Use 112 so
    // SAMPLE_DIV = 112/16 = 7 gives bit period 112 cycles ≈ 8.91 µs vs
    // nominal 8.68 µs — error +2.7%, well inside UART tolerance.
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_DIV(112)) u_uart (
        .clk   (clk_slow),
        .rst_n (rst_n),
        .rx    (uart_rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    // ---- Command Parser & Shadow Registers (slow domain) ----
    wire [1:0]  ctrl_state;
    wire [31:0] phase_inc;
    wire [7:0]  amplitude;

    cmd_parser u_cmd (
        .clk       (clk_slow),
        .rst_n     (rst_n),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .pwm_sync  (pwm_sync_wide),
        .ctrl_state(ctrl_state),
        .phase_inc (phase_inc),
        .amplitude (amplitude)
    );

    // ---- Sine LUT (slow domain — read by spwm_tdm) ----
    wire [10:0] lut_addr;
    wire [15:0] lut_data;

    sine_lut u_lut (
        .clk  (clk_slow),
        .addr (lut_addr),
        .data (lut_data)
    );

    // ---- TDM State Machine (slow domain) ----
    // 16 slow cycles × ~80 ns ≈ 1.27 µs per PWM period — well under 41 µs.
    wire [9:0] duty_u, duty_v, duty_w;
    wire       running = (ctrl_state == 2'd1);

    spwm_tdm u_tdm (
        .clk       (clk_slow),
        .rst_n     (rst_n),
        .enable    (running),
        .pwm_sync  (pwm_sync_wide),
        .phase_inc (phase_inc),
        .amplitude (amplitude),
        .lut_data  (lut_data),
        .lut_addr  (lut_addr),
        .duty_u    (duty_u),
        .duty_v    (duty_v),
        .duty_w    (duty_w)
    );

    // ---- Dead-Time Insertion (3 channels, fast domain) ----
    // Dead-time is folded into the duty thresholds upstream rather than enforced
    // by counters in the fast domain — eliminates the deep CE network that was
    // previously the timing bottleneck. cmd_parser inserts an OPEN sandwich
    // around any ctrl_state change to cover state-transition shoot-through.
    localparam [9:0] DEAD_TIME = 10'd50;  // ~995 ns at 50.25 MHz

    // Saturating duty ± dead_time per phase. These are slow-domain combinational
    // signals (stable for the entire PWM period since duty_X is).
    wire [9:0] duty_u_minus_dt = (duty_u >= DEAD_TIME)
                                 ? duty_u - DEAD_TIME : 10'd0;
    wire [9:0] duty_u_plus_dt  = (duty_u + DEAD_TIME > 10'd1023)
                                 ? 10'd1023 : duty_u + DEAD_TIME;
    wire [9:0] duty_v_minus_dt = (duty_v >= DEAD_TIME)
                                 ? duty_v - DEAD_TIME : 10'd0;
    wire [9:0] duty_v_plus_dt  = (duty_v + DEAD_TIME > 10'd1023)
                                 ? 10'd1023 : duty_v + DEAD_TIME;
    wire [9:0] duty_w_minus_dt = (duty_w >= DEAD_TIME)
                                 ? duty_w - DEAD_TIME : 10'd0;
    wire [9:0] duty_w_plus_dt  = (duty_w + DEAD_TIME > 10'd1023)
                                 ? 10'd1023 : duty_w + DEAD_TIME;

    wire gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;

    deadtime u_dt_u (
        .clk           (clk_fast),
        .rst_n         (rst_n),
        .ctrl_state    (ctrl_state),
        .direction     (pwm_dir),
        .counter       (pwm_counter),
        .duty          (duty_u),
        .duty_minus_dt (duty_u_minus_dt),
        .duty_plus_dt  (duty_u_plus_dt),
        .gate_high     (gate_uh),
        .gate_low      (gate_ul)
    );

    deadtime u_dt_v (
        .clk           (clk_fast),
        .rst_n         (rst_n),
        .ctrl_state    (ctrl_state),
        .direction     (pwm_dir),
        .counter       (pwm_counter),
        .duty          (duty_v),
        .duty_minus_dt (duty_v_minus_dt),
        .duty_plus_dt  (duty_v_plus_dt),
        .gate_high     (gate_vh),
        .gate_low      (gate_vl)
    );

    deadtime u_dt_w (
        .clk           (clk_fast),
        .rst_n         (rst_n),
        .ctrl_state    (ctrl_state),
        .direction     (pwm_dir),
        .counter       (pwm_counter),
        .duty          (duty_w),
        .duty_minus_dt (duty_w_minus_dt),
        .duty_plus_dt  (duty_w_plus_dt),
        .gate_high     (gate_wh),
        .gate_low      (gate_wl)
    );

    assign gate = {gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl};

    // ---- Heartbeat LED (slow domain — ~1.5 Hz blink at 12.5625 MHz) ----
    reg [22:0] hb_cnt;
    always @(posedge clk_slow or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 23'd0;
        else        hb_cnt <= hb_cnt + 23'd1;
    end
    assign led_heartbeat = hb_cnt[22];

endmodule
