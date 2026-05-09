// Top-Level Module — iCE40 Three-Phase SPWM Motor Controller
//
// Two synchronously-related clock domains, 4:1 ratio:
//   clk_fast (~82.5 MHz):  pwm_phase_correct, deadtime ×3, gate/adc_sync outputs
//   clk_slow (~20.6 MHz):  uart_rx, cmd_parser, sine_lut, spwm_tdm, heartbeat
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
    output wire       adc_sync,       // CONVST trigger for ADS8319
    output wire       led_heartbeat
);

    // ---- Fast Clock (PLL: 12 MHz → 82.5 MHz) ----
    wire clk_fast, pll_locked;

    pll u_pll (
        .clk_12m  (clk_12m),
        .clk_82m5 (clk_fast),
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
    wire [10:0] pwm_counter;
    wire        pwm_dir;

    pwm_phase_correct u_pwm_cnt (
        .clk       (clk_fast),
        .rst_n     (rst_n),
        .enable    (1'b1),
        .counter   (pwm_counter),
        .sync      (pwm_sync_fast),
        .direction (pwm_dir)
    );

    // ---- UART Receiver (slow domain) ----
    // CLK_DIV recomputed for 20.625 MHz: 20.625e6 / 115200 ≈ 179.
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_DIV(179)) u_uart (
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
    // 16 slow cycles × ~48.5 ns ≈ 776 ns per PWM period — well under 50 µs.
    wire [10:0] duty_u, duty_v, duty_w;
    wire        running = (ctrl_state == 2'd1);

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
    localparam DEAD_TIME = 8'd50;  // ~606 ns at 82.5 MHz

    wire gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;

    deadtime u_dt_u (
        .clk        (clk_fast),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_u),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_uh),
        .gate_low   (gate_ul)
    );

    deadtime u_dt_v (
        .clk        (clk_fast),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_v),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_vh),
        .gate_low   (gate_vl)
    );

    deadtime u_dt_w (
        .clk        (clk_fast),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_w),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_wh),
        .gate_low   (gate_wl)
    );

    assign gate     = {gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl};
    assign adc_sync = pwm_sync_fast;

    // ---- Heartbeat LED (slow domain — ~2.5 Hz blink at 20.625 MHz) ----
    reg [22:0] hb_cnt;
    always @(posedge clk_slow or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 23'd0;
        else        hb_cnt <= hb_cnt + 23'd1;
    end
    assign led_heartbeat = hb_cnt[22];

endmodule
