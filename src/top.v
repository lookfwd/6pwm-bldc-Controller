// Top-Level Module — iCE40 Three-Phase SPWM Motor Controller
//
// Wires together: PLL, UART, Command Parser, Sine LUT,
// TDM State Machine (with integrated NCO), PWM Counter, and Dead-Time units.
//
// Gate control is handled entirely inside the deadtime modules,
// which receive ctrl_state directly. No output mux — this prevents
// shoot-through during state transitions by enforcing dead-time
// on every gate change, including RUNNING→BRAKE.

module top (
    input  wire       clk_12m,
    input  wire       uart_rx,
    output wire [5:0] gate,           // {UH, UL, VH, VL, WH, WL}
    output wire       adc_sync,      // CONVST trigger for ADS8319
    output wire       led_heartbeat
);

    // ---- Clock Generation ----
    wire clk, pll_locked;

    pll u_pll (
        .clk_12m  (clk_12m),
        .clk_82m5 (clk),
        .locked   (pll_locked)
    );

    wire rst_n = pll_locked;

    // ---- UART Receiver ----
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx u_uart (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart_rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    // ---- PWM Counter ----
    wire [10:0] pwm_counter;
    wire        pwm_sync;
    wire        pwm_dir;

    pwm_phase_correct u_pwm_cnt (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (1'b1),
        .counter   (pwm_counter),
        .sync      (pwm_sync),
        .direction (pwm_dir)
    );

    // ---- Command Parser & Shadow Registers ----
    wire [1:0]  ctrl_state;
    wire [31:0] phase_inc;
    wire [7:0]  amplitude;

    cmd_parser u_cmd (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .pwm_sync  (pwm_sync),
        .ctrl_state(ctrl_state),
        .phase_inc (phase_inc),
        .amplitude (amplitude)
    );

    // ---- Sine LUT ----
    wire [10:0] lut_addr;
    wire [15:0] lut_data;

    sine_lut u_lut (
        .clk  (clk),
        .addr (lut_addr),
        .data (lut_data)
    );

    // ---- TDM State Machine (with integrated NCO) ----
    wire [10:0] duty_u, duty_v, duty_w;
    wire        running = (ctrl_state == 2'd1);

    spwm_tdm u_tdm (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (running),
        .pwm_sync  (pwm_sync),
        .phase_inc (phase_inc),
        .amplitude (amplitude),
        .lut_data  (lut_data),
        .lut_addr  (lut_addr),
        .duty_u    (duty_u),
        .duty_v    (duty_v),
        .duty_w    (duty_w)
    );

    // ---- Dead-Time Insertion (3 channels) ----
    // Each deadtime module handles OPEN/RUNNING/BRAKE natively.
    // No output mux — prevents shoot-through on state transitions.
    localparam DEAD_TIME = 8'd50;  // ~606 ns at 82.5 MHz

    wire gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;

    deadtime u_dt_u (
        .clk        (clk),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_u),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_uh),
        .gate_low   (gate_ul)
    );

    deadtime u_dt_v (
        .clk        (clk),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_v),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_vh),
        .gate_low   (gate_vl)
    );

    deadtime u_dt_w (
        .clk        (clk),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (pwm_counter),
        .duty       (duty_w),
        .dead_time  (DEAD_TIME),
        .gate_high  (gate_wh),
        .gate_low   (gate_wl)
    );

    assign gate     = {gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl};
    assign adc_sync = pwm_sync;

    // ---- Heartbeat LED (simple toggle) ----
    reg [22:0] hb_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hb_cnt <= 23'd0;
        else
            hb_cnt <= hb_cnt + 23'd1;
    end
    assign led_heartbeat = hb_cnt[22];  // ~10 Hz blink at 82.5 MHz

endmodule
