// Phase-Correct PWM + 3-Channel Dead-Time Gate Generator
// Twin-counter variant — one incrementing counter (counter_up_reg) and
// one decrementing counter (counter_down_reg) run in parallel, satisfying
// the invariant counter_up + counter_down ≡ 2047. Direction is tracked
// internally (no external addr/adder needed for the compare path); a
// registered mux picks the active counter into counter_reg, isolating
// the mux from s1's combinational input path.
//
// Both halves of the triangle share thresholds, so only one comparator
// lane per phase is needed (mirroring the brams variant's structure).
//
// Symmetric dead-time, centered on the duty boundary:
//
//   gate_high = (counter <  duty - dt/2)
//   gate_low  = (counter >= duty + dt/2)   = ~(counter < duty + dt/2)
//
// Duty thresholds are latched at sync (trough).
//
// Pipeline depth counter_up/down -> gate FF: 4 stages
//   counter_up/down -> counter_reg -> s1 -> s2 -> gate

module pwm_phase_correct_twin(
    input  wire        clk,
    input  wire        rst_n,
    // Direct duty inputs from slow domain (pre-saturated duty±dt/2).
    input  wire [10:0] duty_u_minus_dt_half,  duty_u_plus_dt_half,
    input  wire [10:0] duty_v_minus_dt_half,  duty_v_plus_dt_half,
    input  wire [10:0] duty_w_minus_dt_half,  duty_w_plus_dt_half,
    output reg         sync,
    output reg         gate_uh,
    output reg         gate_ul,
    output reg         gate_vh,
    output reg         gate_vl,
    output reg         gate_wh,
    output reg         gate_wl
);

    // ============================================================
    // Duty thresholds latched at sync (= trough). Stable for the whole
    // PWM period regardless of when spwm_tdm finishes updating duties.
    // ============================================================
    reg [10:0] duty_u_minus_dt_half_reg;
    reg [10:0] duty_u_plus_dt_half_reg;
    reg [10:0] duty_v_minus_dt_half_reg;
    reg [10:0] duty_v_plus_dt_half_reg;
    reg [10:0] duty_w_minus_dt_half_reg;
    reg [10:0] duty_w_plus_dt_half_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_u_minus_dt_half_reg <= 11'd0;
            duty_u_plus_dt_half_reg  <= 11'd0;
            duty_v_minus_dt_half_reg <= 11'd0;
            duty_v_plus_dt_half_reg  <= 11'd0;
            duty_w_minus_dt_half_reg <= 11'd0;
            duty_w_plus_dt_half_reg  <= 11'd0;
        end else if (sync) begin
            duty_u_minus_dt_half_reg <= duty_u_minus_dt_half;
            duty_u_plus_dt_half_reg  <= duty_u_plus_dt_half;
            duty_v_minus_dt_half_reg <= duty_v_minus_dt_half;
            duty_v_plus_dt_half_reg  <= duty_v_plus_dt_half;
            duty_w_minus_dt_half_reg <= duty_w_minus_dt_half;
            duty_w_plus_dt_half_reg  <= duty_w_plus_dt_half;
        end
    end

    // ============================================================
    // Independent up/down counters + internal direction tracking.
    // Invariant: counter_up_reg + counter_down_reg ≡ 2047 (mod 2048).
    // dir toggles whenever counter_up_reg wraps (2047 -> 0).
    // ============================================================
    reg [10:0] counter_up_reg;
    reg [10:0] counter_down_reg;
    reg        dir;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_up_reg   <= 11'd0;
            counter_down_reg <= 11'd2047;
            dir              <= 1'b0;
        end else begin
            counter_up_reg   <= counter_up_reg   + 11'd1;
            counter_down_reg <= counter_down_reg - 11'd1;
            if (counter_up_reg == 11'd2047) dir <= ~dir;
        end
    end

    // Single triangle counter — REGISTERED mux of the active half.
    // The register isolates the mux from s1's combinational input path
    // (otherwise the mux+compare in one LUT level blows fast-clock timing).
    // Triangle sequence: 0,1,...,2047,2047,2046,...,1,0,0,1,...
    reg [10:0] counter_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) counter_reg <= 11'd0;
        else        counter_reg <= dir ? counter_down_reg : counter_up_reg;
    end

    wire [10:0] counter = counter_reg;

    // ============================================================
    // sync: counter_up_reg has just hit 0 with dir == 0 (start of new
    // UP half = trough). Registered twice to absorb the extra pipeline
    // stage introduced by counter_reg, keeping sync aligned with the
    // moment counter == 0 reaches s1.
    // ============================================================
    reg sync_pre;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync_pre <= 1'b0;
        else        sync_pre <= (counter_up_reg == 11'd0) && !dir;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= sync_pre;
    end

    // ============================================================
    // Counter / duty splits (6 lsb + 5 msb).
    // ============================================================
    wire [4:0] counter_hi = counter[10:6];
    wire [5:0] counter_lo = counter[5:0];

    wire [4:0] duty_u_dmh_hi = duty_u_minus_dt_half_reg[10:6];
    wire [5:0] duty_u_dmh_lo = duty_u_minus_dt_half_reg[5:0];
    wire [4:0] duty_u_dph_hi = duty_u_plus_dt_half_reg [10:6];
    wire [5:0] duty_u_dph_lo = duty_u_plus_dt_half_reg [5:0];

    wire [4:0] duty_v_dmh_hi = duty_v_minus_dt_half_reg[10:6];
    wire [5:0] duty_v_dmh_lo = duty_v_minus_dt_half_reg[5:0];
    wire [4:0] duty_v_dph_hi = duty_v_plus_dt_half_reg [10:6];
    wire [5:0] duty_v_dph_lo = duty_v_plus_dt_half_reg [5:0];

    wire [4:0] duty_w_dmh_hi = duty_w_minus_dt_half_reg[10:6];
    wire [5:0] duty_w_dmh_lo = duty_w_minus_dt_half_reg[5:0];
    wire [4:0] duty_w_dph_hi = duty_w_plus_dt_half_reg [10:6];
    wire [5:0] duty_w_dph_lo = duty_w_plus_dt_half_reg [5:0];

    // ============================================================
    // Phase U pipelined dead-time
    // ============================================================
    reg s1u_hi_lt_dmh, s1u_hi_eq_dmh, s1u_lo_lt_dmh;
    reg s1u_hi_lt_dph, s1u_hi_eq_dph, s1u_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1u_hi_lt_dmh <= 1'b0; s1u_hi_eq_dmh <= 1'b0; s1u_lo_lt_dmh <= 1'b0;
            s1u_hi_lt_dph <= 1'b0; s1u_hi_eq_dph <= 1'b0; s1u_lo_lt_dph <= 1'b0;
        end else begin
            s1u_hi_lt_dmh <= (counter_hi <  duty_u_dmh_hi);
            s1u_hi_eq_dmh <= (counter_hi == duty_u_dmh_hi);
            s1u_lo_lt_dmh <= (counter_lo <  duty_u_dmh_lo);
            s1u_hi_lt_dph <= (counter_hi <  duty_u_dph_hi);
            s1u_hi_eq_dph <= (counter_hi == duty_u_dph_hi);
            s1u_lo_lt_dph <= (counter_lo <  duty_u_dph_lo);
        end
    end

    reg s2u_lt_dmh, s2u_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2u_lt_dmh <= 1'b0;
            s2u_lt_dph <= 1'b0;
        end else begin
            s2u_lt_dmh <= s1u_hi_lt_dmh | (s1u_hi_eq_dmh & s1u_lo_lt_dmh);
            s2u_lt_dph <= s1u_hi_lt_dph | (s1u_hi_eq_dph & s1u_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_uh <= 1'b0;
            gate_ul <= 1'b0;
        end else begin
            gate_uh <=  s2u_lt_dmh;
            gate_ul <= ~s2u_lt_dph;
        end
    end

    // ============================================================
    // Phase V pipelined dead-time
    // ============================================================
    reg s1v_hi_lt_dmh, s1v_hi_eq_dmh, s1v_lo_lt_dmh;
    reg s1v_hi_lt_dph, s1v_hi_eq_dph, s1v_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1v_hi_lt_dmh <= 1'b0; s1v_hi_eq_dmh <= 1'b0; s1v_lo_lt_dmh <= 1'b0;
            s1v_hi_lt_dph <= 1'b0; s1v_hi_eq_dph <= 1'b0; s1v_lo_lt_dph <= 1'b0;
        end else begin
            s1v_hi_lt_dmh <= (counter_hi <  duty_v_dmh_hi);
            s1v_hi_eq_dmh <= (counter_hi == duty_v_dmh_hi);
            s1v_lo_lt_dmh <= (counter_lo <  duty_v_dmh_lo);
            s1v_hi_lt_dph <= (counter_hi <  duty_v_dph_hi);
            s1v_hi_eq_dph <= (counter_hi == duty_v_dph_hi);
            s1v_lo_lt_dph <= (counter_lo <  duty_v_dph_lo);
        end
    end

    reg s2v_lt_dmh, s2v_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2v_lt_dmh <= 1'b0;
            s2v_lt_dph <= 1'b0;
        end else begin
            s2v_lt_dmh <= s1v_hi_lt_dmh | (s1v_hi_eq_dmh & s1v_lo_lt_dmh);
            s2v_lt_dph <= s1v_hi_lt_dph | (s1v_hi_eq_dph & s1v_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_vh <= 1'b0;
            gate_vl <= 1'b0;
        end else begin
            gate_vh <=  s2v_lt_dmh;
            gate_vl <= ~s2v_lt_dph;
        end
    end

    // ============================================================
    // Phase W pipelined dead-time
    // ============================================================
    reg s1w_hi_lt_dmh, s1w_hi_eq_dmh, s1w_lo_lt_dmh;
    reg s1w_hi_lt_dph, s1w_hi_eq_dph, s1w_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1w_hi_lt_dmh <= 1'b0; s1w_hi_eq_dmh <= 1'b0; s1w_lo_lt_dmh <= 1'b0;
            s1w_hi_lt_dph <= 1'b0; s1w_hi_eq_dph <= 1'b0; s1w_lo_lt_dph <= 1'b0;
        end else begin
            s1w_hi_lt_dmh <= (counter_hi <  duty_w_dmh_hi);
            s1w_hi_eq_dmh <= (counter_hi == duty_w_dmh_hi);
            s1w_lo_lt_dmh <= (counter_lo <  duty_w_dmh_lo);
            s1w_hi_lt_dph <= (counter_hi <  duty_w_dph_hi);
            s1w_hi_eq_dph <= (counter_hi == duty_w_dph_hi);
            s1w_lo_lt_dph <= (counter_lo <  duty_w_dph_lo);
        end
    end

    reg s2w_lt_dmh, s2w_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2w_lt_dmh <= 1'b0;
            s2w_lt_dph <= 1'b0;
        end else begin
            s2w_lt_dmh <= s1w_hi_lt_dmh | (s1w_hi_eq_dmh & s1w_lo_lt_dmh);
            s2w_lt_dph <= s1w_hi_lt_dph | (s1w_hi_eq_dph & s1w_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_wh <= 1'b0;
            gate_wl <= 1'b0;
        end else begin
            gate_wh <=  s2w_lt_dmh;
            gate_wl <= ~s2w_lt_dph;
        end
    end

endmodule
