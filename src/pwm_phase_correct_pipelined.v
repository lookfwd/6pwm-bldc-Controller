// Phase-Correct PWM + 3-Channel Dead-Time Gate Generator
// Pipelined variant — single triangle counter formed by a registered
// mux of address halves at the counter stage. Both halves of the
// triangle share thresholds, so only one comparator lane per phase is
// needed (mirroring the brams variant's structure).
//
// Symmetric dead-time, centered on the duty boundary:
//
//   gate_high = (counter <  duty - dt/2)
//   gate_low  = (counter >= duty + dt/2)   = ~(counter < duty + dt/2)
//
// Duty thresholds are latched at sync (trough) so they remain stable
// across the entire PWM period regardless of when spwm_tdm finishes.
//
// Pipeline depth addr -> gate FF: 4 stages
//   addr -> counter_reg -> s1 -> s2 -> gate
//
// Companion modules: pwm_phase_correct_twin, pwm_phase_correct_brams.

module pwm_phase_correct_pipelined(
    input  wire        clk,
    input  wire        rst_n,
    // Direct duty inputs from slow domain (pre-saturated duty±dt/2).
    // *_minus is 11-bit (0..2047, "never on" at 0).
    // *_plus is 12-bit (0..2048, "never on" at 2048).
    input  wire [10:0] duty_u_minus_dt_half,
    input  wire [11:0] duty_u_plus_dt_half,
    input  wire [10:0] duty_v_minus_dt_half,
    input  wire [11:0] duty_v_plus_dt_half,
    input  wire [10:0] duty_w_minus_dt_half,
    input  wire [11:0] duty_w_plus_dt_half,
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
    reg [11:0] duty_u_plus_dt_half_reg;
    reg [10:0] duty_v_minus_dt_half_reg;
    reg [11:0] duty_v_plus_dt_half_reg;
    reg [10:0] duty_w_minus_dt_half_reg;
    reg [11:0] duty_w_plus_dt_half_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_u_minus_dt_half_reg <= 11'd0;
            duty_u_plus_dt_half_reg  <= 12'd0;
            duty_v_minus_dt_half_reg <= 11'd0;
            duty_v_plus_dt_half_reg  <= 12'd0;
            duty_w_minus_dt_half_reg <= 11'd0;
            duty_w_plus_dt_half_reg  <= 12'd0;
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
    // Free-running 12-bit address counter.
    // ============================================================
    reg [11:0] addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  addr <= 12'd0;
        else         addr <= addr + 12'd1;
    end

    // ============================================================
    // Single triangle counter — registered mux of address halves.
    //   UP half   (addr[11]=0): counter = addr[10:0]   = 0..2047
    //   DOWN half (addr[11]=1): counter = ~addr[10:0]  = 2047..0
    // Triangle sequence: 0,1,...,2047,2047,2046,...,1,0,0,1,...
    // ============================================================
    reg [10:0] counter_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  counter_reg <= 11'd0;
        else         counter_reg <= addr[11] ? ~addr[10:0] : addr[10:0];
    end

    wire [10:0] counter = counter_reg;

    // ============================================================
    // sync: two-stage register pipeline matching twin's structure.
    // sync_pre registers (addr == 0); sync is one cycle later, so it
    // fires the cycle after counter_reg hits the trough — which is when
    // the duty-latch picks up the new period's thresholds.
    // ============================================================
    reg sync_pre;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync_pre <= 1'b0;
        else        sync_pre <= (addr == 12'd0);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= sync_pre;
    end

    // ============================================================
    // Counter / duty splits.
    //   Both duty_minus and the LOW 11 bits of duty_plus split 5 hi + 6 lo,
    //   so the s1 comparators are identical-sized between dmh and dph.
    //   The 12th bit of duty_plus is a "force off" sentinel and is OR'd
    //   directly into the s2 reduction (see Phase U/V/W blocks below).
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
    //   stage 1: hi-lt + hi-eq + lo-lt for two RHSs (dmh, dph)
    //   stage 2: combine to two final lt results
    //   stage 3: gate FFs
    //     gate_uh =  lt_dmh
    //     gate_ul = ~lt_dph
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
            s2u_lt_dmh <=                                s1u_hi_lt_dmh | (s1u_hi_eq_dmh & s1u_lo_lt_dmh);
            s2u_lt_dph <= duty_u_plus_dt_half_reg[11] | s1u_hi_lt_dph | (s1u_hi_eq_dph & s1u_lo_lt_dph);
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
            s2v_lt_dmh <=                                s1v_hi_lt_dmh | (s1v_hi_eq_dmh & s1v_lo_lt_dmh);
            s2v_lt_dph <= duty_v_plus_dt_half_reg[11] | s1v_hi_lt_dph | (s1v_hi_eq_dph & s1v_lo_lt_dph);
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
            s2w_lt_dmh <=                                s1w_hi_lt_dmh | (s1w_hi_eq_dmh & s1w_lo_lt_dmh);
            s2w_lt_dph <= duty_w_plus_dt_half_reg[11] | s1w_hi_lt_dph | (s1w_hi_eq_dph & s1w_lo_lt_dph);
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
