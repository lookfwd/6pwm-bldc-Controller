// Phase-Correct PWM + 3-Channel Dead-Time Gate Generator
// Twin-counter variant — one incrementing and one decrementing counter
// fan out to the compare lanes, eliminating the shared up/down adder.
//
// Symmetric dead-time, centered on the duty boundary. The fast-domain
// compares only against duty±dt/2, never against duty itself:
//
//   gate_high = (counter <  duty - dt/2)
//   gate_low  = (counter >= duty + dt/2)   = ~(counter < duty + dt/2)
//
// Pipeline depth counter_reg -> gate FF: 3 stages. addr[11] pipelined
// through dir_d0..d1.

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
    // Free-running 12-bit address counter (used for direction + sync).
    // ============================================================
    reg [11:0] addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  addr <= 12'd0;
        else         addr <= addr + 12'd1;
    end

    // ============================================================
    // Independent up / down counters.
    // Invariant: counter_up_reg + counter_down_reg ≡ 2047 (mod 2048).
    // ============================================================
    reg [10:0] counter_up_reg;
    reg [10:0] counter_down_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_up_reg   <= 11'd0;
            counter_down_reg <= 11'd2047;
        end else begin
            counter_up_reg   <= counter_up_reg   + 11'd1;
            counter_down_reg <= counter_down_reg - 11'd1;
        end
    end

    wire [10:0] counter_up   = counter_up_reg;
    wire [10:0] counter_down = counter_down_reg;

    // ============================================================
    // Direction pipeline (2 explicit flops to match 3-stage data pipe).
    // ============================================================
    reg dir_d0, dir_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_d0 <= 1'b0;
            dir_d1 <= 1'b0;
        end else begin
            dir_d0 <= addr[11];
            dir_d1 <= dir_d0;
        end
    end

    // ============================================================
    // sync: registered (addr == 0).
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= (addr == 12'd0);
    end

    // ============================================================
    // Counter / duty splits (6 lsb + 5 msb).
    // ============================================================
    wire [4:0] counter_up_hi   = counter_up[10:6];
    wire [5:0] counter_up_lo   = counter_up[5:0];
    wire [4:0] counter_down_hi = counter_down[10:6];
    wire [5:0] counter_down_lo = counter_down[5:0];

    wire [4:0] duty_u_dmh_hi = duty_u_minus_dt_half[10:6];
    wire [5:0] duty_u_dmh_lo = duty_u_minus_dt_half[5:0];
    wire [4:0] duty_u_dph_hi = duty_u_plus_dt_half [10:6];
    wire [5:0] duty_u_dph_lo = duty_u_plus_dt_half [5:0];

    wire [4:0] duty_v_dmh_hi = duty_v_minus_dt_half[10:6];
    wire [5:0] duty_v_dmh_lo = duty_v_minus_dt_half[5:0];
    wire [4:0] duty_v_dph_hi = duty_v_plus_dt_half [10:6];
    wire [5:0] duty_v_dph_lo = duty_v_plus_dt_half [5:0];

    wire [4:0] duty_w_dmh_hi = duty_w_minus_dt_half[10:6];
    wire [5:0] duty_w_dmh_lo = duty_w_minus_dt_half[5:0];
    wire [4:0] duty_w_dph_hi = duty_w_plus_dt_half [10:6];
    wire [5:0] duty_w_dph_lo = duty_w_plus_dt_half [5:0];

    // ============================================================
    // Phase U pipelined dead-time
    // ============================================================
    reg s1u_up_hi_lt_dmh, s1u_up_hi_eq_dmh, s1u_up_lo_lt_dmh;
    reg s1u_up_hi_lt_dph, s1u_up_hi_eq_dph, s1u_up_lo_lt_dph;
    reg s1u_dn_hi_lt_dmh, s1u_dn_hi_eq_dmh, s1u_dn_lo_lt_dmh;
    reg s1u_dn_hi_lt_dph, s1u_dn_hi_eq_dph, s1u_dn_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1u_up_hi_lt_dmh <= 1'b0; s1u_up_hi_eq_dmh <= 1'b0; s1u_up_lo_lt_dmh <= 1'b0;
            s1u_up_hi_lt_dph <= 1'b0; s1u_up_hi_eq_dph <= 1'b0; s1u_up_lo_lt_dph <= 1'b0;
            s1u_dn_hi_lt_dmh <= 1'b0; s1u_dn_hi_eq_dmh <= 1'b0; s1u_dn_lo_lt_dmh <= 1'b0;
            s1u_dn_hi_lt_dph <= 1'b0; s1u_dn_hi_eq_dph <= 1'b0; s1u_dn_lo_lt_dph <= 1'b0;
        end else begin
            s1u_up_hi_lt_dmh <= (counter_up_hi   <  duty_u_dmh_hi);
            s1u_up_hi_eq_dmh <= (counter_up_hi   == duty_u_dmh_hi);
            s1u_up_lo_lt_dmh <= (counter_up_lo   <  duty_u_dmh_lo);
            s1u_up_hi_lt_dph <= (counter_up_hi   <  duty_u_dph_hi);
            s1u_up_hi_eq_dph <= (counter_up_hi   == duty_u_dph_hi);
            s1u_up_lo_lt_dph <= (counter_up_lo   <  duty_u_dph_lo);
            s1u_dn_hi_lt_dmh <= (counter_down_hi <  duty_u_dmh_hi);
            s1u_dn_hi_eq_dmh <= (counter_down_hi == duty_u_dmh_hi);
            s1u_dn_lo_lt_dmh <= (counter_down_lo <  duty_u_dmh_lo);
            s1u_dn_hi_lt_dph <= (counter_down_hi <  duty_u_dph_hi);
            s1u_dn_hi_eq_dph <= (counter_down_hi == duty_u_dph_hi);
            s1u_dn_lo_lt_dph <= (counter_down_lo <  duty_u_dph_lo);
        end
    end

    reg s2u_up_lt_dmh, s2u_up_lt_dph;
    reg s2u_dn_lt_dmh, s2u_dn_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2u_up_lt_dmh <= 1'b0; s2u_up_lt_dph <= 1'b0;
            s2u_dn_lt_dmh <= 1'b0; s2u_dn_lt_dph <= 1'b0;
        end else begin
            s2u_up_lt_dmh <= s1u_up_hi_lt_dmh | (s1u_up_hi_eq_dmh & s1u_up_lo_lt_dmh);
            s2u_up_lt_dph <= s1u_up_hi_lt_dph | (s1u_up_hi_eq_dph & s1u_up_lo_lt_dph);
            s2u_dn_lt_dmh <= s1u_dn_hi_lt_dmh | (s1u_dn_hi_eq_dmh & s1u_dn_lo_lt_dmh);
            s2u_dn_lt_dph <= s1u_dn_hi_lt_dph | (s1u_dn_hi_eq_dph & s1u_dn_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_uh <= 1'b0;
            gate_ul <= 1'b0;
        end else begin
            gate_uh <= dir_d1 ?  s2u_dn_lt_dmh :  s2u_up_lt_dmh;
            gate_ul <= dir_d1 ? ~s2u_dn_lt_dph : ~s2u_up_lt_dph;
        end
    end

    // ============================================================
    // Phase V pipelined dead-time
    // ============================================================
    reg s1v_up_hi_lt_dmh, s1v_up_hi_eq_dmh, s1v_up_lo_lt_dmh;
    reg s1v_up_hi_lt_dph, s1v_up_hi_eq_dph, s1v_up_lo_lt_dph;
    reg s1v_dn_hi_lt_dmh, s1v_dn_hi_eq_dmh, s1v_dn_lo_lt_dmh;
    reg s1v_dn_hi_lt_dph, s1v_dn_hi_eq_dph, s1v_dn_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1v_up_hi_lt_dmh <= 1'b0; s1v_up_hi_eq_dmh <= 1'b0; s1v_up_lo_lt_dmh <= 1'b0;
            s1v_up_hi_lt_dph <= 1'b0; s1v_up_hi_eq_dph <= 1'b0; s1v_up_lo_lt_dph <= 1'b0;
            s1v_dn_hi_lt_dmh <= 1'b0; s1v_dn_hi_eq_dmh <= 1'b0; s1v_dn_lo_lt_dmh <= 1'b0;
            s1v_dn_hi_lt_dph <= 1'b0; s1v_dn_hi_eq_dph <= 1'b0; s1v_dn_lo_lt_dph <= 1'b0;
        end else begin
            s1v_up_hi_lt_dmh <= (counter_up_hi   <  duty_v_dmh_hi);
            s1v_up_hi_eq_dmh <= (counter_up_hi   == duty_v_dmh_hi);
            s1v_up_lo_lt_dmh <= (counter_up_lo   <  duty_v_dmh_lo);
            s1v_up_hi_lt_dph <= (counter_up_hi   <  duty_v_dph_hi);
            s1v_up_hi_eq_dph <= (counter_up_hi   == duty_v_dph_hi);
            s1v_up_lo_lt_dph <= (counter_up_lo   <  duty_v_dph_lo);
            s1v_dn_hi_lt_dmh <= (counter_down_hi <  duty_v_dmh_hi);
            s1v_dn_hi_eq_dmh <= (counter_down_hi == duty_v_dmh_hi);
            s1v_dn_lo_lt_dmh <= (counter_down_lo <  duty_v_dmh_lo);
            s1v_dn_hi_lt_dph <= (counter_down_hi <  duty_v_dph_hi);
            s1v_dn_hi_eq_dph <= (counter_down_hi == duty_v_dph_hi);
            s1v_dn_lo_lt_dph <= (counter_down_lo <  duty_v_dph_lo);
        end
    end

    reg s2v_up_lt_dmh, s2v_up_lt_dph;
    reg s2v_dn_lt_dmh, s2v_dn_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2v_up_lt_dmh <= 1'b0; s2v_up_lt_dph <= 1'b0;
            s2v_dn_lt_dmh <= 1'b0; s2v_dn_lt_dph <= 1'b0;
        end else begin
            s2v_up_lt_dmh <= s1v_up_hi_lt_dmh | (s1v_up_hi_eq_dmh & s1v_up_lo_lt_dmh);
            s2v_up_lt_dph <= s1v_up_hi_lt_dph | (s1v_up_hi_eq_dph & s1v_up_lo_lt_dph);
            s2v_dn_lt_dmh <= s1v_dn_hi_lt_dmh | (s1v_dn_hi_eq_dmh & s1v_dn_lo_lt_dmh);
            s2v_dn_lt_dph <= s1v_dn_hi_lt_dph | (s1v_dn_hi_eq_dph & s1v_dn_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_vh <= 1'b0;
            gate_vl <= 1'b0;
        end else begin
            gate_vh <= dir_d1 ?  s2v_dn_lt_dmh :  s2v_up_lt_dmh;
            gate_vl <= dir_d1 ? ~s2v_dn_lt_dph : ~s2v_up_lt_dph;
        end
    end

    // ============================================================
    // Phase W pipelined dead-time
    // ============================================================
    reg s1w_up_hi_lt_dmh, s1w_up_hi_eq_dmh, s1w_up_lo_lt_dmh;
    reg s1w_up_hi_lt_dph, s1w_up_hi_eq_dph, s1w_up_lo_lt_dph;
    reg s1w_dn_hi_lt_dmh, s1w_dn_hi_eq_dmh, s1w_dn_lo_lt_dmh;
    reg s1w_dn_hi_lt_dph, s1w_dn_hi_eq_dph, s1w_dn_lo_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1w_up_hi_lt_dmh <= 1'b0; s1w_up_hi_eq_dmh <= 1'b0; s1w_up_lo_lt_dmh <= 1'b0;
            s1w_up_hi_lt_dph <= 1'b0; s1w_up_hi_eq_dph <= 1'b0; s1w_up_lo_lt_dph <= 1'b0;
            s1w_dn_hi_lt_dmh <= 1'b0; s1w_dn_hi_eq_dmh <= 1'b0; s1w_dn_lo_lt_dmh <= 1'b0;
            s1w_dn_hi_lt_dph <= 1'b0; s1w_dn_hi_eq_dph <= 1'b0; s1w_dn_lo_lt_dph <= 1'b0;
        end else begin
            s1w_up_hi_lt_dmh <= (counter_up_hi   <  duty_w_dmh_hi);
            s1w_up_hi_eq_dmh <= (counter_up_hi   == duty_w_dmh_hi);
            s1w_up_lo_lt_dmh <= (counter_up_lo   <  duty_w_dmh_lo);
            s1w_up_hi_lt_dph <= (counter_up_hi   <  duty_w_dph_hi);
            s1w_up_hi_eq_dph <= (counter_up_hi   == duty_w_dph_hi);
            s1w_up_lo_lt_dph <= (counter_up_lo   <  duty_w_dph_lo);
            s1w_dn_hi_lt_dmh <= (counter_down_hi <  duty_w_dmh_hi);
            s1w_dn_hi_eq_dmh <= (counter_down_hi == duty_w_dmh_hi);
            s1w_dn_lo_lt_dmh <= (counter_down_lo <  duty_w_dmh_lo);
            s1w_dn_hi_lt_dph <= (counter_down_hi <  duty_w_dph_hi);
            s1w_dn_hi_eq_dph <= (counter_down_hi == duty_w_dph_hi);
            s1w_dn_lo_lt_dph <= (counter_down_lo <  duty_w_dph_lo);
        end
    end

    reg s2w_up_lt_dmh, s2w_up_lt_dph;
    reg s2w_dn_lt_dmh, s2w_dn_lt_dph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2w_up_lt_dmh <= 1'b0; s2w_up_lt_dph <= 1'b0;
            s2w_dn_lt_dmh <= 1'b0; s2w_dn_lt_dph <= 1'b0;
        end else begin
            s2w_up_lt_dmh <= s1w_up_hi_lt_dmh | (s1w_up_hi_eq_dmh & s1w_up_lo_lt_dmh);
            s2w_up_lt_dph <= s1w_up_hi_lt_dph | (s1w_up_hi_eq_dph & s1w_up_lo_lt_dph);
            s2w_dn_lt_dmh <= s1w_dn_hi_lt_dmh | (s1w_dn_hi_eq_dmh & s1w_dn_lo_lt_dmh);
            s2w_dn_lt_dph <= s1w_dn_hi_lt_dph | (s1w_dn_hi_eq_dph & s1w_dn_lo_lt_dph);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_wh <= 1'b0;
            gate_wl <= 1'b0;
        end else begin
            gate_wh <= dir_d1 ?  s2w_dn_lt_dmh :  s2w_up_lt_dmh;
            gate_wl <= dir_d1 ? ~s2w_dn_lt_dph : ~s2w_up_lt_dph;
        end
    end

endmodule
