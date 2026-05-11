// Phase-Correct PWM Counter + 3-Channel Dead-Time Gate Generator
// 11-bit counter / duty values (range 0..2047, period 4096 fast cycles).
//
// Counter source: registered passthrough copies of addr.
//   counter_up_reg   <=  addr[10:0]
//   counter_down_reg <= ~addr[10:0]
// One register hop between addr and the comparator inputs; gives nextpnr
// a placement anchor near the comparator cluster (same trick the BRAM
// version used with rom_out_pipe, but in fabric only).
//
// Pipeline depth from addr -> gate FF: 4 register stages
//   addr -> counter_reg -> s1 -> s2 -> gate
// addr[11] (direction) is pipelined through 3 explicit flops (dir_d0..d2)
// so dir_d2 aligns with the s2 outputs at the gate-FF mux.
//
// Per-phase dead-time logic, fully parallel:
//   Up   path: gate_high_up   = (counter_up   <  duty)         = lt_du_up
//              gate_low_up    = (counter_up   >= duty + dt)    = ~lt_dupdt_up
//   Down path: gate_high_down = (counter_down <  duty - dt)    =  lt_dumdt_dn
//              gate_low_down  = (counter_down >= duty)         = ~lt_du_dn
// Final:       gate_h <= dir_d2 ?  gate_high_down : gate_high_up
//              gate_l <= dir_d2 ?  gate_low_down  : gate_low_up
//
// Each 11-bit compare is carry-split 6 lsb + 5 msb across two pipeline
// stages so neither stage carries the full 11-bit chain.
//
// Companion module: pwm_phase_correct_twin (independent up/down counters
// instead of registered passthrough, 1 cycle less latency, same Fmax).

module pwm_phase_correct_pipelined(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  bus_ctrl,
    input  wire [10:0] bus,
    output reg         sync,
    output reg         gate_uh,
    output reg         gate_ul,
    output reg         gate_vh,
    output reg         gate_vl,
    output reg         gate_wh,
    output reg         gate_wl
);

    // ============================================================
    // Bus-decoded duty registers (slow-domain writes)
    //   bus_ctrl 0..2: duty_u/v/w
    //   bus_ctrl 3..8: duty_X_minus_dt / duty_X_plus_dt (pre-saturated
    //                  by the slow-domain pre-calc upstream)
    // ============================================================
    reg [10:0] duty_u, duty_v, duty_w;
    reg [10:0] duty_u_minus_dt, duty_u_plus_dt;
    reg [10:0] duty_v_minus_dt, duty_v_plus_dt;
    reg [10:0] duty_w_minus_dt, duty_w_plus_dt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_u          <= 11'd0;
            duty_v          <= 11'd0;
            duty_w          <= 11'd0;
            duty_u_minus_dt <= 11'd0;
            duty_u_plus_dt  <= 11'd0;
            duty_v_minus_dt <= 11'd0;
            duty_v_plus_dt  <= 11'd0;
            duty_w_minus_dt <= 11'd0;
            duty_w_plus_dt  <= 11'd0;
        end else begin
            if      (bus_ctrl == 4'd0) duty_u          <= bus;
            else if (bus_ctrl == 4'd1) duty_v          <= bus;
            else if (bus_ctrl == 4'd2) duty_w          <= bus;
            else if (bus_ctrl == 4'd3) duty_u_minus_dt <= bus;
            else if (bus_ctrl == 4'd4) duty_u_plus_dt  <= bus;
            else if (bus_ctrl == 4'd5) duty_v_minus_dt <= bus;
            else if (bus_ctrl == 4'd6) duty_v_plus_dt  <= bus;
            else if (bus_ctrl == 4'd7) duty_w_minus_dt <= bus;
            else if (bus_ctrl == 4'd8) duty_w_plus_dt  <= bus;
        end
    end

    // ============================================================
    // Free-running 12-bit address counter
    //   addr[10:0] : up-direction counter value (identity)
    //   addr[11]   : direction select (0 = up, 1 = down)
    // ============================================================
    reg [11:0] addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  addr <= 12'd0;
        else         addr <= addr + 12'd1;
    end

    // ============================================================
    // Registered passthrough copies of addr[10:0] and ~addr[10:0].
    // Pure DFFs in fabric — no add/sub logic between them and the
    // comparator cluster. The flop bank gets placed near the
    // comparators, breaking the long addr-fanout routing hop.
    // ============================================================
    reg [10:0] counter_up_reg;
    reg [10:0] counter_down_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_up_reg   <= 11'd0;
            counter_down_reg <= 11'd2047;
        end else begin
            counter_up_reg   <=  addr[10:0];
            counter_down_reg <= ~addr[10:0];
        end
    end

    wire [10:0] counter_up   = counter_up_reg;
    wire [10:0] counter_down = counter_down_reg;

    // ============================================================
    // addr[11] (direction) pipelined to align with s2 at the gate-FF mux.
    // Data path is addr -> counter_reg -> s1 -> s2 -> gate (4 stages),
    // so 3 explicit dir flops (dir_d0..d2). Gate-FF mux reads dir_d2.
    // ============================================================
    reg dir_d0, dir_d1, dir_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_d0 <= 1'b0;
            dir_d1 <= 1'b0;
            dir_d2 <= 1'b0;
        end else begin
            dir_d0 <= addr[11];
            dir_d1 <= dir_d0;
            dir_d2 <= dir_d1;
        end
    end

    // ============================================================
    // sync: registered (addr == 0). Fires once per 4096-cycle period.
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= (addr == 12'd0);
    end

    // ============================================================
    // Counter / duty splits (combinational): 6 lsb + 5 msb.
    // ============================================================
    wire [4:0] counter_up_hi   = counter_up[10:6];
    wire [5:0] counter_up_lo   = counter_up[5:0];
    wire [4:0] counter_down_hi = counter_down[10:6];
    wire [5:0] counter_down_lo = counter_down[5:0];

    wire [4:0] duty_u_hi              = duty_u[10:6];
    wire [5:0] duty_u_lo              = duty_u[5:0];
    wire [4:0] duty_u_minus_dt_hi     = duty_u_minus_dt[10:6];
    wire [5:0] duty_u_minus_dt_lo     = duty_u_minus_dt[5:0];
    wire [4:0] duty_u_plus_dt_hi      = duty_u_plus_dt[10:6];
    wire [5:0] duty_u_plus_dt_lo      = duty_u_plus_dt[5:0];

    wire [4:0] duty_v_hi              = duty_v[10:6];
    wire [5:0] duty_v_lo              = duty_v[5:0];
    wire [4:0] duty_v_minus_dt_hi     = duty_v_minus_dt[10:6];
    wire [5:0] duty_v_minus_dt_lo     = duty_v_minus_dt[5:0];
    wire [4:0] duty_v_plus_dt_hi      = duty_v_plus_dt[10:6];
    wire [5:0] duty_v_plus_dt_lo      = duty_v_plus_dt[5:0];

    wire [4:0] duty_w_hi              = duty_w[10:6];
    wire [5:0] duty_w_lo              = duty_w[5:0];
    wire [4:0] duty_w_minus_dt_hi     = duty_w_minus_dt[10:6];
    wire [5:0] duty_w_minus_dt_lo     = duty_w_minus_dt[5:0];
    wire [4:0] duty_w_plus_dt_hi      = duty_w_plus_dt[10:6];
    wire [5:0] duty_w_plus_dt_lo      = duty_w_plus_dt[5:0];

    // ============================================================
    // Phase U pipelined dead-time
    //   Up path:   counter_up   <  duty_u            -> lt_du_up
    //              counter_up   <  duty_u_plus_dt    -> lt_dupdt_up
    //   Down path: counter_down <  duty_u_minus_dt   -> lt_dumdt_dn
    //              counter_down <  duty_u            -> lt_du_dn
    // ============================================================
    reg s1u_up_hi_lt_du,    s1u_up_hi_eq_du,    s1u_up_lo_lt_du;
    reg s1u_up_hi_lt_dupdt, s1u_up_hi_eq_dupdt, s1u_up_lo_lt_dupdt;
    reg s1u_dn_hi_lt_dumdt, s1u_dn_hi_eq_dumdt, s1u_dn_lo_lt_dumdt;
    reg s1u_dn_hi_lt_du,    s1u_dn_hi_eq_du,    s1u_dn_lo_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1u_up_hi_lt_du    <= 1'b0; s1u_up_hi_eq_du    <= 1'b0; s1u_up_lo_lt_du    <= 1'b0;
            s1u_up_hi_lt_dupdt <= 1'b0; s1u_up_hi_eq_dupdt <= 1'b0; s1u_up_lo_lt_dupdt <= 1'b0;
            s1u_dn_hi_lt_dumdt <= 1'b0; s1u_dn_hi_eq_dumdt <= 1'b0; s1u_dn_lo_lt_dumdt <= 1'b0;
            s1u_dn_hi_lt_du    <= 1'b0; s1u_dn_hi_eq_du    <= 1'b0; s1u_dn_lo_lt_du    <= 1'b0;
        end else begin
            s1u_up_hi_lt_du    <= (counter_up_hi <  duty_u_hi);
            s1u_up_hi_eq_du    <= (counter_up_hi == duty_u_hi);
            s1u_up_lo_lt_du    <= (counter_up_lo <  duty_u_lo);
            s1u_up_hi_lt_dupdt <= (counter_up_hi <  duty_u_plus_dt_hi);
            s1u_up_hi_eq_dupdt <= (counter_up_hi == duty_u_plus_dt_hi);
            s1u_up_lo_lt_dupdt <= (counter_up_lo <  duty_u_plus_dt_lo);
            s1u_dn_hi_lt_dumdt <= (counter_down_hi <  duty_u_minus_dt_hi);
            s1u_dn_hi_eq_dumdt <= (counter_down_hi == duty_u_minus_dt_hi);
            s1u_dn_lo_lt_dumdt <= (counter_down_lo <  duty_u_minus_dt_lo);
            s1u_dn_hi_lt_du    <= (counter_down_hi <  duty_u_hi);
            s1u_dn_hi_eq_du    <= (counter_down_hi == duty_u_hi);
            s1u_dn_lo_lt_du    <= (counter_down_lo <  duty_u_lo);
        end
    end

    reg s2u_up_lt_du, s2u_up_lt_dupdt;
    reg s2u_dn_lt_dumdt, s2u_dn_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2u_up_lt_du    <= 1'b0;
            s2u_up_lt_dupdt <= 1'b0;
            s2u_dn_lt_dumdt <= 1'b0;
            s2u_dn_lt_du    <= 1'b0;
        end else begin
            s2u_up_lt_du    <= s1u_up_hi_lt_du    | (s1u_up_hi_eq_du    & s1u_up_lo_lt_du);
            s2u_up_lt_dupdt <= s1u_up_hi_lt_dupdt | (s1u_up_hi_eq_dupdt & s1u_up_lo_lt_dupdt);
            s2u_dn_lt_dumdt <= s1u_dn_hi_lt_dumdt | (s1u_dn_hi_eq_dumdt & s1u_dn_lo_lt_dumdt);
            s2u_dn_lt_du    <= s1u_dn_hi_lt_du    | (s1u_dn_hi_eq_du    & s1u_dn_lo_lt_du);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_uh <= 1'b0;
            gate_ul <= 1'b0;
        end else begin
            gate_uh <= dir_d2 ?  s2u_dn_lt_dumdt :  s2u_up_lt_du;
            gate_ul <= dir_d2 ? ~s2u_dn_lt_du    : ~s2u_up_lt_dupdt;
        end
    end

    // ============================================================
    // Phase V pipelined dead-time
    // ============================================================
    reg s1v_up_hi_lt_du,    s1v_up_hi_eq_du,    s1v_up_lo_lt_du;
    reg s1v_up_hi_lt_dupdt, s1v_up_hi_eq_dupdt, s1v_up_lo_lt_dupdt;
    reg s1v_dn_hi_lt_dumdt, s1v_dn_hi_eq_dumdt, s1v_dn_lo_lt_dumdt;
    reg s1v_dn_hi_lt_du,    s1v_dn_hi_eq_du,    s1v_dn_lo_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1v_up_hi_lt_du    <= 1'b0; s1v_up_hi_eq_du    <= 1'b0; s1v_up_lo_lt_du    <= 1'b0;
            s1v_up_hi_lt_dupdt <= 1'b0; s1v_up_hi_eq_dupdt <= 1'b0; s1v_up_lo_lt_dupdt <= 1'b0;
            s1v_dn_hi_lt_dumdt <= 1'b0; s1v_dn_hi_eq_dumdt <= 1'b0; s1v_dn_lo_lt_dumdt <= 1'b0;
            s1v_dn_hi_lt_du    <= 1'b0; s1v_dn_hi_eq_du    <= 1'b0; s1v_dn_lo_lt_du    <= 1'b0;
        end else begin
            s1v_up_hi_lt_du    <= (counter_up_hi <  duty_v_hi);
            s1v_up_hi_eq_du    <= (counter_up_hi == duty_v_hi);
            s1v_up_lo_lt_du    <= (counter_up_lo <  duty_v_lo);
            s1v_up_hi_lt_dupdt <= (counter_up_hi <  duty_v_plus_dt_hi);
            s1v_up_hi_eq_dupdt <= (counter_up_hi == duty_v_plus_dt_hi);
            s1v_up_lo_lt_dupdt <= (counter_up_lo <  duty_v_plus_dt_lo);
            s1v_dn_hi_lt_dumdt <= (counter_down_hi <  duty_v_minus_dt_hi);
            s1v_dn_hi_eq_dumdt <= (counter_down_hi == duty_v_minus_dt_hi);
            s1v_dn_lo_lt_dumdt <= (counter_down_lo <  duty_v_minus_dt_lo);
            s1v_dn_hi_lt_du    <= (counter_down_hi <  duty_v_hi);
            s1v_dn_hi_eq_du    <= (counter_down_hi == duty_v_hi);
            s1v_dn_lo_lt_du    <= (counter_down_lo <  duty_v_lo);
        end
    end

    reg s2v_up_lt_du, s2v_up_lt_dupdt;
    reg s2v_dn_lt_dumdt, s2v_dn_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2v_up_lt_du    <= 1'b0;
            s2v_up_lt_dupdt <= 1'b0;
            s2v_dn_lt_dumdt <= 1'b0;
            s2v_dn_lt_du    <= 1'b0;
        end else begin
            s2v_up_lt_du    <= s1v_up_hi_lt_du    | (s1v_up_hi_eq_du    & s1v_up_lo_lt_du);
            s2v_up_lt_dupdt <= s1v_up_hi_lt_dupdt | (s1v_up_hi_eq_dupdt & s1v_up_lo_lt_dupdt);
            s2v_dn_lt_dumdt <= s1v_dn_hi_lt_dumdt | (s1v_dn_hi_eq_dumdt & s1v_dn_lo_lt_dumdt);
            s2v_dn_lt_du    <= s1v_dn_hi_lt_du    | (s1v_dn_hi_eq_du    & s1v_dn_lo_lt_du);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_vh <= 1'b0;
            gate_vl <= 1'b0;
        end else begin
            gate_vh <= dir_d2 ?  s2v_dn_lt_dumdt :  s2v_up_lt_du;
            gate_vl <= dir_d2 ? ~s2v_dn_lt_du    : ~s2v_up_lt_dupdt;
        end
    end

    // ============================================================
    // Phase W pipelined dead-time
    // ============================================================
    reg s1w_up_hi_lt_du,    s1w_up_hi_eq_du,    s1w_up_lo_lt_du;
    reg s1w_up_hi_lt_dupdt, s1w_up_hi_eq_dupdt, s1w_up_lo_lt_dupdt;
    reg s1w_dn_hi_lt_dumdt, s1w_dn_hi_eq_dumdt, s1w_dn_lo_lt_dumdt;
    reg s1w_dn_hi_lt_du,    s1w_dn_hi_eq_du,    s1w_dn_lo_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1w_up_hi_lt_du    <= 1'b0; s1w_up_hi_eq_du    <= 1'b0; s1w_up_lo_lt_du    <= 1'b0;
            s1w_up_hi_lt_dupdt <= 1'b0; s1w_up_hi_eq_dupdt <= 1'b0; s1w_up_lo_lt_dupdt <= 1'b0;
            s1w_dn_hi_lt_dumdt <= 1'b0; s1w_dn_hi_eq_dumdt <= 1'b0; s1w_dn_lo_lt_dumdt <= 1'b0;
            s1w_dn_hi_lt_du    <= 1'b0; s1w_dn_hi_eq_du    <= 1'b0; s1w_dn_lo_lt_du    <= 1'b0;
        end else begin
            s1w_up_hi_lt_du    <= (counter_up_hi <  duty_w_hi);
            s1w_up_hi_eq_du    <= (counter_up_hi == duty_w_hi);
            s1w_up_lo_lt_du    <= (counter_up_lo <  duty_w_lo);
            s1w_up_hi_lt_dupdt <= (counter_up_hi <  duty_w_plus_dt_hi);
            s1w_up_hi_eq_dupdt <= (counter_up_hi == duty_w_plus_dt_hi);
            s1w_up_lo_lt_dupdt <= (counter_up_lo <  duty_w_plus_dt_lo);
            s1w_dn_hi_lt_dumdt <= (counter_down_hi <  duty_w_minus_dt_hi);
            s1w_dn_hi_eq_dumdt <= (counter_down_hi == duty_w_minus_dt_hi);
            s1w_dn_lo_lt_dumdt <= (counter_down_lo <  duty_w_minus_dt_lo);
            s1w_dn_hi_lt_du    <= (counter_down_hi <  duty_w_hi);
            s1w_dn_hi_eq_du    <= (counter_down_hi == duty_w_hi);
            s1w_dn_lo_lt_du    <= (counter_down_lo <  duty_w_lo);
        end
    end

    reg s2w_up_lt_du, s2w_up_lt_dupdt;
    reg s2w_dn_lt_dumdt, s2w_dn_lt_du;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2w_up_lt_du    <= 1'b0;
            s2w_up_lt_dupdt <= 1'b0;
            s2w_dn_lt_dumdt <= 1'b0;
            s2w_dn_lt_du    <= 1'b0;
        end else begin
            s2w_up_lt_du    <= s1w_up_hi_lt_du    | (s1w_up_hi_eq_du    & s1w_up_lo_lt_du);
            s2w_up_lt_dupdt <= s1w_up_hi_lt_dupdt | (s1w_up_hi_eq_dupdt & s1w_up_lo_lt_dupdt);
            s2w_dn_lt_dumdt <= s1w_dn_hi_lt_dumdt | (s1w_dn_hi_eq_dumdt & s1w_dn_lo_lt_dumdt);
            s2w_dn_lt_du    <= s1w_dn_hi_lt_du    | (s1w_dn_hi_eq_du    & s1w_dn_lo_lt_du);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_wh <= 1'b0;
            gate_wl <= 1'b0;
        end else begin
            gate_wh <= dir_d2 ?  s2w_dn_lt_dumdt :  s2w_up_lt_du;
            gate_wl <= dir_d2 ? ~s2w_dn_lt_du    : ~s2w_up_lt_dupdt;
        end
    end

endmodule
