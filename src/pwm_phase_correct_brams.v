// Phase-Correct PWM + 3-Channel Dead-Time Gate Generator
// BRAM-table variant — the up/down counter is materialized as a
// 4096 × 12-bit BRAM lookup table indexed by a free-running addr.
//   data[10:0] = counter value at that point in the period
//   data[11]   = sync (high at addr 0)
// Direction is NOT stored in the table — symmetric dead-time means
// both halves of the triangle compare against the same thresholds, so
// the gate FF needs no direction mux.
//
// Symmetric dead-time, centered on the duty boundary:
//   gate_high = (counter <  duty - dt/2)
//   gate_low  = (counter >= duty + dt/2)   = ~(counter < duty + dt/2)
//
// Per phase: two compare lanes:
//   lt_dmh = (counter < duty_X_minus_dt_half)
//   lt_dph = (counter < duty_X_plus_dt_half)
//
// Pipeline depth from addr -> gate FF: 5 register stages
//   addr -> BRAM-internal -> rom_out_pipe -> s1 -> s2 -> gate

module pwm_phase_correct_brams(
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

    reg [10:0] duty_u_minus_dt_half_reg;
	reg [10:0] duty_u_plus_dt_half_reg;
    reg [10:0] duty_v_minus_dt_half_reg;
	reg [10:0] duty_v_plus_dt_half_reg;
    reg [10:0] duty_w_minus_dt_half_reg;
	reg [10:0] duty_w_plus_dt_half_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  begin
            duty_u_minus_dt_half_reg <= 11'd0;
            duty_u_plus_dt_half_reg <= 11'd0;
            duty_v_minus_dt_half_reg <= 11'd0;
            duty_v_plus_dt_half_reg <= 11'd0;
            duty_w_minus_dt_half_reg <= 11'd0;
            duty_w_plus_dt_half_reg <= 11'd0;
		end
        else if (sync) begin
            duty_u_minus_dt_half_reg <= duty_u_minus_dt_half;
            duty_u_plus_dt_half_reg <= duty_u_plus_dt_half;
            duty_v_minus_dt_half_reg <= duty_v_minus_dt_half;
            duty_v_plus_dt_half_reg <= duty_v_plus_dt_half;
            duty_w_minus_dt_half_reg <= duty_w_minus_dt_half;
            duty_w_plus_dt_half_reg <= duty_w_plus_dt_half;
		end
    end

    // ============================================================
    // BRAM-based counter / sync source (12-bit data).
    // Reuses src/counter_table.hex (which is 13-bit per entry); the
    // direction bit at position 12 gets truncated by $readmemh into
    // this 12-bit table.
    // ============================================================
    reg [11:0] addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) addr <= 12'd0;
        else        addr <= addr + 12'd1;
    end

    reg [11:0] table_data [0:4095];
    initial $readmemh("src/counter_table.hex", table_data);

    reg [11:0] rom_out;     // BRAM read latch (yosys absorbs into BRAM)
    always @(posedge clk) begin
        rom_out <= table_data[addr];
    end

    // Fabric pipeline register between BRAM column and consumers.
    (* keep *) reg [11:0] rom_out_pipe;
    always @(posedge clk) begin
        rom_out_pipe <= rom_out;
    end

    wire [10:0] counter  = rom_out_pipe[10:0];
    wire        rom_sync = rom_out_pipe[11];

    // sync output: one extra register to match the previous design's
    // sync-output depth.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= rom_sync;
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
    //   stage 1: hi-lt + hi-eq + lo-lt for two RHSs (dmh, dph)
    //   stage 2: combine to two final lt results
    //   stage 3: gate FFs — NO direction mux (both halves identical)
    //     gate_uh = lt_dmh
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
