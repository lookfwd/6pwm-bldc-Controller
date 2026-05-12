// Phase-Correct PWM + 3-Channel Dead-Time Gate Generator
//
// Variant selected at synthesis time via -DVARIANT_*:
//
//   VARIANT_PIPE   — Free-running 12-bit addr; counter is the registered
//                    mux of addr[10:0] vs ~addr[10:0].
//   VARIANT_BRAMS  — Triangle counter (and sync flag) materialized in a
//                    4096-entry BRAM lookup table.
//
// Both variants share the duty-at-sync latch, the comparator pipeline
// (s1 → s2 → gate FF), and the gate outputs. They differ only in how
// `counter` and `sync` are generated.
//
// Symmetric dead-time, centered on the duty boundary:
//
//   gate_high = (counter <  duty - dt/2)
//   gate_low  = (counter >= duty + dt/2) = ~(counter < duty + dt/2)
//
// Both halves of the triangle share thresholds, so one comparator lane
// per phase is sufficient.
//
// duty_minus is 11-bit. duty_plus is 12-bit: bit 11 is a "force off"
// sentinel OR'd directly into the s2 reduction, so gate_low can be fully
// suppressed at the sine peak without widening the 11-bit comparators.
//
// Pipeline depth (source register → gate FF): 4 stages.

module pwm_phase_correct(
    input  wire        clk,
    input  wire        rst_n,
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
    // Variant-specific triangle counter + sync generation.
    // Each block drives `wire [10:0] counter` and `reg sync`.
    // ============================================================
`ifdef VARIANT_PIPE
    // Free-running 12-bit addr → registered mux of address halves.
    //   UP half   (addr[11]=0): counter = addr[10:0]   = 0..2047
    //   DOWN half (addr[11]=1): counter = ~addr[10:0]  = 2047..0
    reg [11:0] addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) addr <= 12'd0;
        else        addr <= addr + 12'd1;
    end

    reg [10:0] counter_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) counter_reg <= 11'd0;
        else        counter_reg <= addr[11] ? ~addr[10:0] : addr[10:0];
    end

    wire [10:0] counter = counter_reg;

    // Two-stage sync to align with counter_reg's pipeline depth.
    reg sync_pre_l, sync_pre_h;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
		  sync_pre_l <= 1'b0;
		  sync_pre_h <= 1'b0;
		end
        else begin
		   sync_pre_l <= (addr[5:0] == 6'd0);
		   sync_pre_h <= (addr[11:6] == 6'd0);
		end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= sync_pre_l && sync_pre_h;
    end

`elsif VARIANT_BRAMS
    // BRAM-materialized triangle table. data[10:0] = counter value,
    // data[11] = sync (high at addr 0). 4096 × 12 bits.
    reg [11:0] addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) addr <= 12'd0;
        else        addr <= addr + 12'd1;
    end

    reg [11:0] table_data [0:4095];
    initial $readmemh("src/counter_table.hex", table_data);

    reg [11:0] rom_out;           // BRAM read latch (yosys absorbs into BRAM)
    always @(posedge clk) rom_out <= table_data[addr];

    (* keep *) reg [11:0] rom_out_pipe;   // fabric pipeline between BRAM and consumers
    always @(posedge clk) rom_out_pipe <= rom_out;

    wire [10:0] counter  = rom_out_pipe[10:0];
    wire        rom_sync = rom_out_pipe[11];

    // One extra register stage to match the other variants' sync depth.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 1'b0;
        else        sync <= rom_sync;
    end

`else
    initial begin
        $display("ERROR: pwm_phase_correct: no VARIANT_* define passed.");
        $display("       Use -DVARIANT_PIPE or -DVARIANT_BRAMS.");
        $finish;
    end
`endif

    // ============================================================
    // Counter / duty splits (shared across variants).
    //   Both duty_minus and the LOW 11 bits of duty_plus split 5 hi + 6 lo,
    //   so the s1 comparators are identical-sized between dmh and dph.
    //   Bit 11 of duty_plus is the "force off" sentinel, OR'd into s2.
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
    //   s1 : hi-lt + hi-eq + lo-lt for two RHSs (dmh, dph)
    //   s2 : reduce hi/lo to lt_dmh, lt_dph (bit-11 sentinel folded in)
    //   gate: gate_uh =  lt_dmh ;  gate_ul = ~lt_dph
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
