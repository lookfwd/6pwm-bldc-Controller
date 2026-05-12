// TDM State Machine — Shared Multiplier for 3-Phase Duty Calculation
//
// Includes an integrated 32-bit NCO phase accumulator. The accumulator
// is advanced in IDLE, then phase offsets for U (+0), V (+683), W (+1365)
// are computed sequentially as each phase accesses the single-port sine LUT.
//
// Per-phase flow: SET_ADDR -> WAIT_BRAM -> LOAD_MULT -> WAIT_MULT -> ADJUST -> STORE
// Total: 19 states. ADJUST registers `rounded_duty` (the bias-adjusted
// multiplier output), so the 25-bit `mult_result + K - amp_offset` subtractor
// and the 11-bit dead-time sat add/sub sit in separate clock cycles.
//
// LUT encoding: unsigned offset-binary — 0x0000=-1, 0x8000=0, 0xFFFF=+1.
//
// Math: duty = (sine_ob * amp + 8392704 - amp * 32768) >> 13
//            = (amp * (sine_ob - 32768) + 8392704) >> 13
// All arithmetic unsigned, matching the unsigned PWM up/down counter.
// Duty midpoint = 1024 for every amplitude; range [4 .. 2044] at amp=255.

module spwm_tdm #(
    parameter [10:0] HALF_DEAD_TIME = 11'd25
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pwm_sync,    // pulse when counter == 0
    input  wire [31:0] phase_inc,   // NCO frequency control
    input  wire [7:0]  amplitude,

    // Saturating duty ± HALF_DEAD_TIME.
    //   *_minus is 11-bit (range 0..2047). Saturating to 0 naturally
    //   represents "gate_high never fires" since counter is unsigned >= 0.
    //   *_plus is 12-bit (range 0..2048). Saturating to 2048 represents
    //   "gate_low never fires" since 11-bit counter is always < 2048.
    output reg  [10:0] u_minus,
    output reg  [11:0] u_plus,
    output reg  [10:0] v_minus,
    output reg  [11:0] v_plus,
    output reg  [10:0] w_minus,
    output reg  [11:0] w_plus
);

    // Sine LUT — 2048 × 16-bit, unsigned offset-binary
    // (0x0000=-peak, 0x8000=zero, 0xFFFF=+peak). Synchronous read,
    // 1-cycle latency (yosys infers a BRAM).
    reg  [10:0] lut_addr;
    reg  [15:0] lut_data;
    reg  [15:0] sine_mem [0:2047];

    initial $readmemh("sine_init.hex", sine_mem);

    always @(posedge clk) lut_data <= sine_mem[lut_addr];

    // Phase offsets for 120-degree spacing in 11-bit address space
    localparam [10:0] OFFSET_V = 11'd683;   // 2048/3 ≈ 683
    localparam [10:0] OFFSET_W = 11'd1365;  // 2*2048/3 ≈ 1365

    // State encoding
    localparam [4:0]
        IDLE        = 5'd0,
        SET_ADDR_U  = 5'd1,
        WAIT_BRAM_U = 5'd2,
        LOAD_MULT_U = 5'd3,
        WAIT_MULT_U = 5'd4,
        ADJUST_U    = 5'd5,
        STORE_U     = 5'd6,
        SET_ADDR_V  = 5'd7,
        WAIT_BRAM_V = 5'd8,
        LOAD_MULT_V = 5'd9,
        WAIT_MULT_V = 5'd10,
        ADJUST_V    = 5'd11,
        STORE_V     = 5'd12,
        SET_ADDR_W  = 5'd13,
        WAIT_BRAM_W = 5'd14,
        LOAD_MULT_W = 5'd15,
        WAIT_MULT_W = 5'd16,
        ADJUST_W    = 5'd17,
        STORE_W     = 5'd18;

    reg [4:0] state;

    // Integrated NCO — 32-bit phase accumulator
    reg [31:0] nco_acc;
    wire [10:0] base_phase = nco_acc[31:21];

    // Multiplier pipeline (registered output for timing closure).
    // All unsigned: LUT is offset-binary, amplitude is unsigned 0..255.
    // Max product: 65535 * 255 = 16,711,425, fits in 24 bits.
    reg  [15:0] mult_a;
    reg  [7:0]  mult_b;
    reg  [23:0] mult_result;

    always @(posedge clk) begin
        mult_result <= mult_a * mult_b;
    end

    // Subtract the amplitude-scaled LUT midpoint (amp * 32768) so the sine
    // zero-crossing always lands on the same duty regardless of amplitude,
    // then add K = 1024*8192 + 4096 to center that duty at 1024 with
    // round-to-nearest. All operands unsigned.
    //
    // adjusted = mult_result + K - amp*32768
    //          = amp * (sine_ob - 32768) + K
    // For amp <= 255: adjusted in [36864 .. 16748289] (always positive,
    // no underflow). Top bit (24) is always 0, so adjusted[23:13] = duty.
    // ADJUST_* registers `rounded_duty` here, so the 25-bit bias-adjust
    // subtractor (combinational below) and the 11-bit dead-time sat add/sub
    // (combinational into u/v/w_minus/plus in STORE_*) sit in separate clocks.
    wire [24:0] amp_offset       = {mult_b, 15'b0};
    wire [24:0] adjusted          = mult_result + 25'd8392704 - amp_offset;
    wire [10:0] rounded_duty_next = adjusted[23:13];

    reg  [10:0] rounded_duty;       // captured in ADJUST_*, consumed in STORE_*

	wire [10:0] rounded_duty_minus = (rounded_duty >= HALF_DEAD_TIME)
                                   ? (rounded_duty - HALF_DEAD_TIME) : 11'd0;
	// Saturate to 2048 (one above the 11-bit counter max) so that the
	// downstream `counter >= duty_plus` comparison is always false at
	// saturation, giving a true 0-cycle minimum for gate_low.
	wire [11:0] rounded_duty_plus  = (rounded_duty <= (11'd2047 - HALF_DEAD_TIME))
                                   ? (rounded_duty + HALF_DEAD_TIME) : 12'd2048;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            nco_acc  <= 32'd0;
            lut_addr <= 11'd0;
		    u_minus   <= 11'd0;
		    u_plus   <= 12'd0;
		    v_minus   <= 11'd0;
		    v_plus   <= 12'd0;
		    w_minus   <= 11'd0;
		    w_plus   <= 12'd0;
            mult_a   <= 16'd0;
            mult_b   <= 8'd0;
            rounded_duty <= 11'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (pwm_sync) begin
                        nco_acc <= nco_acc + phase_inc;  // advance NCO
                        state   <= SET_ADDR_U;
                    end
                end

                // --- Phase U (offset +0) ---
                SET_ADDR_U: begin
                    lut_addr <= base_phase;
                    state    <= WAIT_BRAM_U;
                end
                WAIT_BRAM_U: begin
                    state <= LOAD_MULT_U;
                end
                LOAD_MULT_U: begin
                    mult_a <= lut_data;
                    mult_b <= amplitude;
                    state  <= WAIT_MULT_U;
                end
                WAIT_MULT_U: begin
                    state <= ADJUST_U;
                end
                ADJUST_U: begin
                    rounded_duty <= rounded_duty_next;   // bias-adjust pipeline reg
                    state        <= STORE_U;
                end
                STORE_U: begin
                    u_minus <= rounded_duty_minus;       // sat add/sub (combinational)
                    u_plus  <= rounded_duty_plus;
                    state   <= SET_ADDR_V;
                end

                // --- Phase V (offset +683 = +120°) ---
                SET_ADDR_V: begin
                    lut_addr <= base_phase + OFFSET_V;
                    state    <= WAIT_BRAM_V;
                end
                WAIT_BRAM_V: begin
                    state <= LOAD_MULT_V;
                end
                LOAD_MULT_V: begin
                    mult_a <= lut_data;
                    mult_b <= amplitude;
                    state  <= WAIT_MULT_V;
                end
                WAIT_MULT_V: begin
                    state <= ADJUST_V;
                end
                ADJUST_V: begin
                    rounded_duty <= rounded_duty_next;
                    state        <= STORE_V;
                end
                STORE_V: begin
                    v_minus <= rounded_duty_minus;
                    v_plus  <= rounded_duty_plus;
                    state   <= SET_ADDR_W;
                end

                // --- Phase W (offset +1365 = +240°) ---
                SET_ADDR_W: begin
                    lut_addr <= base_phase + OFFSET_W;
                    state    <= WAIT_BRAM_W;
                end
                WAIT_BRAM_W: begin
                    state <= LOAD_MULT_W;
                end
                LOAD_MULT_W: begin
                    mult_a <= lut_data;
                    mult_b <= amplitude;
                    state  <= WAIT_MULT_W;
                end
                WAIT_MULT_W: begin
                    state <= ADJUST_W;
                end
                ADJUST_W: begin
                    rounded_duty <= rounded_duty_next;
                    state        <= STORE_W;
                end
                STORE_W: begin
                    w_minus <= rounded_duty_minus;
                    w_plus  <= rounded_duty_plus;
                    state   <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
