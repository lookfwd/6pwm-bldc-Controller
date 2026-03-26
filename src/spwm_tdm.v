// TDM State Machine — Shared Multiplier for 3-Phase Duty Calculation
//
// Includes an integrated 32-bit NCO phase accumulator. The accumulator
// is advanced in IDLE, then phase offsets for U (+0), V (+683), W (+1365)
// are computed sequentially as each phase accesses the single-port sine LUT.
//
// Per-phase flow: SET_ADDR -> WAIT_BRAM -> LOAD_MULT -> WAIT_MULT -> STORE
// Total: 16 states, 16 clock cycles (~194 ns at 82.5 MHz)
//
// Math: duty = (sine_16bit * amplitude_8bit + 0x1000) >> 13
//       24-bit product, rounded, truncated to 11 bits

module spwm_tdm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,      // high when state == RUNNING
    input  wire        pwm_sync,    // pulse when counter == 0
    input  wire [31:0] phase_inc,   // NCO frequency control
    input  wire [7:0]  amplitude,
    input  wire [15:0] lut_data,    // from sine_lut (1-cycle latency)
    output reg  [10:0] lut_addr,    // to sine_lut
    output reg  [10:0] duty_u,
    output reg  [10:0] duty_v,
    output reg  [10:0] duty_w
);

    // Phase offsets for 120-degree spacing in 11-bit address space
    localparam [10:0] OFFSET_V = 11'd683;   // 2048/3 ≈ 683
    localparam [10:0] OFFSET_W = 11'd1365;  // 2*2048/3 ≈ 1365

    // State encoding
    localparam [3:0]
        IDLE        = 4'd0,
        SET_ADDR_U  = 4'd1,
        WAIT_BRAM_U = 4'd2,
        LOAD_MULT_U = 4'd3,
        WAIT_MULT_U = 4'd4,
        STORE_U     = 4'd5,
        SET_ADDR_V  = 4'd6,
        WAIT_BRAM_V = 4'd7,
        LOAD_MULT_V = 4'd8,
        WAIT_MULT_V = 4'd9,
        STORE_V     = 4'd10,
        SET_ADDR_W  = 4'd11,
        WAIT_BRAM_W = 4'd12,
        LOAD_MULT_W = 4'd13,
        WAIT_MULT_W = 4'd14,
        STORE_W     = 4'd15;

    reg [3:0] state;

    // Integrated NCO — 32-bit phase accumulator
    reg [31:0] nco_acc;
    wire [10:0] base_phase = nco_acc[31:21];

    // Multiplier pipeline (registered output for timing closure)
    reg  [15:0] mult_a;       // sine value
    reg  [7:0]  mult_b;       // amplitude
    reg  [23:0] mult_result;  // registered product

    always @(posedge clk) begin
        mult_result <= mult_a * mult_b;
    end

    // Rounding: add half-LSB (2^12 = 0x1000) then shift right 13
    wire [10:0] rounded_duty = (mult_result + 24'h1000) >> 13;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            nco_acc  <= 32'd0;
            lut_addr <= 11'd0;
            duty_u   <= 11'd0;
            duty_v   <= 11'd0;
            duty_w   <= 11'd0;
            mult_a   <= 16'd0;
            mult_b   <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (enable && pwm_sync) begin
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
                    state <= STORE_U;
                end
                STORE_U: begin
                    duty_u <= rounded_duty;
                    state  <= SET_ADDR_V;
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
                    state <= STORE_V;
                end
                STORE_V: begin
                    duty_v <= rounded_duty;
                    state  <= SET_ADDR_W;
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
                    state <= STORE_W;
                end
                STORE_W: begin
                    duty_w <= rounded_duty;
                    state  <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
