// Command Parser & Shadow Registers
//
// Decodes 5-byte UART packets: [CMD] [D3] [D2] [D1] [D0]
// Writes to shadow (next_*) registers, then swaps to active outputs
// atomically at pwm_sync (counter == 0).
//
// Commands:
//   0x01 — Set state:     D0 = 0 (OPEN), 1 (RUNNING), 2 (BRAKE)
//   0x02 — Set speed:     D3..D0 = 32-bit NCO phase increment
//   0x03 — Set amplitude: D0 = 8-bit amplitude (0–255)

module cmd_parser (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        pwm_sync,

    output reg  [1:0]  ctrl_state,    // 0=OPEN, 1=RUNNING, 2=BRAKE
    output reg  [31:0] phase_inc,     // NCO phase increment
    output reg  [7:0]  amplitude      // PWM amplitude scale
);

    // Command IDs
    localparam CMD_STATE = 8'h01;
    localparam CMD_SPEED = 8'h02;
    localparam CMD_AMP   = 8'h03;

    // Packet receiver state
    reg [7:0]  cmd_reg;
    reg [2:0]  byte_cnt;    // 0 = waiting for CMD, 1-4 = data bytes
    reg [31:0] data_shift;

    // Shadow (next) registers
    reg [1:0]  next_state;
    reg [31:0] next_phase_inc;
    reg [7:0]  next_amplitude;

    // Flags indicating shadow register has new data
    reg        upd_state;
    reg        upd_phase_inc;
    reg        upd_amplitude;

    // State-transition lock: forces ctrl_state to OPEN for ≥ dead_time fast cycles
    // before any non-OPEN destination, preventing shoot-through across RUNNING ↔
    // BRAKE. dead_time = 50 fast cycles; with the 4:1 gear ratio, 13 slow cycles
    // gives 52 fast cycles of OPEN — comfortably above the floor.
    localparam [3:0] STATE_LOCK_SLOW = 4'd13;
    reg [3:0] state_lock_cnt;
    reg [1:0] pending_state;

    // Packet reception
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt       <= 3'd0;
            cmd_reg        <= 8'd0;
            data_shift     <= 32'd0;
            next_state     <= 2'd0;
            next_phase_inc <= 32'd0;
            next_amplitude <= 8'd0;
            upd_state      <= 1'b0;
            upd_phase_inc  <= 1'b0;
            upd_amplitude  <= 1'b0;
            state_lock_cnt <= 4'd0;
            pending_state  <= 2'd0;
        end else begin
            if (rx_valid) begin
                if (byte_cnt == 3'd0) begin
                    // Command byte
                    cmd_reg    <= rx_data;
                    byte_cnt   <= 3'd1;
                    data_shift <= 32'd0;
                end else begin
                    // Data bytes (big-endian: D3 first, D0 last)
                    data_shift <= {data_shift[23:0], rx_data};

                    if (byte_cnt == 3'd4) begin
                        // Packet complete — store to shadow register
                        byte_cnt <= 3'd0;
                        case (cmd_reg)
                            CMD_STATE: begin
                                next_state <= rx_data[1:0];
                                upd_state  <= 1'b1;
                            end
                            CMD_SPEED: begin
                                next_phase_inc <= {data_shift[23:0], rx_data};
                                upd_phase_inc  <= 1'b1;
                            end
                            CMD_AMP: begin
                                next_amplitude <= rx_data;
                                upd_amplitude  <= 1'b1;
                            end
                            default: ;  // unknown command — ignore
                        endcase
                    end else begin
                        byte_cnt <= byte_cnt + 3'd1;
                    end
                end
            end

            // Atomic swap: shadow -> active at pwm_sync
            if (pwm_sync) begin
                if (upd_state) begin
                    // Force OPEN sandwich on every actual state change.
                    // The fast-domain deadtime no longer self-enforces transition
                    // dead-time, so the lock here is the sole shoot-through guard.
                    if (next_state != ctrl_state) begin
                        ctrl_state     <= 2'd0;            // OPEN
                        pending_state  <= next_state;
                        state_lock_cnt <= STATE_LOCK_SLOW;
                    end
                    upd_state <= 1'b0;
                end
                if (upd_phase_inc) begin
                    phase_inc     <= next_phase_inc;
                    upd_phase_inc <= 1'b0;
                end
                if (upd_amplitude) begin
                    amplitude     <= next_amplitude;
                    upd_amplitude <= 1'b0;
                end
            end

            // Tick the transition lock; release into pending_state when it expires.
            if (state_lock_cnt != 4'd0) begin
                state_lock_cnt <= state_lock_cnt - 4'd1;
                if (state_lock_cnt == 4'd1)
                    ctrl_state <= pending_state;
            end
        end
    end

    // Initialize active registers to safe defaults
    initial begin
        ctrl_state = 2'd0;      // OPEN
        phase_inc  = 32'd0;
        amplitude  = 8'd0;
    end

endmodule
