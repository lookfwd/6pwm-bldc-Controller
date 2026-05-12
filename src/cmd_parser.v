// Command Parser & Shadow Registers
//
// Robust UART protocol (MIDI-style framing + XOR checksum):
//
//   7-byte packet:
//     [1xxx_xxxx]                     sync/status byte (MSB=1)
//     [0xxx_xxxx] × 5                 5 data bytes (7-bit each)
//     [0xxx_xxxx]                     XOR checksum (7-bit)
//
//   The MSB rule (1 only in sync, 0 in data/checksum) lets the receiver
//   resync at any frame boundary: any byte with MSB=1 immediately
//   restarts the state machine on the new packet, dropping whatever
//   half-received frame was in progress.
//
//   Status byte: `1xxx_xxxx` where the low 7 bits select the command:
//     0x80 (cmd 0x00) — SET_STATE  reserved (currently no-op)
//     0x81 (cmd 0x01) — SET_SPEED  payload[31:0]  = NCO phase increment
//     0x82 (cmd 0x02) — SET_AMP    payload[7:0]   = amplitude
//
//   Payload reconstruction (35 bits, big-endian 7-bit chunks):
//     payload = (D4 << 28) | (D3 << 21) | (D2 << 14) | (D1 << 7) | D0
//
//     SPEED uses payload[31:0]  (D4's top 3 bits unused).
//     AMP   uses payload[7:0]   (= {D1[0], D0}).
//
//   Checksum: XOR of the low 7 bits of all 6 preceding bytes
//             (cmd ^ D4 ^ D3 ^ D2 ^ D1 ^ D0).
//
// Shadow / atomic-swap behavior is unchanged: decoded values land in
// next_* shadow registers and are copied to the live outputs at the
// next pwm_sync, guaranteeing all three phases see a coherent update.

module cmd_parser (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        pwm_sync,

    output reg  [31:0] phase_inc,     // NCO phase increment
    output reg  [7:0]  amplitude      // PWM amplitude scale
);

    // Command codes (low 7 bits of the sync byte).
    localparam [6:0] CMD_STATE = 7'h00;   // reserved
    localparam [6:0] CMD_SPEED = 7'h01;
    localparam [6:0] CMD_AMP   = 7'h02;

    // Receiver state machine.
    localparam [2:0]
        S_IDLE = 3'd0,    // waiting for sync (MSB=1)
        S_D4   = 3'd1,
        S_D3   = 3'd2,
        S_D2   = 3'd3,
        S_D1   = 3'd4,
        S_D0   = 3'd5,
        S_CHK  = 3'd6;

    reg [2:0] state;
    reg [6:0] cmd_reg;
    reg [6:0] d4_reg, d3_reg, d2_reg, d1_reg, d0_reg;
    reg [6:0] running_xor;

    // Shadow registers (updated when a valid packet completes, copied to
    // live outputs at pwm_sync).
    reg [31:0] next_phase_inc;
    reg [7:0]  next_amplitude;
    reg        upd_phase_inc;
    reg        upd_amplitude;

    wire is_sync  = rx_data[7];
    wire [6:0] d7 = rx_data[6:0];

    // Decoded payload views.
    wire [31:0] payload_speed = {d4_reg[3:0], d3_reg, d2_reg, d1_reg, d0_reg};
    wire [7:0]  payload_amp   = {d1_reg[0], d0_reg};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            cmd_reg        <= 7'd0;
            d4_reg         <= 7'd0;
            d3_reg         <= 7'd0;
            d2_reg         <= 7'd0;
            d1_reg         <= 7'd0;
            d0_reg         <= 7'd0;
            running_xor    <= 7'd0;
            next_phase_inc <= 32'd0;
            next_amplitude <= 8'd0;
            upd_phase_inc  <= 1'b0;
            upd_amplitude  <= 1'b0;
        end else begin

            if (rx_valid) begin
                if (is_sync) begin
                    // Status byte — always restarts the state machine.
                    cmd_reg     <= d7;
                    running_xor <= d7;
                    state       <= S_D4;
                end else begin
                    case (state)
                        S_D4: begin
                            d4_reg      <= d7;
                            running_xor <= running_xor ^ d7;
                            state       <= S_D3;
                        end
                        S_D3: begin
                            d3_reg      <= d7;
                            running_xor <= running_xor ^ d7;
                            state       <= S_D2;
                        end
                        S_D2: begin
                            d2_reg      <= d7;
                            running_xor <= running_xor ^ d7;
                            state       <= S_D1;
                        end
                        S_D1: begin
                            d1_reg      <= d7;
                            running_xor <= running_xor ^ d7;
                            state       <= S_D0;
                        end
                        S_D0: begin
                            d0_reg      <= d7;
                            running_xor <= running_xor ^ d7;
                            state       <= S_CHK;
                        end
                        S_CHK: begin
                            // Verify checksum. Note d0_reg is already latched
                            // by the time we're in S_CHK, so running_xor here
                            // includes D0.
                            if (d7 == running_xor) begin
                                case (cmd_reg)
                                    CMD_SPEED: begin
                                        next_phase_inc <= payload_speed;
                                        upd_phase_inc  <= 1'b1;
                                    end
                                    CMD_AMP: begin
                                        next_amplitude <= payload_amp;
                                        upd_amplitude  <= 1'b1;
                                    end
                                    default: ;   // CMD_STATE or unknown — drop
                                endcase
                            end
                            // Always return to IDLE after the checksum byte,
                            // whether it matched or not. Bad checksum = silent
                            // drop; the next sync byte starts a fresh frame.
                            state <= S_IDLE;
                        end
                        default: ;   // S_IDLE — data byte with no preceding sync, ignore.
                    endcase
                end
            end

            // Atomic swap: shadow -> active at pwm_sync.
            if (pwm_sync) begin
                if (upd_phase_inc) begin
                    phase_inc     <= next_phase_inc;
                    upd_phase_inc <= 1'b0;
                end
                if (upd_amplitude) begin
                    amplitude     <= next_amplitude;
                    upd_amplitude <= 1'b0;
                end
            end

        end
    end

    initial begin
        phase_inc = 32'd0;
        amplitude = 8'd0;
    end

endmodule
