// Debug Telemetry Framer
//
// Watches the six 11-bit dead-time-adjusted duty thresholds emitted by
// pwm_gate_unit. Whenever any of them differs from the last snapshot
// that was successfully transmitted, latches a new snapshot and ships
// it out byte-by-byte through `uart_tx`.
//
// Frame layout (14 bytes total, big-endian):
//   [0]  0xAA          sync byte 1
//   [1]  0x55          sync byte 2
//   [2]  duty_u_minus[10:8] (high 5 bits = 0)
//   [3]  duty_u_minus[7:0]
//   [4]  duty_u_plus[10:8]
//   [5]  duty_u_plus[7:0]
//   [6]  duty_v_minus[10:8]
//   [7]  duty_v_minus[7:0]
//   [8]  duty_v_plus[10:8]
//   [9]  duty_v_plus[7:0]
//   [10] duty_w_minus[10:8]
//   [11] duty_w_minus[7:0]
//   [12] duty_w_plus[10:8]
//   [13] duty_w_plus[7:0]
//
// The duty values are 11-bit unsigned, so the "high" byte never exceeds
// 0x07 — the sync bytes 0xAA/0x55 cannot occur in data, so the host can
// resync on any frame boundary by hunting for the AA-55 pattern.
//
// At 115200 baud a 14-byte frame takes ~1.2 ms, so the effective sample
// rate is ~820 Hz — naturally rate-limiting the (much faster) duty
// updates. While a frame is in flight, new duty changes are simply
// observed at the end and trigger the next frame.

module debug_tx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [10:0] duty_u_minus,
    input  wire [10:0] duty_u_plus,
    input  wire [10:0] duty_v_minus,
    input  wire [10:0] duty_v_plus,
    input  wire [10:0] duty_w_minus,
    input  wire [10:0] duty_w_plus,
    // To uart_tx
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_ready
);

    // Latched snapshot — the values currently being shipped.
    reg [10:0] u_minus_l, u_plus_l;
    reg [10:0] v_minus_l, v_plus_l;
    reg [10:0] w_minus_l, w_plus_l;

    reg [3:0] byte_idx;   // 0..13
    reg       sending;
    reg       prev_ready; // rising-edge detector for tx_ready

    wire ready_rise = tx_ready & ~prev_ready;

    wire change = (u_minus_l != duty_u_minus) |
                  (u_plus_l  != duty_u_plus)  |
                  (v_minus_l != duty_v_minus) |
                  (v_plus_l  != duty_v_plus)  |
                  (w_minus_l != duty_w_minus) |
                  (w_plus_l  != duty_w_plus);

    // Byte selector — combinational from byte_idx + latched values.
    always @(*) begin
        case (byte_idx)
            4'd0:  tx_data = 8'hAA;
            4'd1:  tx_data = 8'h55;
            4'd2:  tx_data = {5'b0, u_minus_l[10:8]};
            4'd3:  tx_data = u_minus_l[7:0];
            4'd4:  tx_data = {5'b0, u_plus_l[10:8]};
            4'd5:  tx_data = u_plus_l[7:0];
            4'd6:  tx_data = {5'b0, v_minus_l[10:8]};
            4'd7:  tx_data = v_minus_l[7:0];
            4'd8:  tx_data = {5'b0, v_plus_l[10:8]};
            4'd9:  tx_data = v_plus_l[7:0];
            4'd10: tx_data = {5'b0, w_minus_l[10:8]};
            4'd11: tx_data = w_minus_l[7:0];
            4'd12: tx_data = {5'b0, w_plus_l[10:8]};
            4'd13: tx_data = w_plus_l[7:0];
            default: tx_data = 8'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_minus_l  <= 11'd0;
            u_plus_l   <= 11'd0;
            v_minus_l  <= 11'd0;
            v_plus_l   <= 11'd0;
            w_minus_l  <= 11'd0;
            w_plus_l   <= 11'd0;
            byte_idx   <= 4'd0;
            sending    <= 1'b0;
            tx_start   <= 1'b0;
            prev_ready <= 1'b1;
        end else begin
            prev_ready <= tx_ready;
            tx_start   <= 1'b0;       // default — overridden when a byte is launched

            if (!sending) begin
                if (change && tx_ready) begin
                    // Take a fresh snapshot and kick off byte 0 (= 0xAA).
                    u_minus_l <= duty_u_minus;
                    u_plus_l  <= duty_u_plus;
                    v_minus_l <= duty_v_minus;
                    v_plus_l  <= duty_v_plus;
                    w_minus_l <= duty_w_minus;
                    w_plus_l  <= duty_w_plus;
                    byte_idx  <= 4'd0;
                    sending   <= 1'b1;
                    tx_start  <= 1'b1;
                end
            end else begin
                // Each tx_ready rising edge means the previous byte just
                // finished; either advance to the next byte or close the
                // frame.
                if (ready_rise) begin
                    if (byte_idx == 4'd13) begin
                        sending <= 1'b0;
                    end else begin
                        byte_idx <= byte_idx + 4'd1;
                        tx_start <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
