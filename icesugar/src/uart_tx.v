// UART Transmitter — 8N1, matches uart_rx CLK_DIV.
//
// Hand-shake:
//   - When `ready` is high, the transmitter is idle. Drive `data` and
//     pulse `start` high for one clock to begin sending that byte.
//   - `ready` goes low for the duration of transmission (~10*CLK_DIV
//     clocks) and returns high when the stop bit completes.
//   - `tx` is combinational and idles high.

module uart_tx #(
    parameter CLK_DIV = 176   // slow_clk / baud  (20.625 MHz / 115200 ≈ 179)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       start,
    output wire       ready,
    output wire       tx
);

    localparam IDLE = 1'b0;
    localparam SEND = 1'b1;

    reg        state;
    reg  [9:0] shift;       // {stop=1, data[7:0], start=0} — LSB out first
    reg  [3:0] bit_idx;     // 0..9
    reg [15:0] tick_cnt;    // counts 0..CLK_DIV-1 per bit

    assign ready = (state == IDLE);
    assign tx    = (state == IDLE) ? 1'b1 : shift[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            shift    <= 10'b11_1111_1111;
            bit_idx  <= 4'd0;
            tick_cnt <= 16'd0;
        end else case (state)
            IDLE: begin
                if (start) begin
                    shift    <= {1'b1, data, 1'b0};
                    state    <= SEND;
                    bit_idx  <= 4'd0;
                    tick_cnt <= 16'd0;
                end
            end
            SEND: begin
                if (tick_cnt == CLK_DIV - 1) begin
                    tick_cnt <= 16'd0;
                    if (bit_idx == 4'd9) begin
                        state <= IDLE;
                    end else begin
                        shift   <= {1'b1, shift[9:1]};
                        bit_idx <= bit_idx + 4'd1;
                    end
                end else begin
                    tick_cnt <= tick_cnt + 16'd1;
                end
            end
        endcase
    end

endmodule
