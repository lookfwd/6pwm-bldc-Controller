// UART Receiver — 8N1, 115200 baud
// Double-flop synchronizer on RX input for metastability protection.
// Oversamples at 16x bit rate for robust edge detection.

module uart_rx #(
    parameter CLK_DIV = 716    // 82.5 MHz / 115200 = 716.14
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid    // 1-cycle pulse when byte is ready
);

    // Metastability synchronizer (2-FF)
    reg rx_sync1, rx_sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    wire rx_in = rx_sync2;

    // Baud rate generator (16x oversampling)
    localparam SAMPLE_DIV = CLK_DIV / 16;  // ~44 clocks per sample

    reg [9:0] sample_cnt;
    wire sample_tick = (sample_cnt == 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sample_cnt <= 0;
        else if (sample_cnt == SAMPLE_DIV - 1)
            sample_cnt <= 0;
        else
            sample_cnt <= sample_cnt + 1;
    end

    // State machine
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [3:0]  sample_idx;   // 0-15 within each bit
    reg [2:0]  bit_idx;      // 0-7 data bits
    reg [7:0]  shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            sample_idx <= 4'd0;
            bit_idx    <= 3'd0;
            shift_reg  <= 8'd0;
            data       <= 8'd0;
            valid      <= 1'b0;
        end else begin
            valid <= 1'b0;  // default: no valid pulse

            if (sample_tick) begin
                case (state)
                    S_IDLE: begin
                        if (rx_in == 1'b0) begin
                            // Detected falling edge — potential start bit
                            state      <= S_START;
                            sample_idx <= 4'd0;
                        end
                    end

                    S_START: begin
                        if (sample_idx == 4'd7) begin
                            // Mid-point of start bit — verify still low
                            if (rx_in == 1'b0) begin
                                sample_idx <= 4'd0;
                                bit_idx    <= 3'd0;
                                state      <= S_DATA;
                            end else begin
                                state <= S_IDLE;  // false start
                            end
                        end else begin
                            sample_idx <= sample_idx + 4'd1;
                        end
                    end

                    S_DATA: begin
                        if (sample_idx == 4'd15) begin
                            // Sample at mid-point of data bit
                            shift_reg  <= {rx_in, shift_reg[7:1]};  // LSB first
                            sample_idx <= 4'd0;
                            if (bit_idx == 3'd7) begin
                                state <= S_STOP;
                            end else begin
                                bit_idx <= bit_idx + 3'd1;
                            end
                        end else begin
                            sample_idx <= sample_idx + 4'd1;
                        end
                    end

                    S_STOP: begin
                        if (sample_idx == 4'd15) begin
                            if (rx_in == 1'b1) begin
                                // Valid stop bit
                                data  <= shift_reg;
                                valid <= 1'b1;
                            end
                            state <= S_IDLE;
                        end else begin
                            sample_idx <= sample_idx + 4'd1;
                        end
                    end
                endcase
            end
        end
    end

endmodule
