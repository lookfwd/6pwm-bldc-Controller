// Testbench: UART Receiver
`timescale 1ns / 1ps

module tb_uart_rx;

    reg clk;
    reg rst_n;
    reg rx;

    // Clock: 82.5 MHz
    initial clk = 0;
    always #6.06 clk = ~clk;

    wire [7:0] data;
    wire       valid;

    uart_rx #(
        .CLK_DIV(716)
    ) u_uart (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (rx),
        .data  (data),
        .valid (valid)
    );

    // Bit period for 115200 baud = ~8.68 us
    localparam BIT_PERIOD = 8681;  // ns

    // Task to send one byte (8N1, LSB first)
    task send_byte(input [7:0] byte_val);
        integer i;
        begin
            // Start bit
            rx = 1'b0;
            #BIT_PERIOD;
            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                rx = byte_val[i];
                #BIT_PERIOD;
            end
            // Stop bit
            rx = 1'b1;
            #BIT_PERIOD;
        end
    endtask

    // Capture received bytes
    reg [7:0] received [0:15];
    integer rx_count = 0;

    always @(posedge clk) begin
        if (valid) begin
            received[rx_count] = data;
            $display("Received byte %0d: 0x%02X (expected 0x%02X)",
                     rx_count, data,
                     rx_count == 0 ? 8'hA5 :
                     rx_count == 1 ? 8'h5A :
                     rx_count == 2 ? 8'h00 :
                     rx_count == 3 ? 8'hFF : 8'hXX);
            rx_count = rx_count + 1;
        end
    end

    initial begin
        $dumpfile("tb_uart_rx.vcd");
        $dumpvars(0, tb_uart_rx);

        rst_n = 0;
        rx    = 1'b1;  // idle high
        #200;
        rst_n = 1;
        #1000;

        // Send test bytes
        send_byte(8'hA5);
        #(BIT_PERIOD * 2);  // inter-byte gap

        send_byte(8'h5A);
        #(BIT_PERIOD * 2);

        send_byte(8'h00);
        #(BIT_PERIOD * 2);

        send_byte(8'hFF);
        #(BIT_PERIOD * 2);

        #10000;

        $display("--- UART RX Test Results ---");
        $display("Bytes received: %0d (expected 4)", rx_count);

        if (rx_count == 4 &&
            received[0] == 8'hA5 &&
            received[1] == 8'h5A &&
            received[2] == 8'h00 &&
            received[3] == 8'hFF)
            $display("PASS: All bytes received correctly.");
        else
            $display("FAIL: Mismatch in received bytes.");

        $finish;
    end

    // Timeout
    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
