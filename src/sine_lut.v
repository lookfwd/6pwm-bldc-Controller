// 2048 x 16-bit Sine Lookup Table (Synchronous BRAM)
// Unsigned Offset Binary: 0x0000 = -peak, 0x8000 = zero, 0xFFFF = +peak
// Read latency: 1 clock cycle

module sine_lut (
    input  wire        clk,
    input  wire [10:0] addr,
    output reg  [15:0] data
);

    reg [15:0] mem [0:2047];

    initial begin
        $readmemh("sine_init.hex", mem);
    end

    always @(posedge clk) begin
        data <= mem[addr];
    end

endmodule
