// Phase-Correct (Up-Down) PWM Counter
// Counts 0 -> 2047 -> 0 (triangular waveform)
// sync pulse at counter == 0 (bottom of triangle)

module pwm_phase_correct (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    output reg  [10:0] counter,
    (* keep = "true" *)
    output reg         sync,       // registered: pulses the cycle after counter == 0
    output reg         direction   // 0 = counting up, 1 = counting down
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter   <= 11'd0;
            direction <= 1'b0;
            sync      <= 1'b0;
        end else if (enable) begin
            sync <= (counter == 11'd0) && (direction == 1'b0);
            if (direction == 1'b0) begin
                // Counting up: 0, 1, 2, ..., 2046, 2047, then reverse
                if (counter == 11'd2047)
                    direction <= 1'b1;
                else
                    counter <= counter + 11'd1;
            end else begin
                // Counting down: 2046, 2045, ..., 1, 0, then reverse
                if (counter == 11'd0)
                    direction <= 1'b0;
                else
                    counter <= counter - 11'd1;
            end
        end
    end

endmodule
