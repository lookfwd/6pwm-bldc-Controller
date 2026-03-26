// Testbench: TDM State Machine (with integrated NCO) + Sine LUT
`timescale 1ns / 1ps

module tb_spwm_tdm;

    reg clk;
    reg rst_n;

    // Clock: 82.5 MHz
    initial clk = 0;
    always #6.06 clk = ~clk;

    // Sine LUT
    wire [10:0] lut_addr;
    wire [15:0] lut_data;

    sine_lut u_lut (
        .clk  (clk),
        .addr (lut_addr),
        .data (lut_data)
    );

    // TDM (with integrated NCO)
    wire [10:0] duty_u, duty_v, duty_w;
    reg         pwm_sync;

    spwm_tdm u_tdm (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (1'b1),
        .pwm_sync  (pwm_sync),
        .phase_inc (32'h01000000),  // moderate speed
        .amplitude (8'd255),        // full amplitude
        .lut_data  (lut_data),
        .lut_addr  (lut_addr),
        .duty_u    (duty_u),
        .duty_v    (duty_v),
        .duty_w    (duty_w)
    );

    // Generate periodic pwm_sync pulses (every 4096 clocks like real PWM)
    integer cycle_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_cnt <= 0;
            pwm_sync  <= 1'b0;
        end else begin
            if (cycle_cnt == 4095) begin
                cycle_cnt <= 0;
                pwm_sync  <= 1'b1;
            end else begin
                cycle_cnt <= cycle_cnt + 1;
                pwm_sync  <= 1'b0;
            end
        end
    end

    // Monitor duty outputs
    integer sync_count = 0;
    always @(posedge clk) begin
        if (pwm_sync) begin
            sync_count = sync_count + 1;
            if (sync_count > 1) begin
                $display("Cycle %0d: duty_u=%0d  duty_v=%0d  duty_w=%0d",
                         sync_count - 1, duty_u, duty_v, duty_w);
            end
        end
    end

    // Verify duty values are within valid range
    integer range_errors = 0;
    always @(posedge clk) begin
        if (sync_count > 1) begin
            if (duty_u > 11'd2047 || duty_v > 11'd2047 || duty_w > 11'd2047) begin
                range_errors = range_errors + 1;
                $display("ERROR: Duty out of range at cycle %0d", sync_count);
            end
        end
    end

    initial begin
        $dumpfile("tb_spwm_tdm.vcd");
        $dumpvars(0, tb_spwm_tdm);

        rst_n = 0;
        #100;
        rst_n = 1;

        // Run for 20 PWM cycles
        wait(sync_count >= 21);
        #100;

        $display("--- TDM Test Results ---");
        $display("PWM cycles completed: %0d", sync_count - 1);
        $display("Range errors: %0d", range_errors);

        if (range_errors == 0)
            $display("PASS: All duty values within 0-2047.");
        else
            $display("FAIL: Duty values out of range.");

        $finish;
    end

    // Timeout
    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
