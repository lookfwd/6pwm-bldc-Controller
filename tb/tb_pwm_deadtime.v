// Testbench: PWM Phase-Correct Counter + Dead-Time Insertion
// Tests RUNNING mode SPWM and RUNNING→BRAKE transition for shoot-through
`timescale 1ns / 1ps

module tb_pwm_deadtime;

    reg clk;
    reg rst_n;
    reg [1:0] ctrl_state;

    // Clock: 82.5 MHz -> 12.12 ns period
    initial clk = 0;
    always #6.06 clk = ~clk;

    // PWM counter
    wire [10:0] counter;
    wire        sync;
    wire        direction;

    pwm_phase_correct u_pwm (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (1'b1),
        .counter   (counter),
        .sync      (sync),
        .direction (direction)
    );

    // Dead-time unit under test
    wire gate_h, gate_l;

    deadtime u_dt (
        .clk        (clk),
        .rst_n      (rst_n),
        .ctrl_state (ctrl_state),
        .counter    (counter),
        .duty       (11'd1024),     // 50% duty
        .dead_time  (8'd10),        // 10 clocks dead-time
        .gate_high  (gate_h),
        .gate_low   (gate_l)
    );

    // Shoot-through detection
    integer shoot_through_count = 0;
    always @(posedge clk) begin
        if (gate_h && gate_l) begin
            shoot_through_count = shoot_through_count + 1;
            $display("ERROR: Shoot-through at time %0t! gate_H=%b gate_L=%b",
                     $time, gate_h, gate_l);
        end
    end

    // Sync pulse counter
    integer sync_count = 0;
    always @(posedge clk) begin
        if (sync) sync_count = sync_count + 1;
    end

    // Test sequence
    initial begin
        $dumpfile("tb_pwm_deadtime.vcd");
        $dumpvars(0, tb_pwm_deadtime);

        rst_n = 0;
        ctrl_state = 2'd0;  // OPEN
        #100;
        rst_n = 1;

        // --- Test OPEN: both gates should stay LOW ---
        wait(sync_count >= 1);
        #100;
        $display("OPEN: gate_H=%b gate_L=%b (expect 0,0)", gate_h, gate_l);

        // --- Test RUNNING: SPWM with dead-time ---
        ctrl_state = 2'd1;
        wait(sync_count >= 4);
        #1000;
        $display("RUNNING: shoot-through events so far = %0d", shoot_through_count);

        // --- Test RUNNING→BRAKE transition (the critical case) ---
        // Switch to BRAKE mid-cycle while high-side might be active.
        // Dead-time module must turn off high-side immediately, then
        // wait dead_time clocks before turning on low-side.
        @(posedge clk);
        ctrl_state = 2'd2;  // BRAKE — deliberately mid-cycle
        $display("Switched to BRAKE at time %0t, counter=%0d", $time, counter);

        // Run a couple more cycles in BRAKE
        wait(sync_count >= 7);
        #1000;

        // --- Test BRAKE→OPEN ---
        ctrl_state = 2'd0;
        wait(sync_count >= 9);
        #1000;

        $display("--- PWM + Dead-Time Test Results ---");
        $display("Sync pulses observed: %0d", sync_count);
        $display("Shoot-through events: %0d", shoot_through_count);

        if (shoot_through_count == 0)
            $display("PASS: No shoot-through detected (including state transitions).");
        else
            $display("FAIL: Shoot-through detected!");

        $finish;
    end

    // Timeout
    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
