// Testbench: Top-Level Integration
// Sends UART commands and verifies gate output behavior.
// Dead-time modules handle OPEN/RUNNING/BRAKE natively — no output mux.
`timescale 1ns / 1ps

module tb_top;

    reg clk_sim;
    reg rst_n;
    reg rx;

    // 82.5 MHz sim clock
    initial clk_sim = 0;
    always #6.06 clk_sim = ~clk_sim;

    // Sub-module wiring
    wire [7:0]  rx_data;
    wire        rx_valid;
    wire [10:0] pwm_counter;
    wire        pwm_sync, pwm_dir;
    wire [1:0]  ctrl_state;
    wire [31:0] phase_inc;
    wire [7:0]  amplitude;
    wire [10:0] lut_addr;
    wire [15:0] lut_data;
    wire [10:0] duty_u, duty_v, duty_w;

    uart_rx u_uart (
        .clk(clk_sim), .rst_n(rst_n), .rx(rx),
        .data(rx_data), .valid(rx_valid)
    );

    pwm_phase_correct u_pwm (
        .clk(clk_sim), .rst_n(rst_n), .enable(1'b1),
        .counter(pwm_counter), .sync(pwm_sync), .direction(pwm_dir)
    );

    cmd_parser u_cmd (
        .clk(clk_sim), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .pwm_sync(pwm_sync),
        .ctrl_state(ctrl_state), .phase_inc(phase_inc), .amplitude(amplitude)
    );

    sine_lut u_lut (
        .clk(clk_sim), .addr(lut_addr), .data(lut_data)
    );

    wire running = (ctrl_state == 2'd1);
    spwm_tdm u_tdm (
        .clk(clk_sim), .rst_n(rst_n), .enable(running), .pwm_sync(pwm_sync),
        .phase_inc(phase_inc), .amplitude(amplitude),
        .lut_data(lut_data), .lut_addr(lut_addr),
        .duty_u(duty_u), .duty_v(duty_v), .duty_w(duty_w)
    );

    // Dead-time modules handle state natively — no output mux
    wire gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;

    deadtime u_dt_u (.clk(clk_sim), .rst_n(rst_n), .ctrl_state(ctrl_state),
        .counter(pwm_counter), .duty(duty_u), .dead_time(8'd10),
        .gate_high(gate_uh), .gate_low(gate_ul));
    deadtime u_dt_v (.clk(clk_sim), .rst_n(rst_n), .ctrl_state(ctrl_state),
        .counter(pwm_counter), .duty(duty_v), .dead_time(8'd10),
        .gate_high(gate_vh), .gate_low(gate_vl));
    deadtime u_dt_w (.clk(clk_sim), .rst_n(rst_n), .ctrl_state(ctrl_state),
        .counter(pwm_counter), .duty(duty_w), .dead_time(8'd10),
        .gate_high(gate_wh), .gate_low(gate_wl));

    wire [5:0] gate_out = {gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl};

    // Shoot-through detection across all channels
    integer shoot_through_count = 0;
    always @(posedge clk_sim) begin
        if ((gate_uh && gate_ul) || (gate_vh && gate_vl) || (gate_wh && gate_wl)) begin
            shoot_through_count = shoot_through_count + 1;
            $display("ERROR: Shoot-through at time %0t! gates=%06b", $time, gate_out);
        end
    end

    // UART bit-bang
    localparam BIT_PERIOD = 8681;  // ns for 115200 baud

    task send_byte(input [7:0] byte_val);
        integer i;
        begin
            rx = 1'b0;           // start bit
            #BIT_PERIOD;
            for (i = 0; i < 8; i = i + 1) begin
                rx = byte_val[i];
                #BIT_PERIOD;
            end
            rx = 1'b1;           // stop bit
            #BIT_PERIOD;
        end
    endtask

    task send_cmd(input [7:0] cmd, input [31:0] data_val);
        begin
            send_byte(cmd);
            send_byte(data_val[31:24]);
            send_byte(data_val[23:16]);
            send_byte(data_val[15:8]);
            send_byte(data_val[7:0]);
            #(BIT_PERIOD * 2);  // inter-packet gap
        end
    endtask

    // Wait for N sync pulses
    task wait_sync(input integer n);
        integer cnt;
        begin
            cnt = 0;
            while (cnt < n) begin
                @(posedge clk_sim);
                if (pwm_sync) cnt = cnt + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        rst_n = 0;
        rx = 1'b1;
        #200;
        rst_n = 1;

        // Wait for system to stabilize
        wait_sync(2);

        // --- Test 1: OPEN state (default) ---
        $display("Test 1: OPEN state — gates should be 000000");
        #100;
        if (gate_out == 6'b000000)
            $display("  PASS: gate_out = %06b", gate_out);
        else
            $display("  FAIL: gate_out = %06b (expected 000000)", gate_out);

        // --- Test 2: Set amplitude and speed, then switch to RUNNING ---
        $display("Test 2: Switch to RUNNING");
        send_cmd(8'h03, 32'h000000FF);       // amplitude = 255
        send_cmd(8'h02, 32'h01000000);       // speed
        send_cmd(8'h01, 32'h00000001);       // state = RUNNING

        // Wait for shadow registers to swap and a few PWM cycles
        wait_sync(5);
        #100;

        $display("  ctrl_state = %0d (expected 1)", ctrl_state);
        $display("  duty_u = %0d, duty_v = %0d, duty_w = %0d", duty_u, duty_v, duty_w);

        if (ctrl_state == 2'd1)
            $display("  PASS: In RUNNING state");
        else
            $display("  FAIL: Not in RUNNING state");

        // --- Test 3: Switch to BRAKE (tests dead-time on transition) ---
        $display("Test 3: Switch to BRAKE");
        send_cmd(8'h01, 32'h00000002);  // state = BRAKE

        wait_sync(2);
        #1000;  // allow dead-time to complete

        $display("  gate_out = %06b (expect 010101 after dead-time)", gate_out);
        if (gate_out == 6'b010101)
            $display("  PASS: BRAKE state correct");
        else
            $display("  FAIL: BRAKE state incorrect");

        // --- Test 4: Back to OPEN ---
        $display("Test 4: Back to OPEN");
        send_cmd(8'h01, 32'h00000000);  // state = OPEN

        wait_sync(2);
        #100;

        if (gate_out == 6'b000000)
            $display("  PASS: gate_out = %06b (OPEN)", gate_out);
        else
            $display("  FAIL: gate_out = %06b (expected 000000)", gate_out);

        $display("--- Integration Test Results ---");
        $display("Shoot-through events: %0d", shoot_through_count);
        if (shoot_through_count == 0)
            $display("PASS: No shoot-through across all state transitions.");
        else
            $display("FAIL: Shoot-through detected!");

        $display("--- Integration Test Complete ---");
        $finish;
    end

    // Timeout
    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
