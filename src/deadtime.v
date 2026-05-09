// Dead-Time Insertion for One Half-Bridge — direct counter-vs-threshold compares.
//
// The asymmetry between up- and down-counting paths inserts dead-time naturally:
//   Up-counting   : gate_high = (counter <  duty),           gate_low = (counter >= duty + dead_time)
//   Down-counting : gate_high = (counter <  duty - dead_time), gate_low = (counter >= duty)
//
// The four 11-bit compares are registered to keep the carry chains off the
// gate-FF setup path (otherwise nextpnr can't route the full chain + state mux
// in one fast cycle). Net latency: 2 fast cycles from counter change to gate
// (~24 ns at 82.5 MHz) — invisible compared to the 50 µs PWM period.
//
// Dead-time on ctrl_state transitions is enforced upstream in cmd_parser, which
// inserts an OPEN period of ≥ dead_time fast cycles before any non-OPEN state.
// That guarantees no shoot-through across RUNNING ↔ BRAKE.

module deadtime (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  ctrl_state,    // 0=OPEN, 1=RUNNING, 2=BRAKE
    input  wire        direction,     // 0 = counting up, 1 = counting down
    input  wire [9:0]  counter,
    input  wire [9:0]  duty,
    input  wire [9:0]  duty_minus_dt, // saturating: max(duty - dead_time, 0)
    input  wire [9:0]  duty_plus_dt,  // saturating: min(duty + dead_time, 1023)
    output reg         gate_high,
    output reg         gate_low
);

    // Registered compare results — cuts the 11-bit carry chain off the
    // gate-FF critical path. Each compare is its own short flop-to-flop hop.
    reg cmp_lt_duty;          // counter <  duty
    reg cmp_lt_duty_minus_dt; // counter <  duty - dead_time
    reg cmp_ge_duty;          // counter >= duty
    reg cmp_ge_duty_plus_dt;  // counter >= duty + dead_time
    reg dir_reg;              // registered alongside the compares for phase alignment

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmp_lt_duty          <= 1'b0;
            cmp_lt_duty_minus_dt <= 1'b0;
            cmp_ge_duty          <= 1'b0;
            cmp_ge_duty_plus_dt  <= 1'b0;
            dir_reg              <= 1'b0;
        end else begin
            cmp_lt_duty          <= (counter <  duty);
            cmp_lt_duty_minus_dt <= (counter <  duty_minus_dt);
            cmp_ge_duty          <= (counter >= duty);
            cmp_ge_duty_plus_dt  <= (counter >= duty_plus_dt);
            dir_reg              <= direction;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_high <= 1'b0;
            gate_low  <= 1'b0;
        end else begin
            case (ctrl_state)
                2'd1: begin    // RUNNING — pick the right registered compare
                    gate_high <= (dir_reg == 1'b0) ? cmp_lt_duty
                                                   : cmp_lt_duty_minus_dt;
                    gate_low  <= (dir_reg == 1'b0) ? cmp_ge_duty_plus_dt
                                                   : cmp_ge_duty;
                end
                2'd2: begin    // BRAKE — caller guarantees OPEN-sandwich before entry
                    gate_high <= 1'b0;
                    gate_low  <= 1'b1;
                end
                default: begin // OPEN (00) or invalid (11)
                    gate_high <= 1'b0;
                    gate_low  <= 1'b0;
                end
            endcase
        end
    end

endmodule
