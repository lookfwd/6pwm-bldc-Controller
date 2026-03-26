// Dead-Time Insertion for One Half-Bridge
// Handles OPEN/RUNNING/BRAKE natively — no external output mux needed.
// This guarantees dead-time is always respected during state transitions.
//
// OPEN    (0): both gates LOW
// RUNNING (1): SPWM with dead-time insertion
// BRAKE   (2): high-side LOW, low-side HIGH (with dead-time on entry)

module deadtime (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  ctrl_state,   // 0=OPEN, 1=RUNNING, 2=BRAKE
    input  wire [10:0] counter,
    input  wire [10:0] duty,
    input  wire [7:0]  dead_time,    // guard interval in clock cycles
    output reg         gate_high,
    output reg         gate_low
);

    // Desired gate state before dead-time enforcement
    reg req_high, req_low;

    always @(*) begin
        case (ctrl_state)
            2'd0: begin    // OPEN — all off
                req_high = 1'b0;
                req_low  = 1'b0;
            end
            2'd1: begin    // RUNNING — SPWM
                req_high = (counter < duty);
                req_low  = ~(counter < duty);
            end
            2'd2: begin    // BRAKE — low-side on
                req_high = 1'b0;
                req_low  = 1'b1;
            end
            default: begin
                req_high = 1'b0;
                req_low  = 1'b0;
            end
        endcase
    end

    // Dead-time enforcement: any 0→1 transition is delayed by dead_time clocks.
    // Any 1→0 transition is immediate. This ensures both gates are never on
    // simultaneously, including during state transitions (e.g. RUNNING→BRAKE).

    reg [7:0] dt_cnt_h, dt_cnt_l;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_high <= 1'b0;
            gate_low  <= 1'b0;
            dt_cnt_h  <= 8'd0;
            dt_cnt_l  <= 8'd0;
        end else begin
            // High-side dead-time logic
            if (!req_high) begin
                gate_high <= 1'b0;
                dt_cnt_h  <= 8'd0;
            end else if (dt_cnt_h < dead_time) begin
                dt_cnt_h  <= dt_cnt_h + 8'd1;
                gate_high <= 1'b0;
            end else begin
                gate_high <= 1'b1;
            end

            // Low-side dead-time logic
            if (!req_low) begin
                gate_low <= 1'b0;
                dt_cnt_l <= 8'd0;
            end else if (dt_cnt_l < dead_time) begin
                dt_cnt_l <= dt_cnt_l + 8'd1;
                gate_low <= 1'b0;
            end else begin
                gate_low <= 1'b1;
            end
        end
    end

endmodule
