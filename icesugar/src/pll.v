// PLL Clock Generator: 12 MHz -> 82.5 MHz
// F_VCO = 12 MHz * (DIVF+1) / (DIVR+1) = 12 * 55 / 1 = 660 MHz
// F_OUT = F_VCO / 2^DIVQ = 660 / 8 = 82.5 MHz
//
// 82.5 MHz is the lowest power-of-two-counter-friendly clock that gives
// PWM_freq = clk / (2 * 2048) = 20.14 kHz with 11-bit resolution. The
// exact target 81.92 MHz (for 20.000 kHz) is not reachable from 12 MHz
// via integer PLL divisors; 82.5 MHz is +0.7%, well within motor-control
// tolerance.

module pll (
    input  wire clk_12m,
    output wire clk_fast,
    output wire locked
);

    SB_PLL40_PAD #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'b0000),        // DIVR = 0   → ref divider = 1
        .DIVF(7'b0110110),     // DIVF = 54  → feedback divider = 55
        .DIVQ(3'b011),         // DIVQ = 3   → output divider = 2^3 = 8
        .FILTER_RANGE(3'b001)  // PFD range for 12 MHz ref
    ) u_pll (
        .PACKAGEPIN(clk_12m),
        .PLLOUTCORE(clk_fast),
        .LOCK(locked),
        .RESETB(1'b1),
        .BYPASS(1'b0)
    );

endmodule
