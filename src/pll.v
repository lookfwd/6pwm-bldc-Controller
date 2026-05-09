// PLL Clock Generator: 12 MHz -> 82.5 MHz
// F_VCO = 12 MHz * (DIVF+1) / (DIVR+1) = 12 * 55 / 1 = 660 MHz
// F_OUT = F_VCO / 2^DIVQ = 660 / 8 = 82.5 MHz

module pll (
    input  wire clk_12m,
    output wire clk_82m5,
    output wire locked
);

    SB_PLL40_PAD #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'b0000),        // DIVR = 0
        .DIVF(7'b0110110),     // DIVF = 54
        .DIVQ(3'b011),         // DIVQ = 3
        .FILTER_RANGE(3'b001)  // PFD range for 12 MHz ref
    ) u_pll (
        .PACKAGEPIN(clk_12m),
        .PLLOUTCORE(clk_82m5),
        .LOCK(locked),
        .RESETB(1'b1),
        .BYPASS(1'b0)
    );

endmodule
