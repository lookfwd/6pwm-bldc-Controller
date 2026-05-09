// PLL Clock Generator: 12 MHz -> 50.25 MHz
// F_VCO = 12 MHz * (DIVF+1) / (DIVR+1) = 12 * 67 / 1 = 804 MHz
// F_OUT = F_VCO / 2^DIVQ = 804 / 16 = 50.25 MHz

module pll (
    input  wire clk_12m,
    output wire clk_50m,
    output wire locked
);

    SB_PLL40_PAD #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'b0000),        // DIVR = 0
        .DIVF(7'b1000010),     // DIVF = 66
        .DIVQ(3'b100),         // DIVQ = 4
        .FILTER_RANGE(3'b001)  // PFD range for 12 MHz ref
    ) u_pll (
        .PACKAGEPIN(clk_12m),
        .PLLOUTCORE(clk_50m),
        .LOCK(locked),
        .RESETB(1'b1),
        .BYPASS(1'b0)
    );

endmodule
