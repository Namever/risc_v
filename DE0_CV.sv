module DE0_CV(
	input clk,
	input RESET_N,
	output [6:0] HEX0,
	output [6:0] HEX1,
	output [6:0] HEX2,
	output [6:0] HEX3,
	output [6:0] HEX4,
	output [6:0] HEX5
);

logic [31:0] regs_31;

Top u_Core(
    .clk    (clk),
    .rst    (~RESET_N),
    .regs_31(regs_31)
);

// Instance for 7-segment display for each HEX display
seven_segment_display u_seven_0(
	.digit (regs_31[3:0]),
	.seg (HEX0)
);

seven_segment_display u_seven_1(
	.digit (regs_31[7:4]),
	.seg (HEX1)
);

seven_segment_display u_seven_2(
	.digit (regs_31[11:8]),
	.seg (HEX2)
);

seven_segment_display u_seven_3(
	.digit (regs_31[15:12]),
	.seg (HEX3)
);

seven_segment_display u_seven_4(
	.digit (regs_31[19:16]),
	.seg (HEX4)
);

seven_segment_display u_seven_5(
	.digit (regs_31[23:20]),
	.seg (HEX5)
);

endmodule
