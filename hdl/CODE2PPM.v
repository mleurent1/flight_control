module CODE2PPM
(
	input             CLK,
	input             RST,
	input      [15:0] CODE,
	output reg        PPM
);

reg [15:0] cnt;

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		cnt <= 0;
		PPM <= 0;
	end else begin
		cnt <= cnt + 16'd1;
		if ((cnt == 0) && (CODE != 0))
			PPM <= 1;
		else if (cnt == CODE)
			PPM <= 0;
	end
end

endmodule