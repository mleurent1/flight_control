module CODE2PPM
(
	input             CLK,
	input             RST,
	input      [15:0] CODE,
	input             VALID,
	output reg        PPM
);

reg [15:0] cnt;
reg [15:0] CODE_latch;

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		cnt <= 0;
		CODE_latch <= 0;
		PPM <= 0;
	end else begin
		if (~&cnt) //<65535
			cnt <= cnt + 16'd1;
		else if (VALID) begin
			cnt <= 0;
			CODE_latch <= CODE;
		end
		if ((cnt < CODE_latch) && (CODE_latch != 0))
			PPM <= 1;
		else 
			PPM <= 0;
	end
end

endmodule