module SERVO
(
	input            CLK,
	input            RST,
	input      [9:0] COMMAND,
	input      [9:0] OFFSET,
	output reg       SERVO
);

reg [15:0] cnt;

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		SERVO <= 0;
		cnt <= 0;
	end else begin
		cnt <= cnt + 16'd1;
		if (cnt == 0)
			SERVO <= 1;
		else if (cnt[15:5] == ({1'b0,COMMAND}+{1'b0,OFFSET}))
			SERVO <= 0;
	end
end

endmodule