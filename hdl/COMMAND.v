module COMMAND
(
	input            CLK,
	input            RST,
	input            SERVO,
	input      [9:0] OFFSET,
	output reg [9:0] COMMAND
);

reg  [15:0] cnt;
reg  [ 3:0] SERVO_dl;
wire [10:0] x = cnt[15:5] - {1'b0,OFFSET};

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		COMMAND <= 0;
		cnt <= 0;
		SERVO_dl <= 0;
	end else begin
		SERVO_dl <= {SERVO_dl[2:0],SERVO};
		if (SERVO_dl[3])
			cnt <= cnt + 16'd1;
		else
			cnt <= 0;
		if (SERVO_dl[3] && ~SERVO_dl[2])
			COMMAND <= x[9:0];
	end
end

endmodule