module MPU_INIT
(
	input            CLK,
	input            RST,
	output reg [7:0] I2C_ADDR,
	output reg [7:0] I2C_WRITE_DATA,
	output reg       I2C_WRITE_EN,
	output reg       DONE
);

`ifdef FPGA
localparam [15:0] PAUSE = 16'd65535;
`else
localparam [15:0] PAUSE = 16'd512;
`endif

reg [7:0] write_cnt;
reg [15:0] pause_cnt;

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		write_cnt <= 0;
		pause_cnt <= 0;
		I2C_ADDR <= 0;
		I2C_WRITE_DATA <= 0;
		I2C_WRITE_EN <= 0;
		DONE <= 0;
	end else begin
		
		if (pause_cnt == PAUSE)
			pause_cnt <= 0;
		else
			pause_cnt <= pause_cnt + 15'd1;
		
		if ((pause_cnt == PAUSE) && (write_cnt <= 8'd5))
			write_cnt <= write_cnt + 8'd1;
		
		if (pause_cnt == 0) begin
			case (write_cnt)
				8'd0   : begin I2C_ADDR <= 107; I2C_WRITE_DATA <= 128; end
				8'd1   : begin I2C_ADDR <= 107; I2C_WRITE_DATA <= 0;   end
				8'd2   : begin I2C_ADDR <= 107; I2C_WRITE_DATA <= 1;   end
				8'd3   : begin I2C_ADDR <= 26;  I2C_WRITE_DATA <= 1;   end
				8'd4   : begin I2C_ADDR <= 56;  I2C_WRITE_DATA <= 1;   end
				default: begin I2C_ADDR <= 0;   I2C_WRITE_DATA <= 0;   end
			endcase
		end
		
		if (pause_cnt == 0)
			I2C_WRITE_EN <= 0;
		else
			I2C_WRITE_EN <= 1;
		
		if (write_cnt <= 8'd4)
			DONE <= 0;
		else
			DONE <= 1;
	end
end

endmodule