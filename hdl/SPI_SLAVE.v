module SPI_SLAVE
(
	input            CLK,
	input            MOSI,
	output reg       MISO,
	input            CSN,
	output reg [6:0] ADDR,
	output reg       WEN,
	output reg       REN,
	output reg [7:0] WD,
	input      [7:0] RD,
	output     [1:0] DBG
);

reg [3:0] b_cnt; // bit counter
reg [7:0] sr_in;
reg rnw; // read not write
reg [7:0] sr_out;

assign DBG[0] = b_cnt[0];
assign DBG[1] = rnw;

always @ (posedge CLK, posedge CSN) begin
	if (CSN) begin
		b_cnt <= 0;
		sr_in <= 0;
		rnw <= 0;
		ADDR <= 0;
		WEN <= 0;
		REN <= 0;
		WD <= 0;
	end else begin
		// Bit counter
		if (b_cnt == 15) begin
			b_cnt <= 8;
		end else begin
			b_cnt <= b_cnt + 4'd1;
		end
		
		// Input shift register
		sr_in <= {sr_in[6:0], MOSI};
		
		// Address
		if (b_cnt == 7) begin
			ADDR <= {sr_in[5:0], MOSI};
		end
		
		// Read/Write state
		if ((b_cnt == 1) && ~sr_in[0]) begin
			rnw <= 1;
		end
		
		// Read pulse
		if (((b_cnt == 7) || (b_cnt == 15)) && rnw) begin
			REN <= 1;
		end else begin
			REN <= 0;
		end
		
		// Write pulse
		if ((b_cnt == 15) && ~rnw) begin
			WEN <= 1;
			WD <= sr_in;
		end else begin
			WEN <= 0;
		end
		
		// Latch write data
		if ((b_cnt == 15) && ~rnw) begin
			WD <= {sr_in[6:0], MOSI};
		end
	end
end

always @ (negedge CLK, posedge CSN) begin
	if (CSN) begin
		MISO <= 0;
		sr_out <= 0;
	end else begin
		// Output shift register
		if (b_cnt == 8) begin
			sr_out <= {RD[6:0],1'b0};
		end else begin
			sr_out <= {sr_out[6:0],1'b0};
		end
		
		// MISO
		if (b_cnt == 8) begin
			MISO <= RD[7];
		end else if (b_cnt > 8) begin
			MISO <= sr_out[7];
		end
	end
end

endmodule