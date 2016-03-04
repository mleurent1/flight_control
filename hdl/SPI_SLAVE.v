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
	input            AUTO_INC_EN,
	output     [1:0] DBG
);

reg [2:0] bit_cnt; // bit counter
reg [7:0] byte_cnt; // byte counter
reg [7:0] sr_in;
reg rnw; // read not write
reg [7:0] sr_out;

assign DBG[0] = bit_cnt[0];
assign DBG[1] = rnw;

always @ (posedge CLK, posedge CSN) begin
	if (CSN) begin
		bit_cnt <= 0;
		byte_cnt <= 0;
		sr_in <= 0;
		rnw <= 0;
		ADDR <= 0;
		WEN <= 0;
		REN <= 0;
		WD <= 0;
	end else begin
		// Bit counter
		bit_cnt <= bit_cnt + 3'd1;
		
		// Byte counter
		if ((bit_cnt == 7) && (byte_cnt < 255))
			byte_cnt <= byte_cnt + 8'd1;
		
		// Input shift register
		sr_in <= {sr_in[6:0], MOSI};
		
		// Address
		if (byte_cnt == 0) begin
			if (bit_cnt == 7)
				ADDR <= {sr_in[5:0], MOSI};
		end else if (AUTO_INC_EN && ((rnw && (bit_cnt == 7)) || (~rnw && (bit_cnt == 0) && (byte_cnt > 1))))
			ADDR <= ADDR + 7'd1;
		
		// Read/Write state
		if ((bit_cnt == 1) && (byte_cnt == 0) && ~sr_in[0])
			rnw <= 1;
		
		// Read pulse
		//if ((bit_cnt == 7) && rnw)
		if ((bit_cnt == 6) && rnw) // make it happen 1 clock cycle before
			REN <= 1;
		else
			REN <= 0;
		
		// Write pulse
		if ((bit_cnt == 7) && (byte_cnt > 0) && ~rnw)
			WEN <= 1;
		else
			WEN <= 0;
		
		// Latch write data
		if ((bit_cnt == 7) && (byte_cnt > 0) && ~rnw)
			WD <= {sr_in[6:0], MOSI};
	end
end

always @ (negedge CLK, posedge CSN) begin
	if (CSN) begin
		MISO <= 0;
		sr_out <= 0;
	end else begin
		// Output shift register
		if ((bit_cnt == 0) && (byte_cnt > 0)) 
			sr_out <= {RD[6:0],1'b0};
		else
			sr_out <= {sr_out[6:0],1'b0};
		
		// MISO
		if (byte_cnt > 0) begin
			if (bit_cnt == 0) 
				MISO <= RD[7];
			else
				MISO <= sr_out[7];
		end
	end
end

endmodule