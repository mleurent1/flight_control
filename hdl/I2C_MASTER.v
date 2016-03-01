module I2C_MASTER
(
	input            CLK,
	input            RST,
	output reg       SCL,
	input            SDA_IN,
	output reg       SDA_OUT,
	output reg       SDA_DIR, // 0:output 1:input
	input      [6:0] DEVICE_ADDR,
	input      [7:0] REG_ADDR,
	input            READ_EN,
	input      [7:0] READ_SIZE,
	output reg [7:0] READ_DATA,
	output reg       READ_VALID,
	input            WRITE_EN,
	input      [7:0] WRITE_DATA,
	output reg       NACK,
	output reg       BUSY,
	input      [7:0] CLK_DIV
);

localparam [3:0] IDLE = 4'd0;
localparam [3:0] START = 4'd1;
localparam [3:0] START2 = 4'd2;
localparam [3:0] STOP = 4'd3;
localparam [3:0] DEVICE_W = 4'd4;
localparam [3:0] DEVICE_R = 4'd5;
localparam [3:0] ADDR = 4'd6;
localparam [3:0] WRITE = 4'd7;
localparam [3:0] READ = 4'd8;

reg [3:0] state;
reg [3:0] next_state;
reg [7:0] f_cnt;
reg [1:0] c_cnt;
reg [3:0] b_cnt;
reg c_pulse;
//reg b_pulse;
wire b_pulse = c_pulse && (c_cnt == 2'd3);
reg [3:0] c_pulse_dl;
reg [1:0] b_pulse_dl;
reg [3:0] WEN_dl;
reg [3:0] REN_dl;
reg [8:0] sr_in;
reg [8:0] sr_out;
reg [7:0] bytes;

always @ * begin
	case (state)
		IDLE:
			if ((~WEN_dl[3] && WEN_dl[2]) || (~REN_dl[3] && REN_dl[2])) begin
				next_state <= START;
			end else begin
				next_state <= IDLE;
			end
		START:
			if (b_pulse) begin
				next_state <= DEVICE_W;
			end else begin
				next_state <= START;
			end
		DEVICE_W:
			if (b_pulse) begin
				if (NACK) begin
					next_state <= STOP;
				end else if (b_cnt == 4'd8) begin
					next_state <= ADDR;
				end else begin
					next_state <= DEVICE_W;
				end
			end else begin
				next_state <= DEVICE_W;
			end
		ADDR:
			if (b_pulse) begin
				if (NACK) begin
					next_state <= STOP;
				end else if (b_cnt == 4'd8) begin
					if (REN_dl[3]) begin
						next_state <= START2;
					end else if (WEN_dl[3]) begin
						next_state <= WRITE;
					end else begin
						next_state <= STOP;
					end
				end else begin
					next_state <= ADDR;
				end
			end else begin
				next_state <= ADDR;
			end
		WRITE:
			if (((b_cnt == 4'd8) && b_pulse) || (NACK && b_pulse)) begin
				next_state <= STOP;
			end else begin
				next_state <= WRITE;
			end
		START2:
			if (b_pulse) begin
				next_state <= DEVICE_R;
			end else begin
				next_state <= START2;
			end
		DEVICE_R:
			if (b_pulse) begin
				if (NACK) begin
					next_state <= STOP;
				end else if (b_cnt == 4'd8) begin
					next_state <= READ;
				end else begin
					next_state <= DEVICE_R;
				end
			end else begin
				next_state <= DEVICE_R;
			end
		READ:
			if ((b_cnt == 4'd8) && b_pulse && (bytes == READ_SIZE)) begin
				next_state <= STOP;
			end else begin
				next_state <= READ;
			end
		STOP:
			if (b_pulse) begin
				next_state <= IDLE;
			end else begin
				next_state <= STOP;
			end
		default:
			next_state <= IDLE;
	endcase
end

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		state <= IDLE;
		WEN_dl <= 0;
		REN_dl <= 0;
		f_cnt <= 0;
		c_cnt <= 0;
		b_cnt <= 0;
		c_pulse <= 0;
		//b_pulse <= 0;
		c_pulse_dl <= 0;
		b_pulse_dl <= 0;
		sr_out <= 0;
		sr_in <= 0;
		SCL <= 1;
		SDA_OUT <= 1;
		SDA_DIR <= 0;
		NACK <= 0;
		BUSY <= 0;
		READ_DATA <= 0;
		READ_VALID <= 0;
		bytes <= 0;
	end else begin
		state <= next_state;
		
		// delays
		WEN_dl <= {WEN_dl[2:0],WRITE_EN};
		REN_dl <= {REN_dl[2:0],READ_EN};
		c_pulse_dl <= {c_pulse_dl[2:0],c_pulse};
		b_pulse_dl <= {b_pulse_dl[0],b_pulse};
		
		// freq counter
		if ((state == IDLE) || (f_cnt == CLK_DIV)) begin
			f_cnt <= 0;
		end else begin
			f_cnt <= f_cnt + 8'd1;
		end
		
		// clock counter pulse
		if (f_cnt == CLK_DIV) begin
			c_pulse <= 1;
		end else begin
			c_pulse <= 0;
		end
		
		// clock counter
		if (state == IDLE) begin
			c_cnt <= 0;
		end else if (c_pulse) begin
			c_cnt <= c_cnt + 2'd1;
		end
		
		// bit counter pulse
		//b_pulse <= c_pulse && (c_cnt == 2'd3);
		
		// bit counter
		if ((state == IDLE) || (state == START) || (state == START2)) begin
			b_cnt <= 0;
		end else if (b_pulse) begin
			if (b_cnt == 4'd8) begin
				b_cnt <= 0;
			end else begin
				b_cnt <= b_cnt + 4'd1;
			end
		end
		
		// sr_out
		if (b_pulse_dl[1]) begin
			if (b_cnt == 4'd0) begin
				if (state == DEVICE_W) begin
					sr_out <= {DEVICE_ADDR,1'b0,1'b0};
				end else if (state == DEVICE_R) begin
					sr_out <= {DEVICE_ADDR,1'b1,1'b0};
				end else if (state == ADDR) begin
					sr_out <= {REG_ADDR,1'b0};
				end else if (state == WRITE) begin
					sr_out <= {WRITE_DATA,1'b0};
				end
			end else begin
				sr_out <= {sr_out[7:0],1'b0};
			end
		end
		
		// sr_in
		if ((c_cnt == 2'd2) && c_pulse_dl[3]) begin
			sr_in <= {sr_in[7:0],SDA_IN};
		end
		
		// SCL
		if (state == IDLE) begin
			SCL <= 1;
		end else if (c_pulse_dl[3]) begin
			if (state == START) begin
				SCL <= (c_cnt == 2'd3) ? 1'b0 : 1'b1;
			end else if (state == STOP) begin
				SCL <= (c_cnt == 2'd0) ? 1'b0 : 1'b1;
			end else begin
				SCL <= ((c_cnt == 2'd0) || (c_cnt == 2'd3)) ? 1'b0 : 1'b1;
			end
		end
		
		// SDA_OUT
		if (state == IDLE) begin
			SDA_OUT <= 1;
		end else if (c_pulse_dl[3]) begin
			if ((state == START) || (state == START2)) begin
				SDA_OUT <= (c_cnt >= 2'd2) ? 1'b0 : 1'b1;
			end else if (state == STOP) begin
				SDA_OUT <= (c_cnt <= 2'd1) ? 1'b0 : 1'b1;
			end else if ((state == READ) && (b_cnt == 4'd8) && (c_cnt == 2'd0)) begin
				SDA_OUT <= (bytes < READ_SIZE) ? 1'b0: 1'b1;
			end else if (c_cnt == 2'd0) begin
				SDA_OUT <= sr_out[8];
			end
		end
		
		// SDA_DIR
		if ((((state == DEVICE_W) || (state == DEVICE_R) || (state == ADDR) || (state == WRITE)) && (b_cnt == 4'd8)) || ((state == READ) && (b_cnt < 4'd8))) begin
			SDA_DIR <= 1;
		end else begin
			SDA_DIR <= 0;
		end
		
		// BUSY
		if (state == IDLE) begin
			BUSY <= 0;
		end else begin
			BUSY <= 1;
		end
		
		// NACK
		if (~WEN_dl[3] && ~REN_dl[3]) begin
			NACK <= 0;
		end else if (((state == DEVICE_W) || (state == DEVICE_R) || (state == ADDR) || (state == WRITE)) && (b_cnt == 4'd8) && (c_cnt == 2'd2) && c_pulse_dl[3] && SDA_IN) begin
			NACK <= 1;
		end
		
		// READ_DATA
		if ((state == READ) && (b_cnt == 4'd7) && (c_cnt == 2'd3) && c_pulse_dl[3]) begin
			READ_DATA <= sr_in[7:0];
		end
		
		// READ_VALID
		if ((state == READ) && (b_cnt == 4'd8)) begin
			READ_VALID <= 1;
		end else begin
			READ_VALID <= 0;
		end
		
		// bytes
		if (state == IDLE) begin
			bytes <= 0;
		end else if ((state == READ) && (b_cnt == 4'd7) && b_pulse) begin
			bytes <= bytes + 7'd1;
		end
		
	end
end



endmodule