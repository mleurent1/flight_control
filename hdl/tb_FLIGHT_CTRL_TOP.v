`timescale 1ns/1ps

module tb_FLIGHT_CTRL_TOP();

parameter SPI_CLK_HALF_PERIOD_NS = 50;
parameter I2C_CLK_QUARTER_PERIOD_NS = 200;

reg clk_25;
reg rst;
reg spi_clk, spi_csn, spi_mosi;
wire spi_miso;
reg [7:0] spi_data [4095:0];
wire scl, sda_out0, sda_dir;
reg sda_in;
reg [3:0] ack_en;
reg mpu_int;
integer j;

wire sda_out = sda_out0 & ~sda_dir;

initial begin
	clk_25 = 0;
	rst = 1;
	spi_clk = 0;
	spi_csn = 1;
	spi_mosi = 0;
	sda_in = 1;
	ack_en = 15;
	mpu_int = 0;
	
	#6000
	rst = 0;
	
	// I2C clock div
	spi_data[0] = I2C_CLK_QUARTER_PERIOD_NS/40-1;
	spi_write(4,1); // clock div
	
	// Wait for end of MPU initialisation
	@ (posedge FLIGHT_CTRL_TOP_inst.mpu_init_done);
	
	// Read
	spi_read(5,1);
	spi_read(26,1);
	
	// Read burst
	spi_read(0,5);
	
	// Write (motor test)
	spi_data[0] = 70;
	spi_write(7,1);
	spi_data[0] = 80;
	spi_write(8,1);
	
	// Read back
	spi_read(7,2);
	
	// Write burst (sensor test)
	spi_data[0] = 90;
	spi_data[1] = 100;
	spi_data[2] = 110;
	spi_data[3] = 120;
	spi_data[4] = 130;
	spi_write(19,5);
	
	// Read back
	spi_read(19,5);
	
	#1000
	
	// I2C write
	spi_data[0] = 170;
	spi_write(1,1); // addr
	spi_data[0] = 207;
	spi_write(2,1); // data to write
	spi_data[0] = 16+8+2;
	spi_write(5,1); // WEN
	
	i2c_slave(0);
	
	spi_data[0] = 16+8;
	spi_write(5,1); // unset WEN
	
	// I2C read
	spi_data[0] = 78;
	spi_write(1,1); // addr
	spi_data[0] = 16+8+4;
	spi_write(5,1); // REN
	
	i2c_slave(202);
	
	spi_data[0] = 16+8;
	spi_write(5,1); // unset REN
	
	// I2C read burst
	spi_data[0] = 98;
	spi_write(1,1); // addr
	spi_data[0] = 3;
	spi_write(3,1); // read size
	spi_data[0] = 16+8+4;
	spi_write(5,1); // REN
	
	i2c_slave(143);
	
	spi_data[0] = 16+8;
	spi_write(5,1); // unset REN
	
	// I2C test ack_en
	for (j=0; j<3; j=j+1) begin
		ack_en = 15;
		ack_en[j] = 0;
		spi_data[0] = 16+8+2;
		spi_write(5,1); // WEN
		
		i2c_slave(0);
		
		spi_data[0] = 16+8;
		spi_write(5,1); // unset WEN
	end
	ack_en = 15;
	ack_en[3] = 0;
	spi_data[0] = 16+8+4;
	spi_write(5,1); // REN
	
	i2c_slave(0);
	
	spi_data[0] = 16+8;
	spi_write(5,1); // unset REN
	
	#1000
	
	// sensor collect test
	ack_en = 15;
	spi_data[0] = 16;
	spi_write(5,1); // release SPI host control of I2C
	mpu_int = 1;
	i2c_slave(74);
	mpu_int = 0;
	
	#1000
	
	mpu_int = 1;
	i2c_slave(119);
	
end

always begin
	#20 clk_25 <= ~clk_25;
end

FLIGHT_CTRL_TOP FLIGHT_CTRL_TOP_inst
(
	.CLK         (clk_25),
	.RST         (rst),
	.SPI_CLK     (spi_clk),
	.SPI_MOSI    (spi_mosi),
	.SPI_MISO    (spi_miso),
	.SPI_CSN     (spi_csn),
	.THROTTLE    (1'b0),
	.AILERON     (1'b0),
	.ELEVATOR    (1'b0),
	.RUDDER      (1'b0),
	.MOTOR       (),
	.MPU_SCL     (scl),
	.MPU_SDA_IN  (sda_in & sda_dir),
	.MPU_SDA_OUT (sda_out0),
	.MPU_SDA_DIR (sda_dir),
	.MPU_INT     (mpu_int),
	.FPGA_CTRL   (),
	.FPGA_STATUS (8'd0),
	.LED         (),
	.DBG         ()
);


task spi_write;

	input [6:0] addr;
	input [11:0] size;
	
	reg [7:0] serout;
	reg [7:0] serin;
	
	integer i,j;
	
	begin
		
		spi_clk = 1'b0;
		#SPI_CLK_HALF_PERIOD_NS
		spi_csn = 1'b0;
		
		// Address
		serout = {1'b1,addr};
		for (i=0; i<8; i=i+1) begin
			spi_mosi = serout[7];
			#SPI_CLK_HALF_PERIOD_NS
			spi_clk = 1'b1;
			serin[7-i] = spi_miso;
			#SPI_CLK_HALF_PERIOD_NS
			spi_clk = 1'b0;
			serout = serout << 1;
		end
		
		// Data
		for (j=0; j<size; j=j+1) begin
			serout = spi_data[j];
			for (i=0; i<8; i=i+1) begin
				spi_mosi = serout[7];
				#SPI_CLK_HALF_PERIOD_NS
				spi_clk = 1'b1;
				serin[7-i] = spi_miso;
				#SPI_CLK_HALF_PERIOD_NS
				spi_clk = 1'b0;
				serout = serout << 1;
			end
			$write("SPI write @ %d: %d\n", addr, spi_data[j]);
		end
		
		#SPI_CLK_HALF_PERIOD_NS
		spi_csn = 1'b1;
		#SPI_CLK_HALF_PERIOD_NS;
		
	end
	
endtask

task spi_read;

	input [6:0] addr;
	input [11:0] size;
	
	reg [7:0] serout;
	reg [7:0] serin;
	
	integer i,j;
	
	begin
		
		spi_clk = 1'b0;
		#SPI_CLK_HALF_PERIOD_NS
		spi_csn = 1'b0;
		
		// Address
		serout = {1'b0,addr};
		for (i=0; i<8; i=i+1) begin
			spi_mosi = serout[7];
			#SPI_CLK_HALF_PERIOD_NS
			spi_clk = 1'b1;
			serin[7-i] = spi_miso;
			#SPI_CLK_HALF_PERIOD_NS
			spi_clk = 1'b0;
			serout = serout << 1;
		end
		
		// Data
		serout = 8'd0;
		for (j=0; j<size; j=j+1) begin
			for (i=0; i<8; i=i+1) begin
				spi_mosi = serout[7];
				#SPI_CLK_HALF_PERIOD_NS
				spi_clk = 1'b1;
				serin[7-i] = spi_miso;
				#SPI_CLK_HALF_PERIOD_NS
				spi_clk = 1'b0;
				serout = serout << 1;
			end
			spi_data[j] = serin;
			$write("SPI read  @ %d: %d\n", addr, spi_data[j]);
		end
		
		#SPI_CLK_HALF_PERIOD_NS
		spi_csn = 1'b1;
		#SPI_CLK_HALF_PERIOD_NS;
		
	end
	
endtask

task i2c_slave;
	
	input [7:0] rd;
	
	reg [7:0] serout;
	reg [7:0] serin;
	reg [6:0] device;
	reg wen;
	reg [7:0] addr;
	reg [7:0] wd;
	integer i;
	reg sda_out_dl;
	reg ack;
	reg [7:0] rd1;
	
	begin : task_i2c
	
		sda_in = 1;
		ack = 1;
		rd1 = rd;
		
		@ (negedge sda_out);
		if (scl) begin
			$write("\nI2C START\n");
		end
		
		for (i=0; i<8; i=i+1) begin
			@ (posedge scl);
			serin = {serin[6:0],sda_out};
		end
		device = serin[7:1];
		wen = ~serin[0];
		if (wen) begin
			$write("I2C write @ device %d\n", device);
		end else begin
			$write("I2C read @ device %d\n", device);
		end
		
		@ (posedge scl);
		sda_in = ~ack_en[0];
		if (~ack_en[0]) begin
			$write("\n");
			disable task_i2c;
		end
		@ (negedge scl);
		sda_in = 1;
		
		for (i=0; i<8; i=i+1) begin
			@ (posedge scl);
			serin = {serin[6:0],sda_out};
		end
		addr = serin;
		$write("I2C addr: %d\n", addr);
		
		@ (posedge scl);
		sda_in = ~ack_en[1];
		if (~ack_en[1]) begin
			$write("\n");
			disable task_i2c;
		end
		@ (negedge scl);
		sda_in = 1;
		
		@ (posedge scl);
		sda_out_dl = sda_out;
		#(2.5*I2C_CLK_QUARTER_PERIOD_NS);
		if (~sda_out && sda_out_dl) begin
			wen = 0;
			$write("I2C START\n");
		end else begin
			wen = 1;
			serin = {serin[6:0],sda_out};
		end
			
		if (wen) begin
		
			//for (i=0; i<8; i=i+1) begin
			for (i=1; i<8; i=i+1) begin
				@ (posedge scl);
				serin = {serin[6:0],sda_out};
			end
			wd = serin;
			$write("I2C data: %d\n", wd);
			
			@ (posedge scl);
			sda_in = ~ack_en[2];
			if (~ack_en[2]) begin
				$write("\n");
				disable task_i2c;
			end
			@ (negedge scl);
			sda_in = 1;
			
		end else begin
			
			for (i=0; i<8; i=i+1) begin
				@ (posedge scl);
				serin = {serin[6:0],sda_out};
			end
			device = serin[7:1];
			wen = ~serin[0];
			if (wen) begin
				$write("I2C write @ device %d\n", device);
			end else begin
				$write("I2C read @ device %d\n", device);
			end
			
			@ (posedge scl);
			sda_in = ~ack_en[3];
			if (~ack_en[3]) begin
				$write("\n");
				disable task_i2c;
			end
			@ (negedge scl);
			sda_in = 1;
			
			while (ack) begin
			
				serout = rd1;
				for (i=0; i<8; i=i+1) begin
					@ (posedge scl);
					sda_in = serout[7];
					serout = {serout[6:0],1'b0};
				end
				@ (negedge scl);
				sda_in = 1;
				
				@ (posedge scl);
				ack = ~sda_out;
				if (ack) begin
					$write("I2C Master ACK\n");
				end else begin
					$write("I2C Master NACK\n");
				end
				
				rd1 = rd1 + 8'd1;
			end
		end
		
		@ (posedge scl);
		@ (posedge sda_out);
		if (scl) begin
			$write("I2C STOP\n\n");
		end
		
		#1000;
		
	end
	
endtask

endmodule
