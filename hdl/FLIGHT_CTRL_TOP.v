module FLIGHT_CTRL_TOP
(
	input        CLK,
	input        RST,
	input        SPI_CLK,
	input        SPI_MOSI,
	output       SPI_MISO,
	input        SPI_CSN,
	input        THROTTLE,
	input        AILERON,
	input        ELEVATOR,
	input        RUDDER,
	output [3:0] MOTOR,
	output       MPU_SCL,
	input        MPU_SDA_IN,
	output       MPU_SDA_OUT,
	output       MPU_SDA_DIR,
	input        MPU_INT,
	output [7:0] FPGA_CTRL,
	input  [7:0] FPGA_STATUS,
	output [7:0] LED,
	output [7:0] DBG
);

genvar i;

wire rst;

wire [6:0] spi_addr;
wire       spi_wen;
wire       spi_ren;
wire [7:0] spi_wd;
wire [7:0] spi_rd;

//--------- Registers ----------//

localparam NB_ADDR_W = 44;
localparam NB_ADDR_R = 15;

reg  [7:0] regw [NB_ADDR_W-1:0];
wire [7:0] regr [NB_ADDR_R-1:0];

assign spi_rd = (spi_addr < NB_ADDR_W) ? regw[spi_addr] : regr[spi_addr-NB_ADDR_W];

always @ (negedge SPI_CLK, posedge rst) begin
	if (rst) begin
		regw[0] <= 170;
		regw[1] <= 0;
		regw[2] <= 0;
		regw[3] <= 1;
		regw[4] <= 24;
		regw[5] <= 228;
		regw[6] <= 87;
		regw[7] <= 0;
		regw[8] <= 0;
		regw[9] <= 0;
		regw[10] <= 0;
		regw[11] <= 0;
		regw[12] <= 0;
		regw[13] <= 0;
		regw[14] <= 0;
		regw[15] <= 0;
		regw[16] <= 0;
		regw[17] <= 0;
		regw[18] <= 0;
		regw[19] <= 0;
		regw[20] <= 0;
		regw[21] <= 0;
		regw[22] <= 0;
		regw[23] <= 0;
		regw[24] <= 0;
		regw[25] <= 0;
		regw[26] <= 0;
		regw[27] <= 0;
		regw[28] <= 0;
		regw[29] <= 0;
		regw[30] <= 0;
		regw[31] <= 0;
		regw[32] <= 0;
		regw[33] <= 0;
		regw[34] <= 0;
		regw[35] <= 0;
		regw[36] <= 0;
		regw[37] <= 0;
		regw[38] <= 0;
		regw[39] <= 0;
		regw[40] <= 0;
		regw[41] <= 56;
		regw[42] <= 0;
		regw[43] <= 0;
	end else if (spi_wen && (spi_addr < NB_ADDR_W))
		regw[spi_addr] <= spi_wd;
end

// Write registers declaration and assignement
wire [ 7:0] LED_out                  =  regw[0][7:0];
wire [ 7:0] I2C_ADDR_out             =  regw[1][7:0];
wire [ 7:0] I2C_DATA_W_out           =  regw[2][7:0];
wire [ 7:0] I2C_READ_SIZE_out        =  regw[3][7:0];
wire [ 7:0] I2C_CLK_DIV_out          =  regw[4][7:0];
wire [15:0] MOTOR_TEST_out           = {regw[6][7:0], regw[5][7:0]};
wire [15:0] THROTTLE_TEST_out        = {regw[8][7:0], regw[7][7:0]};
wire [15:0] AILERON_TEST_out         = {regw[10][7:0], regw[9][7:0]};
wire [15:0] ELEVATOR_TEST_out        = {regw[12][7:0], regw[11][7:0]};
wire [15:0] RUDDER_TEST_out          = {regw[14][7:0], regw[13][7:0]};
wire [15:0] GYRO_X_TEST_out          = {regw[16][7:0], regw[15][7:0]};
wire [15:0] GYRO_Y_TEST_out          = {regw[18][7:0], regw[17][7:0]};
wire [15:0] GYRO_Z_TEST_out          = {regw[20][7:0], regw[19][7:0]};
wire [15:0] THROTTLE_OFFSET_out      = {regw[22][7:0], regw[21][7:0]};
wire [15:0] AILERON_OFFSET_out       = {regw[24][7:0], regw[23][7:0]};
wire [15:0] ELEVATOR_OFFSET_out      = {regw[26][7:0], regw[25][7:0]};
wire [15:0] RUDDER_OFFSET_out        = {regw[28][7:0], regw[27][7:0]};
wire [15:0] THROTTLE_SCALE_out       = {regw[30][7:0], regw[29][7:0]};
wire [15:0] AILERON_SCALE_out        = {regw[32][7:0], regw[31][7:0]};
wire [15:0] ELEVATOR_SCALE_out       = {regw[34][7:0], regw[33][7:0]};
wire [15:0] RUDDER_SCALE_out         = {regw[36][7:0], regw[35][7:0]};
wire [15:0] MOTOR_OFFSET_out         = {regw[38][7:0], regw[37][7:0]};
wire [15:0] MOTOR_SCALE_out          = {regw[40][7:0], regw[39][7:0]};
wire        FPGA_RESET_out           =  regw[41][0];
wire        I2C_WEN_out              =  regw[41][1];
wire        I2C_REN_out              =  regw[41][2];
wire        I2C_HOST_CTRL_out        =  regw[41][3];
wire        SPI_ADDR_AUTO_INC_EN_out =  regw[41][4];
wire        MOTOR_TEST_EN_out        =  regw[41][5];
wire        COMMAND_TEST_EN_out      =  regw[41][6];
wire        SENSOR_TEST_EN_out       =  regw[41][7];
wire [ 1:0] MOTOR_SELECT_out         =  regw[42][1:0];
wire [ 3:0] LED_MUX_out              =  regw[42][5:2];
wire [ 3:0] DBG_MUX_out              = {regw[43][1:0], regw[42][7:6]};
wire [ 3:0] CAPTURE_MUX_out          =  regw[43][5:2];

// Write pulses assignement

// Read registers declaration
wire [ 7:0] VERSION_in;
wire [ 7:0] I2C_DATA_R_in;
wire [ 7:0] CAPTURE_READ_in;
wire [15:0] THROTTLE_in;
wire [15:0] AILERON_in;
wire [15:0] ELEVATOR_in;
wire [15:0] RUDDER_in;
wire [15:0] MOTOR_in;
wire        I2C_BUSY_in;
wire        I2C_NACK_in;
wire [12:0] CAPTURE_AVAILABLE_in;

// Read registers assignement
assign regr[ 0][7:0] = VERSION_in;
assign regr[ 1][7:0] = I2C_DATA_R_in;
assign regr[ 2][7:0] = CAPTURE_READ_in;
assign regr[ 3][7:0] = THROTTLE_in[7:0];
assign regr[ 4][7:0] = THROTTLE_in[15:8];
assign regr[ 5][7:0] = AILERON_in[7:0];
assign regr[ 6][7:0] = AILERON_in[15:8];
assign regr[ 7][7:0] = ELEVATOR_in[7:0];
assign regr[ 8][7:0] = ELEVATOR_in[15:8];
assign regr[ 9][7:0] = RUDDER_in[7:0];
assign regr[10][7:0] = RUDDER_in[15:8];
assign regr[11][7:0] = MOTOR_in[7:0];
assign regr[12][7:0] = MOTOR_in[15:8];
assign regr[13][0]   = I2C_BUSY_in;
assign regr[13][1]   = I2C_NACK_in;
assign regr[13][7:2] = CAPTURE_AVAILABLE_in[5:0];
assign regr[14][6:0] = CAPTURE_AVAILABLE_in[12:6];
assign regr[14][7]   = 0;

// Read pulses assignement
wire CAPTURE_READ_rpulse      = (spi_addr == 7'd46) ? spi_ren : 1'b0;
wire THROTTLE_rpulse          = (spi_addr == 7'd47) ? spi_ren : 1'b0;
wire AILERON_rpulse           = (spi_addr == 7'd49) ? spi_ren : 1'b0;
wire ELEVATOR_rpulse          = (spi_addr == 7'd51) ? spi_ren : 1'b0;
wire RUDDER_rpulse            = (spi_addr == 7'd53) ? spi_ren : 1'b0;
wire MOTOR_rpulse             = (spi_addr == 7'd55) ? spi_ren : 1'b0;
wire CAPTURE_AVAILABLE_rpulse = (spi_addr == 7'd57) ? spi_ren : 1'b0;

//-------- Host SPI interface --------//

wire [1:0] spi_dbg;

SPI_SLAVE SPI_SLAVE_inst
(
	.CLK         (SPI_CLK),
	.MOSI        (SPI_MOSI),
	.MISO        (SPI_MISO),
	.CSN         (SPI_CSN),
	.ADDR        (spi_addr),
	.WEN         (spi_wen),
	.REN         (spi_ren),
	.WD          (spi_wd),
	.RD          (spi_rd),
	.AUTO_INC_EN (SPI_ADDR_AUTO_INC_EN_out),
	.DBG         (spi_dbg)
);

//------------------//

assign VERSION_in = 3;
assign FPGA_CTRL = 0;

//------- Reset sync --------//

reg [100:0] rRST;
assign rst = &rRST | FPGA_RESET_out;

always @ (posedge CLK) begin
	rRST <= {rRST[99:0],RST};
end

//------- Collect sensors data -------//

wire i2c_read_en_sensor;
wire i2c_read_valid_sensor;
wire [15:0] gyro_x_in;
wire [15:0] gyro_y_in;
wire [15:0] gyro_z_in;
wire [15:0] accel_x_in;
wire [15:0] accel_y_in;
wire [15:0] accel_z_in;
wire gyro_x_valid;
wire gyro_y_valid;
wire gyro_z_valid;
wire accel_x_valid;
wire accel_y_valid;
wire accel_z_valid;

wire [15:0] gyro_x = SENSOR_TEST_EN_out ? GYRO_X_TEST_out : gyro_x_in;
wire [15:0] gyro_y = SENSOR_TEST_EN_out ? GYRO_Y_TEST_out : gyro_y_in;
wire [15:0] gyro_z = SENSOR_TEST_EN_out ? GYRO_Z_TEST_out : gyro_z_in;

COLLECT_SENSOR COLLECT_SENSOR_inst
(
	.CLK            (CLK),
	.RST            (rst),
	.ICU_INT        (MPU_INT),
	.I2C_READ_DATA  (I2C_DATA_R_in),
	.I2C_READ_VALID (i2c_read_valid_sensor),
	.I2C_BUSY       (I2C_BUSY_in),
	.I2C_READ_EN    (i2c_read_en_sensor),
	.GYRO_X         (gyro_x_in),
	.GYRO_Y         (gyro_y_in),
	.GYRO_Z         (gyro_z_in),
	.ACCEL_X        (accel_x_in),
	.ACCEL_Y        (accel_y_in),
	.ACCEL_Z        (accel_z_in),
	.GYRO_X_VALID   (gyro_x_valid),
	.GYRO_Y_VALID   (gyro_y_valid),
	.GYRO_Z_VALID   (gyro_z_valid),
	.ACCEL_X_VALID  (accel_x_valid),
	.ACCEL_Y_VALID  (accel_y_valid),
	.ACCEL_Z_VALID  (accel_z_valid)
);

//----- MPU initialisation ------//

wire [7:0] i2c_reg_addr_init;
wire [7:0] i2c_write_data_init;
wire i2c_write_en_init;
wire mpu_init_done;

MPU_INIT MPU_INIT_inst
(
	.CLK            (CLK),
	.RST            (rst),
	.I2C_ADDR       (i2c_reg_addr_init),
	.I2C_WRITE_DATA (i2c_write_data_init),
	.I2C_WRITE_EN   (i2c_write_en_init),
	.DONE           (mpu_init_done)
);

//------- I2C Master to MPU -------//

wire i2c_init_ctrl = ~mpu_init_done;

wire [7:0] i2c_reg_addr   = i2c_init_ctrl ? i2c_reg_addr_init   : I2C_HOST_CTRL_out ? I2C_ADDR_out      : 8'd59;
wire       i2c_read_en    = i2c_init_ctrl ? 0                   : I2C_HOST_CTRL_out ? I2C_REN_out       : i2c_read_en_sensor;
wire [7:0] i2c_read_size  = i2c_init_ctrl ? 0                   : I2C_HOST_CTRL_out ? I2C_READ_SIZE_out : 8'd14;
wire       i2c_write_en   = i2c_init_ctrl ? i2c_write_en_init   : I2C_HOST_CTRL_out ? I2C_WEN_out       : 0;
wire [7:0] i2c_write_data = i2c_init_ctrl ? i2c_write_data_init : I2C_HOST_CTRL_out ? I2C_DATA_W_out    : 0;

I2C_MASTER I2C_MASTER_inst
(
	.CLK         (CLK),
	.RST         (rst),
	.SCL         (MPU_SCL),
	.SDA_IN      (MPU_SDA_IN),
	.SDA_OUT     (MPU_SDA_OUT),
	.SDA_DIR     (MPU_SDA_DIR),
	.DEVICE_ADDR (7'b1101000),
	.REG_ADDR    (i2c_reg_addr),
	.READ_EN     (i2c_read_en),
	.READ_SIZE   (i2c_read_size),
	.READ_DATA   (I2C_DATA_R_in),
	.READ_VALID  (i2c_read_valid_sensor),
	.WRITE_EN    (i2c_write_en),
	.WRITE_DATA  (i2c_write_data),
	.NACK        (I2C_NACK_in),
	.BUSY        (I2C_BUSY_in),
	.CLK_DIV     (I2C_CLK_DIV_out)
);


//------- Commands from receiver ------//

// Output from PPM conversion
wire [15:0] Tin;
wire [15:0] Ain;
wire [15:0] Ein;
wire [15:0] Rin;
wire Tvalid;
wire Avalid;
wire Evalid;
wire Rvalid;

// Commands after offset and scale
wire [15:0] Tout;
wire [15:0] Aout;
wire [15:0] Eout;
wire [15:0] Rout;

// Host can overwrite commands
wire [15:0] T = COMMAND_TEST_EN_out ? THROTTLE_TEST_out : Tout;
wire [15:0] A = COMMAND_TEST_EN_out ?  AILERON_TEST_out : Aout;
wire [15:0] E = COMMAND_TEST_EN_out ? ELEVATOR_TEST_out : Eout;
wire [15:0] R = COMMAND_TEST_EN_out ?   RUDDER_TEST_out : Rout;

// Read back for HOST
reg [15:0] Tin_latch;
reg [15:0] Ain_latch;
reg [15:0] Ein_latch;
reg [15:0] Rin_latch;
always @ (posedge SPI_CLK, posedge RST) begin
	if (RST) begin
		Tin_latch = 0;
		Ain_latch = 0;
		Ein_latch = 0;
		Rin_latch = 0;
	end else begin
		if (THROTTLE_rpulse) Tin_latch = Tin;
		if (AILERON_rpulse)  Ain_latch = Ain;
		if (ELEVATOR_rpulse) Ein_latch = Ein;
		if (RUDDER_rpulse)   Rin_latch = Rin;		
	end
end
assign THROTTLE_in = Tin_latch;
assign AILERON_in  = Ain_latch;
assign ELEVATOR_in = Ein_latch;
assign RUDDER_in   = Rin_latch;

// Conversion from PPM to 16-bits signed (2's complement)
PPM2CODE PPM2CODE_inst1
(
	.CLK   (CLK),
	.RST   (rst),
	.PPM   (THROTTLE),
	.CODE  (Tin),
	.VALID (Tvalid)
);

PPM2CODE PPM2CODE_inst2
(
	.CLK   (CLK),
	.RST   (rst),
	.PPM   (AILERON),
	.CODE  (Ain),
	.VALID (Avalid)
);

PPM2CODE PPM2CODE_inst3
(
	.CLK   (CLK),
	.RST   (rst),
	.PPM   (ELEVATOR),
	.CODE  (Ein),
	.VALID (Evalid)
);

PPM2CODE PPM2CODE_inst4
(
	.CLK   (CLK),
	.RST   (rst),
	.PPM   (RUDDER),
	.CODE  (Rin),
	.VALID (Rvalid)
);

//------ Motor control -------//

//   ^
//   |
// 0  1
// 3  2

// Motor control from PID
wire [15:0] M [3:0];
assign M[0] = T;// + (A-10'd512) - (E-10'd512) - (R-10'd512);
assign M[1] = T;// - (A-10'd512) - (E-10'd512) + (R-10'd512);
assign M[2] = T;// - (A-10'd512) + (E-10'd512) - (R-10'd512);
assign M[3] = T;// + (A-10'd512) + (E-10'd512) + (R-10'd512);

// Motor control after offset and scale
wire [15:0] Mout [3:0];

// Read back for HOST
reg [15:0] motor_latch;
always @ (posedge SPI_CLK, posedge RST) begin
	if (RST)
		motor_latch = 0;
	else if (MOTOR_rpulse)
		motor_latch = Mout[MOTOR_SELECT_out];
end
assign MOTOR_in = motor_latch;

reg [22:0] CLK_cnt; // Defined in debug section

generate
	for (i=0; i<4; i=i+1) begin: gen_motor
		CODE2PPM CODE2PPM_inst
		(
			.CLK   (CLK),
			.RST   (rst),
			.CODE  (MOTOR_TEST_EN_out ? ((MOTOR_SELECT_out == i) ? MOTOR_TEST_out : 16'd22500) : Mout[i]),
			.VALID (&CLK_cnt[15:0]),
			.PPM   (MOTOR[i])
		);
	end
endgenerate

//------- Substract and multiply ------//

ADD_MULTIPLY ADD_MULTIPLY_inst
(
	.CLK  (CLK),
	.RST  (rst),
	
	.IN0  (Tin),
	.S0   (THROTTLE_OFFSET_out),
	.M0   (THROTTLE_SCALE_out),
	.OUT0 (Tout),
	
	.IN1  (Ain),
	.S1   (AILERON_OFFSET_out),
	.M1   (AILERON_SCALE_out),
	.OUT1 (Aout),
	
	.IN2  (Ein),
	.S2   (ELEVATOR_OFFSET_out),
	.M2   (ELEVATOR_SCALE_out),
	.OUT2 (Eout),
	
	.IN3  (Rin),
	.S3   (RUDDER_OFFSET_out),
	.M3   (RUDDER_SCALE_out),
	.OUT3 (Rout),
	
	.IN4  (M[0]),
	.S4   (MOTOR_OFFSET_out),
	.M4   (MOTOR_SCALE_out),
	.OUT4 (Mout[0]),
	
	.IN5  (M[1]),
	.S5   (MOTOR_OFFSET_out),
	.M5   (MOTOR_SCALE_out),
	.OUT5 (Mout[1]),
	
	.IN6  (M[2]),
	.S6   (MOTOR_OFFSET_out),
	.M6   (MOTOR_SCALE_out),
	.OUT6 (Mout[2]),
	
	.IN7  (M[3]),
	.S7   (MOTOR_OFFSET_out),
	.M7   (MOTOR_SCALE_out),
	.OUT7 (Mout[3]),
	
	.IN8  (16'sd0),
	.S8   (16'sd0),
	.M8   (16'sd0),
	.OUT8 (),
	
	.IN9  (16'sd0),
	.S9   (16'sd0),
	.M9   (16'sd0),
	.OUT9 (),
	
	.IN10  (16'sd0),
	.S10   (16'sd0),
	.M10   (16'sd0),
	.OUT10 (),
	
	.IN11  (16'sd0),
	.S11   (16'sd0),
	.M11   (16'sd0),
	.OUT11 (),
	
	.IN12  (16'sd0),
	.S12   (16'sd0),
	.M12   (16'sd0),
	.OUT12 (),
	
	.IN13  (16'sd0),
	.S13   (16'sd0),
	.M13   (16'sd0),
	.OUT13 (),
	
	.IN14  (16'sd0),
	.S14   (16'sd0),
	.M14   (16'sd0),
	.OUT14 (),
	
	.IN15  (16'sd0),
	.S15   (16'sd0),
	.M15   (16'sd0),
	.OUT15 ()
);

//------- Debug -------//

wire [15:0] capture_data = 
	(CAPTURE_MUX_out == 0 ) ? gyro_x_in :
	(CAPTURE_MUX_out == 1 ) ? gyro_y_in :
	(CAPTURE_MUX_out == 2 ) ? gyro_z_in :
	(CAPTURE_MUX_out == 3 ) ? accel_x_in :
	(CAPTURE_MUX_out == 4 ) ? accel_y_in :
	(CAPTURE_MUX_out == 5 ) ? accel_z_in :
	(CAPTURE_MUX_out == 6 ) ? Tout :
	(CAPTURE_MUX_out == 7 ) ? Aout :
	(CAPTURE_MUX_out == 8 ) ? Eout :
	(CAPTURE_MUX_out == 9 ) ? Rout :
	(CAPTURE_MUX_out == 10) ? M[MOTOR_SELECT_out] : 16'd0;

wire capture_wen =
	(CAPTURE_MUX_out == 0 ) ? gyro_x_valid :
	(CAPTURE_MUX_out == 1 ) ? gyro_y_valid :
	(CAPTURE_MUX_out == 2 ) ? gyro_z_valid :
	(CAPTURE_MUX_out == 3 ) ? accel_x_valid :
	(CAPTURE_MUX_out == 4 ) ? accel_y_valid :
	(CAPTURE_MUX_out == 5 ) ? accel_z_valid :
	(CAPTURE_MUX_out == 6 ) ? Tvalid :
	(CAPTURE_MUX_out == 7 ) ? Avalid :
	(CAPTURE_MUX_out == 8 ) ? Evalid :
	(CAPTURE_MUX_out == 9 ) ? Rvalid :
	(CAPTURE_MUX_out == 10) ? gyro_z_valid : 1'b0;

FIFO_4096x16 FIFO_4096x16_inst
(
	.wr_clk        (CLK),
	.rd_clk        (SPI_CLK),
	.din           (capture_data),
	.wr_en         (capture_wen),
	.rd_en         (CAPTURE_READ_rpulse),
	.dout          (CAPTURE_READ_in),
	.full          (),
	.empty         (),
	.rd_data_count (CAPTURE_AVAILABLE_in)
);

always @ (posedge CLK, posedge rst) begin
	if (rst)
		CLK_cnt <= 0;
	else
		CLK_cnt <= CLK_cnt + 23'd1;
end

assign LED =
	(LED_MUX_out == 0) ? {5'd0,mpu_init_done,FPGA_STATUS[0],CLK_cnt[22]} : LED_out;
assign DBG =
	(DBG_MUX_out == 0) ? {spi_dbg,spi_wen,spi_ren,SPI_CSN,SPI_MISO,SPI_MOSI,SPI_CLK} :
	(DBG_MUX_out == 1) ? {MPU_INT,I2C_NACK_in,I2C_BUSY_in,i2c_read_valid_sensor,MPU_SDA_DIR,MPU_SDA_IN,MPU_SDA_OUT,MPU_SCL} : 8'd0;

endmodule