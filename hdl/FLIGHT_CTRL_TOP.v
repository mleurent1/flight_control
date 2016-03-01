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

wire rst;

//-------- Host SPI interface --------//

wire [6:0] spi_addr;
wire       spi_wen;
wire       spi_ren;
wire [7:0] spi_wd;
wire [7:0] spi_rd;
wire [1:0] spi_dbg;

SPI_SLAVE SPI_SLAVE_inst
(
	.CLK  (SPI_CLK),
	.MOSI (SPI_MOSI),
	.MISO (SPI_MISO),
	.CSN  (SPI_CSN),
	.ADDR (spi_addr),
	.WEN  (spi_wen),
	.REN  (spi_ren),
	.WD   (spi_wd),
	.RD   (spi_rd),
	.DBG  (spi_dbg)
);

//--------- Registers ----------//

localparam NB_ADDR_W = 20;
localparam NB_ADDR_R = 17;

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
		regw[5] <= 72;
		regw[6] <= 41;
		regw[7] <= 0;
		regw[8] <= 0;
		regw[9] <= 0;
		regw[10] <= 0;
		regw[11] <= 0;
		regw[12] <= 74;
		regw[13] <= 3;
		regw[14] <= 0;
		regw[15] <= 0;
		regw[16] <= 0;
		regw[17] <= 0;
		regw[18] <= 0;
		regw[19] <= 0;
	end else if (spi_wen && (spi_addr < NB_ADDR_W)) begin
		regw[spi_addr] <= spi_wd;
	end
end

//Write registers declaration and assignement
wire [ 7:0] LED_out             =  regw[0][7:0];
wire [ 7:0] I2C_ADDR_out        =  regw[1][7:0];
wire [ 7:0] I2C_DATA_W_out      =  regw[2][7:0];
wire [ 7:0] I2C_READ_SIZE_out   =  regw[3][7:0];
wire [ 7:0] I2C_CLK_DIV_out     =  regw[4][7:0];
wire        FPGA_RESET_out      =  regw[5][0];
wire        I2C_WEN_out         =  regw[5][1];
wire        I2C_REN_out         =  regw[5][2];
wire        I2C_HOST_CTRL_out   =  regw[5][3];
wire [ 9:0] SERVO_OFFSET_out    = {regw[6][5:0], regw[5][7:4]};
wire        MOTOR_TEST_EN_out   =  regw[6][6];
wire [ 9:0] MOTOR1_TEST_out     = {regw[8][0:0], regw[7][7:0], regw[6][7:7]};
wire [ 9:0] MOTOR2_TEST_out     = {regw[9][2:0], regw[8][7:1]};
wire [ 9:0] MOTOR3_TEST_out     = {regw[10][4:0], regw[9][7:3]};
wire [ 9:0] MOTOR4_TEST_out     = {regw[11][6:0], regw[10][7:5]};
wire [ 9:0] COMMAND_OFFSET_out  = {regw[13][0:0], regw[12][7:0], regw[11][7:7]};
wire        COMMAND_TEST_EN_out =  regw[13][1];
wire [ 9:0] THROTTLE_TEST_out   = {regw[14][3:0], regw[13][7:2]};
wire [ 9:0] AILERON_TEST_out    = {regw[15][5:0], regw[14][7:4]};
wire [ 9:0] ELEVATOR_TEST_out   = {regw[16][7:0], regw[15][7:6]};
wire [ 9:0] RUDDER_TEST_out     = {regw[18][1:0], regw[17][7:0]};
wire [ 3:0] LED_MUX_out         =  regw[18][5:2];
wire [ 3:0] DBG_MUX_out         = {regw[19][1:0], regw[18][7:6]};

//Read registers declaration
wire [ 7:0] VERSION_in;
wire [ 7:0] I2C_DATA_R_in;
wire [ 7:0] GYRO_X_in;
wire [ 7:0] GYRO_Y_in;
wire [ 7:0] GYRO_Z_in;
wire [ 7:0] ACCEL_X_in;
wire [ 7:0] ACCEL_Y_in;
wire [ 7:0] ACCEL_Z_in;
wire [ 7:0] THROTTLE_in;
wire [ 7:0] AILERON_in;
wire [ 7:0] ELEVATOR_in;
wire [ 7:0] RUDDER_in;
wire [ 7:0] MOTOR1_in;
wire [ 7:0] MOTOR2_in;
wire [ 7:0] MOTOR3_in;
wire [ 7:0] MOTOR4_in;
wire        I2C_BUSY_in;
wire        I2C_NACK_in;

//Read registers assignement
assign regr[ 0][7:0] = VERSION_in;
assign regr[ 1][7:0] = I2C_DATA_R_in;
assign regr[ 2][7:0] = GYRO_X_in;
assign regr[ 3][7:0] = GYRO_Y_in;
assign regr[ 4][7:0] = GYRO_Z_in;
assign regr[ 5][7:0] = ACCEL_X_in;
assign regr[ 6][7:0] = ACCEL_Y_in;
assign regr[ 7][7:0] = ACCEL_Z_in;
assign regr[ 8][7:0] = THROTTLE_in;
assign regr[ 9][7:0] = AILERON_in;
assign regr[10][7:0] = ELEVATOR_in;
assign regr[11][7:0] = RUDDER_in;
assign regr[12][7:0] = MOTOR1_in;
assign regr[13][7:0] = MOTOR2_in;
assign regr[14][7:0] = MOTOR3_in;
assign regr[15][7:0] = MOTOR4_in;
assign regr[16][0]   = I2C_BUSY_in;
assign regr[16][1]   = I2C_NACK_in;
assign regr[16][7:2] = 0;

//Read pulses assignement
wire GYRO_X_rpulse  = (spi_addr == 7'd22) ? spi_ren : 1'b0;
wire GYRO_Y_rpulse  = (spi_addr == 7'd23) ? spi_ren : 1'b0;
wire GYRO_Z_rpulse  = (spi_addr == 7'd24) ? spi_ren : 1'b0;
wire ACCEL_X_rpulse = (spi_addr == 7'd25) ? spi_ren : 1'b0;
wire ACCEL_Y_rpulse = (spi_addr == 7'd26) ? spi_ren : 1'b0;
wire ACCEL_Z_rpulse = (spi_addr == 7'd27) ? spi_ren : 1'b0;

//------------------//

assign VERSION_in = 1;
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
wire [7:0] sample_index;
wire [15:0] gyro_x;
wire [15:0] gyro_y;
wire [15:0] gyro_z;
wire [15:0] accel_x;
wire [15:0] accel_y;
wire [15:0] accel_z;
wire gyro_x_wen;
wire gyro_y_wen;
wire gyro_z_wen;
wire accel_x_wen;
wire accel_y_wen;
wire accel_z_wen;

COLLECT_SENSOR COLLECT_SENSOR_inst
(
	.CLK            (CLK),
	.RST            (rst),
	.ICU_INT        (MPU_INT),
	.I2C_READ_DATA  (I2C_DATA_R_in),
	.I2C_READ_VALID (i2c_read_valid_sensor),
	.I2C_BUSY       (I2C_BUSY_in),
	.I2C_READ_EN    (i2c_read_en_sensor),
	.SAMPLE_INDEX   (sample_index),
	.GYRO_X         (gyro_x),
	.GYRO_Y         (gyro_y),
	.GYRO_Z         (gyro_z),
	.ACCEL_X        (accel_x),
	.ACCEL_Y        (accel_y),
	.ACCEL_Z        (accel_z),
	.GYRO_X_VALID   (gyro_x_wen),
	.GYRO_Y_VALID   (gyro_y_wen),
	.GYRO_Z_VALID   (gyro_z_wen),
	.ACCEL_X_VALID  (accel_x_wen),
	.ACCEL_Y_VALID  (accel_y_wen),
	.ACCEL_Z_VALID  (accel_z_wen)
);

// Gyro X
reg [8:0] gyro_x_addr;
wire [15:0] gyro_x_r;
assign GYRO_X_in = gyro_x_addr[0] ? gyro_x_r[15:8] : gyro_x_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		gyro_x_addr <= 0;
	end else if (GYRO_X_rpulse) begin
		gyro_x_addr <= gyro_x_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_GYRO_X
(
    .CLK_A      (CLK),
    .WEN_A      (gyro_x_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (gyro_x),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (gyro_x_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (gyro_x_r)
);

// Gyro Y
reg [8:0] gyro_y_addr;
wire [15:0] gyro_y_r;
assign GYRO_Y_in = gyro_y_addr[0] ? gyro_y_r[15:8] : gyro_y_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		gyro_y_addr <= 0;
	end else if (GYRO_Y_rpulse) begin
		gyro_y_addr <= gyro_y_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_GYRO_Y
(
    .CLK_A      (CLK),
    .WEN_A      (gyro_y_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (gyro_y),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (gyro_y_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (gyro_y_r)
);

// Gyro Z
reg [8:0] gyro_z_addr;
wire [15:0] gyro_z_r;
assign GYRO_Z_in = gyro_z_addr[0] ? gyro_z_r[15:8] : gyro_z_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		gyro_z_addr <= 0;
	end else if (GYRO_Z_rpulse) begin
		gyro_z_addr <= gyro_z_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_GYRO_Z
(
    .CLK_A      (CLK),
    .WEN_A      (gyro_z_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (gyro_z),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (gyro_z_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (gyro_z_r)
);

// Accel X
reg [8:0] accel_x_addr;
wire [15:0] accel_x_r;
assign ACCEL_X_in = accel_x_addr[0] ? accel_x_r[15:8] : accel_x_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		accel_x_addr <= 0;
	end else if (ACCEL_X_rpulse) begin
		accel_x_addr <= accel_x_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_ACCEL_X
(
    .CLK_A      (CLK),
    .WEN_A      (accel_x_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (accel_x),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (accel_x_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (accel_x_r)
);

// Accel Y
reg [8:0] accel_y_addr;
wire [15:0] accel_y_r;
assign ACCEL_Y_in = accel_y_addr[0] ? accel_y_r[15:8] : accel_y_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		accel_y_addr <= 0;
	end else if (ACCEL_Y_rpulse) begin
		accel_y_addr <= accel_y_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_ACCEL_Y
(
    .CLK_A      (CLK),
    .WEN_A      (accel_y_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (accel_y),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (accel_y_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (accel_y_r)
);

// Accel Z
reg [8:0] accel_z_addr;
wire [15:0] accel_z_r;
assign ACCEL_Z_in = accel_z_addr[0] ? accel_z_r[15:8] : accel_z_r[7:0];

always @ (posedge SPI_CLK, posedge rst) begin
	if (rst) begin
		accel_z_addr <= 0;
	end else if (ACCEL_Z_rpulse) begin
		accel_z_addr <= accel_z_addr + 9'd1;
	end
end

DPRAM_256x16 MEM_ACCEL_Z
(
    .CLK_A      (CLK),
    .WEN_A      (accel_z_wen),
    .ADDR_A     (sample_index),
    .DATA_IN_A  (accel_z),
    .DATA_OUT_A (),
    .CLK_B      (SPI_CLK),
    .WEN_B      (1'b0),
    .ADDR_B     (accel_z_addr[8:1]),
    .DATA_IN_B  (16'd0),
    .DATA_OUT_B (accel_z_r)
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

wire [9:0] Tin;
wire [9:0] Ain;
wire [9:0] Ein;
wire [9:0] Rin;

wire [9:0] T = COMMAND_TEST_EN_out ? THROTTLE_TEST_out : Tin;
wire [9:0] A = COMMAND_TEST_EN_out ?  AILERON_TEST_out : Ain;
wire [9:0] E = COMMAND_TEST_EN_out ? ELEVATOR_TEST_out : Ein;
wire [9:0] R = COMMAND_TEST_EN_out ?   RUDDER_TEST_out : Rin;

assign THROTTLE_in = Tin[9:2];
assign AILERON_in = Ain[9:2];
assign ELEVATOR_in = Ein[9:2];
assign RUDDER_in = Rin[9:2];

COMMAND COMMAND_inst1
(
	.CLK     (CLK),
	.RST     (rst),
	.SERVO   (THROTTLE),
	.OFFSET  (COMMAND_OFFSET_out),
	.COMMAND (Tin)
);

COMMAND COMMAND_inst2
(
	.CLK     (CLK),
	.RST     (rst),
	.SERVO   (AILERON),
	.OFFSET  (COMMAND_OFFSET_out),
	.COMMAND (Ain)
);

COMMAND COMMAND_inst3
(
	.CLK     (CLK),
	.RST     (rst),
	.SERVO   (ELEVATOR),
	.OFFSET  (COMMAND_OFFSET_out),
	.COMMAND (Ein)
);

COMMAND COMMAND_inst4
(
	.CLK     (CLK),
	.RST     (rst),
	.SERVO   (RUDDER),
	.OFFSET  (COMMAND_OFFSET_out),
	.COMMAND (Rin)
);

//------ Motor control -------//

//   ^
//   |
// 0  1
// 3  2

wire [9:0] M [3:0];

assign M[0] = T + (A-10'd512) - (E-10'd512) - (R-10'd512);
assign M[1] = T - (A-10'd512) - (E-10'd512) + (R-10'd512);
assign M[2] = T - (A-10'd512) + (E-10'd512) - (R-10'd512);
assign M[3] = T + (A-10'd512) + (E-10'd512) + (R-10'd512);

assign MOTOR1_in = M[0][9:2];
assign MOTOR2_in = M[1][9:2];
assign MOTOR3_in = M[2][9:2];
assign MOTOR4_in = M[3][9:2];

SERVO SERVO_inst1
(
	.CLK     (CLK),
	.RST     (rst),
	.COMMAND (MOTOR_TEST_EN_out ? MOTOR1_TEST_out : M[0]),
	.OFFSET  (SERVO_OFFSET_out),
	.SERVO   (MOTOR[0])
);

SERVO SERVO_inst2
(
	.CLK     (CLK),
	.RST     (rst),
	.COMMAND (MOTOR_TEST_EN_out ? MOTOR2_TEST_out : M[1]),
	.OFFSET  (SERVO_OFFSET_out),
	.SERVO   (MOTOR[1])
);

SERVO SERVO_inst3
(
	.CLK     (CLK),
	.RST     (rst),
	.COMMAND (MOTOR_TEST_EN_out ? MOTOR3_TEST_out : M[2]),
	.OFFSET  (SERVO_OFFSET_out),
	.SERVO   (MOTOR[2])
);

SERVO SERVO_inst4
(
	.CLK     (CLK),
	.RST     (rst),
	.COMMAND (MOTOR_TEST_EN_out ? MOTOR4_TEST_out : M[3]),
	.OFFSET  (SERVO_OFFSET_out),
	.SERVO   (MOTOR[3])
);

//------- Debug -------//

reg [22:0] CLK_cnt;

always @ (posedge CLK, posedge rst) begin
	if (rst) begin
		CLK_cnt <= 0;
	end else begin
		CLK_cnt <= CLK_cnt + 23'd1;
	end
end

assign LED = (LED_MUX_out == 0) ? {5'd0,mpu_init_done,FPGA_STATUS[0],CLK_cnt[22]} :
			 (LED_MUX_out == 1) ? LED_out :
                                  8'd0;
assign DBG = (DBG_MUX_out == 0) ? {spi_dbg,spi_wen,spi_ren,SPI_CSN,SPI_MISO,SPI_MOSI,SPI_CLK} :
             (DBG_MUX_out == 1) ? {MPU_INT,I2C_NACK_in,I2C_BUSY_in,i2c_read_valid_sensor,MPU_SDA_DIR,MPU_SDA_IN,MPU_SDA_OUT,MPU_SCL} :
                                  8'd0;
endmodule