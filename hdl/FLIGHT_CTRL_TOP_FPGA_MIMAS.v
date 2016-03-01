module FLIGHT_CTRL_TOP_FPGA_MIMAS
(
	input GCLK,
	
	//input  P1_1, // VCCIO
	//input  P1_2, // GND
	input  P1_3,
	output P1_4, // MOTOR1
	input  P1_5,
	output P1_6, // MOTOR2
	input  P1_7,
	output P1_8, // MOTOR3
	input  P1_9,
	output P1_10, // MOTOR3
	output P1_11, // DBG0
	output P1_12, // DBG1
	output P1_13, // DBG2
	output P1_14, // DBG3
	output P1_15, // DBG4
	output P1_16, // DBG5
	output P1_17, // DBG6
	output P1_18, // DBG7
	input  P1_19,
	input  P1_20,
	input  P1_21,
	input  P1_22,
	input  P1_23,
	input  P1_24, // FT232H AD7
	input  P1_25,
	input  P1_26, // FT232H AD6
	input  P1_27,
	input  P1_28, // FT232H AD5
	input  P1_29,
	input  P1_30, // FT232H AD4
	input  P1_31,
	input  P1_32, // FT232H AD3
	input  P1_33,
	output P1_34, // FT232H AD2
	input  P1_35,
	input  P1_36, // FT232H AD1
	input  P1_37,
	input  P1_38, // FT232H AD0
	//input  P1_39, // VCCIO,
	//input  P1_40, // GND, FT232H GND
	
	//input  P2_1, // GND, FT232RL GND
	//input  P2_2, // VCCIO
	input  P2_3,
	inout  P2_4, // MPU SDA
	input  P2_5,
	output P2_6, // MPU SCL
	input  P2_7,
	input  P2_8, // MPU ESD
	input  P2_9,
	input  P2_10, // MPU ESC
	input  P2_11,
	input  P2_12, // MPU INT
	input  P2_13,
	input  P2_14, // MPU CLKIN (shorted to GND on module, do not drive)
	input  P2_15,
	input  P2_16, // MPU AD0 (shorted to GND on module, do not drive)
	input  P2_17,
	input  P2_18, // MPU FSYNC (shorted to GND on module, do not drive)
	//input  P2_19, // GND
	//input  P2_20, // GND
	input  P2_21,
	input  P2_22,
	input  P2_23,
	input  P2_24,
	input  P2_25,
	input  P2_26,
	input  P2_27,
	input  P2_28,
	input  P2_29,
	input  P2_30,
	input  P2_31, // THROTTLE
	input  P2_32,
	input  P2_33, // AILERON
	input  P2_34,
	input  P2_35, // ELEVATOR
	input  P2_36,
	input  P2_37, // RUDDER
	input  P2_38,
	//input  P2_39, // GND
	//input  P2_40, // VCCIO
	
	input  [3:0] SW,
	output [7:0] LED
);

wire [7:0] led1;
assign LED[7] = led1[0];
assign LED[6] = led1[1];
assign LED[5] = led1[2];
assign LED[4] = led1[3];
assign LED[3] = led1[4];
assign LED[2] = led1[5];
assign LED[1] = led1[6];
assign LED[0] = led1[7];

wire clk_25;
wire clk_fb;
wire pll_locked;
wire sda_in;
wire sda_out;
wire sda_dir;

IOBUF
#(
//	.DRIVE(12), // Specify the output drive strength
	.IOSTANDARD("LVCMOS33") // Specify the I/O standard
//	.SLEW("SLOW") // Specify the output slew rate
)
IOBUF_inst
(
	.O(sda_in),// Buffer output
	.IO(P2_4), // Buffer inout port (connect directly to top-level port)
	.I(sda_out), // Buffer input
	.T(sda_dir) // 3-state enable input, high=input, low=output
);

PLL_BASE
#(
	.BANDWIDTH             ("OPTIMIZED"),             // "HIGH", "LOW" or "OPTIMIZED" 
	.CLKFBOUT_MULT         (4),                   // Multiply value for all CLKOUT clock outputs (1-64)
	.CLKIN_PERIOD          (10),                  // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
	.CLKOUT0_DIVIDE        (16),
	.CLKOUT0_DUTY_CYCLE    (0.5),
	.CLKOUT0_PHASE         (0.0), // (-360.0-360.0).
	.CLK_FEEDBACK          ("CLKFBOUT"),           // Clock source to drive CLKFBIN ("CLKFBOUT" or "CLKOUT0")
	.COMPENSATION          ("SYSTEM_SYNCHRONOUS"), // "SYSTEM_SYNCHRONOUS", "SOURCE_SYNCHRONOUS", "EXTERNAL" 
	.DIVCLK_DIVIDE         (1),                   // Division value for all output clocks (1-52)
	.REF_JITTER            (0.1),                    // Reference Clock Jitter in UI (0.000-0.999).
	.RESET_ON_LOSS_OF_LOCK ("FALSE")      // Must be set to FALSE
)
PLL_BASE_inst
(
	.CLKFBOUT (clk_fb), // 1-bit output: PLL_BASE feedback output
	.CLKOUT0  (clk_25),
	.LOCKED   (pll_locked),     // 1-bit output: PLL_BASE lock status output
	.CLKFBIN  (clk_fb),   // 1-bit input: Feedback clock input
	.CLKIN    (GCLK),       // 1-bit input: Clock input
	.RST      (1'b0)            // 1-bit input: Reset input
);

FLIGHT_CTRL_TOP FLIGHT_CTRL_TOP_inst
(
	.CLK         (clk_25),
	.RST         (~SW[3]),
	.SPI_CLK     (P1_38),
	.SPI_MOSI    (P1_36),
	.SPI_MISO    (P1_34),
	.SPI_CSN     (P1_32),
	.THROTTLE    (P2_31),
	.AILERON     (P2_33),
	.ELEVATOR    (P2_35),
	.RUDDER      (P2_37),
	.MOTOR       ({P1_10,P1_8,P1_6,P1_4}),
	.MPU_SCL     (P2_6),
	.MPU_SDA_IN  (sda_in),
	.MPU_SDA_OUT (sda_out),
	.MPU_SDA_DIR (sda_dir),
	.MPU_INT     (P2_12),
	.FPGA_CTRL   (),
	.FPGA_STATUS ({7'd0,pll_locked}),
	.LED         (led1),
	.DBG         ({P1_18,P1_17,P1_16,P1_15,P1_14,P1_13,P1_12,P1_11})
);

endmodule