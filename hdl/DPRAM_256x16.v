module DPRAM_256x16(
    input             CLK_A,
    input             WEN_A,
    input      [ 7:0] ADDR_A,
    input      [15:0] DATA_IN_A,
    output     [15:0] DATA_OUT_A,
    input             CLK_B,
    input             WEN_B,
    input      [ 7:0] ADDR_B,
    input      [15:0] DATA_IN_B,
    output     [15:0] DATA_OUT_B
);

`ifdef FPGA

// BRAM_TDP_MACRO: True Dual Port RAM
//                 Spartan-6
// Xilinx HDL Language Template, version 14.7

//////////////////////////////////////////////////////////////////////////
// DATA_WIDTH_A/B | BRAM_SIZE | RAM Depth | ADDRA/B Width | WEA/B Width //
// ===============|===========|===========|===============|=============//
//     19-36      |  "18Kb"   |     512   |     9-bit     |    4-bit    //
//     10-18      |  "18Kb"   |    1024   |    10-bit     |    2-bit    //
//     10-18      |   "9Kb"   |     512   |     9-bit     |    2-bit    //
//      5-9       |  "18Kb"   |    2048   |    11-bit     |    1-bit    //
//      5-9       |   "9Kb"   |    1024   |    10-bit     |    1-bit    //
//      3-4       |  "18Kb"   |    4096   |    12-bit     |    1-bit    //
//      3-4       |   "9Kb"   |    2048   |    11-bit     |    1-bit    //
//        2       |  "18Kb"   |    8192   |    13-bit     |    1-bit    //
//        2       |   "9Kb"   |    4096   |    12-bit     |    1-bit    //
//        1       |  "18Kb"   |   16384   |    14-bit     |    1-bit    //
//        1       |   "9Kb"   |    8192   |    12-bit     |    1-bit    //
//////////////////////////////////////////////////////////////////////////

BRAM_TDP_MACRO
#(
	.BRAM_SIZE     ("18Kb"), // Target BRAM: "9Kb" or "18Kb" 
	.DEVICE        ("SPARTAN6"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6" 
	.DOA_REG       (0), // Optional port A output register (0 or 1)
	.DOB_REG       (0), // Optional port B output register (0 or 1)
	.INIT_FILE     ("NONE"),
	.READ_WIDTH_A  (16), // Valid values are 1-36
	.READ_WIDTH_B  (16), // Valid values are 1-36
	.WRITE_MODE_A  ("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE" 
	.WRITE_MODE_B  ("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE" 
	.WRITE_WIDTH_A (16), // Valid values are 1-36
	.WRITE_WIDTH_B (16) // Valid values are 1-36
)
BRAM_TDP_MACRO_inst
(
	.DOA    (DATA_OUT_A),       // Output port-A data, width defined by READ_WIDTH_A parameter
	.DOB    (DATA_OUT_B),       // Output port-B data, width defined by READ_WIDTH_B parameter
	.ADDRA  ({2'd0,ADDR_A}),   // Input port-A address, width defined by Port A depth
	.ADDRB  ({2'd0,ADDR_B}),   // Input port-B address, width defined by Port B depth
	.CLKA   (CLK_A),     // 1-bit input port-A clock
	.CLKB   (CLK_B),     // 1-bit input port-B clock
	.DIA    (DATA_IN_A),       // Input port-A data, width defined by WRITE_WIDTH_A parameter
	.DIB    (DATA_IN_B),       // Input port-B data, width defined by WRITE_WIDTH_B parameter
	.ENA    (1'b1),       // 1-bit input port-A enable
	.ENB    (1'b1),       // 1-bit input port-B enable
	.REGCEA (1'b0), // 1-bit input port-A output register enable
	.REGCEB (1'b0), // 1-bit input port-B output register enable
	.RSTA   (1'b0),     // 1-bit input port-A reset
	.RSTB   (1'b0),     // 1-bit input port-B reset
	.WEA    ({WEN_A,WEN_A}),       // Input port-A write enable, width defined by Port A depth
	.WEB    ({WEN_B,WEN_B})        // Input port-B write enable, width defined by Port B depth
);

`else

reg [15:0] mem [0:255];
reg [15:0] doa;
reg [15:0] dob;
integer i;

assign DATA_OUT_A = doa;
assign DATA_OUT_B = dob;

initial begin
    doa = 16'd0;
    dob = 16'd0;
    for (i=0; i<256; i=i+1)
        mem[i] = 16'd65535;
end

always @ (posedge CLK_A) begin
    if (WEN_A)
        mem[ADDR_A] <= DATA_IN_A;
    doa <= mem[ADDR_A];
end

always @ (posedge CLK_B) begin
    if (WEN_B)
        mem[ADDR_B] <= DATA_IN_B;
    dob <= mem[ADDR_B];
end

`endif

endmodule