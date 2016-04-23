module SUBS_MULT_ADD
(
	input CLK,
	input RST,
	
	input [15:0] IN0,
	input [15:0] IN1,
	input [15:0] IN2,
	input [15:0] IN3,
	input [15:0] IN4,
	input [15:0] IN5,
	input [15:0] IN6,
	input [15:0] IN7,
	input [15:0] IN8,
	input [15:0] IN9,
	input [15:0] IN10,
	input [15:0] IN11,
	input [15:0] IN12,
	input [15:0] IN13,
	input [15:0] IN14,
	input [15:0] IN15,
	
	input [15:0] S0,
	input [15:0] S1,
	input [15:0] S2,
	input [15:0] S3,
	input [15:0] S4,
	input [15:0] S5,
	input [15:0] S6,
	input [15:0] S7,
	input [15:0] S8,
	input [15:0] S9,
	input [15:0] S10,
	input [15:0] S11,
	input [15:0] S12,
	input [15:0] S13,
	input [15:0] S14,
	input [15:0] S15,
	
	input [15:0] M0,
	input [15:0] M1,
	input [15:0] M2,
	input [15:0] M3,
	input [15:0] M4,
	input [15:0] M5,
	input [15:0] M6,
	input [15:0] M7,
	input [15:0] M8,
	input [15:0] M9,
	input [15:0] M10,
	input [15:0] M11,
	input [15:0] M12,
	input [15:0] M13,
	input [15:0] M14,
	input [15:0] M15,
	
	input [15:0] A0,
	input [15:0] A1,
	input [15:0] A2,
	input [15:0] A3,
	input [15:0] A4,
	input [15:0] A5,
	input [15:0] A6,
	input [15:0] A7,
	input [15:0] A8,
	input [15:0] A9,
	input [15:0] A10,
	input [15:0] A11,
	input [15:0] A12,
	input [15:0] A13,
	input [15:0] A14,
	input [15:0] A15,
	
	output reg [15:0] OUT0,
	output reg [15:0] OUT1,
	output reg [15:0] OUT2,
	output reg [15:0] OUT3,
	output reg [15:0] OUT4,
	output reg [15:0] OUT5,
	output reg [15:0] OUT6,
	output reg [15:0] OUT7,
	output reg [15:0] OUT8,
	output reg [15:0] OUT9,
	output reg [15:0] OUT10,
	output reg [15:0] OUT11,
	output reg [15:0] OUT12,
	output reg [15:0] OUT13,
	output reg [15:0] OUT14,
	output reg [15:0] OUT15
);

wire signed [35:0] SMout;
reg [3:0] cnt;

always @ (posedge CLK, posedge RST) begin
	if (RST) begin
		cnt <= 0;
		OUT0 <= 0;
		OUT1 <= 0;
		OUT2 <= 0;
		OUT3 <= 0;
		OUT4 <= 0;
		OUT5 <= 0;
		OUT6 <= 0;
		OUT7 <= 0;
		OUT8 <= 0;
		OUT9 <= 0;
		OUT10 <= 0;
		OUT11 <= 0;
		OUT12 <= 0;
		OUT13 <= 0;
		OUT14 <= 0;
		OUT15 <= 0;
	end else begin
		cnt <= cnt + 4'd1;
		case (cnt)
			4'd0 : OUT14 <= SMout[31:16];
			4'd1 : OUT15 <= SMout[31:16];
			4'd2 : OUT0  <= SMout[31:16];
			4'd3 : OUT1  <= SMout[31:16];
			4'd4 : OUT2  <= SMout[31:16];
			4'd5 : OUT3  <= SMout[31:16];
			4'd6 : OUT4  <= SMout[31:16];
			4'd7 : OUT5  <= SMout[31:16];
			4'd8 : OUT6  <= SMout[31:16];
			4'd9 : OUT7  <= SMout[31:16];
			4'd10: OUT8  <= SMout[31:16];
			4'd11: OUT9  <= SMout[31:16];
			4'd12: OUT10 <= SMout[31:16];
			4'd13: OUT11 <= SMout[31:16];
			4'd14: OUT12 <= SMout[31:16];
			4'd15: OUT13 <= SMout[31:16];
		endcase
	end
end

wire signed [17:0] SMin =
	(cnt == 0 ) ? $signed({{2{IN0 [15]}},IN0 }) :
	(cnt == 1 ) ? $signed({{2{IN1 [15]}},IN1 }) :
	(cnt == 2 ) ? $signed({{2{IN2 [15]}},IN2 }) :
	(cnt == 3 ) ? $signed({{2{IN3 [15]}},IN3 }) :
	(cnt == 4 ) ? $signed({{2{IN4 [15]}},IN4 }) :
	(cnt == 5 ) ? $signed({{2{IN5 [15]}},IN5 }) :
	(cnt == 6 ) ? $signed({{2{IN6 [15]}},IN6 }) :
	(cnt == 7 ) ? $signed({{2{IN7 [15]}},IN7 }) :
	(cnt == 8 ) ? $signed({{2{IN8 [15]}},IN8 }) :
	(cnt == 9 ) ? $signed({{2{IN9 [15]}},IN9 }) :
	(cnt == 10) ? $signed({{2{IN10[15]}},IN10}) :
	(cnt == 11) ? $signed({{2{IN11[15]}},IN11}) :
	(cnt == 12) ? $signed({{2{IN12[15]}},IN12}) :
	(cnt == 13) ? $signed({{2{IN13[15]}},IN13}) :
	(cnt == 14) ? $signed({{2{IN14[15]}},IN14}) :
	              $signed({{2{IN15[15]}},IN15});
	
wire signed [17:0] S =
	(cnt == 0 ) ? $signed({{2{S15[15]}},S15}) :
	(cnt == 1 ) ? $signed({{2{S0 [15]}},S0 }) :
	(cnt == 2 ) ? $signed({{2{S1 [15]}},S1 }) :
	(cnt == 3 ) ? $signed({{2{S2 [15]}},S2 }) :
	(cnt == 4 ) ? $signed({{2{S3 [15]}},S3 }) :
	(cnt == 5 ) ? $signed({{2{S4 [15]}},S4 }) :
	(cnt == 6 ) ? $signed({{2{S5 [15]}},S5 }) :
	(cnt == 7 ) ? $signed({{2{S6 [15]}},S6 }) :
	(cnt == 8 ) ? $signed({{2{S7 [15]}},S7 }) :
	(cnt == 9 ) ? $signed({{2{S8 [15]}},S8 }) :
	(cnt == 10) ? $signed({{2{S9 [15]}},S9 }) :
	(cnt == 11) ? $signed({{2{S10[15]}},S10}) :
	(cnt == 12) ? $signed({{2{S11[15]}},S11}) :
	(cnt == 13) ? $signed({{2{S12[15]}},S12}) :
	(cnt == 14) ? $signed({{2{S13[15]}},S13}) :
	              $signed({{2{S14[15]}},S14});

wire signed [17:0] M =
	(cnt == 0 ) ? $signed({{2{M15[15]}},M15}) :
	(cnt == 1 ) ? $signed({{2{M0 [15]}},M0 }) :
	(cnt == 2 ) ? $signed({{2{M1 [15]}},M1 }) :
	(cnt == 3 ) ? $signed({{2{M2 [15]}},M2 }) :
	(cnt == 4 ) ? $signed({{2{M3 [15]}},M3 }) :
	(cnt == 5 ) ? $signed({{2{M4 [15]}},M4 }) :
	(cnt == 6 ) ? $signed({{2{M5 [15]}},M5 }) :
	(cnt == 7 ) ? $signed({{2{M6 [15]}},M6 }) :
	(cnt == 8 ) ? $signed({{2{M7 [15]}},M7 }) :
	(cnt == 9 ) ? $signed({{2{M8 [15]}},M8 }) :
	(cnt == 10) ? $signed({{2{M9 [15]}},M9 }) :
	(cnt == 11) ? $signed({{2{M10[15]}},M10}) :
	(cnt == 12) ? $signed({{2{M11[15]}},M11}) :
	(cnt == 13) ? $signed({{2{M12[15]}},M12}) :
	(cnt == 14) ? $signed({{2{M13[15]}},M13}) :
	              $signed({{2{M14[15]}},M14});

`ifdef FPGA

	DSP48A1
	#(
		.A0REG(0),              // First stage A input pipeline register (0/1)
		.A1REG(0),              // Second stage A input pipeline register (0/1)
		.B0REG(0),              // First stage B input pipeline register (0/1)
		.B1REG(0),              // Second stage B input pipeline register (0/1)
		.CARRYINREG(1),         // CARRYIN input pipeline register (0/1)
		.CARRYINSEL("OPMODE5"), // Specify carry-in source, "CARRYIN" or "OPMODE5" 
		.CARRYOUTREG(1),        // CARRYOUT output pipeline register (0/1)
		.CREG(1),               // C input pipeline register (0/1)
		.DREG(1),               // D pre-adder input pipeline register (0/1)
		.MREG(1),               // M pipeline register (0/1)
		.OPMODEREG(0),          // Enable=1/disable=0 OPMODE input pipeline registers
		.PREG(1),               // P output pipeline register (0/1)
		.RSTTYPE("SYNC")        // Specify reset type, "SYNC" or "ASYNC" 
	)
	DSP48A1_inst
	(
		// Cascade Ports: 18-bit (each) output: Ports to cascade from one DSP48 to another
		.BCOUT     (), // 18-bit output: B port cascade output
		.PCOUT     (), // 48-bit output: P cascade output (if used, connect to PCIN of another DSP48A1)
		
		// Data Ports: 1-bit (each) output: Data input and output ports
		.CARRYOUT  (), // 1-bit output: carry output (if used, connect to CARRYIN pin of another DSP48A1)
		.CARRYOUTF (), // 1-bit output: fabric carry output
		.M         (SMout), // 36-bit output: fabric multiplier data output
		.P         (), // 48-bit output: data output
		
		// Cascade Ports: 48-bit (each) input: Ports to cascade from one DSP48 to another
		.PCIN      (48'd0), // 48-bit input: P cascade input (if used, connect to PCOUT of another DSP48A1)
		
		// Control Input Ports: 1-bit (each) input: Clocking and operation mode
		.CLK       (CLK), // 1-bit input: clock input
		.OPMODE    (8'b0101_0000), // 8-bit input: operation mode input
		
		// Data Ports: 18-bit (each) input: Data input and output ports
		.A         (M), // 18-bit input: A data input
		.B         (S), // 18-bit input: B data input (connected to fabric or BCOUT of adjacent DSP48A1)
		.C         (48'd0), // 48-bit input: C data input
		.CARRYIN   (1'b0), // 1-bit input: carry input signal (if used, connect to CARRYOUT pin of another DSP48A1)
		.D         (SMin), // 18-bit input: B pre-adder data input
		
		// Reset/Clock Enable Input Ports: 1-bit (each) input: Reset and enable input ports
		.CEA(1'b1),       // 1-bit input: active high clock enable input for A registers
		.CEB(1'b1),       // 1-bit input: active high clock enable input for B registers
		.CEC(1'b1),       // 1-bit input: active high clock enable input for C registers
		.CECARRYIN(1'b1), // 1-bit input: active high clock enable input for CARRYIN registers
		.CED(1'b1),       // 1-bit input: active high clock enable input for D registers
		.CEM(1'b1),       // 1-bit input: active high clock enable input for multiplier registers
		.CEOPMODE(1'b1),  // 1-bit input: active high clock enable input for OPMODE registers
		.CEP(1'b1),       // 1-bit input: active high clock enable input for P registers
		.RSTA(RST),       // 1-bit input: reset input for A pipeline registers
		.RSTB(RST),       // 1-bit input: reset input for B pipeline registers
		.RSTC(RST),       // 1-bit input: reset input for C pipeline registers
		.RSTCARRYIN(RST), // 1-bit input: reset input for CARRYIN pipeline registers
		.RSTD(RST),       // 1-bit input: reset input for D pipeline registers
		.RSTM(RST),       // 1-bit input: reset input for M pipeline registers
		.RSTOPMODE(RST),  // 1-bit input: reset input for OPMODE pipeline registers
		.RSTP(RST)        // 1-bit input: reset input for P pipeline registers
	);
	
`else
	
	reg signed [17:0] Sout;
	reg signed [35:0] Mout;
	always @ (posedge CLK, posedge RST) begin
		if (RST) begin
			Sout <= 0;
			Mout <= 0;
		end else begin
			Sout <= SMin - S;
			Mout <= $signed({{18{Sout[17]}},Sout}) * $signed({{18{M[17]}},M});
		end
	end
	assign SMout = Mout;

`endif

endmodule