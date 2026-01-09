classdef max7456_reg
	methods
		function data = read(obj,addr)
			global ser
			if obj.method % direct UART connection to OSD module
				cmd = [];
			else
				cmd = 9;
			end
         fwrite(ser,[cmd,2,128+addr,0]);
			r = fread(ser,2);
			if obj.method % direct UART connection to OSD module
				data = r(2);
			else
				data = r(1); % Bytes order inverted
			end
		end
		function write(obj,addr,data)
			global ser
			if obj.method % direct UART connection to OSD module
				cmd = [];
			else
				cmd = 9;
			end
			if isempty(addr)
				fwrite(ser,[cmd,length(data),data]);
				fread(ser,length(data));
			else
				fwrite(ser,[cmd,length(data)+1,addr,data]);
				fread(ser,length(data)+1);
			end	
		end
		function y = VM0(obj,x)
			if nargin < 2
				y = obj.read(0);
			else
				obj.write(0, uint32(x));
			end
		end
		function y = VM0__BUF_ENB(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 1), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(0, w);
			end
		end
		function y = VM0__SOFT_RST(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 2), -1);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 2) + bitand(r, 4294967293);
				obj.write(0, w);
			end
		end
		function y = VM0__OSD_VSYNC_EN(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 4), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 4) + bitand(r, 4294967291);
				obj.write(0, w);
			end
		end
		function y = VM0__OSD_EN(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 8), -3);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 3), 8) + bitand(r, 4294967287);
				obj.write(0, w);
			end
		end
		function y = VM0__SYNC_SEL_MODE(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 48), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 48) + bitand(r, 4294967247);
				obj.write(0, w);
			end
		end
		function y = VM0__PAL_NOT_NTSC(obj,x)
			r = double(obj.read(0));
			if nargin < 2
				z = bitshift(bitand(r, 64), -6);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 64) + bitand(r, 4294967231);
				obj.write(0, w);
			end
		end
		function y = VM1(obj,x)
			if nargin < 2
				y = obj.read(1);
			else
				obj.write(1, uint32(x));
			end
		end
		function y = VM1__BLINK_DUTY_CYCLE(obj,x)
			r = double(obj.read(1));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(1, w);
			end
		end
		function y = VM1__BLINK_TIME(obj,x)
			r = double(obj.read(1));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(1, w);
			end
		end
		function y = VM1__BACKGND_BRIGTHNESS(obj,x)
			r = double(obj.read(1));
			if nargin < 2
				z = bitshift(bitand(r, 112), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 112) + bitand(r, 4294967183);
				obj.write(1, w);
			end
		end
		function y = VM1__BACKGND_MODE(obj,x)
			r = double(obj.read(1));
			if nargin < 2
				z = bitshift(bitand(r, 128), -7);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 7), 128) + bitand(r, 4294967167);
				obj.write(1, w);
			end
		end
		function y = HOS(obj,x)
			if nargin < 2
				y = obj.read(2);
			else
				obj.write(2, uint32(x));
			end
		end
		function y = VOS(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3, uint32(x));
			end
		end
		function y = DMM(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4, uint32(x));
			end
		end
		function y = DMM__AUTO_INCR_EN(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 1), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(4, w);
			end
		end
		function y = DMM__VSYNC_CLR(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 2), -1);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 2) + bitand(r, 4294967293);
				obj.write(4, w);
			end
		end
		function y = DMM__CLR_DISPLAY_MEM(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 4), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 4) + bitand(r, 4294967291);
				obj.write(4, w);
			end
		end
		function y = DMM__INV(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 8), -3);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 3), 8) + bitand(r, 4294967287);
				obj.write(4, w);
			end
		end
		function y = DMM__BLK(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 16), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 16) + bitand(r, 4294967279);
				obj.write(4, w);
			end
		end
		function y = DMM__LBC(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 32), -5);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 5), 32) + bitand(r, 4294967263);
				obj.write(4, w);
			end
		end
		function y = DMM__16_BITS_MODE(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = bitshift(bitand(r, 64), -6);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 64) + bitand(r, 4294967231);
				obj.write(4, w);
			end
		end
		function y = DMAH(obj,x)
			if nargin < 2
				y = obj.read(5);
			else
				obj.write(5, uint32(x));
			end
		end
		function y = DMAH__DMA_8(obj,x)
			r = double(obj.read(5));
			if nargin < 2
				z = bitshift(bitand(r, 1), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(5, w);
			end
		end
		function y = DMAH__BYTE_SEL(obj,x)
			r = double(obj.read(5));
			if nargin < 2
				z = bitshift(bitand(r, 2), -1);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 2) + bitand(r, 4294967293);
				obj.write(5, w);
			end
		end
		function y = DMAL(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6, uint32(x));
			end
		end
		function y = DMDI(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7, uint32(x));
			end
		end
		function y = CMM(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8, uint32(x));
			end
		end
		function y = CMAH(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9, uint32(x));
			end
		end
		function y = CMAL(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10, uint32(x));
			end
		end
		function y = CDMI(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11, uint32(x));
			end
		end
		function y = CDMI__RIGHT_PIX(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(11, w);
			end
		end
		function y = CDMI__CENTER_RIGHT_PIX(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(11, w);
			end
		end
		function y = CDMI__CENTER_LEFT_PIX(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = bitshift(bitand(r, 48), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 48) + bitand(r, 4294967247);
				obj.write(11, w);
			end
		end
		function y = CDMI__LEFT_PIX(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = bitshift(bitand(r, 192), -6);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 192) + bitand(r, 4294967103);
				obj.write(11, w);
			end
		end
		function y = OSDM(obj,x)
			if nargin < 2
				y = obj.read(12);
			else
				obj.write(12, uint32(x));
			end
		end
		function y = OSDM__OSD_MUX_SWITCH_TIME(obj,x)
			r = double(obj.read(12));
			if nargin < 2
				z = bitshift(bitand(r, 7), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 7) + bitand(r, 4294967288);
				obj.write(12, w);
			end
		end
		function y = OSDM__OSD_RISE_FALL_TIME(obj,x)
			r = double(obj.read(12));
			if nargin < 2
				z = bitshift(bitand(r, 48), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 48) + bitand(r, 4294967247);
				obj.write(12, w);
			end
		end
		function y = RB0(obj,x)
			if nargin < 2
				y = obj.read(13);
			else
				obj.write(13, uint32(x));
			end
		end
		function y = RB0__WHITE_LVL(obj,x)
			r = double(obj.read(13));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(13, w);
			end
		end
		function y = RB0__BLACK_LVL(obj,x)
			r = double(obj.read(13));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(13, w);
			end
		end
		function y = RB1(obj,x)
			if nargin < 2
				y = obj.read(14);
			else
				obj.write(14, uint32(x));
			end
		end
		function y = RB1__WHITE_LVL(obj,x)
			r = double(obj.read(14));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(14, w);
			end
		end
		function y = RB1__BLACK_LVL(obj,x)
			r = double(obj.read(14));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(14, w);
			end
		end
		function y = RB2(obj,x)
			if nargin < 2
				y = obj.read(15);
			else
				obj.write(15, uint32(x));
			end
		end
		function y = RB2__WHITE_LVL(obj,x)
			r = double(obj.read(15));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(15, w);
			end
		end
		function y = RB2__BLACK_LVL(obj,x)
			r = double(obj.read(15));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(15, w);
			end
		end
		function y = RB3(obj,x)
			if nargin < 2
				y = obj.read(16);
			else
				obj.write(16, uint32(x));
			end
		end
		function y = RB3__WHITE_LVL(obj,x)
			r = double(obj.read(16));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(16, w);
			end
		end
		function y = RB3__BLACK_LVL(obj,x)
			r = double(obj.read(16));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(16, w);
			end
		end
		function y = RB4(obj,x)
			if nargin < 2
				y = obj.read(17);
			else
				obj.write(17, uint32(x));
			end
		end
		function y = RB4__WHITE_LVL(obj,x)
			r = double(obj.read(17));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(17, w);
			end
		end
		function y = RB4__BLACK_LVL(obj,x)
			r = double(obj.read(17));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(17, w);
			end
		end
		function y = RB5(obj,x)
			if nargin < 2
				y = obj.read(18);
			else
				obj.write(18, uint32(x));
			end
		end
		function y = RB5__WHITE_LVL(obj,x)
			r = double(obj.read(18));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(18, w);
			end
		end
		function y = RB5__BLACK_LVL(obj,x)
			r = double(obj.read(18));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(18, w);
			end
		end
		function y = RB6(obj,x)
			if nargin < 2
				y = obj.read(19);
			else
				obj.write(19, uint32(x));
			end
		end
		function y = RB6__WHITE_LVL(obj,x)
			r = double(obj.read(19));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(19, w);
			end
		end
		function y = RB6__BLACK_LVL(obj,x)
			r = double(obj.read(19));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(19, w);
			end
		end
		function y = RB7(obj,x)
			if nargin < 2
				y = obj.read(20);
			else
				obj.write(20, uint32(x));
			end
		end
		function y = RB7__WHITE_LVL(obj,x)
			r = double(obj.read(20));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(20, w);
			end
		end
		function y = RB7__BLACK_LVL(obj,x)
			r = double(obj.read(20));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(20, w);
			end
		end
		function y = RB8(obj,x)
			if nargin < 2
				y = obj.read(21);
			else
				obj.write(21, uint32(x));
			end
		end
		function y = RB8__WHITE_LVL(obj,x)
			r = double(obj.read(21));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(21, w);
			end
		end
		function y = RB8__BLACK_LVL(obj,x)
			r = double(obj.read(21));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(21, w);
			end
		end
		function y = RB9(obj,x)
			if nargin < 2
				y = obj.read(22);
			else
				obj.write(22, uint32(x));
			end
		end
		function y = RB9__WHITE_LVL(obj,x)
			r = double(obj.read(22));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(22, w);
			end
		end
		function y = RB9__BLACK_LVL(obj,x)
			r = double(obj.read(22));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(22, w);
			end
		end
		function y = RB10(obj,x)
			if nargin < 2
				y = obj.read(23);
			else
				obj.write(23, uint32(x));
			end
		end
		function y = RB10__WHITE_LVL(obj,x)
			r = double(obj.read(23));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(23, w);
			end
		end
		function y = RB10__BLACK_LVL(obj,x)
			r = double(obj.read(23));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(23, w);
			end
		end
		function y = RB11(obj,x)
			if nargin < 2
				y = obj.read(24);
			else
				obj.write(24, uint32(x));
			end
		end
		function y = RB11__WHITE_LVL(obj,x)
			r = double(obj.read(24));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(24, w);
			end
		end
		function y = RB11__BLACK_LVL(obj,x)
			r = double(obj.read(24));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(24, w);
			end
		end
		function y = RB12(obj,x)
			if nargin < 2
				y = obj.read(25);
			else
				obj.write(25, uint32(x));
			end
		end
		function y = RB12__WHITE_LVL(obj,x)
			r = double(obj.read(25));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(25, w);
			end
		end
		function y = RB12__BLACK_LVL(obj,x)
			r = double(obj.read(25));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(25, w);
			end
		end
		function y = RB13(obj,x)
			if nargin < 2
				y = obj.read(26);
			else
				obj.write(26, uint32(x));
			end
		end
		function y = RB13__WHITE_LVL(obj,x)
			r = double(obj.read(26));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(26, w);
			end
		end
		function y = RB13__BLACK_LVL(obj,x)
			r = double(obj.read(26));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(26, w);
			end
		end
		function y = RB14(obj,x)
			if nargin < 2
				y = obj.read(27);
			else
				obj.write(27, uint32(x));
			end
		end
		function y = RB14__WHITE_LVL(obj,x)
			r = double(obj.read(27));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(27, w);
			end
		end
		function y = RB14__BLACK_LVL(obj,x)
			r = double(obj.read(27));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(27, w);
			end
		end
		function y = RB15(obj,x)
			if nargin < 2
				y = obj.read(28);
			else
				obj.write(28, uint32(x));
			end
		end
		function y = RB15__WHITE_LVL(obj,x)
			r = double(obj.read(28));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(28, w);
			end
		end
		function y = RB15__BLACK_LVL(obj,x)
			r = double(obj.read(28));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(28, w);
			end
		end
		function y = STAT(obj,x)
			if nargin < 2
				y = obj.read(29);
			else
				obj.write(29, uint32(x));
			end
		end
		function y = STAT__PAL_DETECTED(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 1), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(29, w);
			end
		end
		function y = STAT__NTSC_DETECTED(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 2), -1);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 2) + bitand(r, 4294967293);
				obj.write(29, w);
			end
		end
		function y = STAT__LOS(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 4), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 4) + bitand(r, 4294967291);
				obj.write(29, w);
			end
		end
		function y = STAT__HSYNC_LVL(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 8), -3);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 3), 8) + bitand(r, 4294967287);
				obj.write(29, w);
			end
		end
		function y = STAT__VSYNC_LVL(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 16), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 16) + bitand(r, 4294967279);
				obj.write(29, w);
			end
		end
		function y = STAT__CHAR_MEM_BUSY(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 32), -5);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 5), 32) + bitand(r, 4294967263);
				obj.write(29, w);
			end
		end
		function y = STAT__RST_BUSY(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = bitshift(bitand(r, 64), -6);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 64) + bitand(r, 4294967231);
				obj.write(29, w);
			end
		end
		function y = DMDO(obj,x)
			if nargin < 2
				y = obj.read(30);
			else
				obj.write(30, uint32(x));
			end
		end
		function y = CMDO(obj,x)
			if nargin < 2
				y = obj.read(31);
			else
				obj.write(31, uint32(x));
			end
		end
		function y = CMDO__RIGHT_PIX(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = bitshift(bitand(r, 3), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 3) + bitand(r, 4294967292);
				obj.write(31, w);
			end
		end
		function y = CMDO__CENTER_RIGHT_PIX(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = bitshift(bitand(r, 12), -2);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 2), 12) + bitand(r, 4294967283);
				obj.write(31, w);
			end
		end
		function y = CMDO__CENTER_LEFT_PIX(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = bitshift(bitand(r, 48), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 48) + bitand(r, 4294967247);
				obj.write(31, w);
			end
		end
		function y = CMDO__LEFT_PIX(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = bitshift(bitand(r, 192), -6);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 192) + bitand(r, 4294967103);
				obj.write(31, w);
			end
		end
		function y = OSDBL(obj,x)
			if nargin < 2
				y = obj.read(32);
			else
				obj.write(32, uint32(x));
			end
		end
		function y = OSDBL__DO_NOT_CHANGE(obj,x)
			r = double(obj.read(32));
			if nargin < 2
				z = bitshift(bitand(r, 15), 0);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 15) + bitand(r, 4294967280);
				obj.write(32, w);
			end
		end
		function y = OSDBL__AUTO_OSDBL_DISABLE(obj,x)
			r = double(obj.read(32));
			if nargin < 2
				z = bitshift(bitand(r, 16), -4);
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 16) + bitand(r, 4294967279);
				obj.write(32, w);
			end
		end
	end
	properties
      method = 0;
		VM0_addr = 0;
		VM1_addr = 1;
		HOS_addr = 2;
		VOS_addr = 3;
		DMM_addr = 4;
		DMAH_addr = 5;
		DMAL_addr = 6;
		DMDI_addr = 7;
		CMM_addr = 8;
		CMAH_addr = 9;
		CMAL_addr = 10;
		CDMI_addr = 11;
		OSDM_addr = 12;
		RB0_addr = 16;
		RB1_addr = 17;
		RB2_addr = 18;
		RB3_addr = 19;
		RB4_addr = 20;
		RB5_addr = 21;
		RB6_addr = 22;
		RB7_addr = 23;
		RB8_addr = 24;
		RB9_addr = 25;
		RB10_addr = 26;
		RB11_addr = 27;
		RB12_addr = 28;
		RB13_addr = 29;
		RB14_addr = 30;
		RB15_addr = 31;
		STAT_addr = 32;
		DMDO_addr = 48;
		CMDO_addr = 64;
		OSDBL_addr = 108;
	end
end
