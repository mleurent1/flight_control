classdef fc_reg
	methods
		function data = read(obj,addr)
			global TIMEOUT
			ftdi('write',[0,addr,0,0,0,0]);
			r = [];
			tic
			while (length(r) < 4) && (toc < TIMEOUT)
				sleep(1)
				r = [r, ftdi('read')];
			end
			if length(r) < 4
				data = 0;
				warning('read timeout!')
			else
				data = sum(r.*2.^(24:-8:0));
			end
		end
		function write(obj,addr,data)
			ftdi('write',[1,addr,mod(data.*2.^(-24:8:0),2^8)]);
		end
		function y = VERSION(obj,x)
			if nargin < 2
				y = obj.read(0);
			else
				obj.write(0,x);
			end
		end
		function y = CTRL(obj,x)
			if nargin < 2
				y = obj.read(1);
			else
				obj.write(1,x);
			end
		end
		function y = CTRL__READ_SENSOR(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 4294967294);
				obj.write(1,w);
			end
		end
		function y = CTRL__RESET_INT_ON_ARMED(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 4294967293);
				obj.write(1,w);
			end
		end
		function y = CTRL__BEEP(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 4294967291);
				obj.write(1,w);
			end
		end
		function y = CTRL__LED(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 120), -3);
			else
				w = bitand(bitshift(x, 3), 120) + bitand(r, 4294967175);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_SEL(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 1920), -7);
			else
				w = bitand(bitshift(x, 7), 1920) + bitand(r, 4294965375);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_TEST(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 134215680), -11);
			else
				w = bitand(bitshift(x, 11), 134215680) + bitand(r, 4160751615);
				obj.write(1,w);
			end
		end
		function y = DEBUG(obj,x)
			if nargin < 2
				y = obj.read(2);
			else
				obj.write(2,x);
			end
		end
		function y = CLOCK(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3,x);
			end
		end
		function y = ERROR(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4,x);
			end
		end
		function y = ERROR__SENSOR(obj,x)
			r = obj.read(4);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(4,w);
			end
		end
		function y = ERROR__COMMAND(obj,x)
			r = obj.read(4);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(4,w);
			end
		end
		function y = LOOP_TIME(obj,x)
			if nargin < 2
				y = obj.read(5);
			else
				obj.write(5,x);
			end
		end
		function y = VBAT(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6,x);
			end
		end
		function y = VBAT_MIN(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7,x);
			end
		end
		function y = MOTOR(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8,x);
			end
		end
		function y = MOTOR__MIN(obj,x)
			r = obj.read(8);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(8,w);
			end
		end
		function y = MOTOR__MAX(obj,x)
			r = obj.read(8);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(8,w);
			end
		end
		function y = CMD_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9,x);
			end
		end
		function y = CMD_OFFSET__THROTTLE(obj,x)
			r = obj.read(9);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(9,w);
			end
		end
		function y = CMD_OFFSET__AIL_ELE_RUD(obj,x)
			r = obj.read(9);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(9,w);
			end
		end
		function y = CMD_RANGE(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = CMD_RANGE__THROTTLE(obj,x)
			r = obj.read(10);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(10,w);
			end
		end
		function y = CMD_RANGE__AIL_ELE_RUD(obj,x)
			r = obj.read(10);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(10,w);
			end
		end
		function y = THROTTLE(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = THROTTLE__RANGE(obj,x)
			r = obj.read(11);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(11,w);
			end
		end
		function y = THROTTLE__ARMED(obj,x)
			r = obj.read(11);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(11,w);
			end
		end
		function y = RATE(obj,x)
			if nargin < 2
				y = obj.read(12);
			else
				obj.write(12,x);
			end
		end
		function y = RATE__PITCH(obj,x)
			r = obj.read(12);
			if nargin <= 1
				y = bitshift(bitand(r, 255), 0);
			else
				w = bitand(bitshift(x, 0), 255) + bitand(r, 4294967040);
				obj.write(12,w);
			end
		end
		function y = RATE__ROLL(obj,x)
			r = obj.read(12);
			if nargin <= 1
				y = bitshift(bitand(r, 65280), -8);
			else
				w = bitand(bitshift(x, 8), 65280) + bitand(r, 4294902015);
				obj.write(12,w);
			end
		end
		function y = RATE__YAW(obj,x)
			r = obj.read(12);
			if nargin <= 1
				y = bitshift(bitand(r, 16711680), -16);
			else
				w = bitand(bitshift(x, 16), 16711680) + bitand(r, 4278255615);
				obj.write(12,w);
			end
		end
		function y = PITCH_P(obj,x)
			if nargin < 2
				y = obj.read(13);
			else
				obj.write(13,x);
			end
		end
		function y = PITCH_I(obj,x)
			if nargin < 2
				y = obj.read(14);
			else
				obj.write(14,x);
			end
		end
		function y = PITCH_D(obj,x)
			if nargin < 2
				y = obj.read(15);
			else
				obj.write(15,x);
			end
		end
		function y = ROLL_P(obj,x)
			if nargin < 2
				y = obj.read(16);
			else
				obj.write(16,x);
			end
		end
		function y = ROLL_I(obj,x)
			if nargin < 2
				y = obj.read(17);
			else
				obj.write(17,x);
			end
		end
		function y = ROLL_D(obj,x)
			if nargin < 2
				y = obj.read(18);
			else
				obj.write(18,x);
			end
		end
		function y = YAW_P(obj,x)
			if nargin < 2
				y = obj.read(19);
			else
				obj.write(19,x);
			end
		end
		function y = YAW_I(obj,x)
			if nargin < 2
				y = obj.read(20);
			else
				obj.write(20,x);
			end
		end
		function y = YAW_D(obj,x)
			if nargin < 2
				y = obj.read(21);
			else
				obj.write(21,x);
			end
		end
	end
	properties
		VERSION_addr = 0;
		CTRL_addr = 1;
		DEBUG_addr = 2;
		CLOCK_addr = 3;
		ERROR_addr = 4;
		LOOP_TIME_addr = 5;
		VBAT_addr = 6;
		VBAT_MIN_addr = 7;
		MOTOR_addr = 8;
		CMD_OFFSET_addr = 9;
		CMD_RANGE_addr = 10;
		THROTTLE_addr = 11;
		RATE_addr = 12;
		PITCH_P_addr = 13;
		PITCH_I_addr = 14;
		PITCH_D_addr = 15;
		ROLL_P_addr = 16;
		ROLL_I_addr = 17;
		ROLL_D_addr = 18;
		YAW_P_addr = 19;
		YAW_I_addr = 20;
		YAW_D_addr = 21;
	end
end
