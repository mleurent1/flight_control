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
		function y = CTRL__MPU(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 4294967294);
				obj.write(1,w);
			end
		end
		function y = CTRL__RESET_INT(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 4294967293);
				obj.write(1,w);
			end
		end
		function y = CTRL__LED(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 60), -2);
			else
				w = bitand(bitshift(x, 2), 60) + bitand(r, 4294967235);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_SEL(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 960), -6);
			else
				w = bitand(bitshift(x, 6), 960) + bitand(r, 4294966335);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_TEST(obj,x)
			r = obj.read(1);
			if nargin <= 1
				y = bitshift(bitand(r, 67107840), -10);
			else
				w = bitand(bitshift(x, 10), 67107840) + bitand(r, 4227859455);
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
		function y = MOTOR(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3,x);
			end
		end
		function y = MOTOR__MIN(obj,x)
			r = obj.read(3);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(3,w);
			end
		end
		function y = MOTOR__MAX(obj,x)
			r = obj.read(3);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(3,w);
			end
		end
		function y = THROTTLE(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4,x);
			end
		end
		function y = THROTTLE__OFFSET(obj,x)
			r = obj.read(4);
			if nargin <= 1
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(4,w);
			end
		end
		function y = THROTTLE__ARMED(obj,x)
			r = obj.read(4);
			if nargin <= 1
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(4,w);
			end
		end
		function y = THROTTLE_SCALE(obj,x)
			if nargin < 2
				y = obj.read(5);
			else
				obj.write(5,x);
			end
		end
		function y = AILERON_SCALE(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6,x);
			end
		end
		function y = ELEVATOR_SCALE(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7,x);
			end
		end
		function y = RUDDER_SCALE(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8,x);
			end
		end
		function y = PITCH_P(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9,x);
			end
		end
		function y = PITCH_I(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = PITCH_D(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = ROLL_P(obj,x)
			if nargin < 2
				y = obj.read(12);
			else
				obj.write(12,x);
			end
		end
		function y = ROLL_I(obj,x)
			if nargin < 2
				y = obj.read(13);
			else
				obj.write(13,x);
			end
		end
		function y = ROLL_D(obj,x)
			if nargin < 2
				y = obj.read(14);
			else
				obj.write(14,x);
			end
		end
		function y = YAW_P(obj,x)
			if nargin < 2
				y = obj.read(15);
			else
				obj.write(15,x);
			end
		end
		function y = YAW_I(obj,x)
			if nargin < 2
				y = obj.read(16);
			else
				obj.write(16,x);
			end
		end
		function y = YAW_D(obj,x)
			if nargin < 2
				y = obj.read(17);
			else
				obj.write(17,x);
			end
		end
	end
	properties
		VERSION_addr = 0;
		CTRL_addr = 1;
		DEBUG_addr = 2;
		MOTOR_addr = 3;
		THROTTLE_addr = 4;
		THROTTLE_SCALE_addr = 5;
		AILERON_SCALE_addr = 6;
		ELEVATOR_SCALE_addr = 7;
		RUDDER_SCALE_addr = 8;
		PITCH_P_addr = 9;
		PITCH_I_addr = 10;
		PITCH_D_addr = 11;
		ROLL_P_addr = 12;
		ROLL_I_addr = 13;
		ROLL_D_addr = 14;
		YAW_P_addr = 15;
		YAW_I_addr = 16;
		YAW_D_addr = 17;
	end
end
