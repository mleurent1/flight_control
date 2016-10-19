classdef fc_reg
	methods
		function data = read(obj,addr)
			global ser
			fwrite(ser,[0,addr,0,0,0,0]);
			data = sum(fread(ser,4) .* 2.^(24:-8:0)');
		end
		function write(obj,addr,data)
			global ser
			fwrite(ser,[1,addr,floor(mod(double(data) ./ 2.^(24:-8:0),2^8))]);
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
		function y = CTRL__MPU_HOST_CTRL(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 4294967294);
				obj.write(1,w);
			end
		end
		function y = CTRL__RESET_INTEGRAL_ON_ARMED(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 4294967293);
				obj.write(1,w);
			end
		end
		function y = CTRL__BEEP_TEST(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 4294967291);
				obj.write(1,w);
			end
		end
		function y = CTRL__LED_SELECT(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 120), -3);
			else
				w = bitand(bitshift(x, 3), 120) + bitand(r, 4294967175);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_SELECT(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 896), -7);
			else
				w = bitand(bitshift(x, 7), 896) + bitand(r, 4294966399);
				obj.write(1,w);
			end
		end
		function y = CTRL__MOTOR_TEST(obj,x)
			r = obj.read(1);
			if nargin < 2
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
		function y = ERROR(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3,x);
			end
		end
		function y = ERROR__SENSOR(obj,x)
			r = obj.read(3);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(3,w);
			end
		end
		function y = ERROR__COMMMAND(obj,x)
			r = obj.read(3);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(3,w);
			end
		end
		function y = LOOP_TIME(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4,x);
			end
		end
		function y = VBAT(obj,x)
			if nargin < 2
				z = obj.read(5);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(5,z);
			end
		end
		function y = VBAT_MIN(obj,x)
			if nargin < 2
				z = obj.read(6);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(6,z);
			end
		end
		function y = VBAT_MAX(obj,x)
			if nargin < 2
				z = obj.read(7);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(7,z);
			end
		end
		function y = EXPO(obj,x)
			if nargin < 2
				z = obj.read(8);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(8,z);
			end
		end
		function y = MOTOR(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9,x);
			end
		end
		function y = MOTOR__MIN(obj,x)
			r = obj.read(9);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(9,w);
			end
		end
		function y = MOTOR__MAX(obj,x)
			r = obj.read(9);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(9,w);
			end
		end
		function y = THROTTLE(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = THROTTLE__RANGE(obj,x)
			r = obj.read(10);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(10,w);
			end
		end
		function y = THROTTLE__ARMED(obj,x)
			r = obj.read(10);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(10,w);
			end
		end
		function y = RATE(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = RATE__PITCH_ROLL(obj,x)
			r = obj.read(11);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(11,w);
			end
		end
		function y = RATE__YAW(obj,x)
			r = obj.read(11);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(11,w);
			end
		end
		function y = PITCH_P(obj,x)
			if nargin < 2
				z = obj.read(12);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(12,z);
			end
		end
		function y = PITCH_I(obj,x)
			if nargin < 2
				z = obj.read(13);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(13,z);
			end
		end
		function y = PITCH_D(obj,x)
			if nargin < 2
				z = obj.read(14);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(14,z);
			end
		end
		function y = ROLL_P(obj,x)
			if nargin < 2
				z = obj.read(15);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(15,z);
			end
		end
		function y = ROLL_I(obj,x)
			if nargin < 2
				z = obj.read(16);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(16,z);
			end
		end
		function y = ROLL_D(obj,x)
			if nargin < 2
				z = obj.read(17);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(17,z);
			end
		end
		function y = YAW_P(obj,x)
			if nargin < 2
				z = obj.read(18);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(18,z);
			end
		end
		function y = YAW_I(obj,x)
			if nargin < 2
				z = obj.read(19);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(19,z);
			end
		end
		function y = YAW_D(obj,x)
			if nargin < 2
				z = obj.read(20);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(20,z);
			end
		end
	end
	properties
		VERSION_addr = 0;
		CTRL_addr = 1;
		DEBUG_addr = 2;
		ERROR_addr = 3;
		LOOP_TIME_addr = 4;
		VBAT_addr = 5;
		VBAT_MIN_addr = 6;
		VBAT_MAX_addr = 7;
		EXPO_addr = 8;
		MOTOR_addr = 9;
		THROTTLE_addr = 10;
		RATE_addr = 11;
		PITCH_P_addr = 12;
		PITCH_I_addr = 13;
		PITCH_D_addr = 14;
		ROLL_P_addr = 15;
		ROLL_I_addr = 16;
		ROLL_D_addr = 17;
		YAW_P_addr = 18;
		YAW_I_addr = 19;
		YAW_D_addr = 20;
		flash_addr_list = [0 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20];
		flash_float_list = [0,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1];
		flash_name_list = {'VERSION','VBAT_MIN','VBAT_MAX','EXPO','MOTOR','THROTTLE','RATE','PITCH_P','PITCH_I','PITCH_D','ROLL_P','ROLL_I','ROLL_D','YAW_P','YAW_I','YAW_D'};
	end
end
