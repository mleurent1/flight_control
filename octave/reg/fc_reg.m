classdef fc_reg
	methods
		function data = read(obj,addr)
			global ser
			srl_write(ser,sprintf("%s",[0,addr,0,0,0,0]));
			data = sum(double(srl_read(ser,4)) .* 2.^(24:-8:0));
		end
		function write(obj,addr,data)
			global ser
			srl_write(ser,sprintf("%s",[1,addr,floor(mod(double(data) ./ 2.^(24:-8:0),2^8))]));
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
		function y = CTRL__TIME_MAXHOLD(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 4294967287);
				obj.write(1,w);
			end
		end
		function y = CTRL__LED_SELECT(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 48), -4);
			else
				w = bitand(bitshift(x, 4), 48) + bitand(r, 4294967247);
				obj.write(1,w);
			end
		end
		function y = MOTOR_TEST(obj,x)
			if nargin < 2
				y = obj.read(2);
			else
				obj.write(2,x);
			end
		end
		function y = MOTOR_TEST__VALUE(obj,x)
			r = obj.read(2);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(2,w);
			end
		end
		function y = MOTOR_TEST__SELECT(obj,x)
			r = obj.read(2);
			if nargin < 2
				y = bitshift(bitand(r, 983040), -16);
			else
				w = bitand(bitshift(x, 16), 983040) + bitand(r, 4293984255);
				obj.write(2,w);
			end
		end
		function y = DEBUG(obj,x)
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
		function y = ERROR__MPU(obj,x)
			r = obj.read(4);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(4,w);
			end
		end
		function y = ERROR__RADIO(obj,x)
			r = obj.read(4);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(4,w);
			end
		end
		function y = TIME(obj,x)
			if nargin < 2
				y = obj.read(5);
			else
				obj.write(5,x);
			end
		end
		function y = TIME__SPI(obj,x)
			r = obj.read(5);
			if nargin < 2
				y = bitshift(bitand(r, 65535), 0);
			else
				w = bitand(bitshift(x, 0), 65535) + bitand(r, 4294901760);
				obj.write(5,w);
			end
		end
		function y = TIME__PROCESS(obj,x)
			r = obj.read(5);
			if nargin < 2
				y = bitshift(bitand(r, 4294901760), -16);
			else
				w = bitand(bitshift(x, 16), 4294901760) + bitand(r, 65535);
				obj.write(5,w);
			end
		end
		function y = VBAT(obj,x)
			if nargin < 2
				z = obj.read(6);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(6,z);
			end
		end
		function y = VBAT_MIN(obj,x)
			if nargin < 2
				z = obj.read(7);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(7,z);
			end
		end
		function y = RECEIVER_BIND(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8,x);
			end
		end
		function y = EXPO(obj,x)
			if nargin < 2
				z = obj.read(9);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(9,z);
			end
		end
		function y = MOTOR_START(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = MOTOR_ARMED(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = COMMAND_RATE(obj,x)
			if nargin < 2
				z = obj.read(12);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(12,z);
			end
		end
		function y = THROTTLE_RANGE(obj,x)
			if nargin < 2
				z = obj.read(13);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(13,z);
			end
		end
		function y = PITCH_P(obj,x)
			if nargin < 2
				z = obj.read(14);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(14,z);
			end
		end
		function y = PITCH_I(obj,x)
			if nargin < 2
				z = obj.read(15);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(15,z);
			end
		end
		function y = PITCH_D(obj,x)
			if nargin < 2
				z = obj.read(16);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(16,z);
			end
		end
		function y = ROLL_P(obj,x)
			if nargin < 2
				z = obj.read(17);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(17,z);
			end
		end
		function y = ROLL_I(obj,x)
			if nargin < 2
				z = obj.read(18);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(18,z);
			end
		end
		function y = ROLL_D(obj,x)
			if nargin < 2
				z = obj.read(19);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(19,z);
			end
		end
		function y = YAW_P(obj,x)
			if nargin < 2
				z = obj.read(20);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(20,z);
			end
		end
		function y = YAW_I(obj,x)
			if nargin < 2
				z = obj.read(21);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(21,z);
			end
		end
		function y = YAW_D(obj,x)
			if nargin < 2
				z = obj.read(22);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(22,z);
			end
		end
		function y = THROTTLE_PID_ATTEN(obj,x)
			if nargin < 2
				z = obj.read(23);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(23,z);
			end
		end
		function y = RATE_PID_ATTEN(obj,x)
			if nargin < 2
				z = obj.read(24);
				y = typecast(uint32(z), 'single');
			else
				z = typecast(single(x), 'uint32');
				obj.write(24,z);
			end
		end
	end
	properties
		VERSION_addr = 0;
		CTRL_addr = 1;
		MOTOR_TEST_addr = 2;
		DEBUG_addr = 3;
		ERROR_addr = 4;
		TIME_addr = 5;
		VBAT_addr = 6;
		VBAT_MIN_addr = 7;
		RECEIVER_BIND_addr = 8;
		EXPO_addr = 9;
		MOTOR_START_addr = 10;
		MOTOR_ARMED_addr = 11;
		COMMAND_RATE_addr = 12;
		THROTTLE_RANGE_addr = 13;
		PITCH_P_addr = 14;
		PITCH_I_addr = 15;
		PITCH_D_addr = 16;
		ROLL_P_addr = 17;
		ROLL_I_addr = 18;
		ROLL_D_addr = 19;
		YAW_P_addr = 20;
		YAW_I_addr = 21;
		YAW_D_addr = 22;
		THROTTLE_PID_ATTEN_addr = 23;
		RATE_PID_ATTEN_addr = 24;
		flash_addr_list = [0 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24];
		flash_float_list = [0,1,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1];
		flash_name_list = {'VERSION','VBAT_MIN','RECEIVER_BIND','EXPO','MOTOR_START','MOTOR_ARMED','COMMAND_RATE','THROTTLE_RANGE','PITCH_P','PITCH_I','PITCH_D','ROLL_P','ROLL_I','ROLL_D','YAW_P','YAW_I','YAW_D','THROTTLE_PID_ATTEN','RATE_PID_ATTEN'};
	end
end
