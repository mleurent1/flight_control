classdef fc_reg
	methods
		function data = read(obj,addr)
			ftdi('write',[0,addr,0,0]);
			sleep(10)
			r = ftdi('read');
			data = 2^8 * r(1) + r(2);
		end
		function write(obj,addr,data)
			ftdi('write',[1,addr,floor(data/2^8),mod(data,2^8)]);
		end
		function y = VERSION(obj,x)
			if nargin < 2
				y = obj.read(0);
			else
				obj.write(0,x);
			end
		end
		function y = SPI_HOST_CTRL(obj,x)
			if nargin < 2
				y = obj.read(1);
			else
				obj.write(1,x);
			end
		end
		function y = DEBUG_MUX(obj,x)
			if nargin < 2
				y = obj.read(2);
			else
				obj.write(2,x);
			end
		end
		function y = LED_MUX(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3,x);
			end
		end
		function y = MOTOR_SELECT(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4,x);
			end
		end
		function y = MOTOR_TEST(obj,x)
			if nargin < 2
				y = obj.read(5);
			else
				obj.write(5,x);
			end
		end
		function y = SERVO_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6,x);
			end
		end
		function y = THROTTLE_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7,x);
			end
		end
		function y = THROTTLE_SCALE(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8,x);
			end
		end
		function y = AILERON_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9,x);
			end
		end
		function y = AILERON_SCALE(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = ELEVATOR_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = ELEVATOR_SCALE(obj,x)
			if nargin < 2
				y = obj.read(12);
			else
				obj.write(12,x);
			end
		end
		function y = RUDDER_OFFSET(obj,x)
			if nargin < 2
				y = obj.read(13);
			else
				obj.write(13,x);
			end
		end
		function y = RUDDER_SCALE(obj,x)
			if nargin < 2
				y = obj.read(14);
			else
				obj.write(14,x);
			end
		end
		function y = THROTTLE_TEST(obj,x)
			if nargin < 2
				y = obj.read(15);
			else
				obj.write(15,x);
			end
		end
		function y = PITCH_P(obj,x)
			if nargin < 2
				y = obj.read(16);
			else
				obj.write(16,x);
			end
		end
		function y = PITCH_I(obj,x)
			if nargin < 2
				y = obj.read(17);
			else
				obj.write(17,x);
			end
		end
		function y = PITCH_D(obj,x)
			if nargin < 2
				y = obj.read(18);
			else
				obj.write(18,x);
			end
		end
		function y = ROLL_P(obj,x)
			if nargin < 2
				y = obj.read(19);
			else
				obj.write(19,x);
			end
		end
		function y = ROLL_I(obj,x)
			if nargin < 2
				y = obj.read(20);
			else
				obj.write(20,x);
			end
		end
		function y = ROLL_D(obj,x)
			if nargin < 2
				y = obj.read(21);
			else
				obj.write(21,x);
			end
		end
		function y = YAW_P(obj,x)
			if nargin < 2
				y = obj.read(22);
			else
				obj.write(22,x);
			end
		end
		function y = YAW_I(obj,x)
			if nargin < 2
				y = obj.read(23);
			else
				obj.write(23,x);
			end
		end
		function y = YAW_D(obj,x)
			if nargin < 2
				y = obj.read(24);
			else
				obj.write(24,x);
			end
		end
	end
	properties
		VERSION_addr = 0;
		SPI_HOST_CTRL_addr = 1;
		DEBUG_MUX_addr = 2;
		LED_MUX_addr = 3;
		MOTOR_SELECT_addr = 4;
		MOTOR_TEST_addr = 5;
		SERVO_OFFSET_addr = 6;
		THROTTLE_OFFSET_addr = 7;
		THROTTLE_SCALE_addr = 8;
		AILERON_OFFSET_addr = 9;
		AILERON_SCALE_addr = 10;
		ELEVATOR_OFFSET_addr = 11;
		ELEVATOR_SCALE_addr = 12;
		RUDDER_OFFSET_addr = 13;
		RUDDER_SCALE_addr = 14;
		THROTTLE_TEST_addr = 15;
		PITCH_P_addr = 16;
		PITCH_I_addr = 17;
		PITCH_D_addr = 18;
		ROLL_P_addr = 19;
		ROLL_I_addr = 20;
		ROLL_D_addr = 21;
		YAW_P_addr = 22;
		YAW_I_addr = 23;
		YAW_D_addr = 24;
	end
end
