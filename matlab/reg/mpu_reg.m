classdef mpu_reg
	methods
		function data = read(obj,addr)
			global TIMEOUT
			ftdi('write',[2,addr,0,0,0,0]);
			data = [];
			tic
			while isempty(data) && (toc < TIMEOUT)
				sleep(1)
				data = ftdi('read');
			end
			if isempty(data)
				warning('read timeout!')
			end
		end
		function write(obj,addr,data)
			ftdi('write',[3,addr,0,0,0,data]);
		end
		function y = SELF_TST_X(obj,x)
			if nargin <= 1
				y = obj.read(13);
			else
				obj.write(13,x);
			end
		end
		function y = SELF_TST_X__XG_TST(obj,x)
			r = obj.read(13);
			if nargin <= 1
				y = bitshift(bitand(r, 31), 0);
			else
				w = bitand(bitshift(x, 0), 31) + bitand(r, 224);
				obj.write(13,w);
			end
		end
		function y = SELF_TST_X__XA_TST_MSB(obj,x)
			r = obj.read(13);
			if nargin <= 1
				y = bitshift(bitand(r, 224), -5);
			else
				w = bitand(bitshift(x, 5), 224) + bitand(r, 31);
				obj.write(13,w);
			end
		end
		function y = SELF_TST_Y(obj,x)
			if nargin <= 1
				y = obj.read(14);
			else
				obj.write(14,x);
			end
		end
		function y = SELF_TST_Y__YG_TST(obj,x)
			r = obj.read(14);
			if nargin <= 1
				y = bitshift(bitand(r, 31), 0);
			else
				w = bitand(bitshift(x, 0), 31) + bitand(r, 224);
				obj.write(14,w);
			end
		end
		function y = SELF_TST_Y__YA_TST_MSB(obj,x)
			r = obj.read(14);
			if nargin <= 1
				y = bitshift(bitand(r, 224), -5);
			else
				w = bitand(bitshift(x, 5), 224) + bitand(r, 31);
				obj.write(14,w);
			end
		end
		function y = SELF_TST_Z(obj,x)
			if nargin <= 1
				y = obj.read(15);
			else
				obj.write(15,x);
			end
		end
		function y = SELF_TST_Z__ZG_TST(obj,x)
			r = obj.read(15);
			if nargin <= 1
				y = bitshift(bitand(r, 31), 0);
			else
				w = bitand(bitshift(x, 0), 31) + bitand(r, 224);
				obj.write(15,w);
			end
		end
		function y = SELF_TST_Z__ZA_TST_MSB(obj,x)
			r = obj.read(15);
			if nargin <= 1
				y = bitshift(bitand(r, 224), -5);
			else
				w = bitand(bitshift(x, 5), 224) + bitand(r, 31);
				obj.write(15,w);
			end
		end
		function y = SELF_TST_A(obj,x)
			if nargin <= 1
				y = obj.read(16);
			else
				obj.write(16,x);
			end
		end
		function y = SELF_TST_A__XA_TST_LSB(obj,x)
			r = obj.read(16);
			if nargin <= 1
				y = bitshift(bitand(r, 3), 0);
			else
				w = bitand(bitshift(x, 0), 3) + bitand(r, 252);
				obj.write(16,w);
			end
		end
		function y = SELF_TST_A__YA_TST_LSB(obj,x)
			r = obj.read(16);
			if nargin <= 1
				y = bitshift(bitand(r, 12), -2);
			else
				w = bitand(bitshift(x, 2), 12) + bitand(r, 243);
				obj.write(16,w);
			end
		end
		function y = SELF_TST_A__ZA_TST_LSB(obj,x)
			r = obj.read(16);
			if nargin <= 1
				y = bitshift(bitand(r, 48), -4);
			else
				w = bitand(bitshift(x, 4), 48) + bitand(r, 207);
				obj.write(16,w);
			end
		end
		function y = SMPLRT_DIV(obj,x)
			if nargin <= 1
				y = obj.read(25);
			else
				obj.write(25,x);
			end
		end
		function y = CFG(obj,x)
			if nargin <= 1
				y = obj.read(26);
			else
				obj.write(26,x);
			end
		end
		function y = CFG__DLPF_CFG(obj,x)
			r = obj.read(26);
			if nargin <= 1
				y = bitshift(bitand(r, 7), 0);
			else
				w = bitand(bitshift(x, 0), 7) + bitand(r, 248);
				obj.write(26,w);
			end
		end
		function y = CFG__EXT_SYNC_SET(obj,x)
			r = obj.read(26);
			if nargin <= 1
				y = bitshift(bitand(r, 56), -3);
			else
				w = bitand(bitshift(x, 3), 56) + bitand(r, 199);
				obj.write(26,w);
			end
		end
		function y = GYRO_CFG(obj,x)
			if nargin <= 1
				y = obj.read(27);
			else
				obj.write(27,x);
			end
		end
		function y = GYRO_CFG__FS_SEL(obj,x)
			r = obj.read(27);
			if nargin <= 1
				y = bitshift(bitand(r, 24), -3);
			else
				w = bitand(bitshift(x, 3), 24) + bitand(r, 231);
				obj.write(27,w);
			end
		end
		function y = GYRO_CFG__ZG_ST(obj,x)
			r = obj.read(27);
			if nargin <= 1
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(27,w);
			end
		end
		function y = GYRO_CFG__YG_ST(obj,x)
			r = obj.read(27);
			if nargin <= 1
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(27,w);
			end
		end
		function y = GYRO_CFG__XG_ST(obj,x)
			r = obj.read(27);
			if nargin <= 1
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(27,w);
			end
		end
		function y = INT_EN(obj,x)
			if nargin <= 1
				y = obj.read(56);
			else
				obj.write(56,x);
			end
		end
		function y = INT_EN__DATA_RDY_EN(obj,x)
			r = obj.read(56);
			if nargin <= 1
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(56,w);
			end
		end
		function y = INT_EN__I2C_MST_INT_EN(obj,x)
			r = obj.read(56);
			if nargin <= 1
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(56,w);
			end
		end
		function y = INT_EN__FIFO_OFLOW_EN(obj,x)
			r = obj.read(56);
			if nargin <= 1
				y = bitshift(bitand(r, 16), -4);
			else
				w = bitand(bitshift(x, 4), 16) + bitand(r, 239);
				obj.write(56,w);
			end
		end
		function y = ACCEL_X_H(obj,x)
			if nargin <= 1
				y = obj.read(59);
			else
				obj.write(59,x);
			end
		end
		function y = ACCEL_X_L(obj,x)
			if nargin <= 1
				y = obj.read(60);
			else
				obj.write(60,x);
			end
		end
		function y = ACCEL_Y_H(obj,x)
			if nargin <= 1
				y = obj.read(61);
			else
				obj.write(61,x);
			end
		end
		function y = ACCEL_Y_L(obj,x)
			if nargin <= 1
				y = obj.read(62);
			else
				obj.write(62,x);
			end
		end
		function y = ACCEL_Z_H(obj,x)
			if nargin <= 1
				y = obj.read(63);
			else
				obj.write(63,x);
			end
		end
		function y = ACCEL_Z_L(obj,x)
			if nargin <= 1
				y = obj.read(64);
			else
				obj.write(64,x);
			end
		end
		function y = TEMP_H(obj,x)
			if nargin <= 1
				y = obj.read(65);
			else
				obj.write(65,x);
			end
		end
		function y = TEMP_L(obj,x)
			if nargin <= 1
				y = obj.read(66);
			else
				obj.write(66,x);
			end
		end
		function y = GYRO_X_H(obj,x)
			if nargin <= 1
				y = obj.read(67);
			else
				obj.write(67,x);
			end
		end
		function y = GYRO_X_L(obj,x)
			if nargin <= 1
				y = obj.read(68);
			else
				obj.write(68,x);
			end
		end
		function y = GYRO_Y_H(obj,x)
			if nargin <= 1
				y = obj.read(69);
			else
				obj.write(69,x);
			end
		end
		function y = GYRO_Y_L(obj,x)
			if nargin <= 1
				y = obj.read(70);
			else
				obj.write(70,x);
			end
		end
		function y = GYRO_Z_H(obj,x)
			if nargin <= 1
				y = obj.read(71);
			else
				obj.write(71,x);
			end
		end
		function y = GYRO_Z_L(obj,x)
			if nargin <= 1
				y = obj.read(72);
			else
				obj.write(72,x);
			end
		end
		function y = SIGNAL_PATH_RST(obj,x)
			if nargin <= 1
				y = obj.read(104);
			else
				obj.write(104,x);
			end
		end
		function y = SIGNAL_PATH_RST__TEMP_RST(obj,x)
			r = obj.read(104);
			if nargin <= 1
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(104,w);
			end
		end
		function y = SIGNAL_PATH_RST__ACCEL_RST(obj,x)
			r = obj.read(104);
			if nargin <= 1
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 253);
				obj.write(104,w);
			end
		end
		function y = SIGNAL_PATH_RST__GYRO_RST(obj,x)
			r = obj.read(104);
			if nargin <= 1
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(104,w);
			end
		end
		function y = USER_CTRL(obj,x)
			if nargin <= 1
				y = obj.read(106);
			else
				obj.write(106,x);
			end
		end
		function y = USER_CTRL__SIG_COND_RST(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(106,w);
			end
		end
		function y = USER_CTRL__I2C_MST_RST(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 253);
				obj.write(106,w);
			end
		end
		function y = USER_CTRL__FIFO_RST(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(106,w);
			end
		end
		function y = USER_CTRL__I2C_IF_DIS(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 16), -4);
			else
				w = bitand(bitshift(x, 4), 16) + bitand(r, 239);
				obj.write(106,w);
			end
		end
		function y = USER_CTRL__I2C_MST_EN(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(106,w);
			end
		end
		function y = USER_CTRL__FIFO_EN(obj,x)
			r = obj.read(106);
			if nargin <= 1
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(106,w);
			end
		end
		function y = PWR_MGMT_1(obj,x)
			if nargin <= 1
				y = obj.read(107);
			else
				obj.write(107,x);
			end
		end
		function y = PWR_MGMT_1__CLKSEL(obj,x)
			r = obj.read(107);
			if nargin <= 1
				y = bitshift(bitand(r, 7), 0);
			else
				w = bitand(bitshift(x, 0), 7) + bitand(r, 248);
				obj.write(107,w);
			end
		end
		function y = PWR_MGMT_1__TEMP_DIS(obj,x)
			r = obj.read(107);
			if nargin <= 1
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(107,w);
			end
		end
		function y = PWR_MGMT_1__CYCLE(obj,x)
			r = obj.read(107);
			if nargin <= 1
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(107,w);
			end
		end
		function y = PWR_MGMT_1__SLEEP(obj,x)
			r = obj.read(107);
			if nargin <= 1
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(107,w);
			end
		end
		function y = PWR_MGMT_1__DEVICE_RST(obj,x)
			r = obj.read(107);
			if nargin <= 1
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(107,w);
			end
		end
		function y = WHO_AM_I(obj,x)
			if nargin <= 1
				y = obj.read(117);
			else
				obj.write(117,x);
			end
		end
	end
	properties
		SELF_TST_X_addr = 13;
		SELF_TST_Y_addr = 14;
		SELF_TST_Z_addr = 15;
		SELF_TST_A_addr = 16;
		SMPLRT_DIV_addr = 25;
		CFG_addr = 26;
		GYRO_CFG_addr = 27;
		INT_EN_addr = 56;
		ACCEL_X_H_addr = 59;
		ACCEL_X_L_addr = 60;
		ACCEL_Y_H_addr = 61;
		ACCEL_Y_L_addr = 62;
		ACCEL_Z_H_addr = 63;
		ACCEL_Z_L_addr = 64;
		TEMP_H_addr = 65;
		TEMP_L_addr = 66;
		GYRO_X_H_addr = 67;
		GYRO_X_L_addr = 68;
		GYRO_Y_H_addr = 69;
		GYRO_Y_L_addr = 70;
		GYRO_Z_H_addr = 71;
		GYRO_Z_L_addr = 72;
		SIGNAL_PATH_RST_addr = 104;
		USER_CTRL_addr = 106;
		PWR_MGMT_1_addr = 107;
		WHO_AM_I_addr = 117;
	end
end
