classdef fc_reg
	methods
		function data = read(obj,addr)
			global ser
			if obj.method
				r = sx1272_receive(0);
				sx1272_send([0,addr],1);
				sleep(300);
				r = sx1272_receive(0);
				data = uint32(sum(r.payload(3:6) .* 2.^(0:8:24)));
			else
				fwrite(ser,[obj.target*4+0,addr]);
				data = uint32(sum(fread(ser,4) .* 2.^(0:8:24)'));
			end
		end
		function write(obj,addr,data)
			global ser
			if obj.method
				sx1272_send([1,addr,floor(mod(double(data) ./ 2.^(0:8:24),2^8))],1);
			else
				fwrite(ser,[obj.target*4+1,addr,floor(mod(double(data) ./ 2.^(0:8:24),2^8))]);
			end
		end
		function y = VERSION(obj,x)
			if nargin < 2
				y = obj.read(0);
			else
				obj.write(0, uint32(x));
			end
		end
		function y = STATUS(obj,x)
			if nargin < 2
				y = obj.read(1);
			else
				obj.write(1, uint32(x));
			end
		end
		function y = CTRL(obj,x)
			if nargin < 2
				y = obj.read(2);
			else
				obj.write(2, uint32(x));
			end
		end
		function y = CTRL__SENSOR_HOST_CTRL(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 1), 0)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__ARM_TEST(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 6), -1)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 6) + bitand(r, 4294967289);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__BEEP_TEST(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 8), -3)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 3), 8) + bitand(r, 4294967287);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__SENSOR_CAL(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 16), -4)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 4), 16) + bitand(r, 4294967279);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__DEBUG(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 32), -5)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 5), 32) + bitand(r, 4294967263);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__RF_HOST_CTRL(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 64), -6)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 6), 64) + bitand(r, 4294967231);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__DEBUG_RADIO(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 128), -7)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 7), 128) + bitand(r, 4294967167);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__OSD_HOST_CTRL(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 256), -8)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 8), 256) + bitand(r, 4294967039);
				obj.write(2, uint32(w));
			end
		end
		function y = CTRL__SMA_HOST_CTRL(obj,x)
			r = double(obj.read(2));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 512), -9)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 9), 512) + bitand(r, 4294966783);
				obj.write(2, uint32(w));
			end
		end
		function y = MOTOR_TEST(obj,x)
			if nargin < 2
				y = obj.read(3);
			else
				obj.write(3, uint32(x));
			end
		end
		function y = MOTOR_TEST__VALUE(obj,x)
			r = double(obj.read(3));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(3, uint32(w));
			end
		end
		function y = MOTOR_TEST__SELECT(obj,x)
			r = double(obj.read(3));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 983040), -16)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 983040) + bitand(r, 4293984255);
				obj.write(3, uint32(w));
			end
		end
		function y = MOTOR_TEST__TELEMETRY(obj,x)
			r = double(obj.read(3));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 1048576), -20)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 20), 1048576) + bitand(r, 4293918719);
				obj.write(3, uint32(w));
			end
		end
		function y = ERROR(obj,x)
			if nargin < 2
				y = obj.read(4);
			else
				obj.write(4, uint32(x));
			end
		end
		function y = ERROR__SENSOR(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 255), 0)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 255) + bitand(r, 4294967040);
				obj.write(4, uint32(w));
			end
		end
		function y = ERROR__RADIO(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65280), -8)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 8), 65280) + bitand(r, 4294902015);
				obj.write(4, uint32(w));
			end
		end
		function y = ERROR__RF(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 16711680), -16)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 16711680) + bitand(r, 4278255615);
				obj.write(4, uint32(w));
			end
		end
		function y = ERROR__CRC(obj,x)
			r = double(obj.read(4));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4278190080), -24)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 24), 4278190080) + bitand(r, 16777215);
				obj.write(4, uint32(w));
			end
		end
		function y = VBAT_MIN(obj,x)
			if nargin < 2
				y = typecast(obj.read(5), 'single');
			else
				obj.write(5, typecast(single(x), 'uint32'));
			end
		end
		function y = TIME_CONSTANT(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6, uint32(x));
			end
		end
		function y = TIME_CONSTANT__VBAT(obj,x)
			r = double(obj.read(6));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(6, uint32(w));
			end
		end
		function y = TIME_CONSTANT__IBAT(obj,x)
			r = double(obj.read(6));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(6, uint32(w));
			end
		end
		function y = TIME_CONSTANT_2(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7, uint32(x));
			end
		end
		function y = TIME_CONSTANT_2__ACCEL(obj,x)
			r = double(obj.read(7));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(7, uint32(w));
			end
		end
		function y = TIME_CONSTANT_2__RADIO(obj,x)
			r = double(obj.read(7));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(7, uint32(w));
			end
		end
		function y = EXPO_PITCH_ROLL(obj,x)
			if nargin < 2
				y = typecast(obj.read(8), 'single');
			else
				obj.write(8, typecast(single(x), 'uint32'));
			end
		end
		function y = EXPO_YAW(obj,x)
			if nargin < 2
				y = typecast(obj.read(9), 'single');
			else
				obj.write(9, typecast(single(x), 'uint32'));
			end
		end
		function y = MOTOR(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10, uint32(x));
			end
		end
		function y = MOTOR__START(obj,x)
			r = double(obj.read(10));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 1023), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1023) + bitand(r, 4294966272);
				obj.write(10, uint32(w));
			end
		end
		function y = MOTOR__ARMED(obj,x)
			r = double(obj.read(10));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 1047552), -10)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 10), 1047552) + bitand(r, 4293919743);
				obj.write(10, uint32(w));
			end
		end
		function y = MOTOR__RANGE(obj,x)
			r = double(obj.read(10));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 2146435072), -20)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 20), 2146435072) + bitand(r, 2148532223);
				obj.write(10, uint32(w));
			end
		end
		function y = RATE(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11, uint32(x));
			end
		end
		function y = RATE__PITCH_ROLL(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4095), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 4095) + bitand(r, 4294963200);
				obj.write(11, uint32(w));
			end
		end
		function y = RATE__YAW(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 16773120), -12)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 12), 16773120) + bitand(r, 4278194175);
				obj.write(11, uint32(w));
			end
		end
		function y = RATE__ANGLE(obj,x)
			r = double(obj.read(11));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4278190080), -24)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 24), 4278190080) + bitand(r, 16777215);
				obj.write(11, uint32(w));
			end
		end
		function y = P_PITCH(obj,x)
			if nargin < 2
				y = typecast(obj.read(12), 'single');
			else
				obj.write(12, typecast(single(x), 'uint32'));
			end
		end
		function y = I_PITCH(obj,x)
			if nargin < 2
				y = typecast(obj.read(13), 'single');
			else
				obj.write(13, typecast(single(x), 'uint32'));
			end
		end
		function y = D_PITCH(obj,x)
			if nargin < 2
				y = typecast(obj.read(14), 'single');
			else
				obj.write(14, typecast(single(x), 'uint32'));
			end
		end
		function y = P_ROLL(obj,x)
			if nargin < 2
				y = typecast(obj.read(15), 'single');
			else
				obj.write(15, typecast(single(x), 'uint32'));
			end
		end
		function y = I_ROLL(obj,x)
			if nargin < 2
				y = typecast(obj.read(16), 'single');
			else
				obj.write(16, typecast(single(x), 'uint32'));
			end
		end
		function y = D_ROLL(obj,x)
			if nargin < 2
				y = typecast(obj.read(17), 'single');
			else
				obj.write(17, typecast(single(x), 'uint32'));
			end
		end
		function y = P_YAW(obj,x)
			if nargin < 2
				y = typecast(obj.read(18), 'single');
			else
				obj.write(18, typecast(single(x), 'uint32'));
			end
		end
		function y = I_YAW(obj,x)
			if nargin < 2
				y = typecast(obj.read(19), 'single');
			else
				obj.write(19, typecast(single(x), 'uint32'));
			end
		end
		function y = D_YAW(obj,x)
			if nargin < 2
				y = typecast(obj.read(20), 'single');
			else
				obj.write(20, typecast(single(x), 'uint32'));
			end
		end
		function y = P_PITCH_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(21), 'single');
			else
				obj.write(21, typecast(single(x), 'uint32'));
			end
		end
		function y = I_PITCH_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(22), 'single');
			else
				obj.write(22, typecast(single(x), 'uint32'));
			end
		end
		function y = D_PITCH_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(23), 'single');
			else
				obj.write(23, typecast(single(x), 'uint32'));
			end
		end
		function y = P_ROLL_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(24), 'single');
			else
				obj.write(24, typecast(single(x), 'uint32'));
			end
		end
		function y = I_ROLL_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(25), 'single');
			else
				obj.write(25, typecast(single(x), 'uint32'));
			end
		end
		function y = D_ROLL_ANGLE(obj,x)
			if nargin < 2
				y = typecast(obj.read(26), 'single');
			else
				obj.write(26, typecast(single(x), 'uint32'));
			end
		end
		function y = GYRO_DC_XY(obj,x)
			if nargin < 2
				y = obj.read(27);
			else
				obj.write(27, uint32(x));
			end
		end
		function y = GYRO_DC_XY__X(obj,x)
			r = double(obj.read(27));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'int16');
				y = z(1);
			else
				w = bitand(bitshift(double(typecast(int32(x),'uint32')), 0), 65535) + bitand(r, 4294901760);
				obj.write(27, uint32(w));
			end
		end
		function y = GYRO_DC_XY__Y(obj,x)
			r = double(obj.read(27));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'int16');
				y = z(1);
			else
				w = bitand(bitshift(double(typecast(int32(x),'uint32')), 16), 4294901760) + bitand(r, 65535);
				obj.write(27, uint32(w));
			end
		end
		function y = GYRO_DC_Z(obj,x)
			if nargin < 2
				y = typecast(obj.read(28), 'int32');
			else
				obj.write(28, typecast(int32(x), 'uint32'));
			end
		end
		function y = ACCEL_DC_XY(obj,x)
			if nargin < 2
				y = obj.read(29);
			else
				obj.write(29, uint32(x));
			end
		end
		function y = ACCEL_DC_XY__X(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'int16');
				y = z(1);
			else
				w = bitand(bitshift(double(typecast(int32(x),'uint32')), 0), 65535) + bitand(r, 4294901760);
				obj.write(29, uint32(w));
			end
		end
		function y = ACCEL_DC_XY__Y(obj,x)
			r = double(obj.read(29));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'int16');
				y = z(1);
			else
				w = bitand(bitshift(double(typecast(int32(x),'uint32')), 16), 4294901760) + bitand(r, 65535);
				obj.write(29, uint32(w));
			end
		end
		function y = ACCEL_DC_Z(obj,x)
			if nargin < 2
				y = typecast(obj.read(30), 'int32');
			else
				obj.write(30, typecast(int32(x), 'uint32'));
			end
		end
		function y = THROTTLE(obj,x)
			if nargin < 2
				y = obj.read(31);
			else
				obj.write(31, uint32(x));
			end
		end
		function y = THROTTLE__IDLE(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(31, uint32(w));
			end
		end
		function y = THROTTLE__RANGE(obj,x)
			r = double(obj.read(31));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(31, uint32(w));
			end
		end
		function y = AILERON(obj,x)
			if nargin < 2
				y = obj.read(32);
			else
				obj.write(32, uint32(x));
			end
		end
		function y = AILERON__IDLE(obj,x)
			r = double(obj.read(32));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(32, uint32(w));
			end
		end
		function y = AILERON__RANGE(obj,x)
			r = double(obj.read(32));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(32, uint32(w));
			end
		end
		function y = ELEVATOR(obj,x)
			if nargin < 2
				y = obj.read(33);
			else
				obj.write(33, uint32(x));
			end
		end
		function y = ELEVATOR__IDLE(obj,x)
			r = double(obj.read(33));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(33, uint32(w));
			end
		end
		function y = ELEVATOR__RANGE(obj,x)
			r = double(obj.read(33));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(33, uint32(w));
			end
		end
		function y = RUDDER(obj,x)
			if nargin < 2
				y = obj.read(34);
			else
				obj.write(34, uint32(x));
			end
		end
		function y = RUDDER__IDLE(obj,x)
			r = double(obj.read(34));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(34, uint32(w));
			end
		end
		function y = RUDDER__RANGE(obj,x)
			r = double(obj.read(34));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(34, uint32(w));
			end
		end
		function y = AUX(obj,x)
			if nargin < 2
				y = obj.read(35);
			else
				obj.write(35, uint32(x));
			end
		end
		function y = AUX__IDLE(obj,x)
			r = double(obj.read(35));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65535), 0)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 65535) + bitand(r, 4294901760);
				obj.write(35, uint32(w));
			end
		end
		function y = AUX__RANGE(obj,x)
			r = double(obj.read(35));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 4294901760), -16)),'uint16');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 16), 4294901760) + bitand(r, 65535);
				obj.write(35, uint32(w));
			end
		end
		function y = MPU_CFG(obj,x)
			if nargin < 2
				y = obj.read(36);
			else
				obj.write(36, uint32(x));
			end
		end
		function y = MPU_CFG__FILT(obj,x)
			r = double(obj.read(36));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 255), 0)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 255) + bitand(r, 4294967040);
				obj.write(36, uint32(w));
			end
		end
		function y = MPU_CFG__RATE(obj,x)
			r = double(obj.read(36));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65280), -8)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 8), 65280) + bitand(r, 4294902015);
				obj.write(36, uint32(w));
			end
		end
		function y = FC_CFG(obj,x)
			if nargin < 2
				y = obj.read(37);
			else
				obj.write(37, uint32(x));
			end
		end
		function y = FC_CFG__I_TRANSFER(obj,x)
			r = double(obj.read(37));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 1), 0)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 1) + bitand(r, 4294967294);
				obj.write(37, uint32(w));
			end
		end
		function y = FC_CFG__MOTOR_PERIOD(obj,x)
			r = double(obj.read(37));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 510), -1)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 1), 510) + bitand(r, 4294966785);
				obj.write(37, uint32(w));
			end
		end
		function y = VTX(obj,x)
			if nargin < 2
				y = obj.read(38);
			else
				obj.write(38, uint32(x));
			end
		end
		function y = VTX__CHAN(obj,x)
			r = double(obj.read(38));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 255), 0)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 0), 255) + bitand(r, 4294967040);
				obj.write(38, uint32(w));
			end
		end
		function y = VTX__PWR(obj,x)
			r = double(obj.read(38));
			if nargin < 2
				z = typecast(uint32(bitshift(bitand(r, 65280), -8)),'uint8');
				y = z(1);
			else
				w = bitand(bitshift(double(x), 8), 65280) + bitand(r, 4294902015);
				obj.write(38, uint32(w));
			end
		end
		function y = VBAT_SCALE(obj,x)
			if nargin < 2
				y = typecast(obj.read(39), 'single');
			else
				obj.write(39, typecast(single(x), 'uint32'));
			end
		end
		function y = IBAT_SCALE(obj,x)
			if nargin < 2
				y = typecast(obj.read(40), 'single');
			else
				obj.write(40, typecast(single(x), 'uint32'));
			end
		end
		function y = DEBUG_INT(obj,x)
			if nargin < 2
				y = obj.read(41);
			else
				obj.write(41, uint32(x));
			end
		end
		function y = DEBUG_FLOAT(obj,x)
			if nargin < 2
				y = typecast(obj.read(42), 'single');
			else
				obj.write(42, typecast(single(x), 'uint32'));
			end
		end
	end
	properties
		method = 0;
		target = 0;
		info = struct(...
			'VERSION', [0,1,0,0],...
			'STATUS', [1,0,0,0],...
			'CTRL', [2,0,0,1],...
			'CTRL__SENSOR_HOST_CTRL', [2,0,0,2],...
			'CTRL__ARM_TEST', [2,0,0,2],...
			'CTRL__BEEP_TEST', [2,0,0,2],...
			'CTRL__SENSOR_CAL', [2,0,0,2],...
			'CTRL__DEBUG', [2,0,0,2],...
			'CTRL__RF_HOST_CTRL', [2,0,0,2],...
			'CTRL__DEBUG_RADIO', [2,0,0,2],...
			'CTRL__OSD_HOST_CTRL', [2,0,0,2],...
			'CTRL__SMA_HOST_CTRL', [2,0,0,2],...
			'MOTOR_TEST', [3,0,0,1],...
			'MOTOR_TEST__VALUE', [3,0,0,2],...
			'MOTOR_TEST__SELECT', [3,0,0,2],...
			'MOTOR_TEST__TELEMETRY', [3,0,0,2],...
			'ERROR', [4,0,0,1],...
			'ERROR__SENSOR', [4,0,0,2],...
			'ERROR__RADIO', [4,0,0,2],...
			'ERROR__RF', [4,0,0,2],...
			'ERROR__CRC', [4,0,0,2],...
			'VBAT_MIN', [5,1,1,0],...
			'TIME_CONSTANT', [6,1,0,1],...
			'TIME_CONSTANT__VBAT', [6,1,0,2],...
			'TIME_CONSTANT__IBAT', [6,1,0,2],...
			'TIME_CONSTANT_2', [7,1,0,1],...
			'TIME_CONSTANT_2__ACCEL', [7,1,0,2],...
			'TIME_CONSTANT_2__RADIO', [7,1,0,2],...
			'EXPO_PITCH_ROLL', [8,1,1,0],...
			'EXPO_YAW', [9,1,1,0],...
			'MOTOR', [10,1,0,1],...
			'MOTOR__START', [10,1,0,2],...
			'MOTOR__ARMED', [10,1,0,2],...
			'MOTOR__RANGE', [10,1,0,2],...
			'RATE', [11,1,0,1],...
			'RATE__PITCH_ROLL', [11,1,0,2],...
			'RATE__YAW', [11,1,0,2],...
			'RATE__ANGLE', [11,1,0,2],...
			'P_PITCH', [12,1,1,0],...
			'I_PITCH', [13,1,1,0],...
			'D_PITCH', [14,1,1,0],...
			'P_ROLL', [15,1,1,0],...
			'I_ROLL', [16,1,1,0],...
			'D_ROLL', [17,1,1,0],...
			'P_YAW', [18,1,1,0],...
			'I_YAW', [19,1,1,0],...
			'D_YAW', [20,1,1,0],...
			'P_PITCH_ANGLE', [21,1,1,0],...
			'I_PITCH_ANGLE', [22,1,1,0],...
			'D_PITCH_ANGLE', [23,1,1,0],...
			'P_ROLL_ANGLE', [24,1,1,0],...
			'I_ROLL_ANGLE', [25,1,1,0],...
			'D_ROLL_ANGLE', [26,1,1,0],...
			'GYRO_DC_XY', [27,1,0,1],...
			'GYRO_DC_XY__X', [27,1,0,2],...
			'GYRO_DC_XY__Y', [27,1,0,2],...
			'GYRO_DC_Z', [28,1,0,0],...
			'ACCEL_DC_XY', [29,1,0,1],...
			'ACCEL_DC_XY__X', [29,1,0,2],...
			'ACCEL_DC_XY__Y', [29,1,0,2],...
			'ACCEL_DC_Z', [30,1,0,0],...
			'THROTTLE', [31,1,0,1],...
			'THROTTLE__IDLE', [31,1,0,2],...
			'THROTTLE__RANGE', [31,1,0,2],...
			'AILERON', [32,1,0,1],...
			'AILERON__IDLE', [32,1,0,2],...
			'AILERON__RANGE', [32,1,0,2],...
			'ELEVATOR', [33,1,0,1],...
			'ELEVATOR__IDLE', [33,1,0,2],...
			'ELEVATOR__RANGE', [33,1,0,2],...
			'RUDDER', [34,1,0,1],...
			'RUDDER__IDLE', [34,1,0,2],...
			'RUDDER__RANGE', [34,1,0,2],...
			'AUX', [35,1,0,1],...
			'AUX__IDLE', [35,1,0,2],...
			'AUX__RANGE', [35,1,0,2],...
			'MPU_CFG', [36,1,0,1],...
			'MPU_CFG__FILT', [36,1,0,2],...
			'MPU_CFG__RATE', [36,1,0,2],...
			'FC_CFG', [37,1,0,1],...
			'FC_CFG__I_TRANSFER', [37,1,0,2],...
			'FC_CFG__MOTOR_PERIOD', [37,1,0,2],...
			'VTX', [38,0,0,1],...
			'VTX__CHAN', [38,0,0,2],...
			'VTX__PWR', [38,0,0,2],...
			'VBAT_SCALE', [39,1,1,0],...
			'IBAT_SCALE', [40,1,1,0],...
			'DEBUG_INT', [41,0,0,0],...
			'DEBUG_FLOAT', [42,0,1,0] );
	end
end
