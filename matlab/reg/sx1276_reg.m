classdef sx1276_reg
	methods
		function data = read(obj,addr)
			global ser
			fwrite(ser,[7,addr,0,0,0,0]);
			data = fread(ser,1);
		end
		function write(obj,addr,data)
			global ser
			fwrite(ser,[8,addr,0,0,0,data]);
		end
		function y = FIFO(obj,x)
			if nargin < 2
				y = obj.read(0);
			else
				obj.write(0,x);
			end
		end
		function y = OP_MODE(obj,x)
			if nargin < 2
				y = obj.read(1);
			else
				obj.write(1,x);
			end
		end
		function y = OP_MODE__LONG_RANGE_MODE(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(1,w);
			end
		end
		function y = OP_MODE__ACCESS_SHARED_REG(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(1,w);
			end
		end
		function y = OP_MODE__LOW_FREQUENCY_MODE_ON(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(1,w);
			end
		end
		function y = OP_MODE__MODE(obj,x)
			r = obj.read(1);
			if nargin < 2
				y = bitshift(bitand(r, 7), 0);
			else
				w = bitand(bitshift(x, 0), 7) + bitand(r, 248);
				obj.write(1,w);
			end
		end
		function y = FR_MSB(obj,x)
			if nargin < 2
				y = obj.read(6);
			else
				obj.write(6,x);
			end
		end
		function y = FR_MID(obj,x)
			if nargin < 2
				y = obj.read(7);
			else
				obj.write(7,x);
			end
		end
		function y = FR_LSB(obj,x)
			if nargin < 2
				y = obj.read(8);
			else
				obj.write(8,x);
			end
		end
		function y = PA_CONFIG(obj,x)
			if nargin < 2
				y = obj.read(9);
			else
				obj.write(9,x);
			end
		end
		function y = PA_CONFIG__PA_SELECT(obj,x)
			r = obj.read(9);
			if nargin < 2
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(9,w);
			end
		end
		function y = PA_CONFIG__MAX_POWER(obj,x)
			r = obj.read(9);
			if nargin < 2
				y = bitshift(bitand(r, 112), -4);
			else
				w = bitand(bitshift(x, 4), 112) + bitand(r, 143);
				obj.write(9,w);
			end
		end
		function y = PA_CONFIG__OUTPUT_POWER(obj,x)
			r = obj.read(9);
			if nargin < 2
				y = bitshift(bitand(r, 15), 0);
			else
				w = bitand(bitshift(x, 0), 15) + bitand(r, 240);
				obj.write(9,w);
			end
		end
		function y = PA_RAMP(obj,x)
			if nargin < 2
				y = obj.read(10);
			else
				obj.write(10,x);
			end
		end
		function y = OCP(obj,x)
			if nargin < 2
				y = obj.read(11);
			else
				obj.write(11,x);
			end
		end
		function y = OCP__OCP_ON(obj,x)
			r = obj.read(11);
			if nargin < 2
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(11,w);
			end
		end
		function y = OCP__OCP_TRIM(obj,x)
			r = obj.read(11);
			if nargin < 2
				y = bitshift(bitand(r, 31), 0);
			else
				w = bitand(bitshift(x, 0), 31) + bitand(r, 224);
				obj.write(11,w);
			end
		end
		function y = LNA(obj,x)
			if nargin < 2
				y = obj.read(12);
			else
				obj.write(12,x);
			end
		end
		function y = LNA__LNA_GAIN(obj,x)
			r = obj.read(12);
			if nargin < 2
				y = bitshift(bitand(r, 224), -5);
			else
				w = bitand(bitshift(x, 5), 224) + bitand(r, 31);
				obj.write(12,w);
			end
		end
		function y = LNA__LNA_BOOST_LF(obj,x)
			r = obj.read(12);
			if nargin < 2
				y = bitshift(bitand(r, 24), -3);
			else
				w = bitand(bitshift(x, 3), 24) + bitand(r, 231);
				obj.write(12,w);
			end
		end
		function y = LNA__LNA_BOOST_HF(obj,x)
			r = obj.read(12);
			if nargin < 2
				y = bitshift(bitand(r, 3), 0);
			else
				w = bitand(bitshift(x, 0), 3) + bitand(r, 252);
				obj.write(12,w);
			end
		end
		function y = FIFO_ADDR_PTR(obj,x)
			if nargin < 2
				y = obj.read(13);
			else
				obj.write(13,x);
			end
		end
		function y = FIFO_TX_BASE_ADDR(obj,x)
			if nargin < 2
				y = obj.read(14);
			else
				obj.write(14,x);
			end
		end
		function y = FIFO_RX_BASE_ADDR(obj,x)
			if nargin < 2
				y = obj.read(15);
			else
				obj.write(15,x);
			end
		end
		function y = FIFO_RX_CURRENT_ADDR(obj,x)
			if nargin < 2
				y = obj.read(16);
			else
				obj.write(16,x);
			end
		end
		function y = IRQ_FLAGS_MASK(obj,x)
			if nargin < 2
				y = obj.read(17);
			else
				obj.write(17,x);
			end
		end
		function y = IRQ_FLAGS_MASK__RX_TIMEOUT_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__RX_DONE_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__PAYLOAD_CRC_ERROR_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__VALID_HEADER_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 16), -4);
			else
				w = bitand(bitshift(x, 4), 16) + bitand(r, 239);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__TX_DONE_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__CAD_DONE_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__FHSS_CHANGE_CHANNEL_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 253);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS_MASK__CAD_DETECTED_MASK(obj,x)
			r = obj.read(17);
			if nargin < 2
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(17,w);
			end
		end
		function y = IRQ_FLAGS(obj,x)
			if nargin < 2
				y = obj.read(18);
			else
				obj.write(18,x);
			end
		end
		function y = IRQ_FLAGS__RX_TIMEOUT(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__RX_DONE(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__PAYLOAD_CRC_ERROR(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 32), -5);
			else
				w = bitand(bitshift(x, 5), 32) + bitand(r, 223);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__VALID_HEADER(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 16), -4);
			else
				w = bitand(bitshift(x, 4), 16) + bitand(r, 239);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__TX_DONE(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__CAD_DONE(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__FHSS_CHANGE_CHANNEL(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 2), -1);
			else
				w = bitand(bitshift(x, 1), 2) + bitand(r, 253);
				obj.write(18,w);
			end
		end
		function y = IRQ_FLAGS__CAD_DETECTED(obj,x)
			r = obj.read(18);
			if nargin < 2
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(18,w);
			end
		end
		function y = RX_NB_BYTES(obj,x)
			if nargin < 2
				y = obj.read(19);
			else
				obj.write(19,x);
			end
		end
		function y = RX_HEADER_CNT_VALUE_MSB(obj,x)
			if nargin < 2
				y = obj.read(20);
			else
				obj.write(20,x);
			end
		end
		function y = RX_HEADER_CNT_VALUE_LSB(obj,x)
			if nargin < 2
				y = obj.read(21);
			else
				obj.write(21,x);
			end
		end
		function y = RX_PACKET_CNT_VALUE_MSB(obj,x)
			if nargin < 2
				y = obj.read(22);
			else
				obj.write(22,x);
			end
		end
		function y = RX_PACKET_CNT_VALUE_LSB(obj,x)
			if nargin < 2
				y = obj.read(23);
			else
				obj.write(23,x);
			end
		end
		function y = MODEM_STAT(obj,x)
			if nargin < 2
				y = obj.read(24);
			else
				obj.write(24,x);
			end
		end
		function y = MODEM_STAT__RX_CODING_RATE(obj,x)
			r = obj.read(24);
			if nargin < 2
				y = bitshift(bitand(r, 224), -5);
			else
				w = bitand(bitshift(x, 5), 224) + bitand(r, 31);
				obj.write(24,w);
			end
		end
		function y = MODEM_STAT__MODEM_STATUS(obj,x)
			r = obj.read(24);
			if nargin < 2
				y = bitshift(bitand(r, 31), 0);
			else
				w = bitand(bitshift(x, 0), 31) + bitand(r, 224);
				obj.write(24,w);
			end
		end
		function y = PKT_SNR_VALUE(obj,x)
			if nargin < 2
				y = obj.read(25);
			else
				obj.write(25,x);
			end
		end
		function y = PKT_RSSI_VALUE(obj,x)
			if nargin < 2
				y = obj.read(26);
			else
				obj.write(26,x);
			end
		end
		function y = RSSI_VALUE(obj,x)
			if nargin < 2
				y = obj.read(27);
			else
				obj.write(27,x);
			end
		end
		function y = HOP_CHANNEL(obj,x)
			if nargin < 2
				y = obj.read(28);
			else
				obj.write(28,x);
			end
		end
		function y = HOP_CHANNEL__PLL_TIMEOUT(obj,x)
			r = obj.read(28);
			if nargin < 2
				y = bitshift(bitand(r, 128), -7);
			else
				w = bitand(bitshift(x, 7), 128) + bitand(r, 127);
				obj.write(28,w);
			end
		end
		function y = HOP_CHANNEL__CRC_ON_PAYLOAD(obj,x)
			r = obj.read(28);
			if nargin < 2
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(28,w);
			end
		end
		function y = HOP_CHANNEL__FHSS_PRESENT_CHANNEL(obj,x)
			r = obj.read(28);
			if nargin < 2
				y = bitshift(bitand(r, 63), 0);
			else
				w = bitand(bitshift(x, 0), 63) + bitand(r, 192);
				obj.write(28,w);
			end
		end
		function y = MODEM_CONFIG_1(obj,x)
			if nargin < 2
				y = obj.read(29);
			else
				obj.write(29,x);
			end
		end
		function y = MODEM_CONFIG_1__BW(obj,x)
			r = obj.read(29);
			if nargin < 2
				y = bitshift(bitand(r, 240), -4);
			else
				w = bitand(bitshift(x, 4), 240) + bitand(r, 15);
				obj.write(29,w);
			end
		end
		function y = MODEM_CONFIG_1__CODING_RATE(obj,x)
			r = obj.read(29);
			if nargin < 2
				y = bitshift(bitand(r, 14), -1);
			else
				w = bitand(bitshift(x, 1), 14) + bitand(r, 241);
				obj.write(29,w);
			end
		end
		function y = MODEM_CONFIG_1__IMPLICIT_HEADER_MODE_ON(obj,x)
			r = obj.read(29);
			if nargin < 2
				y = bitshift(bitand(r, 1), 0);
			else
				w = bitand(bitshift(x, 0), 1) + bitand(r, 254);
				obj.write(29,w);
			end
		end
		function y = MODEM_CONFIG_2(obj,x)
			if nargin < 2
				y = obj.read(30);
			else
				obj.write(30,x);
			end
		end
		function y = MODEM_CONFIG_2__SPREADING_FACTOR(obj,x)
			r = obj.read(30);
			if nargin < 2
				y = bitshift(bitand(r, 240), -4);
			else
				w = bitand(bitshift(x, 4), 240) + bitand(r, 15);
				obj.write(30,w);
			end
		end
		function y = MODEM_CONFIG_2__TX_CONTINUOUS_MODE(obj,x)
			r = obj.read(30);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(30,w);
			end
		end
		function y = MODEM_CONFIG_2__RX_PAYLOAD_CRC_ON(obj,x)
			r = obj.read(30);
			if nargin < 2
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(30,w);
			end
		end
		function y = MODEM_CONFIG_2__SYMB_TIMEOUT_MSB(obj,x)
			r = obj.read(30);
			if nargin < 2
				y = bitshift(bitand(r, 3), 0);
			else
				w = bitand(bitshift(x, 0), 3) + bitand(r, 252);
				obj.write(30,w);
			end
		end
		function y = SYMB_TIMEOUT_LSB(obj,x)
			if nargin < 2
				y = obj.read(31);
			else
				obj.write(31,x);
			end
		end
		function y = PREAMBLE_MSB(obj,x)
			if nargin < 2
				y = obj.read(32);
			else
				obj.write(32,x);
			end
		end
		function y = PREAMBLE_LSB(obj,x)
			if nargin < 2
				y = obj.read(33);
			else
				obj.write(33,x);
			end
		end
		function y = PAYLOAD_LENGTH(obj,x)
			if nargin < 2
				y = obj.read(34);
			else
				obj.write(34,x);
			end
		end
		function y = MAX_PAYLOAD_LENGTH(obj,x)
			if nargin < 2
				y = obj.read(35);
			else
				obj.write(35,x);
			end
		end
		function y = HOP_PERIOD(obj,x)
			if nargin < 2
				y = obj.read(36);
			else
				obj.write(36,x);
			end
		end
		function y = FIFO_RX_BYTE_ADDR(obj,x)
			if nargin < 2
				y = obj.read(37);
			else
				obj.write(37,x);
			end
		end
		function y = MODEM_CONFIG_3(obj,x)
			if nargin < 2
				y = obj.read(38);
			else
				obj.write(38,x);
			end
		end
		function y = MODEM_CONFIG_3__LOW_DATA_RATE_OPTIMIZE(obj,x)
			r = obj.read(38);
			if nargin < 2
				y = bitshift(bitand(r, 8), -3);
			else
				w = bitand(bitshift(x, 3), 8) + bitand(r, 247);
				obj.write(38,w);
			end
		end
		function y = MODEM_CONFIG_3__AGC_AUTO_ON(obj,x)
			r = obj.read(38);
			if nargin < 2
				y = bitshift(bitand(r, 4), -2);
			else
				w = bitand(bitshift(x, 2), 4) + bitand(r, 251);
				obj.write(38,w);
			end
		end
		function y = PPM_CORRECTION(obj,x)
			if nargin < 2
				y = obj.read(39);
			else
				obj.write(39,x);
			end
		end
		function y = FEI_MSB(obj,x)
			if nargin < 2
				y = obj.read(40);
			else
				obj.write(40,x);
			end
		end
		function y = FEI_MIB(obj,x)
			if nargin < 2
				y = obj.read(41);
			else
				obj.write(41,x);
			end
		end
		function y = FEI_LSB(obj,x)
			if nargin < 2
				y = obj.read(42);
			else
				obj.write(42,x);
			end
		end
		function y = RSSI_WIDEBAND(obj,x)
			if nargin < 2
				y = obj.read(44);
			else
				obj.write(44,x);
			end
		end
		function y = DETECT_OPTIMIZE(obj,x)
			if nargin < 2
				y = obj.read(49);
			else
				obj.write(49,x);
			end
		end
		function y = INVERT_IQ(obj,x)
			if nargin < 2
				y = obj.read(51);
			else
				obj.write(51,x);
			end
		end
		function y = INVERT_IQ__INVERT_IQ(obj,x)
			r = obj.read(51);
			if nargin < 2
				y = bitshift(bitand(r, 64), -6);
			else
				w = bitand(bitshift(x, 6), 64) + bitand(r, 191);
				obj.write(51,w);
			end
		end
		function y = DIO_MAPPING_1(obj,x)
			if nargin < 2
				y = obj.read(64);
			else
				obj.write(64,x);
			end
		end
		function y = DIO_MAPPING_1__DIO0_MAPPING(obj,x)
			r = obj.read(64);
			if nargin < 2
				y = bitshift(bitand(r, 192), -6);
			else
				w = bitand(bitshift(x, 6), 192) + bitand(r, 63);
				obj.write(64,w);
			end
		end
		function y = DIO_MAPPING_1__DIO1_MAPPING(obj,x)
			r = obj.read(64);
			if nargin < 2
				y = bitshift(bitand(r, 48), -4);
			else
				w = bitand(bitshift(x, 4), 48) + bitand(r, 207);
				obj.write(64,w);
			end
		end
		function y = DIO_MAPPING_1__DIO2_MAPPING(obj,x)
			r = obj.read(64);
			if nargin < 2
				y = bitshift(bitand(r, 12), -2);
			else
				w = bitand(bitshift(x, 2), 12) + bitand(r, 243);
				obj.write(64,w);
			end
		end
		function y = DIO_MAPPING_1__DIO3_MAPPING(obj,x)
			r = obj.read(64);
			if nargin < 2
				y = bitshift(bitand(r, 3), 0);
			else
				w = bitand(bitshift(x, 0), 3) + bitand(r, 252);
				obj.write(64,w);
			end
		end
	end
	properties
		FIFO_addr = 0;
		OP_MODE_addr = 1;
		FR_MSB_addr = 6;
		FR_MID_addr = 7;
		FR_LSB_addr = 8;
		PA_CONFIG_addr = 9;
		PA_RAMP_addr = 10;
		OCP_addr = 11;
		LNA_addr = 12;
		FIFO_ADDR_PTR_addr = 13;
		FIFO_TX_BASE_ADDR_addr = 14;
		FIFO_RX_BASE_ADDR_addr = 15;
		FIFO_RX_CURRENT_ADDR_addr = 16;
		IRQ_FLAGS_MASK_addr = 17;
		IRQ_FLAGS_addr = 18;
		RX_NB_BYTES_addr = 19;
		RX_HEADER_CNT_VALUE_MSB_addr = 20;
		RX_HEADER_CNT_VALUE_LSB_addr = 21;
		RX_PACKET_CNT_VALUE_MSB_addr = 22;
		RX_PACKET_CNT_VALUE_LSB_addr = 23;
		MODEM_STAT_addr = 24;
		PKT_SNR_VALUE_addr = 25;
		PKT_RSSI_VALUE_addr = 26;
		RSSI_VALUE_addr = 27;
		HOP_CHANNEL_addr = 28;
		MODEM_CONFIG_1_addr = 29;
		MODEM_CONFIG_2_addr = 30;
		SYMB_TIMEOUT_LSB_addr = 31;
		PREAMBLE_MSB_addr = 32;
		PREAMBLE_LSB_addr = 33;
		PAYLOAD_LENGTH_addr = 34;
		MAX_PAYLOAD_LENGTH_addr = 35;
		HOP_PERIOD_addr = 36;
		FIFO_RX_BYTE_ADDR_addr = 37;
		MODEM_CONFIG_3_addr = 38;
		PPM_CORRECTION_addr = 39;
		FEI_MSB_addr = 40;
		FEI_MIB_addr = 41;
		FEI_LSB_addr = 42;
		RSSI_WIDEBAND_addr = 44;
		DETECT_OPTIMIZE_addr = 49;
		INVERT_IQ_addr = 51;
		DIO_MAPPING_1_addr = 64;
	end
end
