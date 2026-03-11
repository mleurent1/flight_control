#include <stdint.h>
#include <stdbool.h>

#include "board.h" // sma_transfer()
#include "smart_audio.h" // sma_cmd_e()
#include "utils.h" // crc8()
#include "reg.h" // REG_VTX

/* Global variables -----------------------*/

uint8_t sma_data_to_send[6];
uint8_t sma_data_received[11];

/* Public Functions -----------------------*/

void sma_send_cmd(enum sma_cmd_e sma_cmd, uint8_t data)
{
	uint8_t sma_nbytes_to_send = 0;
	uint8_t sma_nbytes_to_receive = 0;

	sma_data_to_send[0] = 0xAA; // sync
	sma_data_to_send[1] = 0x55; // header

	switch (sma_cmd) {
		case SMA_GET_SETTINGS : {
			sma_data_to_send[2] = 0x03;
			sma_data_to_send[3] = 0x00;
			sma_nbytes_to_send = 4;
			sma_nbytes_to_receive = 11;
			break;
		}
		case SMA_SET_POWER : {
			sma_data_to_send[2] = 0x05;
			sma_data_to_send[3] = 0x01;
			sma_data_to_send[4] = data;
			sma_nbytes_to_send = 5;
			sma_nbytes_to_receive = 8;
			break;
		}
		case SMA_SET_CHANNEL : {
			sma_data_to_send[2] = 0x07;
			sma_data_to_send[3] = 0x01;
			sma_data_to_send[4] = data;
			sma_nbytes_to_send = 5;
			sma_nbytes_to_receive = 8;
			break;
		}
	}

	// Append CRC
	sma_data_to_send[sma_nbytes_to_send] = crc8(sma_data_to_send, sma_nbytes_to_send);
	sma_nbytes_to_send++;

	sma_transfer(sma_data_to_send, sma_data_received, sma_nbytes_to_send, sma_nbytes_to_receive);
}

bool sma_process_resp(void)
{
	uint8_t cmd, len, crc_calc;

	// Check sync + header
	if ((sma_data_received[0] != 0xAA) || (sma_data_received[1] != 0x55))
		return false;

	// Check command
	cmd = sma_data_received[2] & 0x07;
	if ((cmd == 0) || (cmd > 4))
		return false;

	// Check length
	len = sma_data_received[3]; // length + payload
	if ((len != 6) && (len != 3))
		return false;
	
	// Check CRC
	crc_calc = crc8(&sma_data_received[2], len + 1); // CRC computation does not include sync and header, but includes command
	if (sma_data_received[2+len+1] != crc_calc)
		return false;

	switch (cmd) {
		case 0x01 : {
			REG_VTX = ((uint32_t)sma_data_received[4] << REG_VTX__CHAN_Pos) | ((uint32_t)sma_data_received[5] << REG_VTX__PWR_Pos);
			break;
		}
		case 0x02 : {
			REG_VTX &= ~REG_VTX__PWR_Msk;
			REG_VTX |= (uint32_t)sma_data_received[4] << REG_VTX__PWR_Pos;
			break;
		}
		case 0x03 : {
			REG_VTX &= ~REG_VTX__CHAN_Msk;
			REG_VTX |= (uint32_t)sma_data_received[4] << REG_VTX__CHAN_Pos;
			break;
		}
	}

	return true;
}
