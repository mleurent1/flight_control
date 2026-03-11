#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memcpy()

#include "osd.h"
#include "max7456_reg.h"
#include "board.h" // osd_send()
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include "radio.h" // struct radio_s
#include "reg.h" // reg_save()
#include "utils.h" // crc8()
#include "smart_audio.h" // sma_send_cmd()

/* Private defines --------------------------------------*/

#define DISP_ADDR_MENU (9*30+5)
#define DISP_ADDR_TELEMETRY_NTSC (10*30+1) // Last row index is 12
#define DISP_ADDR_TELEMETRY_PAL (13*30+1) // Last row index is 15

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

enum state_e {TELEMETRY,
	MENU_ENTER, MENU, MENU_UP, MENU_DOWN, MENU_RIGHT, MENU_EXIT,
	REG, REG_UP, REG_DOWN, REG_EXIT,
	SAVED, SAVED_EXIT,
	RUNCAM_MENU, RUNCAM_LEFT, RUNCAM_RIGHT, RUNCAM_UP, RUNCAM_DOWN, RUNCAM_ENTER, RUNCAM_EXIT};

enum menu_e {P_PITCH_ROLL=0, I_PITCH_ROLL, D_PITCH_ROLL, P_YAW, I_YAW, RATE, EXPO, MOTOR_START, MOTOR_ARMED, MOTOR_RANGE,
	VTX_CHANNEL, VTX_POWER, SAVE_REG, RUNCAM};

enum runcam_cmd_e {LEFT, RIGHT, UP, DOWN, ENTER, RELEASE, OPEN, CLOSE};

/* Private variables -----------------------*/

#ifdef IBAT
	uint16_t telem_str[43] = {10, 10, 65, 10, 10, 32,  0, 10, 10, 10, 65, 10, 11,  0, 10, 10, 10, 10, 49, 11, 44,  0, 10, 10, 68, 10, 10,  0,  0, 73, 10, 10, 10, 40, 12, 49,  0, 73, 10, 10, 40, 12, 255}; // 00.00V 000.0A 0000mAh 00:00  -000dBm -00dB
#else
	uint16_t telem_str[27] = {10, 10, 65, 10, 10, 32,  0, 10, 10, 68, 10, 10,  0, 73, 10, 10, 10, 40, 12, 49,  0, 73, 10, 10, 40, 12, 255}; // 00.00V 00:00 -000dBm -00dB
#endif
uint16_t menu_str[15][16] = {
	{26,  0, 26, 19, 30, 13, 18,  0, 28, 25, 22, 22,  0,  0,  0, 255}, // P PITCH ROLL
	{19,  0, 26, 19, 30, 13, 18,  0, 28, 25, 22, 22,  0,  0,  0, 255}, // I PITCH ROLL
	{14,  0, 26, 19, 30, 13, 18,  0, 28, 25, 22, 22,  0,  0,  0, 255}, // D PITCH ROLL
	{26,  0, 35, 11, 33,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}, // P YAW
	{19,  0, 35, 11, 33,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}, // I YAW
	{28, 11, 30, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}, // RATE
	{15, 34, 26, 25,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}, // EXPO
	{23, 25, 30, 25, 28,  0, 29, 30, 11, 28, 30,  0,  0,  0,  0, 255}, // MOTOR START
	{23, 25, 30, 25, 28,  0, 11, 28, 23, 15, 14,  0,  0,  0,  0, 255}, // MOTOR ARMED
	{23, 25, 30, 25, 28,  0, 28, 11, 24, 17, 15,  0,  0,  0,  0, 255}, // MOTOR RANGE
	{32, 30, 34,  0, 13, 18, 11, 24, 24, 15, 22,  0,  0,  0,  0, 255}, // VTX CHANNEL
	{32, 30, 34,  0, 26, 25, 33, 15, 28,  0,  0,  0,  0,  0,  0, 255}, // VTX POWER
	{29, 11, 32, 15,  0, 28, 15, 17,  0,  0,  0,  0,  0,  0,  0, 255}, // SAVE REG
	{29, 11, 32, 15,  0, 32, 30, 34,  0,  0,  0,  0,  0,  0,  0, 255}, // SAVE VTX
	{28, 31, 24, 13, 11, 23,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}  // RUNCAM
};
uint16_t vtx_str[48][16] = {
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  1,  0,  5,  8,  6,  5, 255}, // BAND A   1 5865
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  2,  0,  5,  8,  4,  5, 255}, // BAND A   2 5845
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  3,  0,  5,  8,  2,  5, 255}, // BAND A   3 5825
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  4,  0,  5,  8, 10,  5, 255}, // BAND A   4 5805
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  5,  0,  5,  7,  8,  5, 255}, // BAND A   5 5785
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  6,  0,  5,  7,  6,  5, 255}, // BAND A   6 5765
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  7,  0,  5,  7,  4,  5, 255}, // BAND A   7 5745
	{12, 11, 24, 14,  0, 11,  0,  0,  0,  8,  0,  5,  7,  2,  5, 255}, // BAND A   8 5725
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  1,  0,  5,  7,  3,  3, 255}, // BAND B   1 5733
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  2,  0,  5,  7,  5,  2, 255}, // BAND B   2 5752
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  3,  0,  5,  7,  7,  1, 255}, // BAND B   3 5771
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  4,  0,  5,  7,  9, 10, 255}, // BAND B   4 5790
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  5,  0,  5,  8, 10,  9, 255}, // BAND B   5 5809
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  6,  0,  5,  8,  2,  8, 255}, // BAND B   6 5828
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  7,  0,  5,  8,  4,  7, 255}, // BAND B   7 5847
	{12, 11, 24, 14,  0, 12,  0,  0,  0,  8,  0,  5,  8,  6,  6, 255}, // BAND B   8 5866
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  1,  0,  5,  7, 10,  5, 255}, // BAND E   1 5705
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  2,  0,  5,  6,  8,  5, 255}, // BAND E   2 5685
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  3,  0,  5,  6,  6,  5, 255}, // BAND E   3 5665
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  4,  0,  5,  6,  4,  5, 255}, // BAND E   4 5645
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  5,  0,  5,  8,  8,  5, 255}, // BAND E   5 5885
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  6,  0,  5,  9, 10,  5, 255}, // BAND E   6 5905
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  7,  0,  5,  9,  2,  5, 255}, // BAND E   7 5925
	{12, 11, 24, 14,  0, 15,  0,  0,  0,  8,  0,  5,  9,  4,  5, 255}, // BAND E   8 5945
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  1,  0,  5,  7,  4, 10, 255}, // AIRWAVE  1 5740
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  2,  0,  5,  7,  6, 10, 255}, // AIRWAVE  2 5760
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  3,  0,  5,  7,  8, 10, 255}, // AIRWAVE  3 5780
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  4,  0,  5,  8, 10, 10, 255}, // AIRWAVE  4 5800
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  5,  0,  5,  8,  2, 10, 255}, // AIRWAVE  5 5820
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  6,  0,  5,  8,  4, 10, 255}, // AIRWAVE  6 5840
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  7,  0,  5,  8,  6, 10, 255}, // AIRWAVE  7 5860
	{11, 19, 28, 33, 11, 32, 15,  0,  0,  8,  0,  5,  8,  8, 10, 255}, // AIRWAVE  8 5880
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  1,  0,  5,  6,  5,  8, 255}, // RACE     1 5658
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  2,  0,  5,  6,  9,  5, 255}, // RACE     2 5695
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  3,  0,  5,  7,  3,  2, 255}, // RACE     3 5732
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  4,  0,  5,  7,  6,  9, 255}, // RACE     4 5769
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  5,  0,  5,  8, 10,  6, 255}, // RACE     5 5806
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  6,  0,  5,  8,  4,  3, 255}, // RACE     6 5843
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  7,  0,  5,  8,  8, 10, 255}, // RACE     7 5880
	{28, 11, 13, 15,  0,  0,  0,  0,  0,  8,  0,  5,  9,  1,  7, 255}, // RACE     8 5917
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  1,  0,  5,  6,  2,  1, 255}, // LOW RACE 1 5621
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  2,  0,  5,  5,  8,  4, 255}, // LOW RACE 2 5584
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  3,  0,  5,  5,  4,  7, 255}, // LOW RACE 3 5547
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  4,  0,  5,  5,  1, 10, 255}, // LOW RACE 4 5510
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  5,  0,  5,  4,  7,  3, 255}, // LOW RACE 5 5473
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  6,  0,  5,  4,  3,  6, 255}, // LOW RACE 6 5436
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  7,  0,  5,  3,  9,  9, 255}, // LOW RACE 7 5399
	{22, 25, 33,  0, 28, 11, 13, 15,  0,  8,  0,  5,  3,  6,  2, 255}  // LOW RACE 8 5362
};
uint16_t saved_str[16] = {29, 11, 32, 15, 14,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 255}; // SAVED
uint16_t reg_val_str[16] = {10, 10, 10, 10, 65, 10, 10, 10,  0,  0,  0,  0,  0,  0,  0, 255}; // 0000.000

uint8_t osd_data_to_send[2];
uint8_t osd_data_received[sizeof(telem_str)];
uint8_t* osd_next_data_to_send;
uint8_t osd_next_nbytes_to_send = 0;
uint16_t disp_addr_telem;
enum state_e state = TELEMETRY;
enum state_e state_prev = TELEMETRY;
uint8_t menu_idx = 0;

/* Private Functions -----------------------*/

void osd_write(uint8_t addr, uint8_t data)
{
	osd_data_to_send[0] = addr & 0x7F;
	osd_data_to_send[1] = data;
	osd_transfer(osd_data_to_send, osd_data_received, 2);
	while (osd_busy) {}
}

uint8_t osd_read(uint8_t addr)
{
	osd_data_to_send[0] = 0x80 | (addr & 0x7F);
	osd_transfer(osd_data_to_send, osd_data_received, 2);
	while (osd_busy) {}
	return osd_data_received[1];
}

void osd_write_str(uint8_t* str, uint8_t size)
{
	osd_next_data_to_send = str;
	osd_next_nbytes_to_send = size;
	// Enable auto-increment
	osd_data_to_send[0] = MAX7456_DMM;
	osd_data_to_send[1] = MAX7456_DMM__AUTO_INCR_EN;
	osd_transfer(osd_data_to_send, osd_data_received, 2);
}

void float_to_str(float num, uint16_t* str_int, uint16_t* str_frac, uint8_t int_size, uint8_t frac_size)
{
	uint8_t dec_int[4], dec_frac[3];
	bool nonzero = false;
	uint8_t i,j;

	float_to_dec(num, dec_int, dec_frac);

	for (i=0; i<int_size; i++) {
		j = 4-int_size+i; // Take LSBs
		if (dec_int[j] > 0)
			nonzero = true;
		if ((dec_int[j] == 0) && (nonzero || (i == int_size-1)))
			str_int[i] = 10;
		else
			str_int[i] = dec_int[j];
	}
	for (i=0; i<frac_size; i++) {
		if (dec_frac[i] == 0)
			str_frac[i] = 10;
		else
			str_frac[i] = dec_frac[i];
	}
}

void runcam_send_cmd(enum runcam_cmd_e runcam_cmd)
{
	uint8_t buf[4];
	uint8_t len = 0;

	buf[0] = 0xCC;
	switch (runcam_cmd) {
		case LEFT : {
			buf[1] = 2;
			buf[2] = 2;
			len = 3;
			break;
		}
		case RIGHT : {
			buf[1] = 2;
			buf[2] = 3;
			len = 3;
			break;
		}
		case UP : {
			buf[1] = 2;
			buf[2] = 4;
			len = 3;
			break;
		}
		case DOWN : {
			buf[1] = 2;
			buf[2] = 5;
			len = 3;
			break;
		}
		case ENTER : {
			buf[1] = 2;
			buf[2] = 1;
			len = 3;
			break;
		}
		case RELEASE : {
			buf[1] = 3;
			len = 2;
			break;
		}
		case OPEN : {
			buf[1] = 4;
			buf[2] = 1;
			len = 3;
			break;
		}
		case CLOSE : {
			buf[1] = 4;
			buf[2] = 2;
			len = 3;
			break;
		}
	}
	buf[len] = crc8(buf, len);
	runcam_send(buf, len + 1);
}

/* Public Functions -----------------------*/

void osd_init(void)
{
	uint8_t r;
	r = osd_read(MAX7456_OSDBL);
	osd_write(MAX7456_OSDBL, r & ~MAX7456_OSDBL__AUTO_OSDBL_DISABLE);
	r = osd_read(MAX7456_STAT);
	if (r & MAX7456_STAT__PAL_DETECTED) {
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN | MAX7456_VM0__PAL_NOT_NTSC);
		disp_addr_telem = DISP_ADDR_TELEMETRY_PAL;
	}
	else {
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN);
		disp_addr_telem = DISP_ADDR_TELEMETRY_NTSC;
	}
	osd_write(MAX7456_HOS, 40);
	osd_write(MAX7456_VOS, 22);
	osd_write(MAX7456_DMAH, disp_addr_telem >> 8);
	osd_write(MAX7456_DMAL, disp_addr_telem & 0xFF);
}

void osd_telemetry(float vbat, float ibat, float imah, uint8_t t_s, uint8_t t_min, uint8_t rssi, int8_t snr)
{
		if ((state == TELEMETRY) && (!osd_busy)) {
		float_to_str(vbat, &telem_str[0], &telem_str[3], 2, 2);
	#ifdef IBAT
		float_to_str(ibat, &telem_str[7], &telem_str[11], 3, 1);
		float_to_str(imah, &telem_str[14], &telem_str[14], 4, 0);
		float_to_str((float)t_min, &telem_str[22], &telem_str[22], 2, 0);
		float_to_str((float)t_s, &telem_str[25], &telem_str[25], 2, 0);
		float_to_str((float)rssi, &telem_str[30], &telem_str[30], 3, 0);
		if (snr >= 0) {
			float_to_str((float)snr, &telem_str[38], &telem_str[38], 2, 0);
			telem_str[37] = 0;
		} else {
			float_to_str(-(float)snr, &telem_str[38], &telem_str[38], 2, 0);
		}
	#else
		float_to_str((float)t_min, &telem_str[7], &telem_str[7], 2, 0);
		float_to_str((float)t_s, &telem_str[10], &telem_str[10], 2, 0);
		float_to_str((float)rssi, &telem_str[14], &telem_str[14], 3, 0);
		if (snr >= 0) {
			float_to_str((float)snr, &telem_str[22], &telem_str[22], 2, 0);
			telem_str[21] = 0;
		} else {
			float_to_str(-(float)snr, &telem_str[22], &telem_str[22], 2, 0);
		}
	#endif
		osd_write_str((uint8_t*)telem_str, sizeof(telem_str)-1); // -1: avoid 0 after last byte 255
	}
}

void osd_menu(struct radio_s * radio)
{
	int32_t incr_int;
	float incr_f;
	int32_t rate;
	int32_t motor_start, motor_armed, motor_range;
	int32_t vtx_chan, vtx_pwr;
	float vtx_pwr_mw;

	// State machine
	switch (state_prev) {
		case TELEMETRY : {
			if (radio->yaw > 0.66f)
				state = MENU_ENTER;
			break;
		}
		case MENU_ENTER : {
			if (radio->yaw < 0.33f)
				state = MENU;
			break;
		}
		case MENU : {
			if (radio->pitch > 0.66f)
				state = MENU_UP;
			else if (radio->pitch < -0.66f)
				state = MENU_DOWN;
			else if (radio->roll > 0.66f)
				state = MENU_RIGHT;
			else if (radio->yaw < -0.66f)
				state = MENU_EXIT;
			break;
		}
		case MENU_UP : {
			if (radio->pitch < 0.33f)
				state = MENU;
			break;
		}
		case MENU_DOWN : {
			if (radio->pitch > -0.33f)
				state = MENU;
			break;
		}
		case MENU_RIGHT : {
			if (radio->roll < 0.33f) {
				if (menu_idx < SAVE_REG)
					state = REG;
				else if (menu_idx == RUNCAM)
					state = RUNCAM_MENU;
				else // if ((menu_idx == SAVE_REG) || (menu_idx == SAVE_VTX))
					state = SAVED;
			}
			break;
		}
		case MENU_EXIT : {
			if (radio->yaw > -0.33f)
				state = TELEMETRY;
			break;
		}
		case REG : {
			if (radio->pitch > 0.66f)
				state = REG_UP;
			else if (radio->pitch < -0.66f)
				state = REG_DOWN;
			else if (radio->roll < -0.66f)
				state = REG_EXIT;
			break;
		}
		case REG_UP : {
			if (radio->pitch < 0.33f)
				state = REG;
			break;
		}
		case REG_DOWN : {
			if (radio->pitch > -0.33f)
				state = REG;
			break;
		}
		case REG_EXIT : {
			if (radio->roll > -0.33f)
				state = MENU;
			break;
		}
		case SAVED : {
			if (radio->roll < -0.66f)
				state = SAVED_EXIT;
			break;
		}
		case SAVED_EXIT : {
			if (radio->roll > -0.33f)
				state = MENU;
			break;
		}
		case RUNCAM_MENU : {
			if (radio->pitch > 0.66f)
				state = RUNCAM_UP;
			else if (radio->pitch < -0.66f)
				state = RUNCAM_DOWN;
			else if (radio->roll < -0.66f)
				state = RUNCAM_LEFT;	
			else if (radio->roll > 0.66f)
				state = RUNCAM_RIGHT;
			else if (radio->yaw > 0.66f)
				state = RUNCAM_ENTER;
			else if (radio->yaw < -0.66f)
				state = RUNCAM_EXIT;
			break;
		}
		case RUNCAM_UP : {
			if (radio->pitch < 0.33f)
				state = RUNCAM_MENU;
			break;
		}
		case RUNCAM_DOWN : {
			if (radio->pitch > -0.33f)
				state = RUNCAM_MENU;
			break;
		}
		case RUNCAM_LEFT : {
			if (radio->roll > -0.33f)
				state = RUNCAM_MENU;
			break;
		}
		case RUNCAM_RIGHT : {
			if (radio->roll < 0.33f)
				state = RUNCAM_MENU;
			break;
		}
		case RUNCAM_ENTER : {
			if (radio->yaw < 0.33f)
				state = RUNCAM_MENU;
			break;
		}
		case RUNCAM_EXIT : {
			if (radio->yaw > -0.33f)
				state = MENU;
			break;
		}

		default : {
			break;
		}
	}

	// Menu index
	if (state_prev == MENU) {
		if (state == MENU_UP) {
			if (menu_idx == 0)
				menu_idx = sizeof(menu_str)/sizeof(menu_str[0])-1;
			else
				menu_idx--;
		} else if (state == MENU_DOWN) {
			if (menu_idx == sizeof(menu_str)/sizeof(menu_str[0])-1)
				menu_idx = 0;
			else
				menu_idx++;
		}
	}

	// Display menu
	if ((state_prev == TELEMETRY) && (state == MENU_ENTER)) {
		while (osd_busy) {}
		osd_write(MAX7456_DMAH, DISP_ADDR_MENU >> 8);
		osd_write(MAX7456_DMAL, DISP_ADDR_MENU & 0xFF);
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM);
		osd_write_str((uint8_t*)menu_str[menu_idx], sizeof(menu_str[0])-1); // -1: avoid 0 after last byte 255
	} else if ( ((state_prev == MENU) && ((state == MENU_UP) || (state == MENU_DOWN)))
		|| ((state_prev == REG) && (state == REG_EXIT))
		|| ((state_prev == SAVED) && (state == SAVED_EXIT))
		|| ((state_prev == RUNCAM_MENU) && (state == RUNCAM_EXIT)) ) {
		osd_write_str((uint8_t*)menu_str[menu_idx], sizeof(menu_str[0])-1); // -1: avoid 0 after last byte 255
	} else if ((state_prev == MENU) && (state == MENU_EXIT)) {
		while (osd_busy) {}
		osd_write(MAX7456_DMAH, disp_addr_telem >> 8);
		osd_write(MAX7456_DMAL, disp_addr_telem & 0xFF);
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM);
	}

	// Update register
	if ((state_prev == REG) && ((state == REG_UP) || (state == REG_DOWN))) {
		// Increment value
		if ((menu_idx == P_PITCH_ROLL) || (menu_idx == D_PITCH_ROLL) || (menu_idx == P_YAW)) // P and D
			incr_f = 0.05f;
		else if ((menu_idx == I_PITCH_ROLL) || (menu_idx == I_YAW)) // I
			incr_f = 0.001f;
		else // expo
			incr_f = 0.01f;
		if ((menu_idx == VTX_CHANNEL) || (menu_idx == VTX_POWER)) // VTX
			incr_int = 1;
		else
			incr_int = 5;

		// Increment direction
		if (state == REG_DOWN) {
			incr_f = -incr_f;
			incr_int = -incr_int;
		}
		
		// Write register
		switch (menu_idx) {
			case P_PITCH_ROLL : {
				REG_P_PITCH += incr_f;
				REG_P_ROLL = REG_P_PITCH;
				break;
			}
			case I_PITCH_ROLL : {
				REG_I_PITCH += incr_f;
				REG_I_ROLL = REG_I_PITCH;
				break;
			}
			case D_PITCH_ROLL : {
				REG_D_PITCH += incr_f;
				REG_D_ROLL = REG_D_PITCH;
				break;
			}
			case P_YAW : {
				REG_P_YAW += incr_f;
				break;
			}
			case I_YAW : {
				REG_I_YAW += incr_f;
				break;
			}
			case RATE : {
				rate = (int32_t)REG_RATE__PITCH_ROLL + incr_int;
				REG_RATE = (rate << REG_RATE__PITCH_ROLL_Pos) | (rate << REG_RATE__YAW_Pos);
				break;
			}
			case EXPO : {
				REG_EXPO_PITCH_ROLL += incr_f;
				REG_EXPO_YAW = REG_EXPO_PITCH_ROLL;
				break;
			}
			case MOTOR_START : {
				motor_start = (int32_t)REG_MOTOR__START + incr_int;
				REG_MOTOR &= ~REG_MOTOR__START_Msk;
				REG_MOTOR |= (uint32_t)motor_start << REG_MOTOR__START_Pos;
				break;
			}
			case MOTOR_ARMED : {
				motor_armed = (int32_t)REG_MOTOR__ARMED + incr_int;
				REG_MOTOR &= ~REG_MOTOR__ARMED_Msk;
				REG_MOTOR |= (uint32_t)motor_armed << REG_MOTOR__ARMED_Pos;
				break;
			}
			case MOTOR_RANGE : {
				motor_range = (int32_t)REG_MOTOR__RANGE + incr_int;
				REG_MOTOR &= ~REG_MOTOR__RANGE_Msk;
				REG_MOTOR |= (uint32_t)motor_range << REG_MOTOR__RANGE_Pos;
				break;
			}
			case VTX_CHANNEL : {
				if ((REG_VTX__CHAN == 47) && (incr_int > 0))
					vtx_chan = 0;
				else if ((REG_VTX__CHAN == 0) && (incr_int < 0))
					vtx_chan = 47;
				else
					vtx_chan = (int32_t)REG_VTX__CHAN + incr_int;
				REG_VTX &= ~REG_VTX__CHAN_Msk;
				REG_VTX |= (uint32_t)vtx_chan << REG_VTX__CHAN_Pos;
				break;
			}
			case VTX_POWER : {
				if ((REG_VTX__PWR == 3) && (incr_int > 0))
					vtx_pwr = 0;
				else if ((REG_VTX__PWR == 0) && (incr_int < 0))
					vtx_pwr = 3;
				else
					vtx_pwr = (int32_t)REG_VTX__PWR + incr_int;
				REG_VTX &= ~REG_VTX__PWR_Msk;
				REG_VTX |= (uint32_t)vtx_pwr << REG_VTX__PWR_Pos;
				break;
			}
			default : {
				break;
			}
		}
	}

	// Display register value
	if ( ((state_prev == REG) && ((state == REG_UP) || (state == REG_DOWN)))
		|| ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx < SAVE_REG)) ) {
		switch (menu_idx) {
			case P_PITCH_ROLL : {
				float_to_str(REG_P_PITCH, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case I_PITCH_ROLL : {
				float_to_str(REG_I_PITCH, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case D_PITCH_ROLL : {
				float_to_str(REG_D_PITCH, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case P_YAW : {
				float_to_str(REG_P_YAW, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case I_YAW : {
				float_to_str(REG_I_YAW, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case RATE : {
				float_to_str((float)REG_RATE__PITCH_ROLL, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case EXPO : {
				float_to_str(REG_EXPO_PITCH_ROLL, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case MOTOR_START : {
				float_to_str((float)REG_MOTOR__START, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case MOTOR_ARMED : {
				float_to_str((float)REG_MOTOR__ARMED, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case MOTOR_RANGE : {
				float_to_str((float)REG_MOTOR__RANGE, &reg_val_str[0], &reg_val_str[5], 4, 3);
				break;
			}
			case VTX_POWER : {
				if (REG_VTX__PWR == 0)
					vtx_pwr_mw = 25.0f;
				else if (REG_VTX__PWR == 1)
					vtx_pwr_mw = 200.0f;
				else if (REG_VTX__PWR == 2)
					vtx_pwr_mw = 500.0f;
				else
					vtx_pwr_mw = 800.0f;
				float_to_str(vtx_pwr_mw, &reg_val_str[0], &reg_val_str[5], 4, 3);
			}
			default : {
				break;
			}
		}
		if (menu_idx == VTX_CHANNEL)
			osd_write_str((uint8_t*)vtx_str[REG_VTX__CHAN], sizeof(vtx_str[0])-1); // -1: avoid 0 after last byte 255
		else
			osd_write_str((uint8_t*)reg_val_str, sizeof(reg_val_str)-1); // -1: avoid 0 after last byte 255
	}

	// Save register
	if ((state_prev == MENU) && (menu_idx == SAVE_REG) && (state == MENU_RIGHT)) {
		reg_save();
		osd_write_str((uint8_t*)saved_str, sizeof(saved_str)-1); // -1: avoid 0 after last byte 255
	}

#ifdef SMART_AUDIO
	// Apply VTX settings
	if ((state_prev == REG) && (menu_idx == VTX_CHANNEL) && (state == REG_EXIT)) {
		sma_send_cmd(SMA_SET_CHANNEL, REG_VTX__CHAN);
		while (sma_busy) {}
	}
	if ((state_prev == REG) && (menu_idx == VTX_POWER) && (state == REG_EXIT)) {
		sma_send_cmd(SMA_SET_POWER, REG_VTX__PWR);
		while (sma_busy) {}
	}
#endif

	// Runcam
#ifdef RUNCAM
	if ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx == RUNCAM)) {
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM);
		runcam_send_cmd(OPEN);
	} else if (state_prev == RUNCAM_MENU) {
		switch (state) {
			case RUNCAM_UP : {
				runcam_send_cmd(UP);
				break;
			}
			case RUNCAM_DOWN : {
				runcam_send_cmd(DOWN);
				break;
			}
			case RUNCAM_LEFT : {
				runcam_send_cmd(LEFT);
				break;
			}
			case RUNCAM_RIGHT : {
				runcam_send_cmd(RIGHT);
				break;
			}
			case RUNCAM_ENTER : {
				runcam_send_cmd(ENTER);
				break;
			}
			case RUNCAM_EXIT : {
				runcam_send_cmd(CLOSE);
				
				break;
			}
			default : {
				break;
			}
		}
	} else if ( ((state_prev == RUNCAM_UP) || (state_prev == RUNCAM_DOWN) || (state_prev == RUNCAM_LEFT) || (state_prev == RUNCAM_RIGHT) || (state_prev == RUNCAM_ENTER)) && (state == RUNCAM_MENU) ) {
		runcam_send_cmd(RELEASE);
	}
#endif

	state_prev = state;
}
