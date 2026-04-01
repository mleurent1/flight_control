#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memcpy()

#include "osd.h"
#include "max7456_reg.h"
#include "board.h" // osd_send()
#include "radio.h" // struct radio_s
#include "reg.h" // reg_save()
#include "utils.h" // crc8()
#include "smart_audio.h" // sma_send_cmd()
#include "msp.h" // msp_name()
#include "fc.h" // flag_msp_connected

/* Private defines --------------------------------------*/

#define DISP_ADDR_NTSC (10*30+1) // Last row index is 12
#define DISP_ADDR_PAL  (13*30+1) // Last row index is 15

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

enum state_e {TELEMETRY,
	MENU_ENTER, MENU, MENU_UP, MENU_DOWN, MENU_RIGHT, MENU_EXIT,
	REG, REG_UP, REG_DOWN, REG_EXIT,
#ifdef RUNCAM
	RUNCAM_MENU, RUNCAM_LEFT, RUNCAM_RIGHT, RUNCAM_UP, RUNCAM_DOWN, RUNCAM_ENTER, RUNCAM_EXIT,
#endif
	SAVED, SAVED_EXIT};

enum menu_e {P_PITCH_ROLL=0, I_PITCH_ROLL, D_PITCH_ROLL, P_YAW, I_YAW, RATE, EXPO, MOTOR_START, MOTOR_ARMED, MOTOR_RANGE,
#ifdef OSD
	VTX_CHANNEL, VTX_POWER,
#endif
#ifdef RUNCAM
	RUNCAM_IF,
#endif
	SAVE_REG};

enum runcam_cmd_e {LEFT, RIGHT, UP, DOWN, ENTER, RELEASE, OPEN, CLOSE};

/* Private variables -----------------------*/

#ifdef MSP_OSD
uint8_t telem_str[12] = "-000dBm-00dB";
#else
#ifdef IBAT
uint8_t telem_str[] = "00.00V 000.0A 0000mAh 00:00  -000dBm -00dB";
#else
uint8_t telem_str[] = "00.00V 00:00 -000dBm -00dB";
#endif
#endif

uint8_t menu_str[][12] = {
	"P PITCH ROLL",
	"I PITCH ROLL",
	"D PITCH ROLL",
	"P YAW       ",
	"I YAW       ",
	"RATE        ",
	"EXPO        ",
	"MOTOR START ",
	"MOTOR ARMED ",
	"MOTOR RANGE ",
#ifdef OSD
	"VTX CHANNEL ",
	"VTX POWER   ",
#endif
#ifdef RUNCAM
	"RUNCAM      ",
#endif
	"SAVE REG    "
};

uint8_t saved_str[12]   = "SAVED       ";
uint8_t reg_val_str[12] = "0000.000    ";

uint8_t vtx_str[][12] = {
	"BANDA 1 5865",
	"BANDA 2 5845",
	"BANDA 3 5825",
	"BANDA 4 5805",
	"BANDA 5 5785",
	"BANDA 6 5765",
	"BANDA 7 5745",
	"BANDA 8 5725",
	"BANDB 1 5733",
	"BANDB 2 5752",
	"BANDB 3 5771",
	"BANDB 4 5790",
	"BANDB 5 5809",
	"BANDB 6 5828",
	"BANDB 7 5847",
	"BANDB 8 5866",
	"BANDE 1 5705",
	"BANDE 2 5685",
	"BANDE 3 5665",
	"BANDE 4 5645",
	"BANDE 5 5885",
	"BANDE 6 5905",
	"BANDE 7 5925",
	"BANDE 8 5945",
	"AIRWV 1 5740",
	"AIRWV 2 5760",
	"AIRWV 3 5780",
	"AIRWV 4 5800",
	"AIRWV 5 5820",
	"AIRWV 6 5840",
	"AIRWV 7 5860",
	"AIRWV 8 5880",
	"RACE  1 5658",
	"RACE  2 5695",
	"RACE  3 5732",
	"RACE  4 5769",
	"RACE  5 5806",
	"RACE  6 5843",
	"RACE  7 5880",
	"RACE  8 5917",
	"LRACE 1 5621",
	"LRACE 2 5584",
	"LRACE 3 5547",
	"LRACE 4 5510",
	"LRACE 5 5473",
	"LRACE 6 5436",
	"LRACE 7 5399",
	"LRACE 8 5362",
};

// 16-bit strings
uint16_t menu_str_16[sizeof(menu_str)/sizeof(menu_str[0])][sizeof(menu_str[0])+1];
uint16_t saved_str_16[12+1];
uint16_t reg_val_str_16[12+1];
uint16_t telem_str_16[sizeof(telem_str)+1];
uint16_t vtx_str_16[sizeof(vtx_str)/sizeof(vtx_str[0])][sizeof(vtx_str[0])+1];

uint8_t osd_data_to_send[2];
uint8_t osd_data_received[sizeof(telem_str)];
uint8_t* osd_next_data_to_send;
uint8_t osd_next_nbytes_to_send = 0;
enum state_e state = TELEMETRY;
enum state_e state_prev = TELEMETRY;
uint8_t menu_idx = 0;

/* Private Functions -----------------------*/

void osd_write(uint8_t addr, uint8_t data)
{
	osd_data_to_send[0] = addr & 0x7F;
	osd_data_to_send[1] = data;
	osd_transfer(osd_data_to_send, osd_data_received, 2, 2);
	while (osd_busy) {}
}

uint8_t osd_read(uint8_t addr)
{
	osd_data_to_send[0] = 0x80 | (addr & 0x7F);
	osd_transfer(osd_data_to_send, osd_data_received, 2, 2);
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
	osd_transfer(osd_data_to_send, osd_data_received, 2, 2);
}

void float_to_str(float num, uint8_t* str_int, uint8_t* str_frac, uint8_t int_size, uint8_t frac_size)
{
	uint8_t dec_int[4], dec_frac[3];
	bool nonzero = false;
	uint8_t i,j;

	float_to_dec(num, dec_int, dec_frac);

	for (i=0; i<int_size; i++) {
		j = 4-int_size+i; // Take LSBs
		if (dec_int[j] > 0)
			nonzero = true;
		if ((dec_int[j] == 0) && !nonzero && (i < int_size-1))
			str_int[i] = ' ';
		else
			str_int[i] = dec_int[j] + 48;
	}
	for (i=0; i<frac_size; i++)
		str_frac[i] = dec_frac[i] + 48;
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
	runcam_transfer(buf, buf, len + 1, 0);
}

/* Public Functions -----------------------*/

void osd_init(void)
{
	uint8_t i,j;
	uint8_t r;

	// Convert 8-bit strings into 16-bit strings because MAX7456 burst mode needs 16-bit per character
	// Add 255 terminator to stop auto-increment mode
	for (i = 0; i < sizeof(saved_str); i++)
		saved_str_16[i] = saved_str[i];
	saved_str_16[i] = 255;

	for (i = 0; i < sizeof(reg_val_str); i++)
		reg_val_str_16[i] = reg_val_str[i];
	reg_val_str_16[sizeof(reg_val_str)] = 255;

	for (i = 0; i < sizeof(telem_str); i++)
		telem_str_16[i] = telem_str[i];
	telem_str_16[sizeof(telem_str)] = 255;

	for (i = 0; i < sizeof(menu_str)/sizeof(menu_str[0]); i++) {
		for (j = 0; j < sizeof(menu_str[0]); j++)
			menu_str_16[i][j] = menu_str[i][j];
		menu_str_16[i][j] = 255;
	}
	for (i = 0; i < sizeof(vtx_str)/sizeof(vtx_str[0]); i++) {
		for (j = 0; j < sizeof(vtx_str[0]); j++)
			vtx_str_16[i][j] = vtx_str[i][j];
		vtx_str_16[i][j] = 255;
	}

	r = osd_read(MAX7456_OSDBL);
	osd_write(MAX7456_OSDBL, r & ~MAX7456_OSDBL__AUTO_OSDBL_DISABLE);
	r = osd_read(MAX7456_STAT);
	if (r & MAX7456_STAT__PAL_DETECTED) {
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN | MAX7456_VM0__PAL_NOT_NTSC);
		osd_write(MAX7456_DMAH, DISP_ADDR_PAL >> 8);
		osd_write(MAX7456_DMAL, DISP_ADDR_PAL & 0xFF);
	}
	else {
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN);
		osd_write(MAX7456_DMAH, DISP_ADDR_NTSC >> 8);
		osd_write(MAX7456_DMAL, DISP_ADDR_NTSC & 0xFF);
	}
	osd_write(MAX7456_HOS, 40);
	osd_write(MAX7456_VOS, 22);
}

#ifdef MSP_OSD

void osd_telemetry(float vbat, float vbat_cell, float ibat, float imah, uint8_t t_s, uint8_t t_min,
	uint8_t rssi, int8_t snr, bool armed, bool acro)
{
#ifdef OSD_FLIGHT_MODE
	#define MSP_NB_MESSAGES 5
#else
	#define MSP_NB_MESSAGES 4
#endif
	static uint8_t cnt = 0;

	if ((state == TELEMETRY) && (REG_CTRL__MSP_HOST_CTRL == 0) && (!msp_busy) && flag_msp_connected) {
		if (cnt == MSP_NB_MESSAGES-1)
			cnt = 0;
		else
			cnt++;

		if (cnt == 0)
			msp_status(armed, acro);
		else if (cnt == 1)
			msp_analog(vbat, ibat, imah, rssi);
		else if (cnt == 2)
			msp_battery_state(vbat, ibat, imah);
		else if (cnt == 3) {
			float_to_str((float)rssi, &telem_str[1], &telem_str[1], 3, 0);
			if (snr >= 0) {
				float_to_str((float)snr, &telem_str[8], &telem_str[8], 2, 0);
				telem_str[7] = ' ';
			} else {
				float_to_str(-(float)snr, &telem_str[8], &telem_str[8], 2, 0);
			}
			msp_name(telem_str);
		}
		else
			msp_status_ex(armed, acro);
	}
}

#else

void osd_telemetry(float vbat, float vbat_cell, float ibat, float imah, uint8_t t_s, uint8_t t_min,
	uint8_t rssi, int8_t snr, bool armed, bool acro)
{
	if ((state == TELEMETRY) && (REG_CTRL__OSD_HOST_CTRL == 0) && (!osd_busy)) {
		float_to_str(vbat_cell, &telem_str[0], &telem_str[3], 2, 2);
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
		// Convert 8-bit strings into 16-bit strings because MAX7456 burst mode needs 16-bit per character
		for (uint8_t i = 0; i < sizeof(telem_str); i++)
			telem_str_16[i] = telem_str[i];
		osd_write_str((uint8_t*)telem_str_16, sizeof(telem_str_16)-1); // -1: avoid 0 after last byte 255
	}
}

#endif

void osd_menu(struct radio_s * radio)
{
	int32_t incr_int;
	float incr_f;
	int32_t rate;
	int32_t motor_start, motor_armed, motor_range;
#ifdef OSD
	int32_t vtx_chan, vtx_pwr;
	float vtx_pwr_mw;
#endif

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
#ifdef RUNCAM
				else if (menu_idx == RUNCAM_IF)
					state = RUNCAM_MENU;
#endif
				else // if (menu_idx == SAVE_REG)
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
#ifdef RUNCAM
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
#endif
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
	if ((state_prev == TELEMETRY) && (state == MENU_ENTER))
	{
#ifdef MSP_OSD
		while (msp_busy) {} // Wait for end of telemetry transaction
		msp_name(menu_str[menu_idx]);
#else
		while (osd_busy) {} // Wait for end of telemetry transaction
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM); // Clear display since telemetry string is longer than menu string
		osd_write_str((uint8_t*)menu_str_16[menu_idx], sizeof(menu_str_16[0])-1); // -1: avoid 0 after last byte 255
#endif
	}
	else if ( ((state_prev == MENU) && ((state == MENU_UP) || (state == MENU_DOWN)))
		|| ((state_prev == REG) && (state == REG_EXIT))
#ifdef RUNCAM
		|| ((state_prev == RUNCAM_MENU) && (state == RUNCAM_EXIT))
#endif
		|| ((state_prev == SAVED) && (state == SAVED_EXIT)) )
	{
#ifdef MSP_OSD
		msp_name(menu_str[menu_idx]);
#else		
		osd_write_str((uint8_t*)menu_str_16[menu_idx], sizeof(menu_str_16[0])-1); // -1: avoid 0 after last byte 255
#endif
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
		incr_int = 5;
#ifdef OSD
		if ((menu_idx == VTX_CHANNEL) || (menu_idx == VTX_POWER)) // VTX
			incr_int = 1;
#endif

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
				REG_RATE &= ~(REG_RATE__PITCH_ROLL_Msk	| REG_RATE__YAW_Msk);
				REG_RATE |= (rate << REG_RATE__PITCH_ROLL_Pos) | (rate << REG_RATE__YAW_Pos);
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
#ifdef OSD
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
#endif
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
#ifdef OSD
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
#endif
			default : {
				break;
			}
		}
#ifdef MSP_OSD
		msp_name(reg_val_str);
#else
		if (menu_idx == VTX_CHANNEL)
			osd_write_str((uint8_t*)vtx_str_16[REG_VTX__CHAN], sizeof(vtx_str_16[0])-1); // -1: avoid 0 after last byte 255
		else {
			// Convert 8-bit strings into 16-bit strings because MAX7456 burst mode needs 16-bit per character
			for (uint8_t i = 0; i < sizeof(reg_val_str); i++)
				reg_val_str_16[i] = reg_val_str[i];
			osd_write_str((uint8_t*)reg_val_str_16, sizeof(reg_val_str_16)-1); // -1: avoid 0 after last byte 255
		}
#endif
	}

	// Save register
	if ((state_prev == MENU) && (menu_idx == SAVE_REG) && (state == MENU_RIGHT)) {
		reg_save();
#ifdef MSP_OSD
		msp_name(saved_str);
#else
		osd_write_str((uint8_t*)saved_str_16, sizeof(saved_str_16)-1); // -1: avoid 0 after last byte 255
#endif
	}

#ifdef SMART_AUDIO
	// Apply VTX settings
	if ((state_prev == REG) && (menu_idx == VTX_CHANNEL) && (state == REG_EXIT)) {
		sma_send_cmd(SMA_SET_CHANNEL, REG_VTX__CHAN);
	}
	if ((state_prev == REG) && (menu_idx == VTX_POWER) && (state == REG_EXIT)) {
		sma_send_cmd(SMA_SET_POWER, REG_VTX__PWR);
	}
#endif

	// Runcam
#ifdef RUNCAM
	if ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx == RUNCAM_IF)) {
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
