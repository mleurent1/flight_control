#ifdef OSD

#include <stdint.h>
#include "osd.h"
#include "max7456_reg.h"
#include "board.h" // osd_send()
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include "radio.h" // struct radio_s
#include <string.h> // memcpy()
#include "reg.h" // reg_save()
#include <math.h> // pow()
#include "utils.h" // crc8()

/* Private defines --------------------------------------*/

#define DISP_ADDR_MENU (uint8_t)(9*30+5)
#define DISP_ADDR_TELEMETRY (uint8_t)(13*30+4)

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

enum state_e {TELEMETRY, TELEMETRY_ENTER,
	MENU, MENU_UP, MENU_DOWN, MENU_RIGHT, MENU_EXIT,
	REG, REG_UP, REG_DOWN, REG_LEFT,
	RUNCAM_MENU, RUNCAM_LEFT, RUNCAM_RIGHT, RUNCAM_UP, RUNCAM_DOWN, RUNCAM_ENTER, RUNCAM_EXIT};

enum runcam_cmd_e {LEFT, RIGHT, UP, DOWN, ENTER, RELEASE, OPEN, CLOSE};

/* Private variables -----------------------*/

const uint8_t menu_str[13][13] = {
	{0xFF, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x1A}, // P PITCH ROLL
	{0xFF, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x13}, // I PITCH ROLL
	{0xFF, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x0E}, // D PITCH ROLL
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0B, 0x23, 0x00, 0x1A}, // P YAW
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0B, 0x23, 0x00, 0x13}, // I YAW
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1E, 0x0B, 0x1C}, // RATE
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x1A, 0x22, 0x0F}, // EXPO
	{0xFF, 0x00, 0x1E, 0x1C, 0x0B, 0x1E, 0x1D, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR START
	{0xFF, 0x00, 0x0E, 0x0F, 0x17, 0x1C, 0x0B, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR ARMED
	{0xFF, 0x00, 0x0F, 0x11, 0x18, 0x0B, 0x1C, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR RANGE
	{0xFF, 0x00, 0x00, 0x1C, 0x0F, 0x10, 0x1D, 0x18, 0x0B, 0x1C, 0x1E, 0x00, 0x13}, // I TRANSFER
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x1D}, // SAVE
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x0B, 0x0D, 0x18, 0x1F, 0x1C}  // RUNCAM
};
const uint8_t saved_str[6] = {0xFF, 0x0E, 0x0F, 0x20, 0x0B, 0x1D}; // SAVED

/* Global variables -----------------------*/

volatile uint8_t osd_data_to_send[22];
volatile uint8_t osd_nbytes_to_send = 0;
volatile uint8_t osd_data_received[2];
volatile uint8_t osd_nbytes_to_receive = 0;
volatile enum state_e state = TELEMETRY;

/* Private Functions -----------------------*/

// Wait for end of OSD transaction
void wait_osd(void)
{
	while ((osd_nbytes_to_send > 0) || (osd_nbytes_to_receive > 0))
		__WFI();
}

void osd_write(uint8_t addr, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = addr & 0x7F;
	buf[1] = data;
	osd_send(buf,2);
	while (osd_nbytes_to_receive > 0)
		__WFI();
}

uint8_t osd_read(uint8_t addr)
{
	uint8_t buf[2];
	buf[0] = 0x80 | (addr & 0x7F);
	osd_send(buf,2);
	while (osd_nbytes_to_receive > 0)
		__WFI();
	return osd_data_received[0];
}

void osd_write_str(uint8_t * str, uint8_t size)
{
	uint8_t buf[2];
	memcpy((uint8_t*)osd_data_to_send, str, size);
	osd_nbytes_to_send = size;
	// Enable auto-increment
	buf[0] = MAX7456_DMM;
	buf[1] = MAX7456_DMM__AUTO_INCR_EN;
	osd_send(buf,2);
}

void float_to_str(float num, uint8_t * str_int, uint8_t * str_frac)
{
	float b10_mult[3] = {1000.0,100.0,10.0};
	float b10_div[3] = {0.001,0.01,0.1};
	_Bool first_nonzero = 0;
	float q;
	int i;

	for (i=0; i<3; i++) {
		q = floor(num * b10_div[i]);
		if (q > 0)
			first_nonzero = 1;
		if ((q == 0) && (first_nonzero || (i == 3)))
			str_int[3-i] = 10;
		else
			str_int[3-i] = (uint8_t)q;
		num = num - q * b10_mult[i];
	}

	q = floor(num);
	if (q == 0)
		str_int[0] = 10;
	else
		str_int[0] = (uint8_t)q;
	num = num - q;

	for (i=0; i<3; i++) {
		q = floor(num * b10_mult[2-i]);
		if (q == 0)
			str_frac[2-i] = 10;
		else
			str_frac[2-i] = (uint8_t)q;
		num = num - q * b10_div[2-i];
	}
}

void float_to_str_short(float num, uint8_t * str_int, uint8_t * str_frac)
{
	float q;

	q = floor(num * 0.01f);
	str_int[2] = (uint8_t)q;
	num = num - q * 100.0f;

	q = floor(num * 0.1f);
	if ((q == 0) && (str_int[2] > 0))
		str_int[1] = 10;
	else
		str_int[1] = (uint8_t)q;
	num = num - q * 10.0f;

	q = floor(num);
	if (q == 0)
		str_int[0] = 10;
	else
		str_int[0] = (uint8_t)q;
	num = num - q;

	q = floor(num * 10.0f);
	if (q == 0)
		str_frac[0] = 10;
	else
		str_frac[0] = (uint8_t)q;
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
	if (r & MAX7456_STAT__PAL_DETECTED)
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN | MAX7456_VM0__PAL_NOT_NTSC);
	else
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN);
	//osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN | MAX7456_VM0__PAL_NOT_NTSC);
	//osd_write(MAX7456_HOS, 45); // 40
	//osd_write(MAX7456_VOS, 28); // 22
	osd_write(MAX7456_DMAH, MAX7456_DMAH__DMA_8);
	osd_write(MAX7456_DMAL, (13*30+4) & 0xFF);
}

void osd_telemetry(float vbat, float ibat, float imah)
{
	uint8_t str[22] = {0xFF,44,11,49,0,0,0,0, 0, 11,0,65,0,0,0, 0, 32,0,65,0,0,0};

	if ((state == TELEMETRY) && (osd_nbytes_to_send == 0) && (osd_nbytes_to_receive == 0)) {
		float_to_str_short(vbat, &str[19], &str[17]);
		float_to_str_short(ibat, &str[12], &str[10]);
		float_to_str_short(imah*0.1f, &str[5], &str[4]);
		osd_write_str(str, sizeof(str));
	}
}

void osd_menu(struct radio_s * radio)
{
	static enum state_e state_prev = TELEMETRY;
	static uint8_t menu_idx = 0;
	uint32_t incr_int;
	float incr_f;
	uint32_t rate;
	uint32_t motor_start, motor_armed, motor_range;
	uint8_t reg_val_str[13] = {0xFF,0,0,0,0,0,0,0,65,0,0,0,0};

	// State machine
	switch (state_prev) {
		case TELEMETRY : {
			if (radio->yaw > 0.66f)
				state = TELEMETRY_ENTER;
			break;
		}
		case TELEMETRY_ENTER : {
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
				if (menu_idx < 11)
					state = REG;
				else if (menu_idx == 12)
					state = RUNCAM_MENU;
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
				state = REG_LEFT;
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
		case REG_LEFT : {
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
	if ((state_prev == TELEMETRY) && (state == TELEMETRY_ENTER)) {
		wait_osd();
		osd_write(MAX7456_DMAL, DISP_ADDR_MENU);
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM);
		osd_write_str((uint8_t*)menu_str[menu_idx], sizeof(menu_str[0]));
	} else if ( ((state_prev == MENU) && ((state == MENU_UP) || (state == MENU_DOWN))) || ((state_prev == REG) && (state == REG_LEFT)) || ((state_prev == RUNCAM_MENU) && (state == RUNCAM_EXIT)) ) {
		osd_write_str((uint8_t*)menu_str[menu_idx], sizeof(menu_str[0]));
	} else if ((state_prev == MENU) && (state == MENU_EXIT)) {
		osd_write(MAX7456_DMAL, DISP_ADDR_TELEMETRY);
		osd_write(MAX7456_DMM, MAX7456_DMM__CLR_DISPLAY_MEM); 
	}

	// Update register
	if ((state_prev == REG) && ((state == REG_UP) || (state == REG_DOWN))) {
		// Increment value
		if ((menu_idx == 0) || (menu_idx == 2) || (menu_idx == 3)) // P and D
			incr_f = 0.05f;
		else if ((menu_idx == 1) || (menu_idx == 4)) // I
			incr_f = 0.001f;
		else // expo
			incr_f = 0.01f;
		incr_int = 5;

		// Increment direction
		if (state == REG_DOWN) {
			incr_f = -incr_f;
			incr_int = -incr_int;
		}
		
		// Write register
		switch (menu_idx) {
			case 0 : {
				REG_P_PITCH += incr_f;
				REG_P_ROLL = REG_P_PITCH;
				break;
			}
			case 1 : {
				REG_I_PITCH += incr_f;
				REG_I_ROLL = REG_I_PITCH;
				break;
			}
			case 2 : {
				REG_D_PITCH += incr_f;
				REG_D_ROLL = REG_D_PITCH;
				break;
			}
			case 3 : {
				REG_P_YAW += incr_f;
				break;
			}
			case 4 : {
				REG_I_YAW += incr_f;
				break;
			}
			case 5 : {
				rate = (uint32_t)REG_RATE__PITCH_ROLL + incr_int;
				REG_RATE = (rate << REG_RATE__PITCH_ROLL_Pos) | (rate << REG_RATE__YAW_Pos);
				break;
			}
			case 6 : {
				REG_EXPO_PITCH_ROLL += incr_f;
				REG_EXPO_YAW = REG_EXPO_PITCH_ROLL;
				break;
			}
			case 7 : {
				motor_start = (uint32_t)REG_MOTOR__START + incr_int;
				REG_MOTOR &= ~REG_MOTOR__START_Msk;
				REG_MOTOR |= motor_start << REG_MOTOR__START_Pos;
				break;
			}
			case 8 : {
				motor_armed = (uint32_t)REG_MOTOR__ARMED + incr_int;
				REG_MOTOR &= ~REG_MOTOR__ARMED_Msk;
				REG_MOTOR |= motor_armed << REG_MOTOR__ARMED_Pos;
				break;
			}
			case 9 : {
				motor_range = (uint32_t)REG_MOTOR__RANGE + incr_int;
				REG_MOTOR &= ~REG_MOTOR__RANGE_Msk;
				REG_MOTOR |= motor_range << REG_MOTOR__RANGE_Pos;
				break;
			}
			case 10 : {
				if (state == REG_UP)
					REG_I_TRANSFER = 1;
				else if (state == REG_DOWN)
					REG_I_TRANSFER = 0;
				break;
			}
			default : {
				break;
			}
		}
	}

	// Display register value
	if ( ((state_prev == REG) && ((state == REG_UP) || (state == REG_DOWN))) || ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx < 11)) ) {
		switch (menu_idx) {
			case 0 : {
				float_to_str(REG_P_PITCH, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 1 : {
				float_to_str(REG_I_PITCH, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 2 : {
				float_to_str(REG_D_PITCH, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 3 : {
				float_to_str(REG_P_YAW, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 4 : {
				float_to_str(REG_I_YAW, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 5 : {
				float_to_str((float)REG_RATE__PITCH_ROLL, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 6 : {
				float_to_str(REG_EXPO_PITCH_ROLL, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 7 : {
				float_to_str((float)REG_MOTOR__START, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 8 : {
				float_to_str((float)REG_MOTOR__ARMED, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 9 : {
				float_to_str((float)REG_MOTOR__RANGE, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			case 10 : {
				float_to_str((float)REG_I_TRANSFER, &reg_val_str[9], &reg_val_str[5]);
				break;
			}
			default : {
				break;
			}
		}
		osd_write_str(reg_val_str, sizeof(reg_val_str));
	}

	// Save register
	if ((state_prev == MENU) && (menu_idx == 11) && (state == MENU_RIGHT)) {
		reg_save();
		osd_write_str((uint8_t*)saved_str, sizeof(saved_str));
	}

	// Runcam
	if ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx == 12)) {
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

	state_prev = state;
}

#endif
