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
#include "utils.h" // crc8()
#include "smart_audio.h" // sma_send_cmd()

/* Private defines --------------------------------------*/

#define DISP_ADDR_MENU (uint8_t)(9*30+5)
#define DISP_ADDR_TELEMETRY (uint8_t)(13*30+2)

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

enum state_e {TELEMETRY, TELEMETRY_ENTER,
	MENU, MENU_UP, MENU_DOWN, MENU_RIGHT, MENU_EXIT,
	REG, REG_UP, REG_DOWN, REG_LEFT,
	RUNCAM_MENU, RUNCAM_LEFT, RUNCAM_RIGHT, RUNCAM_UP, RUNCAM_DOWN, RUNCAM_ENTER, RUNCAM_EXIT};

enum runcam_cmd_e {LEFT, RIGHT, UP, DOWN, ENTER, RELEASE, OPEN, CLOSE};

/* Private variables -----------------------*/

const uint8_t menu_str[16][16] = {
	{0xFF, 0x00, 0x00, 0x00, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x1A}, // P PITCH ROLL
	{0xFF, 0x00, 0x00, 0x00, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x13}, // I PITCH ROLL
	{0xFF, 0x00, 0x00, 0x00, 0x16, 0x16, 0x19, 0x1C, 0x00, 0x12, 0x0D, 0x1E, 0x13, 0x1A, 0x00, 0x0E}, // D PITCH ROLL
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0B, 0x23, 0x00, 0x1A}, // P YAW
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0B, 0x23, 0x00, 0x13}, // I YAW
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1E, 0x0B, 0x1C}, // RATE
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x1A, 0x22, 0x0F}, // EXPO
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1C, 0x0B, 0x1E, 0x1D, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR START
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x0F, 0x17, 0x1C, 0x0B, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR ARMED
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x11, 0x18, 0x0B, 0x1C, 0x00, 0x1C, 0x19, 0x1E, 0x19, 0x17}, // MOTOR RANGE
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x0F, 0x10, 0x1D, 0x18, 0x0B, 0x1C, 0x1E, 0x00, 0x13}, // I TRANSFER
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x16, 0x0F, 0x18, 0x18, 0x0B, 0x12, 0x0D, 0x00, 0x22, 0x1E, 0x20}, // VTX CHANNEL
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x0F, 0x21, 0x19, 0x1A, 0x00, 0x22, 0x1E, 0x20}, // VTX POWER
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x0F, 0x1C, 0x00, 0x0F, 0x20, 0x0B, 0x1D}, // SAVE REG
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x1E, 0x20, 0x00, 0x0F, 0x20, 0x0B, 0x1D}, // SAVE VTX
	{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x0B, 0x0D, 0x18, 0x1F, 0x1C}  // RUNCAM
};
const uint8_t vtx_str[48][16] = {
	{0xFF, 0x05, 0x06, 0x08, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   1 5865
	{0xFF, 0x05, 0x04, 0x08, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   2 5845
	{0xFF, 0x05, 0x02, 0x08, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   3 5825
	{0xFF, 0x05, 0x0A, 0x08, 0x05, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   4 5805
	{0xFF, 0x05, 0x08, 0x07, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   5 5785
	{0xFF, 0x05, 0x06, 0x07, 0x05, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   6 5765
	{0xFF, 0x05, 0x04, 0x07, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   7 5745
	{0xFF, 0x05, 0x02, 0x07, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND A   8 5725
	{0xFF, 0x03, 0x03, 0x07, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   1 5733
	{0xFF, 0x02, 0x05, 0x07, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   2 5752
	{0xFF, 0x01, 0x07, 0x07, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   3 5771
	{0xFF, 0x0A, 0x09, 0x07, 0x05, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   4 5790
	{0xFF, 0x09, 0x0A, 0x08, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   5 5809
	{0xFF, 0x08, 0x02, 0x08, 0x05, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   6 5828
	{0xFF, 0x07, 0x04, 0x08, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   7 5847
	{0xFF, 0x06, 0x06, 0x08, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND B   8 5866
	{0xFF, 0x05, 0x0A, 0x07, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   1 5705
	{0xFF, 0x05, 0x08, 0x06, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   2 5685
	{0xFF, 0x05, 0x06, 0x06, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   3 5665
	{0xFF, 0x05, 0x04, 0x06, 0x05, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   4 5645
	{0xFF, 0x05, 0x08, 0x08, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   5 5885
	{0xFF, 0x05, 0x0A, 0x09, 0x05, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   6 5905
	{0xFF, 0x05, 0x02, 0x09, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   7 5925
	{0xFF, 0x05, 0x04, 0x09, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x0E, 0x18, 0x0B, 0x0C}, // BAND E   8 5945
	{0xFF, 0x0A, 0x04, 0x07, 0x05, 0x00, 0x01, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  1 5740
	{0xFF, 0x0A, 0x06, 0x07, 0x05, 0x00, 0x02, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  2 5760
	{0xFF, 0x0A, 0x08, 0x07, 0x05, 0x00, 0x03, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  3 5780
	{0xFF, 0x0A, 0x0A, 0x08, 0x05, 0x00, 0x04, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  4 5800
	{0xFF, 0x0A, 0x02, 0x08, 0x05, 0x00, 0x05, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  5 5820
	{0xFF, 0x0A, 0x04, 0x08, 0x05, 0x00, 0x06, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  6 5840
	{0xFF, 0x0A, 0x06, 0x08, 0x05, 0x00, 0x07, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  7 5860
	{0xFF, 0x0A, 0x08, 0x08, 0x05, 0x00, 0x08, 0x00, 0x00, 0x0F, 0x20, 0x0B, 0x21, 0x1C, 0x13, 0x0B}, // AIRWAVE  8 5880
	{0xFF, 0x08, 0x05, 0x06, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     1 5658
	{0xFF, 0x05, 0x09, 0x06, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     2 5695
	{0xFF, 0x02, 0x03, 0x07, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     3 5732
	{0xFF, 0x09, 0x06, 0x07, 0x05, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     4 5769
	{0xFF, 0x06, 0x0A, 0x08, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     5 5806
	{0xFF, 0x03, 0x04, 0x08, 0x05, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     6 5843
	{0xFF, 0x0A, 0x08, 0x08, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     7 5880
	{0xFF, 0x07, 0x01, 0x09, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0D, 0x0B, 0x1C}, // RACE     8 5917
	{0xFF, 0x01, 0x02, 0x06, 0x05, 0x00, 0x01, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 1 5621
	{0xFF, 0x04, 0x08, 0x05, 0x05, 0x00, 0x02, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 2 5584
	{0xFF, 0x07, 0x04, 0x05, 0x05, 0x00, 0x03, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 3 5547
	{0xFF, 0x0A, 0x01, 0x05, 0x05, 0x00, 0x04, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 4 5510
	{0xFF, 0x03, 0x07, 0x04, 0x05, 0x00, 0x05, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 5 5473
	{0xFF, 0x06, 0x03, 0x04, 0x05, 0x00, 0x06, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 6 5436
	{0xFF, 0x09, 0x09, 0x03, 0x05, 0x00, 0x07, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}, // LOW RACE 7 5399
	{0xFF, 0x02, 0x06, 0x03, 0x05, 0x00, 0x08, 0x00, 0x0F, 0x0D, 0x0B, 0x1C, 0x00, 0x21, 0x19, 0x16}  // LOW RACE 8 5362
};
const uint8_t saved_str[16] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x0E, 0x0F, 0x20, 0x0B, 0x1D}; // SAVED

/* Global variables -----------------------*/

volatile uint8_t osd_data_to_send[30]; // 30 = nb of char in 1 line
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

void float_to_str(float num, uint8_t * str_int, uint8_t * str_frac, uint8_t int_size, uint8_t frac_size)
{
	uint8_t dec_frac[3];
	_Bool nonzero = 0;
	int i;

	float_to_dec(num, str_int, dec_frac, int_size, frac_size);

	for (i=int_size-1; i>=0; i--) {
		if (str_int[i] > 0)
			nonzero = 1;
		if ((str_int[i] == 0) && (nonzero || (i == 0)))
			str_int[i] = 10;
	}
	for (i=0; i<frac_size; i++) {
		if (dec_frac[i] == 0)
			str_frac[frac_size-i-1] = 10;
		else
			str_frac[frac_size-i-1] = dec_frac[i];
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
	if (r & MAX7456_STAT__PAL_DETECTED)
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN | MAX7456_VM0__PAL_NOT_NTSC);
	else
		osd_write(MAX7456_VM0, MAX7456_VM0__OSD_EN);
	//osd_write(MAX7456_HOS, 45); // 40
	//osd_write(MAX7456_VOS, 28); // 22
	osd_write(MAX7456_DMAH, MAX7456_DMAH__DMA_8);
	osd_write(MAX7456_DMAL, DISP_ADDR_TELEMETRY);
}

void osd_telemetry(float vbat, float ibat, float imah, uint8_t t_s, uint8_t t_min)
{
	#ifdef IBAT
		uint8_t str[28] = {0xFF, 0,0,68,0,0, 0, 44,11,49,0,0,0,0, 0, 11,0,65,0,0,0, 0, 32,0,0,65,0,0};
	#else
		uint8_t str[28] = {0xFF, 0,0,68,0,0, 0,  0, 0, 0,0,0,0,0, 0,  0,0, 0,0,0,0, 0, 32,0,0,65,0,0};
	#endif

	if ((state == TELEMETRY) && (osd_nbytes_to_send == 0) && (osd_nbytes_to_receive == 0)) {
		float_to_str(vbat, &str[26], &str[23], 2, 2);
		#ifdef IBAT
			float_to_str(ibat, &str[18], &str[16], 3, 1);
			float_to_str(imah, &str[10], &str[10], 4, 0);
		#endif
		float_to_str((float)t_min, &str[4], &str[4], 2, 0);
		float_to_str((float)t_s, &str[1], &str[1], 2, 0);
		osd_write_str(str, sizeof(str));
	}
}

void osd_menu(struct radio_s * radio)
{
	static enum state_e state_prev = TELEMETRY;
	static uint8_t menu_idx = 0;
	int32_t incr_int;
	float incr_f;
	int32_t rate;
	int32_t motor_start, motor_armed, motor_range;
	int32_t vtx_chan, vtx_pwr;
	float vtx_pwr_mw;
	uint8_t reg_val_str[16] = {0xFF,0,0,0,0,0,0,0,0,0,0,65,0,0,0,0};

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
				if (menu_idx < 13)
					state = REG;
				else if (menu_idx == 15)
					state = RUNCAM_MENU;
				else
					state = MENU;
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
		REG_VTX = (vtx_current_chan << REG_VTX__CHAN_Pos) | (vtx_current_pwr << REG_VTX__PWR_Pos); // Update VTX register at exit
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
		if ((menu_idx == 11) || (menu_idx == 12)) // VTX
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
				rate = (int32_t)REG_RATE__PITCH_ROLL + incr_int;
				REG_RATE = (rate << REG_RATE__PITCH_ROLL_Pos) | (rate << REG_RATE__YAW_Pos);
				break;
			}
			case 6 : {
				REG_EXPO_PITCH_ROLL += incr_f;
				REG_EXPO_YAW = REG_EXPO_PITCH_ROLL;
				break;
			}
			case 7 : {
				motor_start = (int32_t)REG_MOTOR__START + incr_int;
				REG_MOTOR &= ~REG_MOTOR__START_Msk;
				REG_MOTOR |= (uint32_t)motor_start << REG_MOTOR__START_Pos;
				break;
			}
			case 8 : {
				motor_armed = (int32_t)REG_MOTOR__ARMED + incr_int;
				REG_MOTOR &= ~REG_MOTOR__ARMED_Msk;
				REG_MOTOR |= (uint32_t)motor_armed << REG_MOTOR__ARMED_Pos;
				break;
			}
			case 9 : {
				motor_range = (int32_t)REG_MOTOR__RANGE + incr_int;
				REG_MOTOR &= ~REG_MOTOR__RANGE_Msk;
				REG_MOTOR |= (uint32_t)motor_range << REG_MOTOR__RANGE_Pos;
				break;
			}
			case 10 : {
				if (state == REG_UP)
					REG_I_TRANSFER = 1;
				else if (state == REG_DOWN)
					REG_I_TRANSFER = 0;
				break;
			}
			case 11 : {
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
			case 12 : {
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
	if ( ((state_prev == REG) && ((state == REG_UP) || (state == REG_DOWN))) || ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx < 13)) ) {
		switch (menu_idx) {
			case 0 : {
				float_to_str(REG_P_PITCH, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 1 : {
				float_to_str(REG_I_PITCH, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 2 : {
				float_to_str(REG_D_PITCH, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 3 : {
				float_to_str(REG_P_YAW, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 4 : {
				float_to_str(REG_I_YAW, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 5 : {
				float_to_str((float)REG_RATE__PITCH_ROLL, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 6 : {
				float_to_str(REG_EXPO_PITCH_ROLL, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 7 : {
				float_to_str((float)REG_MOTOR__START, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 8 : {
				float_to_str((float)REG_MOTOR__ARMED, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 9 : {
				float_to_str((float)REG_MOTOR__RANGE, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 10 : {
				float_to_str((float)REG_I_TRANSFER, &reg_val_str[12], &reg_val_str[8], 4, 3);
				break;
			}
			case 11 : {
				osd_write_str((uint8_t*)vtx_str[REG_VTX__CHAN], sizeof(vtx_str[0]));
				break;
			}
			case 12 : {
				if (REG_VTX__PWR == 0)
					vtx_pwr_mw = 25.0f;
				else if (REG_VTX__PWR == 1)
					vtx_pwr_mw = 200.0f;
				else if (REG_VTX__PWR == 2)
					vtx_pwr_mw = 500.0f;
				else
					vtx_pwr_mw = 800.0f;
				float_to_str(vtx_pwr_mw, &reg_val_str[12], &reg_val_str[8], 4, 3);
			}
			default : {
				break;
			}
		}
		if (menu_idx != 11)
			osd_write_str(reg_val_str, sizeof(reg_val_str));
	}

	// Save register
	if ((state_prev == MENU) && (menu_idx == 13) && (state == MENU_RIGHT)) {
		reg_save();
		osd_write_str((uint8_t*)saved_str, sizeof(saved_str));
	}

	// Apply VTX settings
	if ((state_prev == MENU) && (menu_idx == 14) && (state == MENU_RIGHT)) {
		sma_send_cmd(SMA_SET_CHANNEL, REG_VTX__CHAN);
		wait_sma();
		sma_send_cmd(SMA_SET_POWER, REG_VTX__PWR);
		wait_sma();
		vtx_current_chan = REG_VTX__CHAN;
		vtx_current_pwr = REG_VTX__PWR;
		osd_write_str((uint8_t*)saved_str, sizeof(saved_str));
	}

	// Runcam
	if ((state_prev == MENU) && (state == MENU_RIGHT) && (menu_idx == 15)) {
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

void __attribute__((weak)) osd_send(uint8_t * data, uint8_t size) {}
void __attribute__((weak)) runcam_send(uint8_t * data, uint8_t size) {}