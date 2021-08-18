#include <stdint.h>
#include "board.h" // sma_send()
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include "smart_audio.h" // sma_cmd_e()
#include <string.h> // memcpy()
#include "utils.h" // crc8()

/* Global variables -----------------------*/

volatile uint8_t sma_data_received[16];
volatile uint8_t sma_nbytes_to_receive = 0;
volatile uint8_t vtx_current_chan = 0;
volatile uint8_t vtx_current_pwr = 0;

/* Private Functions -----------------------*/

// Wait for end of Smart Audio transaction
void wait_sma(void)
{
	while (sma_nbytes_to_receive > 0)
		__WFI();
}

/* Public Functions -----------------------*/

void sma_send_cmd(enum sma_cmd_e sma_cmd, uint8_t data)
{
	uint8_t buf[5];
	uint8_t len = 0;

	buf[0] = 0x00; // line must be low before the frame
	buf[1] = 0xAA; // sync
	buf[2] = 0x55; // header

	switch (sma_cmd) {
		case SMA_GET_SETTINGS : {
			buf[3] = 0x03;
			buf[4] = 0x00;
			len = 5;
			sma_nbytes_to_receive = 11;
			break;
		}
		case SMA_SET_POWER : {
			buf[3] = 0x05;
			buf[4] = 0x01;
			buf[5] = data;
			len = 6;
			sma_nbytes_to_receive = 8;
			break;
		}
		case SMA_SET_CHANNEL : {
			buf[3] = 0x07;
			buf[4] = 0x01;
			buf[5] = data;;
			len = 6;
			sma_nbytes_to_receive = 8;
			break;
		}
	}

	buf[len] = crc8(buf, len);

	sma_send(buf, len + 1);
}

void sma_process_resp(void)
{
	uint8_t cmd, len, crc_calc;

	wait_sma();

	cmd = sma_data_received[3];
	len = sma_data_received[4] + 5;

	if ((len > 11) || (len < 8))
		return;
	
	crc_calc = crc8((uint8_t *)&sma_data_received[3], len-4);

	if (sma_data_received[len-1] != crc_calc)
		return;

	switch (cmd) {
		case 0x09 : {
			vtx_current_chan = sma_data_received[5];
			vtx_current_pwr = sma_data_received[6];
			break;
		}
		case 0x02 : {
			vtx_current_pwr = sma_data_received[5];
			break;
		}
		case 0x03 : {
			vtx_current_chan = sma_data_received[5];
			break;
		}
	}
}

void __attribute__((weak)) sma_send(uint8_t * data, uint8_t size) {}