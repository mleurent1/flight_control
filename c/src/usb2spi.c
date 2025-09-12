#include <string.h> // memcpy()
#include <stdbool.h>

#include "usb2spi.h"
#include "board.h" // board_init()
#include "usbd_cdc.h" // CDC_DATA_FS_MAX_PACKET_SIZE

/* Private defines ------------------------------------*/

#define STATUS_PERIOD 500 // ms

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint16_t host_bytes_avail = 0;
volatile uint16_t spi_bytes_avail = 0;
volatile bool flag_time = 0;

volatile uint8_t host_rx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
volatile uint8_t host_tx_buffer[BUF_SIZE];
volatile uint8_t rx_buffer[BUF_SIZE];

volatile uint32_t t_ms;

/* Private functions ------------------------------------------------*/

extern void spi_dma_enable(uint8_t size);

/* MAIN ----------------------------------------------------------------
-----------------------------------------------------------------------*/

int main(void)
{
	uint32_t t_status_prev = 0;
	bool first_host_rx = true;
	uint16_t rx_size, rx_buffer_idx;
	uint16_t bytes_avail;

	/* Init -----------------------------------------------------*/

	board_init();

	/* Loop ----------------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/

	while (1)
	{
		/* Host request ---------------------------------------------------------------*/

		if (host_bytes_avail)
		{
			// Clear flag ASAP
			bytes_avail = host_bytes_avail;
			host_bytes_avail = 0;

			if (first_host_rx == true) {
				first_host_rx = false;
				rx_buffer_idx = 0;
				rx_size = ((uint16_t)host_rx_buffer[0] << 8) | (uint16_t)host_rx_buffer[1];
				bytes_avail -= 2;
				memcpy(rx_buffer, &host_rx_buffer[2], bytes_avail);
			} else {
				memcpy(&rx_buffer[rx_buffer_idx], host_rx_buffer, bytes_avail);
			}

			rx_buffer_idx += bytes_avail;

			if (rx_buffer_idx == rx_size) {
				if (rx_buffer[0] == 0xFF) {
					set_dios(rx_buffer[1]);
				} else if (rx_buffer[0] == 0xFE) {
					host_tx_buffer[0] = get_dios();
					host_send(host_tx_buffer, 1);
				} else {
					spi_dma_enable(rx_size);
				}
				first_host_rx = true;
				toggle_led2(true);
			}
		}

		/* SPI response ---------------------------------------------------------------*/

		if (spi_bytes_avail)
		{
			// Clear flag ASAP
			bytes_avail = spi_bytes_avail;
			spi_bytes_avail = 0;

			if (bytes_avail == CDC_DATA_FS_MAX_PACKET_SIZE)
				host_send(host_tx_buffer, bytes_avail+1);
			else
				host_send(host_tx_buffer, bytes_avail);
		}

		/* Status ----------------------------------------------------------------------*/

		if (flag_time)
		{
			flag_time = 0; // Clear flag

			if ((t_ms - t_status_prev) >= STATUS_PERIOD) {
				t_status_prev = t_ms;
				toggle_led();
				toggle_led2(false);
			}
		}
	}
}
