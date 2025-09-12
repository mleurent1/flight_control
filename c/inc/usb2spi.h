#ifndef __USB2SPI_H
#define __USB2SPI_H

#include <stdint.h>
#include <stdbool.h>

#include "usbd_cdc.h" // CDC_DATA_FS_MAX_PACKET_SIZE

/* Public defines -----------------*/

#define BUF_SIZE 512

/* Public types -----------------*/

/* Exported variables -----------------*/

extern volatile uint16_t host_bytes_avail;
extern volatile uint16_t spi_bytes_avail;
extern volatile bool flag_time;

extern volatile uint8_t host_rx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
extern volatile uint8_t host_tx_buffer[BUF_SIZE];
extern volatile uint8_t rx_buffer[BUF_SIZE];

extern volatile uint32_t t_ms;

/* Public functions -----------------*/

#endif
