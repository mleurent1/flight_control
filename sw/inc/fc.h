#include <stdint.h>
#include "usbd_def.h"

#ifndef __FC_H
#define __FC_H

/* Public defines -----------------*/

#define FLAG__MPU 0x01
#define FLAG__RADIO 0x02
#define FLAG__MOTOR 0x04
#define FLAG__HOST 0x08
#define FLAG__REG 0x10
#define FLAG__MPU_HOST_READ 0x20
#define FLAG__RADIO_SYNCH 0x40
#define FLAG__MPU_CAL 0x80
#define FLAG__VBAT 0x100
#define FLAG__ERROR_MPU 0x200
#define FLAG__ERROR_RADIO 0x400

/* Public types -----------------*/

typedef struct {
	uint8_t instr;
	uint8_t addr;
	union { 
		uint8_t u8[4];
		uint16_t u16[2];
		uint32_t u32;
		float f;
	} data;
} __attribute__ ((__packed__)) usb_buffer_rx_t;

/* Public variables -----------------*/

extern volatile _Bool flag_host;
extern usb_buffer_rx_t usb_buffer_rx;
extern USBD_HandleTypeDef USBD_device_handler;

/* Public functions -----------------*/

#endif
