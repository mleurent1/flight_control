#ifndef __REG_H
#define __REG_H

#include <stdint.h>
#include "reg_def.h"

/* Public defines -----------------*/

/* Public types -----------------*/

typedef struct
{
	_Bool read_only;
	_Bool flash;
	_Bool is_float;
	uint32_t dflt;
} reg_properties_t;

typedef struct {
	uint8_t instr;
	uint8_t addr;
	union {
		uint8_t u8[4*16];
		uint16_t u16[2*16];
		uint32_t u32[1*16];
		float f[1*16];
	} data __attribute__((packed));
} __attribute__((packed)) host_buffer_rx_t;

typedef union {
	uint8_t u8[4*32];
	int8_t i8[4*32];
	uint16_t u16[2*32];
	int16_t i16[2*32];
	uint32_t u32[1*32];
	int32_t i32[1*32];
	float f[1*32];
} host_buffer_tx_t;

/* Exported variables -----------------*/

extern uint32_t reg[NB_REG];
extern float regf[NB_REG];
extern float sensor_rate;
extern float filter_alpha_vbat;
extern float filter_alpha_ibat;
extern float filter_alpha_accel;
extern float filter_alpha_radio;

/* Public functions -----------------*/

void reg_init(void);
void reg_access(host_buffer_rx_t * host_buffer_rx);
void reg_save(void);

#endif
