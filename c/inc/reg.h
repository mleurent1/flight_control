#ifndef __REG_H
#define __REG_H

#include <stdint.h>
#include <stdbool.h>

#include "reg_def.h" // NB_REG

/* Public defines -----------------*/

#define HOST_BUF_LEN_RX 16 // in 32-bit words
#define HOST_BUF_LEN_TX 32 // in 32-bit words

/* Public types -----------------*/

typedef struct
{
	bool read_only;
	bool flash;
	bool is_float;
	uint32_t dflt;
} reg_properties_t;

typedef struct {
	uint8_t instr;
	uint8_t addr;
	union {
		uint8_t   u8[HOST_BUF_LEN_RX*4];
		uint16_t u16[HOST_BUF_LEN_RX*2];
		uint32_t u32[HOST_BUF_LEN_RX];
		float      f[HOST_BUF_LEN_RX];
	} data;
} __attribute__((packed)) host_buffer_rx_t;

typedef union {
	uint8_t   u8[HOST_BUF_LEN_TX*4];
	int8_t    i8[HOST_BUF_LEN_TX*4];
	uint16_t u16[HOST_BUF_LEN_TX*2];
	int16_t  i16[HOST_BUF_LEN_TX*2];
	uint32_t u32[HOST_BUF_LEN_TX];
	int32_t  i32[HOST_BUF_LEN_TX];
	float      f[HOST_BUF_LEN_TX];
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
