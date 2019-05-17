#ifndef __RADIO_H
#define __RADIO_H

#include "board.h"

/* Public defines -----------------*/

/* Public types -----------------*/

 __packed struct radio_frame_s {
	uint16_t header;
	uint16_t chan[14];
	uint16_t checksum;
};

typedef union {
	uint8_t bytes[32];
	struct radio_frame_s frame;
} radio_frame_t;

__packed struct radio_raw_s {
	uint16_t throttle;
	uint16_t aileron;
	uint16_t elevator;
	uint16_t rudder;
	uint16_t aux[4];
};

struct radio_s {
	float throttle;
	float pitch;
	float roll;
	float yaw;
	float aux[4];
};

/* Exported variables -----------------*/

/* Public functions -----------------*/

_Bool radio_decode(radio_frame_t * radio_frame, struct radio_raw_s * radio_raw, struct radio_s * radio);
void radio_expo(struct radio_s * radio, _Bool acro_mode);
void sx1276_init(void);

#endif
