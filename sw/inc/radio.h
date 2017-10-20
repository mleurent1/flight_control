#ifndef __RADIO_H
#define __RADIO_H

#include "board.h"

/* Public defines -----------------*/

#if (RADIO_TYPE == IBUS)
	#define THROTTLE_IDLE_DEFAULT 1000
	#define THROTTLE_RANGE_DEFAULT 1000
	#define AILERON_IDLE_DEFAULT 1500
	#define AILERON_RANGE_DEFAULT 500
	#define ELEVATOR_IDLE_DEFAULT 1500
	#define ELEVATOR_RANGE_DEFAULT 500
	#define RUDDER_IDLE_DEFAULT 1500
	#define RUDDER_RANGE_DEFAULT 500
	#define AUX_IDLE_DEFAULT 1000
	#define AUX_RANGE_DEFAULT 1000
#elif (RADIO_TYPE == SUMD)
	#define THROTTLE_IDLE_DEFAULT 8800
	#define THROTTLE_RANGE_DEFAULT 6400
	#define AILERON_IDLE_DEFAULT 12000
	#define AILERON_RANGE_DEFAULT 3200
	#define ELEVATOR_IDLE_DEFAULT 12000
	#define ELEVATOR_RANGE_DEFAULT 32000
	#define RUDDER_IDLE_DEFAULT 12000
	#define RUDDER_RANGE_DEFAULT 3200
	#define AUX_IDLE_DEFAULT 8800
	#define AUX_RANGE_DEFAULT 6400
#elif (RADIO_TYPE == SBUS)
	#define THROTTLE_IDLE_DEFAULT 368
	#define THROTTLE_RANGE_DEFAULT 1312
	#define AILERON_IDLE_DEFAULT 1024
	#define AILERON_RANGE_DEFAULT 656
	#define ELEVATOR_IDLE_DEFAULT 1024
	#define ELEVATOR_RANGE_DEFAULT 656
	#define RUDDER_IDLE_DEFAULT 1024
	#define RUDDER_RANGE_DEFAULT 656
	#define AUX_IDLE_DEFAULT 144
	#define AUX_RANGE_DEFAULT 1760
#endif

/* Public types -----------------*/

#if (RADIO_TYPE == IBUS)

	 __packed struct radio_frame_s {
		uint16_t header;
		uint16_t chan[14];
		uint16_t checksum;
	};
	
	typedef union {
		uint8_t bytes[32];
		struct radio_frame_s frame;
	} radio_frame_t;
	
#elif (RADIO_TYPE == SUMD)

	__packed struct radio_frame_s {
		uint8_t vendor_id;
		uint8_t status;
		uint8_t nb_chan;
		uint16_t chan[12];
		uint16_t checksum;
	};
	
	typedef union {
		uint8_t bytes[29];
		struct radio_frame_s frame;
	} radio_frame_t;
	
#elif (RADIO_TYPE == SBUS)

	__packed struct radio_frame_s {
		uint8_t header;
		unsigned int chan0  : 11;
		unsigned int chan1  : 11;
		unsigned int chan2  : 11;
		unsigned int chan3  : 11;
		unsigned int chan4  : 11;
		unsigned int chan5  : 11;
		unsigned int chan6  : 11;
		unsigned int chan7  : 11;
		unsigned int chan8  : 11;
		unsigned int chan9  : 11;
		unsigned int chan10 : 11;
		unsigned int chan11 : 11;
		unsigned int chan12 : 11;
		unsigned int chan13 : 11;
		unsigned int chan14 : 11;
		unsigned int chan15 : 11;
		uint8_t flags;
		uint8_t end_byte;
	};
	
	typedef union {
		uint8_t bytes[25];
		struct radio_frame_s frame;
	} radio_frame_t;
	
#endif

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
void radio_cal_idle(radio_frame_t * radio_frame);
void radio_cal_range(radio_frame_t * radio_frame);
void radio_expo(struct radio_s * radio, _Bool acro_mode);
void sx1276_init(void);

#endif
