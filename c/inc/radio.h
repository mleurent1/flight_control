#ifndef __RADIO_H
#define __RADIO_H

#include "board.h"

/* Public defines -----------------*/

#define RADIO_FRAME_ERROR -1
#define RADIO_FRAME_LINK_STAT 0
#define RADIO_FRAME_RC_CHAN 1

/* Public types -----------------*/

struct crsf_rc_chan_s {
	uint32_t ch0 : 11;
	uint32_t ch1 : 11;
	uint32_t ch2 : 11;
	uint32_t ch3 : 11;
	uint32_t ch4 : 11;
	uint32_t ch5 : 11;
	uint32_t ch6 : 11;
	uint32_t ch7 : 11;
	uint32_t ch8 : 11;
	uint32_t ch9 : 11;
	uint32_t ch10 : 11;
	uint32_t ch11 : 11;
	uint32_t ch12 : 11;
	uint32_t ch13 : 11;
	uint32_t ch14 : 11;
	uint32_t ch15 : 11;
} __attribute__((packed));

struct crsf_link_stat_s {
	uint8_t up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
	uint8_t up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
	uint8_t up_link_quality;    // Uplink Package success rate / Link quality (%)
	int8_t  up_snr;             // Uplink SNR (dB)
	uint8_t active_antenna;     // number of currently best antenna
	uint8_t rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
	uint8_t up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW}
	uint8_t down_rssi;          // Downlink RSSI (dBm * -1)
	uint8_t down_link_quality;  // Downlink Package success rate / Link quality (%)
	int8_t  down_snr;           // Downlink SNR (dB)
};

struct crsf_frame_s {
	uint8_t sync_byte;
	uint8_t frame_length;
	uint8_t type;
	uint8_t payload[sizeof(struct crsf_rc_chan_s)];
	uint8_t crc;
};

typedef union {
	uint8_t bytes[sizeof(struct crsf_frame_s)];
	struct crsf_frame_s frame;
} radio_frame_t;

struct radio_raw_s {
	uint16_t throttle;
	uint16_t aileron;
	uint16_t elevator;
	uint16_t rudder;
	uint16_t aux[4];
} __attribute__((packed));

struct radio_s {
	float throttle;
	float pitch;
	float roll;
	float yaw;
	float aux[4];
	uint8_t rssi;
	int8_t snr;
};

/* Exported variables -----------------*/

/* Public functions -----------------*/

int8_t radio_decode(radio_frame_t * radio_frame, struct radio_s * radio);
void radio_expo(struct radio_s * radio, _Bool acro_mode);
void sx1276_init(void);

#endif
