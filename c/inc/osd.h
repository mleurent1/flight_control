#ifndef __OSD_H
#define __OSD_H

#include <stdint.h>

#include "radio.h" // struct radio_s

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

extern uint8_t* osd_next_data_to_send;
extern uint8_t osd_next_nbytes_to_send;
extern uint8_t osd_data_received[];

/* Public functions -----------------*/

void osd_init(void);
void osd_menu(struct radio_s * radio);
void osd_telemetry(float vbat, float ibat, float imah, uint8_t t_s, uint8_t t_min, uint8_t rssi, int8_t snr);

#endif
