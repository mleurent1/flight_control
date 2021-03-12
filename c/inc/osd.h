#ifndef __OSD_H
#define __OSD_H

#include <stdint.h>
#include "radio.h" // struct radio_s

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

extern volatile uint8_t osd_data_to_send[22];
extern volatile uint8_t osd_nbytes_to_send;
extern volatile uint8_t osd_data_received[2];
extern volatile uint8_t osd_nbytes_to_receive;

/* Public functions -----------------*/

void osd_init(void);
void osd_menu(struct radio_s * radio);
void osd_telemetry(float vbat, float ibat, float imah);

#endif
