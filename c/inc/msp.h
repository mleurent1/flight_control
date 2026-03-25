#ifndef __MSP_H
#define __MSP_H

#include <stdint.h>
#include <stdbool.h>

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

extern uint8_t msp_name_str[12];

/* Public functions -----------------*/

void msp_name(void);
void msp_status(bool armed);
void msp_osd_config(void);
void msp_analog(float vbat, float ibat, float imah, uint8_t rssi);
void msp_battery_state(float vbat, float ibat, float imah);

#endif
