#ifndef __MSP_H
#define __MSP_H

#include <stdint.h>
#include <stdbool.h>

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

/* Public functions -----------------*/

void msp_name(uint8_t* str);
void msp_status(bool armed, bool acro);
void msp_status_ex(bool armed, bool acro);
void msp_osd_config(void);
void msp_analog(float vbat, float ibat, float imah, uint8_t rssi);
void msp_battery_state(float vbat, float ibat, float imah);

#endif
