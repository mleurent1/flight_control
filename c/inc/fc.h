#ifndef __FC_H
#define __FC_H

#include <stdint.h>
#include "reg.h" // host_buffer_rx_t
#include "sensor.h" // sensor_raw_t
#include "radio.h" // radio_frame_t

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

extern sensor_raw_t sensor_raw;
extern radio_frame_t radio_frame;
extern volatile float vbat, vbat_smoothed;
extern volatile float ibat;

extern volatile uint8_t sensor_error_count;
extern volatile uint8_t radio_error_count;
extern volatile uint8_t rf_error_count;

extern volatile _Bool flag_sensor;
extern volatile _Bool flag_radio;
extern volatile _Bool flag_vbat;
extern volatile _Bool flag_rf;
extern volatile _Bool flag_host;
extern volatile _Bool flag_time;
extern volatile _Bool flag_rf_host_read;
extern volatile _Bool flag_rf_rxtx_done;

extern host_buffer_rx_t host_buffer_rx;

extern volatile uint32_t t_ms;

/* Public functions -----------------*/

#endif
