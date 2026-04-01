#ifndef __FC_H
#define __FC_H

#include <stdint.h>
#include <stdbool.h>

#include "reg.h" // host_buffer_rx_t
#include "sensor.h" // sensor_raw_t
#include "radio.h" // radio_frame_t

/* Public defines -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

extern radio_frame_t radio_frame;
extern volatile float vbat;
extern volatile float ibat;

extern volatile uint8_t sensor_error_count;
extern volatile uint8_t radio_error_count;
extern volatile uint8_t rf_error_count;
extern volatile uint8_t sma_error_count;

extern volatile bool flag_sensor;
extern volatile bool flag_radio;
extern volatile bool flag_vbat;
extern volatile bool flag_msp;
extern volatile bool flag_rf;
extern volatile bool flag_host;
extern volatile bool flag_time;
extern volatile bool flag_rf_rxtx_done;
extern volatile bool flag_msp_connected;

extern host_buffer_rx_t host_buffer_rx;
extern host_buffer_tx_t host_buffer_tx;

extern volatile uint32_t t_ms;

/* Public functions -----------------*/

#endif
