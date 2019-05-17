#ifndef __FC_H
#define __FC_H

#include <stdint.h>
#include "reg.h" // host_buffer_rx_t
#include "sensor.h" // sensor_raw_t
#include "radio.h" // radio_frame_t

/* Public defines -----------------*/

#define SERVO_MAX 2000 // us
#define SERVO_MIN 1000 // us
#define MOTOR_MAX 2000
#define BEEPER_PERIOD 250 // ms
#define TIMEOUT_RADIO 500 // ms
#define TIMEOUT_SENSOR 10000 // us
#define VBAT_PERIOD 10 // ms

/* Public types -----------------*/

/* Exported variables -----------------*/

extern sensor_raw_t sensor_raw;
extern radio_frame_t radio_frame;

extern volatile uint8_t sensor_error_count;
extern volatile uint8_t radio_error_count;
extern volatile uint8_t rf_error_count;

extern volatile _Bool flag_sensor;
extern volatile _Bool flag_radio;
extern volatile _Bool flag_vbat;
extern volatile _Bool flag_rf;
extern volatile _Bool flag_host;
extern volatile _Bool flag_sensor_host_read;
extern volatile _Bool flag_rf_host_read;
extern volatile _Bool flag_timeout_sensor;
extern volatile _Bool flag_timeout_radio;
extern volatile _Bool flag_rf_rxtx_done;
extern volatile _Bool flag_rf_host_read;
extern volatile _Bool flag_acro;

extern volatile _Bool flag_beep_user;
extern volatile _Bool flag_beep_radio;
extern volatile _Bool flag_beep_sensor;
extern volatile _Bool flag_beep_host;
extern volatile _Bool flag_beep_vbat;

extern host_buffer_rx_t host_buffer_rx;

extern uint16_t timer_sensor[2];
extern uint16_t time_sensor;
extern uint16_t time_process;

/* Public functions -----------------*/

#endif
