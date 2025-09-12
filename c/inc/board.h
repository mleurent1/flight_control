#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>

/* Public defines -----------------*/

#define SERVO_MAX 2000 // us
#define SERVO_MIN 1000 // us

/* Exported variables -----------------*/

extern volatile _Bool sensor_busy;

/* Public functions -----------------*/

void board_init(void);
void host_send(uint8_t * data, uint8_t size);
void sensor_write(uint8_t addr, uint8_t data);
void sensor_read(uint8_t addr, uint8_t size);
void rf_write(uint8_t addr, uint8_t * data, uint8_t size);
void rf_read(uint8_t addr, uint8_t size);
void set_motors(uint16_t * motor_raw, _Bool * motor_telemetry);
void toggle_led(void);
void toggle_led2(_Bool en);
void toggle_beeper(_Bool en);
int32_t get_t_us(void);
void radio_sync(void);
void trig_vbat_meas(void);
void osd_send(uint8_t * data, uint8_t size);
void runcam_send(uint8_t * data, uint8_t size);
void sma_send(uint8_t * data, uint8_t size);
void set_leds(uint8_t * grb);
uint8_t get_dios(void);
void set_dios(uint8_t val);

#endif
