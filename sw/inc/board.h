#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>

/* Public functions -----------------*/

void board_init(void);
void host_send(uint8_t * data, uint8_t size);
void sensor_write(uint8_t addr, uint8_t data);
void sensor_read(uint8_t addr, uint8_t size);
void rf_write(uint8_t addr, uint8_t * data, uint8_t size);
void rf_read(uint8_t addr, uint8_t size);
void set_motors(uint32_t * motor_raw, _Bool * motor_telemetry);
void toggle_led_sensor(void);
void toggle_led_radio(void);
void set_mpu_host(_Bool host);
float get_vbat(void);
void reset_timeout_radio(void);
uint16_t get_timer_process(void);
void radio_sync(void);

#endif
