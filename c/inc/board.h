#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>
#include <stdbool.h>

/* Public defines -----------------*/

#define SERVO_MAX 2000 // us
#define SERVO_MIN 1000 // us

/* Exported variables -----------------*/

extern volatile bool sensor_busy;
extern volatile bool osd_busy;

/* Public functions -----------------*/

uint8_t board_init(void);
void host_send(uint8_t * data, uint8_t size);
void sensor_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size);
void en_sensor_irq(void);
void trig_radio_rx(void);
void trig_delayed_radio_rx(void);
void rf_write(uint8_t addr, uint8_t * data, uint8_t size);
void rf_read(uint8_t addr, uint8_t size);
void set_motors(uint16_t * motor_raw, bool * motor_telemetry);
void toggle_led(bool en);
void toggle_led2(bool en);
void toggle_beeper(bool en);
uint16_t get_t_us(void);
void trig_vbat_meas(void);
void osd_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size);
void runcam_send(uint8_t * data, uint8_t size);
void sma_send(uint8_t * data, uint8_t size);
void set_leds(uint8_t * grb);

#endif
