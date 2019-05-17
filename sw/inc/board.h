#ifndef __BOARD_H
#define __BOARD_H

#include "utils.h"
#if defined(MOTOF3)
	#include "motof3.h"
#elif defined(CYCLONE)
	#include "cyclone.h"
#elif defined(REVOLUTION)
	#include "revolution.h"
#elif defined(NUCLEO)
	#include "nucleo.h"
#elif defined(FEMTO)
	#include "femto.h"
#elif defined(BEECORE)
	#include "beecore.h"
#endif

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
void radio_synch(void);
	
#endif
