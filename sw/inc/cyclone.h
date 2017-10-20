#ifndef __CYCLONE_H
#define __CYCLONE_H

//#define STM32F303xC // Already defined by device pack
#define USE_HAL_DRIVER
#include "stm32f3xx.h"
#define REG_FLASH_ADDR 0x0803F800
#define SENSOR MPU6000
#define SENSOR_ORIENTATION 90
#define RADIO_TYPE IBUS
#define ESC DSHOT

#endif
