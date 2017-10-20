#ifndef __REVOLUTION_H
#define __REVOLUTION_H

//#define STM32F405xx // Already defined by device pack
#define USE_HAL_DRIVER
#include "stm32f4xx.h"
#define REG_FLASH_ADDR 0x080E0000
#define SENSOR MPU6000
#define SENSOR_ORIENTATION 180
#define RADIO_TYPE IBUS
#define ESC DSHOT

#endif
