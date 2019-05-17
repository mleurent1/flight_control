#ifndef __BEECORE_H
#define __BEECORE_H

//#define STM32F303xC // Already defined by device pack
#define USE_HAL_DRIVER
#include "stm32f3xx.h"
#define REG_FLASH_ADDR 0x0803F800

#define SENSOR MPU6000
#define SENSOR_ORIENTATION 0

#undef BEEPER
#undef VBAT
#undef RF

#endif
