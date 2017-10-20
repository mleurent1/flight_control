#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>
#include <math.h>

/* Public defines -----------------*/

#define MPU6000 0
#define MPU6050 1
#define MPU9150 2
#define ONESHOT 0
#define DSHOT 1
#define IBUS 0
#define SUMD 1
#define SBUS 2

#define EXPONENTIAL exp
#define ARCSINUS asin
#define SINUS sin

/* Public macros -----------------*/

/* Public types -----------------*/

/* Public variables -----------------*/

extern volatile uint32_t tick;

/* Public functions -----------------*/

void wait_ms(uint32_t t);
float uint32_to_float(uint32_t x);
uint32_t int32_to_uint32(int32_t x);
int32_t uint32_to_int32(uint32_t x);
void dshot_encode(volatile uint32_t* val, volatile uint32_t buf[17]);
float expo(float lin);
float arcsin(float sin_val);
float sinus(float angle);

#endif
