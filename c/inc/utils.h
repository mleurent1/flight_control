#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>
#include <math.h>

/* Public defines -----------------*/

#define ONESHOT 0
#define DSHOT 1
#define PWM 2

#define EXPONENTIAL expf
#define ARCSINUS asinf
#define SINUS sinf

/* Public macros -----------------*/

/* Public types -----------------*/

/* Exported variables -----------------*/

/* Public functions -----------------*/

void wait_ms(uint32_t t);
float uint32_to_float(uint32_t x);
uint32_t float_to_uint32(float x);
void dshot_encode(volatile uint16_t* val, volatile uint8_t buf[16], _Bool telemetry);
float expo(float lin);
float arcsin(float sin_val);
float sinus(float angle);
uint8_t crc8(uint8_t * data, uint8_t size);
void float_to_dec(float x, uint8_t * dec_int, uint8_t * dec_frac, uint8_t int_size, uint8_t frac_size);

#endif
