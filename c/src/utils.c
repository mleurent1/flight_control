#include <stdint.h>
#include <stdbool.h>

#include "utils.h"
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include "fc.h" // t_ms
#include "board.h" // toggle_led()

void wait_ms(uint32_t t)
{
	uint32_t next_t_ms = t_ms + t;
	while (t_ms != next_t_ms)
		__WFI();
}

void wait(uint8_t t)
{
	for (uint16_t i=t; i>0; i--) {
		for (uint16_t j=0; j<i; j++) {
			toggle_led(true);
			wait_ms(50);
			toggle_led(true);
			wait_ms(50);
		}
		wait_ms(1000-i*100);
	}
}

float uint32_to_float(uint32_t x)
{
	union {
		float f;
		uint32_t u;
	} f2u;
	f2u.u = x;
	return f2u.f;
}

uint32_t float_to_uint32(float x)
{
	union {
		float f;
		uint32_t u;
	} f2u;
	f2u.f = x;
	return f2u.u;
}

void dshot_encode(volatile uint16_t* val, volatile uint8_t buf[16], bool telemetry)
{
	uint8_t i;
	uint8_t bit[11];
	for (i=0; i<11; i++)
	{
		buf[i] = (*val & (1 << (10-i))) ? 60 : 30;
		bit[i] = (*val & (1 << i)) ? 1 : 0;
	}
	buf[11] = telemetry ? 60 : 30;
	buf[12] = (bit[10]^bit[6]^bit[2]) ? 60 : 30;
	buf[13] = (bit[ 9]^bit[5]^bit[1]) ? 60 : 30;
	buf[14] = (bit[ 8]^bit[4]^bit[0]) ? 60 : 30;
	buf[15] = (bit[ 7]^bit[3])        ? 60 : 30;
}

float expo(float lin)
{
	float x;
	float y;
	x =     lin; y  = x;
	x = x * lin; y += x / 2.0f;
	x = x * lin; y += x / 6.0f;
	x = x * lin; y += x / 24.0f;
	x = x * lin; y += x / 120.0f;
	//x = x * lin; y += x / 720.0f;
	return y;
}

float arcsin(float sin_val)
{
	float x;
	float y;
	x =               sin_val; y  = x;
	x = x * sin_val * sin_val; y += x *  0.166666667f;
	x = x * sin_val * sin_val; y += x *  0.075f;
	x = x * sin_val * sin_val; y += x *  0.044642857f;
	//x = x * sin_val * sin_val; y += x *  0.030382f;
	return y;
}

float sinus(float angle)
{
	float x;
	float y;
	x =             angle; y  = x;
	x = x * angle * angle; y += x * -0.166666667f;
	x = x * angle * angle; y += x *  0.2f;
	x = x * angle * angle; y += x *  0.008333333f;
	//x = x * angle * angle; y += x *  2.755731922e-6f;
	return y;
}

uint8_t crc8(uint8_t * data, uint8_t size)
{
	uint8_t crc = 0;
	for (int i=0; i<size; i++) {
		crc ^= data[i];
		for (int j=0; j<8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0xD5;
			else
				crc = crc << 1;
		}
	}
	return crc;
}

void float_to_dec(float x, uint8_t* dec_int, uint8_t* dec_frac)
{
	float b10_mult[4] = {1000.0f,100.0f,10.0f,1.0f};
	float b10_div[4] = {0.001f,0.01f,0.1f,1.0f};
	float q;
	uint8_t i;

	for (i=0; i<4; i++) {
		q = floorf(x * b10_div[i]);
		if (q > 9.0f)
			dec_int[i] = 9;
		else
			dec_int[i] = (uint8_t)q;
		x = x - q * b10_mult[i];
	}

	for (i=0; i<3; i++) {
		q = floorf(x * b10_mult[2-i]);
		dec_frac[i] = (uint8_t)q;
		x = x - q * b10_div[2-i];
	}
}
