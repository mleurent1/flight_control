#include "utils.h"
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif

volatile uint32_t tick;

/*--- System timer ---*/
void SysTick_Handler()
{
	tick++;
}

void wait_ms(uint32_t t)
{
	uint32_t next_tick = tick + t;
	while (tick != next_tick)
		__WFI();
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

uint32_t int32_to_uint32(int32_t x)
{
	union {
		int32_t i;
		uint32_t u;
	} i2u;
	i2u.i = x;
	return i2u.u;
}

int32_t uint32_to_int32(uint32_t x)
{
	union {
		int32_t i;
		uint32_t u;
	} i2u;
	i2u.u = x;
	return i2u.i;
}

void dshot_encode(volatile uint16_t* val, volatile uint8_t buf[16], _Bool telemetry)
{
	int i;
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
