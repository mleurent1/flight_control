#include <stdint.h>

#define REG_NB 4
#define VERSION 0
#define GET_SENSOR 1
#define DEBUG_MUX 2
#define LED_MUX 3

volatile uint16_t reg[REG_NB];

const uint16_t reg_init[REG_NB] = 
{
	2, // VERSION
	1, // GET_SENSOR
	0, // DEBUG_MUX
	3 // LED_MUX
};

