#include <stdint.h>

#define REG_NB 2
#define VERSION 0
#define DEBUG_MUX 1

volatile uint16_t reg[REG_NB];

const uint16_t reg_init[REG_NB] = 
{
	1, // VERSION
	0  // DEBUG_MUX
};

