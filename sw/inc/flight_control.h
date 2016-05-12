#include <stdint.h>

#define REG_NB 8
#define VERSION 0
#define GET_SENSOR 1
#define DEBUG_MUX 2
#define LED_MUX 3
#define MOTOR1_TEST 4
#define MOTOR2_TEST 5
#define MOTOR3_TEST 6
#define MOTOR4_TEST 7

volatile uint16_t reg[REG_NB];

const uint16_t reg_init[REG_NB] = 
{
	3, // VERSION
	1, // GET_SENSOR
	0, // DEBUG_MUX
	3, // LED_MUX
	0, // MOTOR1_TEST
	0, // MOTOR2_TEST
	0, // MOTOR3_TEST
	0  // MOTOR4_TEST
};

