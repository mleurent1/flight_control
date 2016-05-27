#include <stdint.h>

#define SERVO_MIN 1806
#define FLAG_SENSOR 0x01
#define FLAG_COMMAND 0x02
#define FLAG_SERVO 0x04

volatile uint8_t FLAG;

//####### REG ADDR #######

#define VERSION 0
#define GET_SENSOR 1
#define DEBUG_MUX 2
#define LED_MUX 3
#define MOTOR_SELECT 4
#define MOTOR_TEST 5
#define SERVO_OFFSET 6
#define THROTTLE_OFFSET 7
#define THROTTLE_SCALE 8
#define AILERON_OFFSET 9
#define AILERON_SCALE 10
#define ELEVATOR_OFFSET 11
#define ELEVATOR_SCALE 12
#define RUDDER_OFFSET 13
#define RUDDER_SCALE 14
#define THROTTLE_TEST 15
#define PITCH_P 16
#define PITCH_I 17
#define PITCH_D 18
#define ROLL_P 19
#define ROLL_I 20
#define ROLL_D 21
#define YAW_P 22
#define YAW_I 23
#define YAW_D 24


//####### REG DEFINITIONS #######

#define REG_NB 25

volatile uint16_t reg[REG_NB];

const uint16_t reg_init[REG_NB] = 
{
	4, // VERSION
	1, // GET_SENSOR
	0, // DEBUG_MUX
	3, // LED_MUX
	0, // MOTOR_SELECT
	2500, // MOTOR_TEST
	2300, // SERVO_OFFSET
	341, // THROTTLE_OFFSET
	47, // THROTTLE_SCALE
	1024, // AILERON_OFFSET
	47, // AILERON_SCALE
	1024, // AILERON_OFFSET
	47, // AILERON_SCALE
	1024, // RUDDER_OFFSET
	47, // RUDDER_SCALE
	SERVO_MIN, // THROTTLE_TEST
	0, // PITCH_P
	0, // PITCH_I
	0, // PITCH_D
	0, // ROLL_P
	0, // ROLL_I
	0, // ROLL_D
	0, // YAW_P
	0, // YAW_I
	0 // YAW_D
};

