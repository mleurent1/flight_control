#include <stdint.h>

#define REG_NB_ADDR 21

volatile uint32_t reg[REG_NB_ADDR];

#define REG_VERSION reg[0]
#define REG_CTRL reg[1]
#define REG_CTRL__READ_SENSOR ((reg[1] & 1U) >> 0)
#define REG_CTRL__RESET_INT_ON_ARMED ((reg[1] & 2U) >> 1)
#define REG_CTRL__LED ((reg[1] & 60U) >> 2)
#define REG_CTRL__MOTOR_SEL ((reg[1] & 960U) >> 6)
#define REG_CTRL__MOTOR_TEST ((reg[1] & 67107840U) >> 10)
#define REG_DEBUG reg[2]
#define REG_CLOCK reg[3]
#define REG_ERROR reg[4]
#define REG_ERROR__SENSOR ((reg[4] & 65535U) >> 0)
#define REG_ERROR__COMMAND ((reg[4] & 4294901760U) >> 16)
#define REG_TIME reg[5]
#define REG_TIME__LOOP ((reg[5] & 65535U) >> 0)
#define REG_MOTOR reg[6]
#define REG_MOTOR__MIN ((reg[6] & 65535U) >> 0)
#define REG_MOTOR__MAX ((reg[6] & 4294901760U) >> 16)
#define REG_THROTTLE reg[7]
#define REG_THROTTLE__OFFSET ((reg[7] & 65535U) >> 0)
#define REG_THROTTLE__ARMED ((reg[7] & 4294901760U) >> 16)
#define REG_THROTTLE_SCALE reg[8]
#define REG_AILERON_SCALE reg[9]
#define REG_ELEVATOR_SCALE reg[10]
#define REG_RUDDER_SCALE reg[11]
#define REG_PITCH_P reg[12]
#define REG_PITCH_I reg[13]
#define REG_PITCH_D reg[14]
#define REG_ROLL_P reg[15]
#define REG_ROLL_I reg[16]
#define REG_ROLL_D reg[17]
#define REG_YAW_P reg[18]
#define REG_YAW_I reg[19]
#define REG_YAW_D reg[20]

const uint32_t reg_init[REG_NB_ADDR] = 
{
	7, // VERSION
	1024011, // CTRL
	0, // DEBUG
	0, // CLOCK
	0, // ERROR
	0, // TIME
	131073000, // MOTOR
	78643541, // THROTTLE
	1058416009, // THROTTLE_SCALE
	1044098796, // AILERON_SCALE
	1044098796, // ELEVATOR_SCALE
	1044098796, // RUDDER_SCALE
	1073741824, // PITCH_P
	1017370378, // PITCH_I
	1101004800, // PITCH_D
	1073741824, // ROLL_P
	1017370378, // ROLL_I
	1101004800, // ROLL_D
	1073741824, // YAW_P
	1017370378, // YAW_I
	0 // YAW_D
};
