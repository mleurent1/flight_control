#include <stdint.h>

#define REG_NB_ADDR 23

volatile uint32_t reg[REG_NB_ADDR];

#define REG_VERSION reg[0]
#define REG_CTRL reg[1]
#define REG_CTRL__READ_SENSOR ((reg[1] & 1U) >> 0)
#define REG_CTRL__RESET_INT_ON_ARMED ((reg[1] & 2U) >> 1)
#define REG_CTRL__BEEP_TEST ((reg[1] & 4U) >> 2)
#define REG_CTRL__LED ((reg[1] & 120U) >> 3)
#define REG_CTRL__MOTOR_SEL ((reg[1] & 1920U) >> 7)
#define REG_CTRL__MOTOR_TEST ((reg[1] & 134215680U) >> 11)
#define REG_DEBUG reg[2]
#define REG_CLOCK reg[3]
#define REG_ERROR reg[4]
#define REG_ERROR__SENSOR ((reg[4] & 65535U) >> 0)
#define REG_ERROR__COMMAND ((reg[4] & 4294901760U) >> 16)
#define REG_LOOP_TIME reg[5]
#define REG_VBAT reg[6]
#define REG_VBAT_MIN reg[7]
#define REG_EXPO reg[8]
#define REG_MOTOR reg[9]
#define REG_MOTOR__MIN ((reg[9] & 65535U) >> 0)
#define REG_MOTOR__MAX ((reg[9] & 4294901760U) >> 16)
#define REG_CMD_OFFSET reg[10]
#define REG_CMD_OFFSET__THROTTLE ((reg[10] & 65535U) >> 0)
#define REG_CMD_OFFSET__AIL_ELE_RUD ((reg[10] & 4294901760U) >> 16)
#define REG_CMD_RANGE reg[11]
#define REG_CMD_RANGE__THROTTLE ((reg[11] & 65535U) >> 0)
#define REG_CMD_RANGE__AIL_ELE_RUD ((reg[11] & 4294901760U) >> 16)
#define REG_THROTTLE reg[12]
#define REG_THROTTLE__RANGE ((reg[12] & 65535U) >> 0)
#define REG_THROTTLE__ARMED ((reg[12] & 4294901760U) >> 16)
#define REG_RATE reg[13]
#define REG_RATE__PITCH_ROLL ((reg[13] & 65535U) >> 0)
#define REG_RATE__YAW ((reg[13] & 4294901760U) >> 16)
#define REG_PITCH_P reg[14]
#define REG_PITCH_I reg[15]
#define REG_PITCH_D reg[16]
#define REG_ROLL_P reg[17]
#define REG_ROLL_I reg[18]
#define REG_ROLL_D reg[19]
#define REG_YAW_P reg[20]
#define REG_YAW_I reg[21]
#define REG_YAW_D reg[22]

const uint32_t reg_init[REG_NB_ADDR] = 
{
	10, // VERSION
	2048019, // CTRL
	0, // DEBUG
	0, // CLOCK
	0, // ERROR
	0, // LOOP_TIME
	0, // VBAT
	1093559910, // VBAT_MIN
	1045220557, // EXPO
	131073000, // MOTOR
	67108864, // CMD_OFFSET
	67110912, // CMD_RANGE
	72090400, // THROTTLE
	23593320, // RATE
	1073741824, // PITCH_P
	1017370378, // PITCH_I
	1092616192, // PITCH_D
	1073741824, // ROLL_P
	1017370378, // ROLL_I
	1092616192, // ROLL_D
	1082130432, // YAW_P
	1025758986, // YAW_I
	0 // YAW_D
};
