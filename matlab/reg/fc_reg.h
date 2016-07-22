#include <stdint.h>

#define REG_NB_ADDR 22

volatile uint32_t reg[REG_NB_ADDR];

#define REG_VERSION reg[0]
#define REG_CTRL reg[1]
#define REG_CTRL__READ_SENSOR ((reg[1] & 1U) >> 0)
#define REG_CTRL__RESET_INT_ON_ARMED ((reg[1] & 2U) >> 1)
#define REG_CTRL__BEEP ((reg[1] & 4U) >> 2)
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
#define REG_MOTOR reg[8]
#define REG_MOTOR__MIN ((reg[8] & 65535U) >> 0)
#define REG_MOTOR__MAX ((reg[8] & 4294901760U) >> 16)
#define REG_CMD_OFFSET reg[9]
#define REG_CMD_OFFSET__THROTTLE ((reg[9] & 65535U) >> 0)
#define REG_CMD_OFFSET__AIL_ELE_RUD ((reg[9] & 4294901760U) >> 16)
#define REG_CMD_RANGE reg[10]
#define REG_CMD_RANGE__THROTTLE ((reg[10] & 65535U) >> 0)
#define REG_CMD_RANGE__AIL_ELE_RUD ((reg[10] & 4294901760U) >> 16)
#define REG_THROTTLE reg[11]
#define REG_THROTTLE__RANGE ((reg[11] & 65535U) >> 0)
#define REG_THROTTLE__ARMED ((reg[11] & 4294901760U) >> 16)
#define REG_RATE reg[12]
#define REG_RATE__PITCH ((reg[12] & 255U) >> 0)
#define REG_RATE__ROLL ((reg[12] & 65280U) >> 8)
#define REG_RATE__YAW ((reg[12] & 16711680U) >> 16)
#define REG_PITCH_P reg[13]
#define REG_PITCH_I reg[14]
#define REG_PITCH_D reg[15]
#define REG_ROLL_P reg[16]
#define REG_ROLL_I reg[17]
#define REG_ROLL_D reg[18]
#define REG_YAW_P reg[19]
#define REG_YAW_I reg[20]
#define REG_YAW_D reg[21]

const uint32_t reg_init[REG_NB_ADDR] = 
{
	9, // VERSION
	2048019, // CTRL
	0, // DEBUG
	0, // CLOCK
	0, // ERROR
	0, // LOOP_TIME
	0, // VBAT
	1093874483, // VBAT_MIN
	131073000, // MOTOR
	67108864, // CMD_OFFSET
	67110912, // CMD_RANGE
	72090400, // THROTTLE
	8224125, // RATE
	1073741824, // PITCH_P
	1017370378, // PITCH_I
	1101004800, // PITCH_D
	1073741824, // ROLL_P
	1017370378, // ROLL_I
	1101004800, // ROLL_D
	1082130432, // YAW_P
	1025758986, // YAW_I
	0 // YAW_D
};
