#include <stdint.h>

#define NB_REG 26

#define REG_VERSION reg[0]
#define REG_CTRL reg[1]
#define REG_CTRL__MPU_HOST_CTRL (uint8_t)((reg[1] & 1U) >> 0)
#define REG_CTRL__RESET_INTEGRAL_ON_ARMED (uint8_t)((reg[1] & 2U) >> 1)
#define REG_CTRL__BEEP_TEST (uint8_t)((reg[1] & 4U) >> 2)
#define REG_CTRL__TIME_MAXHOLD (uint8_t)((reg[1] & 8U) >> 3)
#define REG_CTRL__LED_SELECT (uint8_t)((reg[1] & 48U) >> 4)
#define REG_MOTOR_TEST reg[2]
#define REG_MOTOR_TEST__VALUE (uint16_t)((reg[2] & 65535U) >> 0)
#define REG_MOTOR_TEST__SELECT (uint8_t)((reg[2] & 983040U) >> 16)
#define REG_DEBUG reg[3]
#define REG_DEBUG__CASE (uint8_t)((reg[3] & 255U) >> 0)
#define REG_DEBUG__MASK (uint16_t)((reg[3] & 16776960U) >> 8)
#define REG_ERROR reg[4]
#define REG_ERROR__MPU (uint16_t)((reg[4] & 65535U) >> 0)
#define REG_ERROR__RADIO (uint16_t)((reg[4] & 4294901760U) >> 16)
#define REG_TIME reg[5]
#define REG_TIME__MPU (uint16_t)((reg[5] & 65535U) >> 0)
#define REG_TIME__LOOP (uint16_t)((reg[5] & 4294901760U) >> 16)
#define REG_VBAT regf[6]
#define REG_VBAT_MIN regf[7]
#define REG_RECEIVER_BIND reg[8]
#define REG_PITCH_ROLL_EXPO regf[9]
#define REG_YAW_EXPO regf[10]
#define REG_MOTOR_START reg[11]
#define REG_MOTOR_ARMED reg[12]
#define REG_PITCH_ROLL_RATE regf[13]
#define REG_YAW_RATE regf[14]
#define REG_THROTTLE_RANGE regf[15]
#define REG_PITCH_P regf[16]
#define REG_PITCH_I regf[17]
#define REG_PITCH_D regf[18]
#define REG_ROLL_P regf[19]
#define REG_ROLL_I regf[20]
#define REG_ROLL_D regf[21]
#define REG_YAW_P regf[22]
#define REG_YAW_I regf[23]
#define REG_YAW_D regf[24]
#define REG_THROTTLE_ATTEN regf[25]

typedef struct
{
	_Bool read_only;
	_Bool flash;
	_Bool is_float;
	uint32_t dflt;
} reg_properties_t;

extern uint32_t reg[NB_REG];
extern float regf[NB_REG];
extern reg_properties_t reg_properties[NB_REG];
