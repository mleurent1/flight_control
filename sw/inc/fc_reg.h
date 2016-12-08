#define NB_REG 25

uint32_t reg[NB_REG];
float regf[NB_REG];

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
#define REG_ERROR reg[4]
#define REG_ERROR__SENSOR (uint16_t)((reg[4] & 65535U) >> 0)
#define REG_ERROR__COMMMAND (uint16_t)((reg[4] & 4294901760U) >> 16)
#define REG_TIME reg[5]
#define REG_TIME__SPI (uint16_t)((reg[5] & 65535U) >> 0)
#define REG_TIME__PROCESS (uint16_t)((reg[5] & 4294901760U) >> 16)
#define REG_VBAT regf[6]
#define REG_VBAT_MIN regf[7]
#define REG_RECEIVER_BIND reg[8]
#define REG_EXPO regf[9]
#define REG_MOTOR_START reg[10]
#define REG_MOTOR_ARMED reg[11]
#define REG_COMMAND_RATE regf[12]
#define REG_THROTTLE_RANGE reg[13]
#define REG_PITCH_P regf[14]
#define REG_PITCH_I regf[15]
#define REG_PITCH_D regf[16]
#define REG_ROLL_P regf[17]
#define REG_ROLL_I regf[18]
#define REG_ROLL_D regf[19]
#define REG_YAW_P regf[20]
#define REG_YAW_I regf[21]
#define REG_YAW_D regf[22]
#define REG_TPA_THRESHOLD regf[23]
#define REG_TPA_SLOPE regf[24]

typedef struct
{
	_Bool read_only;
	_Bool flash;
	_Bool is_float;
	uint32_t dflt;
} reg_properties_t;

reg_properties_t reg_properties[NB_REG] = 
{
	{1, 1, 0, 16}, // VERSION
	{0, 0, 0, 34}, // CTRL
	{0, 0, 0, 0}, // MOTOR_TEST
	{0, 0, 0, 0}, // DEBUG
	{1, 0, 0, 0}, // ERROR
	{1, 0, 0, 0}, // TIME
	{1, 0, 1, 0}, // VBAT
	{0, 1, 1, 1093350195}, // VBAT_MIN
	{0, 1, 0, 0}, // RECEIVER_BIND
	{0, 1, 1, 1045220557}, // EXPO
	{0, 1, 0, 50}, // MOTOR_START
	{0, 1, 0, 150}, // MOTOR_ARMED
	{0, 1, 1, 1075838976}, // COMMAND_RATE
	{0, 1, 0, 1800}, // THROTTLE_RANGE
	{0, 1, 1, 1073741824}, // PITCH_P
	{0, 1, 1, 1008981770}, // PITCH_I
	{0, 1, 1, 0}, // PITCH_D
	{0, 1, 1, 1073741824}, // ROLL_P
	{0, 1, 1, 1008981770}, // ROLL_I
	{0, 1, 1, 0}, // ROLL_D
	{0, 1, 1, 1077936128}, // YAW_P
	{0, 1, 1, 1017370378}, // YAW_I
	{0, 1, 1, 0}, // YAW_D
	{0, 1, 1, 1056964608}, // TPA_THRESHOLD
	{0, 1, 1, 1053609165} // TPA_SLOPE
};
