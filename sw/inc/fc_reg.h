#define NB_REG 22

uint32_t reg[NB_REG];
float regf[NB_REG];

#define REG_VERSION reg[0]
#define REG_CTRL reg[1]
#define REG_CTRL__MPU_HOST_CTRL (uint8_t)((reg[1] & 1U) >> 0)
#define REG_CTRL__RESET_INTEGRAL_ON_ARMED (uint8_t)((reg[1] & 2U) >> 1)
#define REG_CTRL__BEEP_TEST (uint8_t)((reg[1] & 4U) >> 2)
#define REG_CTRL__LED_SELECT (uint8_t)((reg[1] & 120U) >> 3)
#define REG_CTRL__MOTOR_SELECT (uint8_t)((reg[1] & 896U) >> 7)
#define REG_CTRL__MOTOR_TEST (uint16_t)((reg[1] & 67107840U) >> 10)
#define REG_DEBUG reg[2]
#define REG_ERROR reg[3]
#define REG_ERROR__SENSOR (uint16_t)((reg[3] & 65535U) >> 0)
#define REG_ERROR__COMMMAND (uint16_t)((reg[3] & 4294901760U) >> 16)
#define REG_LOOP_TIME reg[4]
#define REG_VBAT regf[5]
#define REG_VBAT_MIN regf[6]
#define REG_VBAT_MAX regf[7]
#define REG_EXPO regf[8]
#define REG_MOTOR reg[9]
#define REG_MOTOR__MIN (uint16_t)((reg[9] & 65535U) >> 0)
#define REG_MOTOR__MAX (uint16_t)((reg[9] & 4294901760U) >> 16)
#define REG_THROTTLE reg[10]
#define REG_THROTTLE__RANGE (uint16_t)((reg[10] & 65535U) >> 0)
#define REG_THROTTLE__ARMED (uint16_t)((reg[10] & 4294901760U) >> 16)
#define REG_RATE reg[11]
#define REG_RATE__PITCH_ROLL (uint16_t)((reg[11] & 65535U) >> 0)
#define REG_RATE__YAW (uint16_t)((reg[11] & 4294901760U) >> 16)
#define REG_PITCH_P regf[12]
#define REG_PITCH_I regf[13]
#define REG_PITCH_D regf[14]
#define REG_ROLL_P regf[15]
#define REG_ROLL_I regf[16]
#define REG_ROLL_D regf[17]
#define REG_YAW_P regf[18]
#define REG_YAW_I regf[19]
#define REG_YAW_D regf[20]
#define REG_TPA regf[21]

typedef struct
{
	_Bool read_only;
	_Bool flash;
	_Bool is_float;
	uint32_t dflt;
} reg_properties_t;

reg_properties_t reg_properties[NB_REG] = 
{
	{1, 1, 0, 12}, // VERSION
	{0, 0, 0, 2066}, // CTRL
	{0, 0, 0, 0}, // DEBUG
	{1, 0, 0, 0}, // ERROR
	{1, 0, 0, 0}, // LOOP_TIME
	{1, 0, 1, 0}, // VBAT
	{0, 1, 1, 1093664768}, // VBAT_MIN
	{0, 1, 1, 1095342490}, // VBAT_MAX
	{0, 1, 1, 1045220557}, // EXPO
	{0, 1, 0, 131073000}, // MOTOR
	{0, 1, 0, 68813700}, // THROTTLE
	{0, 1, 0, 23593320}, // RATE
	{0, 1, 1, 1073741824}, // PITCH_P
	{0, 1, 1, 1017370378}, // PITCH_I
	{0, 1, 1, 1092616192}, // PITCH_D
	{0, 1, 1, 1073741824}, // ROLL_P
	{0, 1, 1, 1017370378}, // ROLL_I
	{0, 1, 1, 1092616192}, // ROLL_D
	{0, 1, 1, 1082130432}, // YAW_P
	{0, 1, 1, 1025758986}, // YAW_I
	{0, 1, 1, 0}, // YAW_D
	{0, 1, 1, 0} // TPA
};
