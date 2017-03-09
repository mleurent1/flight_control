#include "fc_reg.h"

uint32_t reg[NB_REG];
float regf[NB_REG];

reg_properties_t reg_properties[NB_REG] = 
{
	{1, 1, 0, 17}, // VERSION
	{0, 0, 0, 34}, // CTRL
	{0, 0, 0, 0}, // MOTOR_TEST
	{0, 0, 0, 0}, // DEBUG
	{1, 0, 0, 0}, // ERROR
	{1, 0, 0, 0}, // TIME
	{1, 0, 1, 0}, // VBAT
	{0, 1, 1, 1093350195}, // VBAT_MIN
	{0, 1, 0, 0}, // RECEIVER_BIND
	{0, 1, 1, 1082130432}, // EXPO
	{0, 1, 0, 50}, // MOTOR_START
	{0, 1, 0, 150}, // MOTOR_ARMED
	{0, 1, 0, 1080}, // COMMAND_RATE
	{0, 1, 0, 1800}, // THROTTLE_RANGE
	{0, 1, 1, 1082130432}, // PITCH_P
	{0, 1, 1, 998445679}, // PITCH_I
	{0, 1, 1, 1090519040}, // PITCH_D
	{0, 1, 1, 1082130432}, // ROLL_P
	{0, 1, 1, 998445679}, // ROLL_I
	{0, 1, 1, 1090519040}, // ROLL_D
	{0, 1, 1, 1086324736}, // YAW_P
	{0, 1, 1, 1002740646}, // YAW_I
	{0, 1, 1, 0}, // YAW_D
	{0, 1, 1, 1065353216}, // TPA_THRESHOLD
	{0, 1, 1, 0} // TPA_SLOPE
};
