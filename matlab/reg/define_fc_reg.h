#define VERSION 10

#define CTRL 2048019

#define CTRL__READ_SENSOR (1 << 0)
#define CTRL__RESET_INT_ON_ARMED (1 << 1)
#define CTRL__BEEP_TEST (1 << 2)
#define CTRL__LED(x) (((x) & 0xF) << 3)
#define CTRL__MOTOR_SEL(x) (((x) & 0xF) << 7)
#define CTRL__MOTOR_TEST(x) (((x) & 0xFFFF) << 11)

#define DEBUG 0

#define CLOCK 0

#define ERROR 0

#define ERROR__SENSOR(x) (((x) & 0xFFFF) << 0)
#define ERROR__COMMAND(x) (((x) & 0xFFFF) << 16)

#define LOOP_TIME 0

#define VBAT 0

#define VBAT_MIN 1093559910

#define EXPO 1045220557

#define MOTOR 131073000

#define MOTOR__MIN(x) (((x) & 0xFFFF) << 0)
#define MOTOR__MAX(x) (((x) & 0xFFFF) << 16)

#define CMD_OFFSET 67108864

#define CMD_OFFSET__THROTTLE(x) (((x) & 0xFFFF) << 0)
#define CMD_OFFSET__AIL_ELE_RUD(x) (((x) & 0xFFFF) << 16)

#define CMD_RANGE 67110912

#define CMD_RANGE__THROTTLE(x) (((x) & 0xFFFF) << 0)
#define CMD_RANGE__AIL_ELE_RUD(x) (((x) & 0xFFFF) << 16)

#define THROTTLE 72090400

#define THROTTLE__RANGE(x) (((x) & 0xFFFF) << 0)
#define THROTTLE__ARMED(x) (((x) & 0xFFFF) << 16)

#define RATE 23593320

#define RATE__PITCH_ROLL(x) (((x) & 0xFFFF) << 0)
#define RATE__YAW(x) (((x) & 0xFFFF) << 16)

#define PITCH_P 1073741824
#define PITCH_I 1017370378
#define PITCH_D 1092616192

#define ROLL_P 1073741824
#define ROLL_I 1017370378
#define ROLL_D 1092616192

#define YAW_P 1082130432
#define YAW_I 1025758986
#define YAW_D 0