#define VERSION 8

#define CTRL 1024011

#define CTRL__READ_SENSOR (1 << 0)
#define CTRL__RESET_INT_ON_ARMED (1 << 1)
#define CTRL__LED(x) (((x) & 0xF) << 2)
#define CTRL__MOTOR_SEL(x) (((x) & 0xF) << 6)
#define CTRL__MOTOR_TEST(x) (((x) & 0xFFFF) << 10)

#define DEBUG 0

#define CLOCK 0

#define ERROR 0

#define ERROR__SENSOR(x) (((x) & 0xFFFF) << 0)
#define ERROR__COMMAND(x) (((x) & 0xFFFF) << 16)

#define TIME 0

#define TIME__LOOP(x) (((x) & 0xFFFF) << 0)

#define MOTOR 131073000

#define MOTOR__MIN(x) (((x) & 0xFFFF) << 0)
#define MOTOR__MAX(x) (((x) & 0xFFFF) << 16)

#define THROTTLE 72089941

#define THROTTLE__OFFSET(x) (((x) & 0xFFFF) << 0)
#define THROTTLE__ARMED(x) (((x) & 0xFFFF) << 16)

#define THROTTLE_SCALE 1058416009
#define AILERON_SCALE 1044098796
#define ELEVATOR_SCALE 1044098796
#define RUDDER_SCALE 1044098796

#define PITCH_P 1073741824
#define PITCH_I 1017370378
#define PITCH_D 1101004800

#define ROLL_P 1073741824
#define ROLL_I 1017370378
#define ROLL_D 1101004800

#define YAW_P 1082130432
#define YAW_I 1025758986
#define YAW_D 0