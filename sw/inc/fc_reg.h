#define REG_NB_ADDR 25

volatile uint16_t reg[REG_NB_ADDR];

#define REG_VERSION reg[0]
#define REG_SPI_HOST_CTRL reg[1]
#define REG_DEBUG_MUX reg[2]
#define REG_LED_MUX reg[3]
#define REG_MOTOR_SELECT reg[4]
#define REG_MOTOR_TEST reg[5]
#define REG_SERVO_OFFSET reg[6]
#define REG_THROTTLE_OFFSET reg[7]
#define REG_THROTTLE_SCALE reg[8]
#define REG_AILERON_OFFSET reg[9]
#define REG_AILERON_SCALE reg[10]
#define REG_ELEVATOR_OFFSET reg[11]
#define REG_ELEVATOR_SCALE reg[12]
#define REG_RUDDER_OFFSET reg[13]
#define REG_RUDDER_SCALE reg[14]
#define REG_THROTTLE_TEST reg[15]
#define REG_PITCH_P reg[16]
#define REG_PITCH_I reg[17]
#define REG_PITCH_D reg[18]
#define REG_ROLL_P reg[19]
#define REG_ROLL_I reg[20]
#define REG_ROLL_D reg[21]
#define REG_YAW_P reg[22]
#define REG_YAW_I reg[23]
#define REG_YAW_D reg[24]

const uint16_t reg_init[REG_NB_ADDR] = 
{
	5, // VERSION
	0, // SPI_HOST_CTRL
	0, // DEBUG_MUX
	3, // LED_MUX
	0, // MOTOR_SELECT
	2500, // MOTOR_TEST
	2300, // SERVO_OFFSET
	341, // THROTTLE_OFFSET
	47, // THROTTLE_SCALE
	1024, // AILERON_OFFSET
	47, // AILERON_SCALE
	1024, // ELEVATOR_OFFSET
	47, // ELEVATOR_SCALE
	1024, // RUDDER_OFFSET
	47, // RUDDER_SCALE
	4806, // THROTTLE_TEST
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
