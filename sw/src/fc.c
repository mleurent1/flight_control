#include "fc.h"
#include "board.h" // board_init()
#include "sensor.h" // mpu_process_samples()
#include "radio.h" // radio_decode()
#include "reg.h" // REG__*
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include <string.h> // memcpy

/* Private defines ------------------------------------*/

#define MOTOR_MAX 2000
#define I_MAX 300.0f
#define PID_MAX 600.0f
#define TIME_FILTER_ALPHA 0.1f

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

sensor_raw_t sensor_raw;
radio_frame_t radio_frame;
volatile float vbat;

volatile uint8_t sensor_error_count;
volatile uint8_t radio_error_count;
volatile uint8_t rf_error_count;

volatile _Bool flag_sensor;
volatile _Bool flag_radio;
volatile _Bool flag_vbat;
volatile _Bool flag_rf;
volatile _Bool flag_host;
volatile _Bool flag_status;
volatile _Bool flag_rf_host_read;
volatile _Bool flag_rf_rxtx_done;

host_buffer_rx_t host_buffer_rx;

volatile int32_t time_sensor;

/* Functions ------------------------------------------------*/

/* MAIN ----------------------------------------------------------------
-----------------------------------------------------------------------*/

int main(void)
{
	int i;
	_Bool error;

	_Bool flag_radio_connected;
	_Bool flag_armed;
	_Bool flag_acro;
	_Bool flag_acro_z;
#ifdef BEEPER
	_Bool flag_beep_radio;
#endif
#ifdef VBAT
	_Bool flag_vbat_first_value;
#endif
	_Bool flag_sensor_timeout;
	_Bool flag_radio_timeout;

	struct radio_raw_s radio_raw;
	struct radio_s radio;

	float radio_pitch_smooth;
	float radio_roll_smooth;

	struct sensor_s sensor;
	struct angle_s angle;

	uint8_t arm_test_z;

	float error_pitch;
	float error_roll;
	float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;

	float p_pitch;
	float i_pitch;
	float d_pitch;
	float p_roll;
	float i_roll;
	float d_roll;

	float pitch_p_term;
	float pitch_i_term;
	float pitch_d_term;
	float roll_p_term;
	float roll_i_term;
	float roll_d_term;
	float yaw_p_term;
	float yaw_i_term;
	float yaw_d_term;

	float pitch;
	float roll;
	float yaw;

	float motor[4];
	int16_t motor_clip[4];
	uint16_t motor_raw[4];
	_Bool motor_telemetry[4];

	float vbat_smoothed;

	uint8_t status_cnt;
	float time_sensor_mean;
	uint16_t time_sensor_max;
	uint16_t time_sensor_min;
	int32_t time_process;
	float time_process_mean;
	uint16_t time_process_max;
	uint16_t time_process_min;
	host_buffer_tx_t host_buffer_tx;

	/* Register init --------------------------------------------------*/

	reg_init();

	/* Variable initialisation -----------------------------------------------------*/

	flag_sensor = 0;
	flag_radio = 0;
	flag_vbat = 0;
	flag_rf = 0;
	flag_host = 0;
	flag_status = 0;
	flag_rf_host_read = 0;
	flag_rf_rxtx_done = 0;

	flag_radio_connected = 0;
	flag_armed = 0;
	flag_acro = 1;
	flag_acro_z = 0;
#ifdef BEEPER
	flag_beep_radio = 0;
#endif
#ifdef VBAT
	flag_vbat_first_value = 1;
#endif
	flag_sensor_timeout = 0;
	flag_radio_timeout = 0;

	sensor_error_count = 0;
	radio_error_count = 0;
	rf_error_count = 0;

	radio.throttle = 0;
	radio.pitch = 0;
	radio.roll = 0;
	radio.yaw = 0;
	radio.aux[0] = 0;
	radio.aux[1] = 0;

	radio_pitch_smooth = 0;
	radio_roll_smooth = 0;

	angle.pitch = 0;
	angle.roll = 0;

	arm_test_z = 0;

	error_pitch = 0;
	error_roll = 0;
	error_yaw = 0;

	p_pitch = REG_P_PITCH;
	i_pitch = REG_I_PITCH;
	d_pitch = REG_D_PITCH;
	p_roll  = REG_P_ROLL;
	i_roll  = REG_I_ROLL;
	d_roll  = REG_D_ROLL;

	pitch_i_term = 0;
	roll_i_term = 0;
	yaw_i_term = 0;

	pitch = 0;
	roll = 0;
	yaw = 0;

	vbat_smoothed = 0;

	status_cnt = 0;
	time_sensor_mean = 0;
	time_sensor_max = 0;
	time_sensor_min = 0xFFFF;
	time_process_mean = 0;
	time_process_max = 0;
	time_process_min = 0xFFFF;

	/* Init -----------------------------------------------------*/

	board_init();

	// Do not disable Systick interrupt, needed by reg write update
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

	/* Loop ----------------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/

	while (1)
	{
		// record processing time
		time_process = -get_timer_process();

		/* Process radio commands -----------------------------------------------------*/

		if (flag_radio) // new radio commands are ready
		{
			flag_radio = 0; // reset flag

			// Decode radio commands
			error = radio_decode(&radio_frame, &radio_raw, &radio);
			if (error) {
				radio_error_count++;
				radio_sync();
			}
			else
			{
				flag_radio_timeout = 0; // reset timeout flag
				flag_radio_connected = 1; // if we get here, there is a working radio receiver

				// Arm procedure: cannot arm when throttle is not low
				if (radio.aux[0] < 0.33f)
					flag_armed = 0;
				else if (!flag_armed && (radio.aux[0] > 0.33f) && ((radio.throttle < 0.02f)))
					flag_armed = 1;
				if (radio.aux[0] < 0.66f)
					flag_acro = 1;
				else
					flag_acro = 0;

				// Exponential
				radio_expo(&radio, flag_acro);

				// Beep command
#ifdef BEEPER
				if (radio.aux[1] > 0.33f)
					flag_beep_radio = 1;
				else
					flag_beep_radio = 0;
#endif
			}
		}

		/* Process sensors -----------------------------------------------------------------------*/

		if (flag_sensor) // new sensor samples are ready
		{
			flag_sensor = 0; // reset flag
			flag_sensor_timeout = 0; // reset timeout flag

			// Record sensor transaction time
			if (time_sensor < 0)
				time_sensor += 0x0000FFFF;
			time_sensor_mean += TIME_FILTER_ALPHA * (float)time_sensor - TIME_FILTER_ALPHA * time_sensor_mean;
			if ((uint16_t)time_sensor > time_sensor_max)
				time_sensor_max = (uint16_t)time_sensor;
			if ((uint16_t)time_sensor < time_sensor_min)
				time_sensor_min = (uint16_t)time_sensor;

			// Process sensor data
			mpu_process_samples(&sensor_raw, &sensor);

			// Estimate angle
			angle_estimate(&sensor, &angle);

			// Override armed and acro flags for test without radio
			if (REG_CTRL__ARM_TEST > 0) {
				flag_armed = 1;
				flag_acro = (REG_CTRL__ARM_TEST == 1);
				radio.throttle = 0; // Disable throttle
			}
			else if (arm_test_z > 0) {
				flag_armed = 0;
				flag_acro = 1;
			}
			arm_test_z = REG_CTRL__ARM_TEST;

			// Smooth pitch and roll commands in angle mode
			if (!flag_acro) {
				radio_pitch_smooth += filter_alpha_radio * radio.pitch - filter_alpha_radio * radio_pitch_smooth;
				radio_roll_smooth  += filter_alpha_radio * radio.roll  - filter_alpha_radio * radio_roll_smooth;
			}

			// Reset angle when entering angle mode
			if (!flag_acro && flag_acro_z) {
				angle.pitch = 0;
				angle.roll = 0;
			}

			/*------ PID ------*/

			// Previous error
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;

			// Current error
			if (flag_acro) {
				error_pitch = sensor.gyro_x - radio.pitch * (float)REG_RATE__PITCH_ROLL;
				error_roll = sensor.gyro_y - radio.roll * (float)REG_RATE__PITCH_ROLL;
			}
			else {
				error_pitch = angle.pitch - radio_pitch_smooth * (float)REG_RATE__ANGLE;
				error_roll = angle.roll - radio_roll_smooth * (float)REG_RATE__ANGLE;
			}
			error_yaw = sensor.gyro_z - radio.yaw * (float)REG_RATE__YAW;

			// Switch PID coefficients for acro
			if (flag_acro && !flag_acro_z) {
				p_pitch = REG_P_PITCH;
				i_pitch = REG_I_PITCH;
				d_pitch = REG_D_PITCH;
				p_roll  = REG_P_ROLL;
				i_roll  = REG_I_ROLL;
				d_roll  = REG_D_ROLL;
			}
			else if (!flag_acro && flag_acro_z) {
				p_pitch = REG_P_PITCH_ANGLE;
				i_pitch = REG_I_PITCH_ANGLE;
				d_pitch = REG_D_PITCH_ANGLE;
				p_roll  = REG_P_ROLL_ANGLE;
				i_roll  = REG_I_ROLL_ANGLE;
				d_roll  = REG_D_ROLL_ANGLE;
			}

			// P term
			pitch_p_term = error_pitch * p_pitch;
			roll_p_term = error_roll * p_roll;
			yaw_p_term = error_yaw * REG_P_ROLL;

			// Reset integral when disarmed or acro/angle mode change
			if (!flag_armed || (flag_acro != flag_acro_z)) {
				pitch_i_term = 0;
				roll_i_term = 0;
			}
			else {
				pitch_i_term += error_pitch * i_pitch;
				roll_i_term += error_roll * i_roll;
			}
			if (!flag_armed)
				yaw_i_term = 0;
			else
				yaw_i_term += error_yaw * REG_I_YAW;

			// D term
			pitch_d_term = (error_pitch - error_pitch_z) * d_pitch;
			roll_d_term = (error_roll - error_roll_z) * d_roll;
			yaw_d_term = (error_yaw - error_yaw_z) * REG_D_YAW;

			// Previous acro mode value
			flag_acro_z = flag_acro;

			// Clip I
			if      (pitch_i_term < -I_MAX) pitch_i_term = -I_MAX;
			else if (pitch_i_term >  I_MAX) pitch_i_term =  I_MAX;
			if      (roll_i_term  < -I_MAX) roll_i_term  = -I_MAX;
			else if (roll_i_term  >  I_MAX) roll_i_term  =  I_MAX;
			if      (yaw_i_term   < -I_MAX) yaw_i_term   = -I_MAX;
			else if (yaw_i_term   >  I_MAX) yaw_i_term   =  I_MAX;

			// P+I+D
			pitch = pitch_p_term + pitch_i_term + pitch_d_term;
			roll = roll_p_term + roll_i_term + roll_d_term;
			yaw = yaw_p_term + yaw_i_term + yaw_d_term;

			// Clip P+I+D
			if      (pitch < -PID_MAX) pitch = -PID_MAX;
			else if (pitch >  PID_MAX) pitch =  PID_MAX;
			if      (roll  < -PID_MAX) roll  = -PID_MAX;
			else if (roll  >  PID_MAX) roll  =  PID_MAX;
			if      (yaw   < -PID_MAX) yaw   = -PID_MAX;
			else if (yaw   >  PID_MAX) yaw   =  PID_MAX;

			/*------ Motor command ------*/

			// adapt yaw control if motor direction is reversed
			if (REG_MOTOR__REVERSED)
				yaw = -yaw;

			// Motor matrix
			motor[0] = radio.throttle * (float)REG_MOTOR__RANGE + roll + pitch - yaw;
			motor[1] = radio.throttle * (float)REG_MOTOR__RANGE + roll - pitch + yaw;
			motor[2] = radio.throttle * (float)REG_MOTOR__RANGE - roll - pitch - yaw;
			motor[3] = radio.throttle * (float)REG_MOTOR__RANGE - roll + pitch + yaw;

			// Offset and clip motor value
			for (i=0; i<4; i++) {
				motor_clip[i] = (int16_t)motor[i] + (int16_t)REG_MOTOR__ARMED;

				if (motor_clip[i] < (int16_t)REG_MOTOR__START)
					motor_clip[i] = (int16_t)REG_MOTOR__START;
				else if (motor_clip[i] > (int16_t)MOTOR_MAX)
					motor_clip[i] = (int16_t)MOTOR_MAX;
			}

			// Motor command
			for (i=0; i<4; i++) {
				if (REG_MOTOR_TEST__SELECT & (1 << i)) {
					motor_raw[i] = (uint16_t)REG_MOTOR_TEST__VALUE;
					motor_telemetry[i] = REG_MOTOR_TEST__TELEMETRY;
				}
				else if (flag_armed) {
					motor_raw[i] = (uint16_t)motor_clip[i];
					motor_telemetry[i] = 0;
				}
				else {
					motor_raw[i] = 0;
					motor_telemetry[i] = 0;
				}
			}
			set_motors(motor_raw, motor_telemetry);
		}

		/* VBAT ---------------------------------------------------------------------*/

#ifdef VBAT
		if (flag_vbat) // New vbat value is available
		{
			flag_vbat = 0; // reset flag

			if (flag_vbat_first_value) { // initialise low-pass filter
				flag_vbat_first_value = 0;
				vbat_smoothed = vbat;
			}
			else  // low-pass filter
				vbat_smoothed += filter_alpha_vbat * vbat - filter_alpha_vbat * vbat_smoothed;
		}
#endif

		/* Host request ------------------------------------------------------------------*/

		if (flag_host)
		{
			flag_host = 0; // reset flag

			reg_access(&host_buffer_rx);
		}

		/* Flight controller status: LED, beeper, timeout and debug output -----------------------------------------------*/

		if (flag_status)
		{
			flag_status = 0; // reset flag
			status_cnt++;

			// disarm when timeout on radio samples
			if (flag_radio_timeout)
				flag_armed = 0;

			// shut down motors when timeout on sensor samples
			if (flag_sensor_timeout) {
				for (i=0; i<4; i++) {
					motor_raw[i] = 0;
					motor_telemetry[i] = 0;
				}
				set_motors(motor_raw, motor_telemetry);
			}

			// Update status reg
			if (flag_sensor_timeout)
				REG_STATUS |= 0x01;
			if (flag_radio_timeout)
				REG_STATUS |= 0x02;
			if ((vbat_smoothed < REG_VBAT_MIN) && (vbat_smoothed > 3.0f))
				REG_STATUS |= 0x04;

			// LED double blinks when timeout on sensor or radio samples, or vbat too low
			if (flag_sensor_timeout || flag_radio_timeout || ((vbat_smoothed < REG_VBAT_MIN) && (vbat_smoothed > 3.0f))) {
				if ((status_cnt & 0x04))
					toggle_led();
			}
			else {
				if ((status_cnt & 0x03) == 0)
					toggle_led();
			}

			// beep when requested by user or, timeout on sensor or radio samples, or vbat too low
#ifdef BEEPER
			if (flag_beep_radio || flag_sensor_timeout || flag_radio_timeout || ((vbat_smoothed < REG_VBAT_MIN) && (vbat_smoothed > 3.0f)) || (REG_CTRL__BEEP_TEST == 1)) {
				if ((status_cnt & 0x03) == 0)
					toggle_beeper(1);
			}
			else
				toggle_beeper(0);
#endif

			// Set flags, to be cleared when new samples are ready
			flag_sensor_timeout = 1;
			if (flag_radio_connected && ((status_cnt & 0x03) == 0x03))
				flag_radio_timeout = 1;

			// Send data to host
			if (REG_CTRL__DEBUG) {
				host_buffer_tx.f[0] = sensor.gyro_x;
				host_buffer_tx.f[1] = sensor.gyro_y;
				host_buffer_tx.f[2] = sensor.gyro_z;
				host_buffer_tx.f[3] = sensor.accel_x;
				host_buffer_tx.f[4] = sensor.accel_y;
				host_buffer_tx.f[5] = sensor.accel_z;
				host_buffer_tx.f[6] = angle.pitch;
				host_buffer_tx.f[7] = angle.roll;
				host_buffer_tx.f[8] = radio.throttle;
				if (!flag_acro) {
					host_buffer_tx.f[9] = radio_pitch_smooth;
					host_buffer_tx.f[10] = radio_roll_smooth;
				}
				else {
					host_buffer_tx.f[9] = radio.pitch;
					host_buffer_tx.f[10] = radio.roll;
				}
				host_buffer_tx.f[11] = radio.yaw;
				host_buffer_tx.f[12] = radio.aux[0];
				host_buffer_tx.f[13] = radio.aux[1];
				host_buffer_tx.f[14] = vbat_smoothed;
				host_buffer_tx.f[15] = pitch;
				host_buffer_tx.f[16] = roll;
				host_buffer_tx.f[17] = yaw;
				host_buffer_tx.u16[18*2]   = motor_raw[0];
				host_buffer_tx.u16[18*2+1] = motor_raw[1];
				host_buffer_tx.u16[19*2]   = motor_raw[2];
				host_buffer_tx.u16[19*2+1] = motor_raw[3];
				host_buffer_tx.f[20] = time_sensor_mean;
				host_buffer_tx.u16[21*2]   = time_sensor_max;
				host_buffer_tx.u16[21*2+1] = time_sensor_min;
				host_buffer_tx.f[22] = time_process_mean;
				host_buffer_tx.u16[23*2]   = time_process_max;
				host_buffer_tx.u16[23*2+1] = time_process_min;
				host_send(host_buffer_tx.u8, 24*4);
			}

			// Reset min/max time
			time_sensor_max = 0;
			time_sensor_min = 0xFFFF;
			time_process_max = 0;
			time_process_min = 0xFFFF;
		}

		/* Wait for interrupts if all flags are processed ------------------------------------------------*/

		if (!flag_radio && !flag_sensor && !flag_vbat && !flag_host && !flag_status)
		{
			// Record processing time
			time_process += (int32_t)get_timer_process();
			if (time_process < 0)
				time_process += 0x0000FFFF;
			time_process_mean += TIME_FILTER_ALPHA * (float)time_process - TIME_FILTER_ALPHA * time_process_mean;
			if ((uint16_t)time_process > time_process_max)
				time_process_max = (uint16_t)time_process;
			if ((uint16_t)time_process < time_process_min)
				time_process_min = (uint16_t)time_process;

			__WFI();
		}
	}
}
