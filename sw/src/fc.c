#include <stdint.h>
#include <string.h> // memcpy
#include "fc.h"
#include "board.h"
#include "sensor.h"
#include "radio.h"
#include "reg.h"

/* Private defines ------------------------------------*/

#define I_MAX 300.0f
#define PID_MAX 600.0f
#define RECOVERY_TIME 3000 // ms

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

sensor_raw_t sensor_raw;
radio_frame_t radio_frame;

volatile uint8_t sensor_error_count;
volatile uint8_t radio_error_count;
volatile uint8_t rf_error_count;

volatile _Bool flag_sensor;
volatile _Bool flag_radio;
volatile _Bool flag_vbat;
volatile _Bool flag_rf;
volatile _Bool flag_host;
volatile _Bool flag_sensor_host_read;
volatile _Bool flag_rf_host_read;
volatile _Bool flag_timeout_sensor;
volatile _Bool flag_timeout_radio;
volatile _Bool flag_armed;
volatile _Bool flag_acro;
volatile _Bool flag_rf_rxtx_done;
volatile _Bool flag_rf_host_read;

volatile _Bool flag_beep_user;
volatile _Bool flag_beep_radio;
volatile _Bool flag_beep_sensor;
volatile _Bool flag_beep_host;
volatile _Bool flag_beep_vbat;

host_buffer_rx_t host_buffer_rx;

uint16_t timer_sensor[2];
uint16_t time_sensor;
uint16_t time_process;

/* Functions ------------------------------------------------*/

/* MAIN ----------------------------------------------------------------
-----------------------------------------------------------------------*/

int main(void)
{
	int i;
	
	uint16_t sensor_sample_count;
	struct sensor_s sensor;
	struct angle_s angle;
	
	uint16_t radio_frame_count;
	struct radio_raw_s radio_raw;
	struct radio_s radio;
	float radio_pitch_smooth;
	float radio_roll_smooth;
	
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
	int32_t motor_clip[4];
	uint32_t motor_raw[4];
	
	uint16_t vbat_sample_count;
	
	int32_t t1;
	int32_t t2;
	_Bool flag_acro_z;
	_Bool error;
	uint16_t sensor_sample_count1;
	
	host_buffer_tx_t host_buffer_tx;
	
	/* Variable initialisation -----------------------------------------------------*/
	
	flag_sensor = 0;
	flag_radio = 0;
	flag_vbat = 0;
	flag_rf = 0;
	flag_host = 0;
	flag_sensor_host_read = 0;
	flag_rf_host_read = 0;
	flag_timeout_sensor = 0;
	flag_timeout_radio = 0;
	flag_armed = 0;
	flag_acro = 1;
	flag_acro_z = 0;

	flag_beep_user = 0;
	flag_beep_radio = 0;
	flag_beep_sensor = 0;
	flag_beep_host = 0;
	flag_beep_vbat = 0;
	
	sensor_sample_count = 0;
	sensor_error_count = 0;
	angle.pitch = 0;
	angle.roll = 0;
	
	radio_frame_count = 0;
	radio_error_count = 0;
	radio_pitch_smooth = 0;
	radio_roll_smooth = 0;
	
	pitch_i_term = 0;
	roll_i_term = 0;
	yaw_i_term = 0;
	
	vbat_sample_count = 0;
	
	rf_error_count = 0;
	
	sensor_sample_count1 = 0;
	
	/* Setup -----------------------------------------------------*/
	
	board_init(); // BOARD_DEPENDENT
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // Disable Systick interrupt, not needed anymore (but can still use COUNTFLAG)
	reg_init();
	
	/* Loop ----------------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/
	
	while (1)
	{
		// Processing time
		t1 = (int32_t)get_timer_process();
		
		/* Process radio commands -----------------------------------------------------*/
		
		if (flag_radio)
		{
			flag_radio = 0;
			
			// Decode radio commands
			error = radio_decode(&radio_frame, &radio_raw, &radio);
			if (error)
				radio_error_recover();
			else
			{
				reset_timeout_radio();
				flag_beep_radio = 0; // Stop beeping
				radio_frame_count++;
			
				// Arm procedure
				if (radio.aux[0] < 0.33f)
					flag_armed = 0;
				else if (!flag_armed && (radio.aux[0] > 0.33f) && ((radio.throttle < 0.01f)))
					flag_armed = 1;
				if (radio.aux[0] < 0.66f)
					flag_acro = 1;
				else
					flag_acro = 0;
				
				// Expo and smooth
				radio_expo(&radio, flag_acro);
				
				// Beep if requested
				if (radio.aux[1] > 0.33f)
					flag_beep_user = 1;
				else
					flag_beep_user = 0;
				
				// Send data to host
				if ((REG_DEBUG__CASE > 0) && ((radio_frame_count & REG_DEBUG__MASK) == 0)) {
					if (REG_DEBUG__CASE == 4)
						host_send((uint8_t*)&radio_raw, sizeof(radio_raw));
					else if (REG_DEBUG__CASE == 5)
						host_send((uint8_t*)&radio, sizeof(radio));
				}
				
				// Toggle LED at rate of Radio flag
				if ((radio_frame_count & 0x3F) == 0)
					toggle_led_radio();
			}
		}
		
		/* Process sensors -----------------------------------------------------------------------*/
		
		if (flag_sensor)
		{
			flag_sensor = 0;
			
			sensor_sample_count++;
			flag_beep_sensor = 0; // Disable beeping
			
			// Record sensor transaction time
			t2 = (int32_t)timer_sensor[1] - (int32_t)timer_sensor[0];
			if (t2 < 0)
				t2 += 0xFFFF;
			if ((REG_CTRL__TIME_MAXHOLD == 0) || (((uint16_t)t2 > time_sensor) && REG_CTRL__TIME_MAXHOLD))
				time_sensor = (uint16_t)t2;
			
			// Recovery time before activating yaw agnle transfer
			if (flag_acro != flag_acro_z)
				sensor_sample_count1 = 0;
			else if (sensor_sample_count1 < RECOVERY_TIME)
				sensor_sample_count1++;
			
			// Procees sensor data
			mpu_process_samples(&sensor_raw, &sensor);
			
			// Estimate angle
			angle_estimate(&sensor, &angle, (sensor_sample_count1 == RECOVERY_TIME));
			
			// Smooth pitch and roll commands in angle mode
			if (!flag_acro) {
				radio_pitch_smooth += filter_alpha_radio * radio.pitch - filter_alpha_radio * radio_pitch_smooth;
				radio_roll_smooth  += filter_alpha_radio * radio.roll  - filter_alpha_radio * radio_roll_smooth;
			}
			
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
			if (flag_acro != flag_acro_z) {
				if (flag_acro) {
					p_pitch = REG_P_PITCH;
					i_pitch = REG_I_PITCH;
					d_pitch = REG_D_PITCH;
					p_roll  = REG_P_ROLL;
					i_roll  = REG_I_ROLL;
					d_roll  = REG_D_ROLL;
				}
				else {
					p_pitch = REG_P_PITCH_ANGLE;
					i_pitch = REG_I_PITCH_ANGLE;
					d_pitch = REG_D_PITCH_ANGLE;
					p_roll  = REG_P_ROLL_ANGLE;
					i_roll  = REG_I_ROLL_ANGLE;
					d_roll  = REG_D_ROLL_ANGLE;
				}
			}
			
			// P,I and D
			pitch_p_term = error_pitch * p_pitch;
			roll_p_term = error_roll * p_roll;
			yaw_p_term = error_yaw * REG_P_ROLL;
			
			if ((!flag_armed && (REG_CTRL__ARM_TEST == 0)) || (flag_acro != flag_acro_z)) {
				pitch_i_term = 0;
				roll_i_term = 0;
			}
			else {
				pitch_i_term += error_pitch * i_pitch;
				roll_i_term += error_roll * i_roll;
			}
			if (!flag_armed && (REG_CTRL__ARM_TEST == 0))
				yaw_i_term = 0;
			else
				yaw_i_term += error_yaw * REG_I_YAW;
			
			pitch_d_term = (error_pitch - error_pitch_z) * d_pitch;
			roll_d_term = (error_roll - error_roll_z) * d_roll;
			yaw_d_term = (error_yaw - error_yaw_z) * REG_D_YAW;
			
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
			
			// Desactivate throttle when arm test
			if (REG_CTRL__ARM_TEST > 0)
				radio.throttle = 0;
			
			// Motor matrix
			motor[0] = radio.throttle * (float)REG_MOTOR__RANGE + roll + pitch - yaw;
			motor[1] = radio.throttle * (float)REG_MOTOR__RANGE + roll - pitch + yaw;
			motor[2] = radio.throttle * (float)REG_MOTOR__RANGE - roll - pitch - yaw;
			motor[3] = radio.throttle * (float)REG_MOTOR__RANGE - roll + pitch + yaw;
			
			// Offset and clip motor value
			for (i=0; i<4; i++) {
				motor_clip[i] = (int32_t)motor[i] + (int32_t)REG_MOTOR__ARMED;
				
				if (motor_clip[i] < (int32_t)REG_MOTOR__START)
					motor_clip[i] = (int32_t)REG_MOTOR__START;
				else if (motor_clip[i] > (int32_t)MOTOR_MAX)
					motor_clip[i] = (int32_t)MOTOR_MAX;
			}
			
			// Motor command
			for (i=0; i<4; i++) {
				if (REG_MOTOR_TEST__SELECT & (1 << i))
					motor_raw[i] = (uint32_t)REG_MOTOR_TEST__VALUE;
				else if (flag_armed || (REG_CTRL__ARM_TEST > 0))
					motor_raw[i] = (uint32_t)motor_clip[i];
				else
					motor_raw[i] = 0;
			}
			set_motors(motor_raw);
			
			// Send data to host
			if ((REG_DEBUG__CASE > 0) && ((sensor_sample_count & REG_DEBUG__MASK) == 0)) {
				if (REG_DEBUG__CASE == 1) 
					host_send((uint8_t*)&sensor_raw.bytes[2], sizeof(sensor_raw)-2);
				else if (REG_DEBUG__CASE == 2)
					host_send((uint8_t*)&sensor, sizeof(sensor));
				else if (REG_DEBUG__CASE == 3)
					host_send((uint8_t*)&angle, sizeof(angle));
				else if (REG_DEBUG__CASE == 6) {
					host_buffer_tx.f[0] = pitch;
					host_buffer_tx.f[1] = roll;
					host_buffer_tx.f[2] = yaw;
					host_send(host_buffer_tx.u8, 3*4);
				}
				else if (REG_DEBUG__CASE == 7)
					host_send((uint8_t*)&motor_raw, sizeof(motor_raw));
			}
			
			// Toggle LED at rate of sensor flag
			if ((sensor_sample_count & 0x01FF) == 0)
				toggle_led_sensor();
		}
		
		/* VBAT ---------------------------------------------------------------------*/
		
		if (flag_vbat)
		{
			flag_vbat = 0;
			
			vbat_sample_count++;
			
			REG_VBAT += filter_alpha_vbat * get_vbat() - filter_alpha_vbat * REG_VBAT;
			
			// Send VBAT to host
			if ((REG_DEBUG__CASE == 8) && ((vbat_sample_count & REG_DEBUG__MASK) == 0))
				host_send((uint8_t*)&REG_VBAT,4);
			
			// Beep if VBAT too low
			if ((REG_VBAT < REG_VBAT_MIN) && (REG_VBAT > 8.0f))
				flag_beep_vbat = 1;
			else
				flag_beep_vbat = 0;
		}
		
		/* Host requests ------------------------------------------------------------------*/
		
		if (flag_host)
		{
			flag_host = 0;
			reg_access(&host_buffer_rx);
		}
		
		/* Handle timeout -----------------------------------------------------------------------*/
		
		if (flag_timeout_sensor) {
			flag_timeout_sensor = 0;
			for (i=0; i<4; i++)
				motor_raw[i] = 0;
			set_motors(motor_raw);
			flag_beep_sensor = 1;
		}
		if (flag_timeout_radio) {
			flag_timeout_radio = 0;
			flag_armed = 0;
			flag_beep_radio = 1;
		}
		
		/*------------------------------------------------------------------*/
		
		// Record processing time
		t1 = (int32_t)get_timer_process() - t1;
		if (t1 < 0)
			t1 += 0xFFFF;
		if ((REG_CTRL__TIME_MAXHOLD == 0) || (((uint16_t)t1 > time_process) && REG_CTRL__TIME_MAXHOLD))
			time_process = (uint16_t)t1;
		
		// Wait for interrupts if all flags are processed
		if (!flag_radio && !flag_sensor && !flag_vbat && !flag_host && !flag_timeout_sensor && !flag_timeout_radio)
			__wfi();
	}
}
