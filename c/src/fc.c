#include "fc.h"
#include "board.h" // board_init()
#include "usb.h" // usb_init()
#include "sensor.h" // mpu_process_samples()
#include "radio.h" // radio_decode()
#include "reg.h" // REG__*
#include "utils.h" // SINUS
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif
#include <string.h> // memcpy()
#include "osd.h" // osd_menu()
#include "smart_audio.h" // sma_send_cmd()

/* Private defines ------------------------------------*/

#if (ESC == DSHOT)
	#define MOTOR_MAX 2047
#else
	#define MOTOR_MAX SERVO_MAX
#endif
#define I_MAX 300.0f
#define PID_MAX 600.0f
#define STATUS_PERIOD 100 // ms

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

sensor_raw_t sensor_raw;
radio_frame_t radio_frame;
volatile float vbat = 0;
volatile float ibat = 0;

volatile uint8_t sensor_error_count = 0;
volatile uint8_t radio_error_count = 0;
volatile uint8_t rf_error_count = 0;

volatile _Bool flag_sensor = 0;
volatile _Bool flag_radio = 0;
volatile _Bool flag_vbat = 0;
volatile _Bool flag_rf = 0;
volatile _Bool flag_host = 0;
volatile _Bool flag_time = 0;
volatile _Bool flag_rf_host_read = 0;
volatile _Bool flag_rf_rxtx_done = 0;

host_buffer_rx_t host_buffer_rx;

volatile uint32_t t_ms;

/* Private functions ------------------------------------------------*/

/* MAIN ----------------------------------------------------------------
-----------------------------------------------------------------------*/

int main(void)
{
	uint8_t i;
	int8_t status;

	_Bool flag_radio_connected = 0;
	_Bool flag_armed = 0;
	_Bool flag_acro = 0;
	_Bool flag_acro_z = 0;
	_Bool flag_sensor_timeout = 0;
	_Bool flag_radio_timeout = 0;

	radio_frame_t radio_frame1;
	struct radio_s radio;

	float radio_pitch_smooth = 0;
	float radio_roll_smooth = 0;

	struct sensor_s sensor;
	struct angle_s angle;

	uint8_t arm_test_z = 0;

	float error_pitch = 0;
	float error_roll = 0;
	float error_yaw = 0;
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
	float pitch_i_term = 0;
	float pitch_d_term;
	float roll_p_term;
	float roll_i_term = 0;
	float roll_d_term;
	float yaw_p_term;
	float yaw_i_term = 0;
	float yaw_d_term;

	float i_transfer;

	float pitch = 0;
	float roll = 0;
	float yaw = 0;

	float motor[4];
	int16_t motor_clip[4];
	uint16_t motor_raw[4];
	_Bool motor_telemetry[4];
	uint8_t motor_period_cnt = 0;

	float vbat_smoothed;
	float nb_cells;
	float vbat_cell;
	float ibat_smoothed;
	float imah = 0;

	uint16_t cnt_ms = 0;
	uint8_t t_s = 0;
	uint8_t t_min = 0;
	uint32_t t_status_prev = 0;
	uint8_t status_cnt = 0;

	host_buffer_tx_t host_buffer_tx;

	uint16_t t_proc_0;
	uint16_t t_proc;
	uint16_t t_proc_max = 0;
#ifdef LED
	uint8_t grb[3];
#endif
	_Bool led_en = 1;

	/* Init -----------------------------------------------------*/

	radio.throttle = 0;
	radio.pitch = 0;
	radio.roll = 0;
	radio.yaw = 0;
	radio.aux[0] = 0;
	radio.aux[1] = 0;
	radio.aux[2] = 0;
	radio.aux[3] = 0;
	radio.rssi = 0;
	radio.snr = 0;

	angle.pitch = 0;
	angle.roll = 0;

	board_init();

	usb_init();

	reg_init();
	p_pitch = REG_P_PITCH;
	i_pitch = REG_I_PITCH;
	d_pitch = REG_D_PITCH;
	p_roll  = REG_P_ROLL;
	i_roll  = REG_I_ROLL;
	d_roll  = REG_D_ROLL;

	wait_ms(1000); // Wait for regulators to settle

	mpu_init();

	trig_radio_rx();

#ifdef VBAT
	// Get first vbat value
	trig_vbat_meas();
	while (!flag_vbat) __WFI();
	flag_vbat = 0;
#endif

	vbat_smoothed = vbat;
	nb_cells = floor(vbat / 3.6f);
	vbat_cell = vbat / nb_cells;
	ibat_smoothed = ibat;

#ifdef OSD
	osd_init();
#endif

#ifdef SMART_AUDIO
	sma_send_cmd(SMA_GET_SETTINGS, 0);
	//wait_sma();
	wait_ms(1); // In case VTX is not powered by USB
	sma_process_resp();
	REG_VTX = (vtx_current_chan << REG_VTX__CHAN_Pos) | (vtx_current_pwr << REG_VTX__PWR_Pos);
#endif

	en_sensor_irq();

	t_proc_0 = get_t_us();

	/* Loop ----------------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/

	while (1)
	{
		/* Process radio commands -----------------------------------------------------*/

		if (flag_radio) // new radio commands are ready
		{
			flag_radio = 0; // reset flag

			// Decode radio commands
			status = radio_decode(&radio_frame, &radio);
			radio_frame1 = radio_frame; // record buffer for debug

			if (status == RADIO_FRAME_ERROR)
				radio_error_count++;

			if (status == RADIO_FRAME_RC_CHAN) {
				trig_radio_rx(); // to get next frame
				flag_radio_timeout = 0; // reset timeout flag
				flag_radio_connected = 1; // if we get here, there is a working radio receiver

				// Arm procedure: cannot arm when throttle is not low
				if (radio.aux[0] < 0.66f)
					flag_armed = 0;
				else if (!flag_armed && (radio.aux[0] > 0.66f) && ((radio.throttle < 0.02f)))
					flag_armed = 1;
				if (radio.aux[1] < 0.66f)
					flag_acro = 1;
				else
					flag_acro = 0;

			#ifdef OSD
				// OSD menu navigation from stick moves
				if (!flag_armed)
					osd_menu(&radio);
			#endif
				
				// Exponential
				radio_expo(&radio, flag_acro);
			} else {
				trig_delayed_radio_rx();
			}
		}

		/* Process sensors -----------------------------------------------------------------------*/

		if (flag_sensor) // new sensor samples are ready
		{
			flag_sensor = 0; // reset flag
			flag_sensor_timeout = 0; // reset timeout flag

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

			// I term, reset when disarmed or acro/angle mode change
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

			// Yaw induced I transfer
			if (REG_FC_CFG__I_TRANSFER) {
				i_transfer = SINUS(sensor.gyro_z * 1.745329252e-5f);
				pitch_i_term += pitch_i_term  * i_transfer;
				roll_i_term  -= roll_i_term * i_transfer;
			}

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

			// Motor matrix
		#ifdef M1_CCW
			motor[0] = radio.throttle * (float)REG_MOTOR__RANGE + roll + pitch - yaw;
			motor[1] = radio.throttle * (float)REG_MOTOR__RANGE + roll - pitch + yaw;
			motor[2] = radio.throttle * (float)REG_MOTOR__RANGE - roll - pitch - yaw;
			motor[3] = radio.throttle * (float)REG_MOTOR__RANGE - roll + pitch + yaw;
		#else
			motor[0] = radio.throttle * (float)REG_MOTOR__RANGE + roll + pitch + yaw;
			motor[1] = radio.throttle * (float)REG_MOTOR__RANGE + roll - pitch - yaw;
			motor[2] = radio.throttle * (float)REG_MOTOR__RANGE - roll - pitch + yaw;
			motor[3] = radio.throttle * (float)REG_MOTOR__RANGE - roll + pitch - yaw;
		#endif

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
			if (motor_period_cnt == REG_FC_CFG__MOTOR_PERIOD-1) {
				motor_period_cnt = 0;
				set_motors(motor_raw, motor_telemetry);
			} else
				motor_period_cnt++;
		}

		/* VBAT ---------------------------------------------------------------------*/

		if (flag_vbat) // New vbat value is available
		{
			flag_vbat = 0; // reset flag

			vbat_smoothed += filter_alpha_vbat * vbat - filter_alpha_vbat * vbat_smoothed; // low-pass filter
			vbat_cell = vbat_smoothed / nb_cells;
			ibat_smoothed += filter_alpha_ibat * ibat - filter_alpha_ibat * ibat_smoothed; // low-pass filter
			imah += ibat_smoothed / 3600.0f;
		}

		/* Flight controller status: LED, beeper, timeout and debug output -----------------------------------------------*/

		if (flag_time)
		{
			flag_time = 0; // reset flag

			// Flight time
			if (flag_armed) {
				cnt_ms++;
				if (cnt_ms == 1000) {
					cnt_ms = 0;
					t_s++;
					if (t_s == 60) {
						t_s = 0;
						t_min++;
					}
				}
			}
			
		#ifdef VBAT
			// Measure Vbat every ms
			trig_vbat_meas();
		#endif


			if ((t_ms - t_status_prev) >= STATUS_PERIOD) {

				t_status_prev = t_ms;

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
				REG_STATUS = 0;
				if (flag_sensor_timeout)
					REG_STATUS |= 0x01;
				if (flag_radio_timeout)
					REG_STATUS |= 0x02;
				if ((vbat_cell < REG_VBAT_MIN) && (nb_cells > 0))
					REG_STATUS |= 0x04;

				// LED double blinks when timeout on sensor or radio samples, or vbat too low
				if (flag_sensor_timeout || flag_radio_timeout || ((vbat_cell < REG_VBAT_MIN) && (nb_cells > 0))) {
					#ifdef DUAL_LED_STATUS
						if ((status_cnt & 0x03) == 0) toggle_led(1);
						if ((status_cnt & 0x03) == 2) toggle_led2(1);
					#else
						toggle_led(1);
					#endif
				}
				else {
					if ((status_cnt & 0x03) == 0) toggle_led(1);
					toggle_led2(0);
				}

				// beep when requested by user or, timeout on sensor or radio samples, or vbat too low
				if ((radio.aux[2] > 0.5f) || flag_sensor_timeout || flag_radio_timeout || ((vbat_cell < REG_VBAT_MIN) && (nb_cells > 0)) || (REG_CTRL__BEEP_TEST == 1)) {
					if ((status_cnt & 0x03) == 0) {
						toggle_beeper(1);
						led_en ^= 1;
					}
				}
				else {
					toggle_beeper(0);
					led_en = 1;
				}

				// Set flags, to be cleared when new samples are ready
				flag_sensor_timeout = 1;
				if (flag_radio_connected)
					flag_radio_timeout = 1;

			#ifdef OSD
				// OSD telemetry
				osd_telemetry(vbat_cell, ibat_smoothed, imah, t_s, t_min, radio.rssi, radio.snr);
			#endif

			#ifdef LED
				// Change LED color
				if (radio.aux[3] < 0.5f) {
					grb[0] = (uint8_t)((0.5f-radio.aux[3])*510.0f) * led_en;
					grb[1] = (uint8_t)(radio.aux[3]*510.0f) * led_en;
					grb[2] = 0;
				}
				else {
					grb[0] = 0;
					grb[1] = (uint8_t)((1.0f-radio.aux[3])*510.0f) * led_en;
					grb[2] = (uint8_t)((radio.aux[3]-0.5f)*510.0f) * led_en;
				}
				set_leds(grb);
			#endif

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
					host_buffer_tx.f[14] = radio.aux[2];
					host_buffer_tx.f[15] = radio.aux[3];
					host_buffer_tx.f[16] = vbat_smoothed;
					host_buffer_tx.f[17] = ibat_smoothed;
					host_buffer_tx.f[18] = pitch;
					host_buffer_tx.f[19] = roll;
					host_buffer_tx.f[20] = yaw;
					host_buffer_tx.u16[21*2]   = motor_raw[0];
					host_buffer_tx.u16[21*2+1] = motor_raw[1];
					host_buffer_tx.u16[22*2]   = motor_raw[2];
					host_buffer_tx.u16[22*2+1] = motor_raw[3];
					host_buffer_tx.u16[23*2]   = t_proc_max;
					host_send(host_buffer_tx.u8, 21*4 + 5*2);
				}
				else if (REG_CTRL__DEBUG_RADIO) {
					memcpy(host_buffer_tx.u8, radio_frame1.bytes, sizeof(radio_frame1));
					host_send(host_buffer_tx.u8, sizeof(radio_frame1));
				}

				// Reset max processing time
				t_proc_max = 0;
			}
		}

		/* Host request ------------------------------------------------------------------*/

		if (flag_host)
		{
			flag_host = 0; 

			reg_access(&host_buffer_rx);
		}

		/* Wait for interrupts if all flags are processed ------------------------------------------------*/

		if (!flag_radio && !flag_sensor && !flag_vbat && !flag_time && !flag_host)
		{
			// Record processing time
			t_proc = get_t_us() - t_proc_0;
			if (t_proc > t_proc_max)
				t_proc_max = t_proc;

			// Reset processing time measurement
			t_proc_0 = get_t_us();

			//__WFI(); // Can potentially mask a flag that have just been raised after the if condition
		}
	}
}
