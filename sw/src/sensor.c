#include "sensor.h"
#include "sensor_reg.h"
#include "fc.h" // flags
#include "board.h" // toggle_led_sensor
#include "reg.h" // alpha coeff
#include "utils.h" // wait_ms()
#ifdef STM32F4
	#include "stm32f4xx.h" // __WFI()
#else
	#include "stm32f3xx.h" // __WFI()
#endif

/* Private defines --------------------------------------*/

#define MPU_GYRO_SCALE 0.061035f
#define MPU_ACCEL_SCALE 0.00048828f
#define SENSOR_SETTLING_TIME 500 // ms

/* Private macros --------------------------------------*/

/* Global variables ----------------------------------*/

/* Function definitions ----------------------------------*/

void mpu_init(void)
{
	wait_ms(SENSOR_SETTLING_TIME);
	sensor_write(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
#if (SENSOR != 6050)
	sensor_write(MPU_SIGNAL_PATH_RST, MPU_SIGNAL_PATH_RST__ACCEL_RST | MPU_SIGNAL_PATH_RST__GYRO_RST | MPU_SIGNAL_PATH_RST__TEMP_RST);
	wait_ms(100);
	sensor_write(MPU_USER_CTRL, MPU_USER_CTRL__I2C_IF_DIS);
#endif
	sensor_write(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__CLKSEL(1));// | MPU_PWR_MGMT_1__TEMP_DIS); // Get MPU out of sleep, set CLK = gyro X clock, and disable temperature sensor
	wait_ms(100);
	//sensor_write(MPU_PWR_MGMT_2, MPU_PWR_MGMT_2__STDBY_XA | MPU_PWR_MGMT_2__STDBY_YA | MPU_PWR_MGMT_2__STDBY_ZA); // Disable accelerometers
	sensor_write(MPU_SMPLRT_DIV, REG_MPU_CFG__RATE); // Sample rate = Fs/(x+1)
	sensor_write(MPU_CFG, MPU_CFG__DLPF_CFG(REG_MPU_CFG__FILT)); // Filter ON => Fs=1kHz, else 8kHz
	sensor_write(MPU_GYRO_CFG, MPU_GYRO_CFG__FS_SEL(3)); // Full scale = +/-2000 deg/s
	sensor_write(MPU_ACCEL_CFG, MPU_ACCEL_CFG__AFS_SEL(3)); // Full scale = +/- 16g
	//wait_ms(100); // wait for filter to settle
	sensor_write(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
}

void mpu_process_samples(sensor_raw_t * sensor_raw, struct sensor_s * sensor)
{
	int i;
	uint8_t x;

	for (i=1; i<15; i=i+2){
		x = sensor_raw->bytes[i+1];
		sensor_raw->bytes[i+1] = sensor_raw->bytes[i];
		sensor_raw->bytes[i] = x;
	}

	#if (SENSOR_ORIENTATION == 90)
		sensor->gyro_y = -(float)(sensor_raw->sensor.gyro_x - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__X)) * MPU_GYRO_SCALE;
		sensor->gyro_x = -(float)(sensor_raw->sensor.gyro_y - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__Y)) * MPU_GYRO_SCALE;
		sensor->accel_x =  (float)(sensor_raw->sensor.accel_x - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__X)) * MPU_ACCEL_SCALE;
		sensor->accel_y = -(float)(sensor_raw->sensor.accel_y - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__Y)) * MPU_ACCEL_SCALE;
	#elif (SENSOR_ORIENTATION == 180)
		sensor->gyro_x =  (float)(sensor_raw->sensor.gyro_x - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__X)) * MPU_GYRO_SCALE;
		sensor->gyro_y = -(float)(sensor_raw->sensor.gyro_y - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__Y)) * MPU_GYRO_SCALE;
		sensor->accel_y = (float)(sensor_raw->sensor.accel_x - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__X)) * MPU_ACCEL_SCALE;
		sensor->accel_x = (float)(sensor_raw->sensor.accel_y - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__Y)) * MPU_ACCEL_SCALE;
	#else
		sensor->gyro_x = -(float)(sensor_raw->sensor.gyro_x - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__X)) * MPU_GYRO_SCALE;
		sensor->gyro_y =  (float)(sensor_raw->sensor.gyro_y - (int16_t)uint32_to_int32(REG_GYRO_DC_XY__Y)) * MPU_GYRO_SCALE;
		sensor->accel_y = -(float)(sensor_raw->sensor.accel_x - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__X)) * MPU_ACCEL_SCALE;
		sensor->accel_x = -(float)(sensor_raw->sensor.accel_y - (int16_t)uint32_to_int32(REG_ACCEL_DC_XY__Y)) * MPU_ACCEL_SCALE;
	#endif
	sensor->gyro_z = -(float)(sensor_raw->sensor.gyro_z - (int16_t)uint32_to_int32(REG_GYRO_DC_Z)) * MPU_GYRO_SCALE;
	sensor->accel_z = (float)(sensor_raw->sensor.accel_z - (int16_t)uint32_to_int32(REG_ACCEL_DC_Z)) * MPU_ACCEL_SCALE;
	sensor->temperature = (float)sensor_raw->sensor.temperature / 340.0f + 36.53f;
}

void mpu_cal(sensor_raw_t * sensor_raw)
{
	uint16_t sensor_sample_count = 0;
	int i;
	uint8_t x;

	float gyro_x_dc = 0;
	float gyro_y_dc = 0;
	float gyro_z_dc = 0;
	float accel_x_dc = 0;
	float accel_y_dc = 0;
	float accel_z_dc = 0;

	while (sensor_sample_count < 1000)
	{
		if (flag_sensor)
		{
			flag_sensor = 0;

			for (i=1; i<15; i=i+2){
				x = sensor_raw->bytes[i+1];
				sensor_raw->bytes[i+1] = sensor_raw->bytes[i];
				sensor_raw->bytes[i] = x;
			}

			gyro_x_dc += (float)sensor_raw->sensor.gyro_x;
			gyro_y_dc += (float)sensor_raw->sensor.gyro_y;
			gyro_z_dc += (float)sensor_raw->sensor.gyro_z;
			accel_x_dc += (float)sensor_raw->sensor.accel_x;
			accel_y_dc += (float)sensor_raw->sensor.accel_y;
			accel_z_dc += (float)sensor_raw->sensor.accel_z;

			if ((sensor_sample_count & 0x1F) == 0)
				toggle_led();

			sensor_sample_count++;
		}
		__WFI();
	}

	REG_GYRO_DC_XY = int32_to_uint32((int32_t)(gyro_x_dc / 1000.0f)) + (int32_to_uint32((int32_t)(gyro_y_dc / 1000.0f)) << 16);
	REG_GYRO_DC_Z = int32_to_uint32((int32_t)(gyro_z_dc / 1000.0f));
	REG_ACCEL_DC_XY = int32_to_uint32((int32_t)(accel_x_dc / 1000.0f)) + (int32_to_uint32((int32_t)(accel_y_dc / 1000.0f)) << 16);
	REG_ACCEL_DC_Z = int32_to_uint32((int32_t)(accel_z_dc / 1000.0f - 1.0f/MPU_ACCEL_SCALE));
}

void angle_estimate(struct sensor_s * sensor, struct angle_s * angle)
{
	float vector_magnitude;
	float angle_transfer;
	float x;

	// Integrate gyro rate and wrap angle
	x = angle->pitch + (sensor->gyro_x / sensor_rate);
	/*if (x > 180.0f)
		angle->pitch = x - 360.0f;
	else if (x < -180.0f)
		angle->pitch = x + 360.0f;
	else*/
		angle->pitch = x;
	x = angle->roll + (sensor->gyro_y / sensor_rate);
	/*if (x > 180.0f)
		angle->roll = x - 360.0f;
	else if (x < -180.0f)
		angle->roll = x + 360.0f;
	else*/
		angle->roll = x;

	// Angles form accelerometers
	vector_magnitude = sqrt(sensor->accel_x * sensor->accel_x + sensor->accel_y * sensor->accel_y + sensor->accel_z * sensor->accel_z);
	angle->pitch_from_accel = 57.2958f * ARCSINUS(sensor->accel_x / vector_magnitude);
	angle->roll_from_accel  = 57.2958f * ARCSINUS(sensor->accel_y / vector_magnitude);
	/*if (angle->pitch >= 90)
		angle->pitch_from_accel = -angle->pitch_from_accel + 180.0f;
	else if (angle->pitch <= -90)
		angle->pitch_from_accel = -angle->pitch_from_accel - 180.0f;
	if (angle->roll >= 90)
		angle->roll_from_accel = -angle->roll_from_accel + 180.0f;
	else if (angle->roll <= -90)
		angle->roll_from_accel = -angle->roll_from_accel - 180.0f;*/

	// Combine gyro and accel angles
	angle->pitch += filter_alpha_accel * angle->pitch_from_accel - filter_alpha_accel * angle->pitch;
	angle->roll  += filter_alpha_accel * angle->roll_from_accel  - filter_alpha_accel * angle->roll;
	/*if (abs(angle->pitch - angle->pitch_from_accel) > 135)
		angle->pitch = angle->pitch_from_accel;
	if (abs(angle->roll - angle->roll_from_accel) > 135)
		angle->roll = angle->roll_from_accel;*/

	// Yaw induced angle transfer
	angle_transfer = SINUS(sensor->gyro_z * 1.745329252e-5f);
	angle->pitch += angle->roll  * angle_transfer;
	angle->roll  -= angle->pitch * angle_transfer;
}
