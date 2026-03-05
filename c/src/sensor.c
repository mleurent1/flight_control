#include <stdint.h>
#include <stdbool.h>

#include "sensor.h"
#include "mpu_reg.h"
#include "fc.h" // flags
#include "board.h" // sensor_transfer()
#include "reg.h" // alpha coeff
#include "utils.h" // wait_ms()

/* Private defines --------------------------------------*/

#define MPU_GYRO_SCALE 0.061035f
#define MPU_ACCEL_SCALE 0.00048828f

/* Private macros --------------------------------------*/

/* Private types --------------------------------------*/

/* Private variables ----------------------------------*/

#if (SENSOR == 6000)
	uint8_t sensor_data_to_send[sizeof(sensor_raw_t)+1]; // +1: SPI address
	uint16_t sensor_data_received[sizeof(sensor_raw_t)+1]; // +1: SPI dummy byte
	sensor_raw_t* sensor_raw = (sensor_raw_t*)&sensor_data_received[1];
#else
	uint8_t sensor_data_to_send[2];
	uint16_t sensor_data_received[sizeof(sensor_raw_t)];
	sensor_raw_t* sensor_raw = (sensor_raw_t*)sensor_data_received;
#endif

/* Private functions ----------------------------------*/

void sensor_write(uint8_t addr, uint8_t data)
{
	sensor_data_to_send[0] = addr & 0x7F;
	sensor_data_to_send[1] = data;
	sensor_transfer(sensor_data_to_send, (uint8_t*)sensor_data_received, 2);
	while (sensor_busy) {} // Wait for end of transaction
}

void sensor_read(uint8_t addr, uint8_t* data, uint8_t size)
{
	sensor_data_to_send[0] = 0x80 | (addr & 0x7F);
	sensor_transfer(sensor_data_to_send, data, size+1);
}

/* Public functions ----------------------------------*/

sensor_raw_t* sensor_read_samples()
{
#if (SENSOR == 6000)
	sensor_read(MPU_ACCEL_X_H, (uint8_t*)sensor_data_received+1, sizeof(sensor_raw_t));
#else
	sensor_read(MPU_ACCEL_X_H, (uint8_t*)sensor_data_received, sizeof(sensor_raw_t));
#endif
	return sensor_raw;
}

float mpu_update()
{
	while (sensor_busy) {} // Wait for end of current transaction
	sensor_write(MPU_CFG, MPU_CFG__DLPF_CFG(REG_MPU_CFG__FILT));
	sensor_write(MPU_SMPLRT_DIV, REG_MPU_CFG__RATE);
	return 1000.0f / (float)(REG_MPU_CFG__RATE+1);
}

void mpu_init()
{
	sensor_write(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
#if (SENSOR == 6000)
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
	sensor_write(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
}

void mpu_process_samples(struct sensor_s* sensor)
{
	uint8_t i;

	// Change endianness
	for (i=0; i<sizeof(sensor_raw_t)/2; i=i+1) {
		#if (SENSOR == 6000)	
			sensor_data_received[i+1] = __builtin_bswap16(sensor_data_received[i+1]);
		#else
			sensor_data_received[i] = __builtin_bswap16(sensor_data_received[i]);
		#endif
	}

	// Remove DC and scale
	#if (SENSOR_ORIENTATION == 90)
		sensor->gyro_y = -(float)(sensor_raw->gyro_x - REG_GYRO_DC_XY__X) * MPU_GYRO_SCALE;
		sensor->gyro_x = -(float)(sensor_raw->gyro_y - REG_GYRO_DC_XY__Y) * MPU_GYRO_SCALE;
		sensor->accel_x =  (float)(sensor_raw->accel_x - REG_ACCEL_DC_XY__X) * MPU_ACCEL_SCALE;
		sensor->accel_y = -(float)(sensor_raw->accel_y - REG_ACCEL_DC_XY__Y) * MPU_ACCEL_SCALE;
	#elif (SENSOR_ORIENTATION == 180)
		sensor->gyro_x =  (float)(sensor_raw->gyro_x - REG_GYRO_DC_XY__X) * MPU_GYRO_SCALE;
		sensor->gyro_y = -(float)(sensor_raw->gyro_y - REG_GYRO_DC_XY__Y) * MPU_GYRO_SCALE;
		sensor->accel_y = (float)(sensor_raw->accel_x - REG_ACCEL_DC_XY__X) * MPU_ACCEL_SCALE;
		sensor->accel_x = (float)(sensor_raw->accel_y - REG_ACCEL_DC_XY__Y) * MPU_ACCEL_SCALE;
	#else
		sensor->gyro_x = -(float)(sensor_raw->gyro_x - REG_GYRO_DC_XY__X) * MPU_GYRO_SCALE;
		sensor->gyro_y =  (float)(sensor_raw->gyro_y - REG_GYRO_DC_XY__Y) * MPU_GYRO_SCALE;
		sensor->accel_y = -(float)(sensor_raw->accel_x - REG_ACCEL_DC_XY__X) * MPU_ACCEL_SCALE;
		sensor->accel_x = -(float)(sensor_raw->accel_y - REG_ACCEL_DC_XY__Y) * MPU_ACCEL_SCALE;
	#endif
	sensor->gyro_z = -(float)(sensor_raw->gyro_z - (int16_t)REG_GYRO_DC_Z) * MPU_GYRO_SCALE;
	sensor->accel_z = (float)(sensor_raw->accel_z - (int16_t)REG_ACCEL_DC_Z) * MPU_ACCEL_SCALE;
	sensor->temperature = (float)sensor_raw->temperature / 340.0f + 36.53f;
}

void mpu_cal()
{
	uint8_t i;
	uint16_t j;
	float gyro_x_dc = 0;
	float gyro_y_dc = 0;
	float gyro_z_dc = 0;
	float accel_x_dc = 0;
	float accel_y_dc = 0;
	float accel_z_dc = 0;

	for (j=0 ; j<1000; j++) {
		// Wait for sensor sample
		while (!flag_sensor) {}
		flag_sensor = false;

		// Change endianness
		for (i=0; i<sizeof(sensor_raw_t)/2; i=i+1) {
			#if (SENSOR == 6000)	
				sensor_data_received[i+1] = __builtin_bswap16(sensor_data_received[i+1]);
			#else
				sensor_data_received[i] = __builtin_bswap16(sensor_data_received[i]);
			#endif
		}

		// Accumulate
		gyro_x_dc += (float)sensor_raw->gyro_x;
		gyro_y_dc += (float)sensor_raw->gyro_y;
		gyro_z_dc += (float)sensor_raw->gyro_z;
		accel_x_dc += (float)sensor_raw->accel_x;
		accel_y_dc += (float)sensor_raw->accel_y;
		accel_z_dc += (float)sensor_raw->accel_z;

		if ((j & 0x3F) == 0) {
			#ifdef DUAL_LED_STATUS
				toggle_led2(true);
			#else
				toggle_led(true);
			#endif
		}
	}

	// Compute average
	REG_GYRO_DC_XY = (int32_t)(gyro_x_dc / 1000.0f) + ((int32_t)(gyro_y_dc / 1000.0f) << 16);
	REG_GYRO_DC_Z = (int32_t)(gyro_z_dc / 1000.0f);
	REG_ACCEL_DC_XY = (int32_t)(accel_x_dc / 1000.0f) + ((int32_t)(accel_y_dc / 1000.0f) << 16);
	REG_ACCEL_DC_Z = (int32_t)(accel_z_dc / 1000.0f - 1.0f/MPU_ACCEL_SCALE);
}

void angle_estimate(struct sensor_s* sensor, struct angle_s* angle)
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
