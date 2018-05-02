#ifndef __SENSOR_H
#define __SENSOR_H

#include "board.h"

/* Public defines -----------------*/

/* Public macros -----------------*/

/* Public types -----------------*/

__packed struct sensor_raw_s {
	uint8_t dummy;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

typedef union {
	uint8_t bytes[15];
	struct sensor_raw_s sensor;
} sensor_raw_t;

struct sensor_s {
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float accel_x;
	float accel_y;
	float accel_z;
	float temperature;
};

struct angle_s {
	float pitch;
	float roll;
	float pitch_from_accel;
	float roll_from_accel;
};

/* Exported variables -----------------*/

/* Public functions -----------------*/

void mpu_spi_init(void);
void mpu_i2c_init(void);
void mpu_process_samples(sensor_raw_t * sensor_raw, struct sensor_s * sensor);
void mpu_cal(sensor_raw_t * sensor_raw);
void angle_estimate(struct sensor_s * sensor, struct angle_s * angle, _Bool yaw_transfer_is_on);

#endif
