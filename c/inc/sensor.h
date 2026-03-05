#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

/* Public defines -----------------*/

/* Public macros -----------------*/

/* Public types -----------------*/

typedef struct sensor_raw_s {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} __attribute__((packed)) sensor_raw_t;

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

sensor_raw_t* sensor_read_samples();
void mpu_init();
float mpu_update();
void mpu_process_samples(struct sensor_s* sensor);
void mpu_cal();
void angle_estimate(struct sensor_s* sensor, struct angle_s* angle);

#endif
