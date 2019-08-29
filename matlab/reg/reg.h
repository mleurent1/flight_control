#define NB_REG 38

#define REG_VERSION reg[0]
#define REG_VERSION_Addr 0
#define REG_STATUS reg[1]
#define REG_STATUS_Addr 1
#define REG_CTRL reg[2]
#define REG_CTRL__SENSOR_HOST_CTRL (uint8_t)((reg[2] & 1U) >> 0)
#define REG_CTRL__SENSOR_HOST_CTRL_Msk 1U
#define REG_CTRL__SENSOR_HOST_CTRL_Pos 0U
#define REG_CTRL__ARM_TEST (uint8_t)((reg[2] & 6U) >> 1)
#define REG_CTRL__ARM_TEST_Msk 6U
#define REG_CTRL__ARM_TEST_Pos 1U
#define REG_CTRL__BEEP_TEST (uint8_t)((reg[2] & 8U) >> 3)
#define REG_CTRL__BEEP_TEST_Msk 8U
#define REG_CTRL__BEEP_TEST_Pos 3U
#define REG_CTRL__SENSOR_CAL (uint8_t)((reg[2] & 16U) >> 4)
#define REG_CTRL__SENSOR_CAL_Msk 16U
#define REG_CTRL__SENSOR_CAL_Pos 4U
#define REG_CTRL__DEBUG (uint8_t)((reg[2] & 32U) >> 5)
#define REG_CTRL__DEBUG_Msk 32U
#define REG_CTRL__DEBUG_Pos 5U
#define REG_CTRL_Addr 2
#define REG_MOTOR_TEST reg[3]
#define REG_MOTOR_TEST__VALUE (uint16_t)((reg[3] & 65535U) >> 0)
#define REG_MOTOR_TEST__VALUE_Msk 65535U
#define REG_MOTOR_TEST__VALUE_Pos 0U
#define REG_MOTOR_TEST__SELECT (uint8_t)((reg[3] & 983040U) >> 16)
#define REG_MOTOR_TEST__SELECT_Msk 983040U
#define REG_MOTOR_TEST__SELECT_Pos 16U
#define REG_MOTOR_TEST__TELEMETRY (uint8_t)((reg[3] & 1048576U) >> 20)
#define REG_MOTOR_TEST__TELEMETRY_Msk 1048576U
#define REG_MOTOR_TEST__TELEMETRY_Pos 20U
#define REG_MOTOR_TEST_Addr 3
#define REG_ERROR reg[4]
#define REG_ERROR__SENSOR (uint8_t)((reg[4] & 255U) >> 0)
#define REG_ERROR__SENSOR_Msk 255U
#define REG_ERROR__SENSOR_Pos 0U
#define REG_ERROR__RADIO (uint8_t)((reg[4] & 65280U) >> 8)
#define REG_ERROR__RADIO_Msk 65280U
#define REG_ERROR__RADIO_Pos 8U
#define REG_ERROR__RF (uint8_t)((reg[4] & 16711680U) >> 16)
#define REG_ERROR__RF_Msk 16711680U
#define REG_ERROR__RF_Pos 16U
#define REG_ERROR__CRC (uint8_t)((reg[4] & 4278190080U) >> 24)
#define REG_ERROR__CRC_Msk 4278190080U
#define REG_ERROR__CRC_Pos 24U
#define REG_ERROR_Addr 4
#define REG_VBAT_MIN regf[5]
#define REG_VBAT_MIN_Addr 5
#define REG_TIME_CONSTANT reg[6]
#define REG_TIME_CONSTANT__ACCEL (uint16_t)((reg[6] & 65535U) >> 0)
#define REG_TIME_CONSTANT__ACCEL_Msk 65535U
#define REG_TIME_CONSTANT__ACCEL_Pos 0U
#define REG_TIME_CONSTANT__VBAT (uint16_t)((reg[6] & 4294901760U) >> 16)
#define REG_TIME_CONSTANT__VBAT_Msk 4294901760U
#define REG_TIME_CONSTANT__VBAT_Pos 16U
#define REG_TIME_CONSTANT_Addr 6
#define REG_TIME_CONSTANT_RADIO reg[7]
#define REG_TIME_CONSTANT_RADIO_Addr 7
#define REG_EXPO_PITCH_ROLL regf[8]
#define REG_EXPO_PITCH_ROLL_Addr 8
#define REG_EXPO_YAW regf[9]
#define REG_EXPO_YAW_Addr 9
#define REG_MOTOR reg[10]
#define REG_MOTOR__START (uint16_t)((reg[10] & 1023U) >> 0)
#define REG_MOTOR__START_Msk 1023U
#define REG_MOTOR__START_Pos 0U
#define REG_MOTOR__ARMED (uint16_t)((reg[10] & 1047552U) >> 10)
#define REG_MOTOR__ARMED_Msk 1047552U
#define REG_MOTOR__ARMED_Pos 10U
#define REG_MOTOR__RANGE (uint16_t)((reg[10] & 4293918720U) >> 20)
#define REG_MOTOR__RANGE_Msk 4293918720U
#define REG_MOTOR__RANGE_Pos 20U
#define REG_MOTOR_Addr 10
#define REG_RATE reg[11]
#define REG_RATE__PITCH_ROLL (uint16_t)((reg[11] & 4095U) >> 0)
#define REG_RATE__PITCH_ROLL_Msk 4095U
#define REG_RATE__PITCH_ROLL_Pos 0U
#define REG_RATE__YAW (uint16_t)((reg[11] & 16773120U) >> 12)
#define REG_RATE__YAW_Msk 16773120U
#define REG_RATE__YAW_Pos 12U
#define REG_RATE__ANGLE (uint8_t)((reg[11] & 4278190080U) >> 24)
#define REG_RATE__ANGLE_Msk 4278190080U
#define REG_RATE__ANGLE_Pos 24U
#define REG_RATE_Addr 11
#define REG_P_PITCH regf[12]
#define REG_P_PITCH_Addr 12
#define REG_I_PITCH regf[13]
#define REG_I_PITCH_Addr 13
#define REG_D_PITCH regf[14]
#define REG_D_PITCH_Addr 14
#define REG_P_ROLL regf[15]
#define REG_P_ROLL_Addr 15
#define REG_I_ROLL regf[16]
#define REG_I_ROLL_Addr 16
#define REG_D_ROLL regf[17]
#define REG_D_ROLL_Addr 17
#define REG_P_YAW regf[18]
#define REG_P_YAW_Addr 18
#define REG_I_YAW regf[19]
#define REG_I_YAW_Addr 19
#define REG_D_YAW regf[20]
#define REG_D_YAW_Addr 20
#define REG_P_PITCH_ANGLE regf[21]
#define REG_P_PITCH_ANGLE_Addr 21
#define REG_I_PITCH_ANGLE regf[22]
#define REG_I_PITCH_ANGLE_Addr 22
#define REG_D_PITCH_ANGLE regf[23]
#define REG_D_PITCH_ANGLE_Addr 23
#define REG_P_ROLL_ANGLE regf[24]
#define REG_P_ROLL_ANGLE_Addr 24
#define REG_I_ROLL_ANGLE regf[25]
#define REG_I_ROLL_ANGLE_Addr 25
#define REG_D_ROLL_ANGLE regf[26]
#define REG_D_ROLL_ANGLE_Addr 26
#define REG_GYRO_DC_XY reg[27]
#define REG_GYRO_DC_XY__X (int16_t)((reg[27] & 65535U) >> 0)
#define REG_GYRO_DC_XY__X_Msk 65535U
#define REG_GYRO_DC_XY__X_Pos 0U
#define REG_GYRO_DC_XY__Y (int16_t)((reg[27] & 4294901760U) >> 16)
#define REG_GYRO_DC_XY__Y_Msk 4294901760U
#define REG_GYRO_DC_XY__Y_Pos 16U
#define REG_GYRO_DC_XY_Addr 27
#define REG_GYRO_DC_Z reg[28]
#define REG_GYRO_DC_Z_Addr 28
#define REG_ACCEL_DC_XY reg[29]
#define REG_ACCEL_DC_XY__X (int16_t)((reg[29] & 65535U) >> 0)
#define REG_ACCEL_DC_XY__X_Msk 65535U
#define REG_ACCEL_DC_XY__X_Pos 0U
#define REG_ACCEL_DC_XY__Y (int16_t)((reg[29] & 4294901760U) >> 16)
#define REG_ACCEL_DC_XY__Y_Msk 4294901760U
#define REG_ACCEL_DC_XY__Y_Pos 16U
#define REG_ACCEL_DC_XY_Addr 29
#define REG_ACCEL_DC_Z reg[30]
#define REG_ACCEL_DC_Z_Addr 30
#define REG_THROTTLE reg[31]
#define REG_THROTTLE__IDLE (uint16_t)((reg[31] & 65535U) >> 0)
#define REG_THROTTLE__IDLE_Msk 65535U
#define REG_THROTTLE__IDLE_Pos 0U
#define REG_THROTTLE__RANGE (uint16_t)((reg[31] & 4294901760U) >> 16)
#define REG_THROTTLE__RANGE_Msk 4294901760U
#define REG_THROTTLE__RANGE_Pos 16U
#define REG_THROTTLE_Addr 31
#define REG_AILERON reg[32]
#define REG_AILERON__IDLE (uint16_t)((reg[32] & 65535U) >> 0)
#define REG_AILERON__IDLE_Msk 65535U
#define REG_AILERON__IDLE_Pos 0U
#define REG_AILERON__RANGE (uint16_t)((reg[32] & 4294901760U) >> 16)
#define REG_AILERON__RANGE_Msk 4294901760U
#define REG_AILERON__RANGE_Pos 16U
#define REG_AILERON_Addr 32
#define REG_ELEVATOR reg[33]
#define REG_ELEVATOR__IDLE (uint16_t)((reg[33] & 65535U) >> 0)
#define REG_ELEVATOR__IDLE_Msk 65535U
#define REG_ELEVATOR__IDLE_Pos 0U
#define REG_ELEVATOR__RANGE (uint16_t)((reg[33] & 4294901760U) >> 16)
#define REG_ELEVATOR__RANGE_Msk 4294901760U
#define REG_ELEVATOR__RANGE_Pos 16U
#define REG_ELEVATOR_Addr 33
#define REG_RUDDER reg[34]
#define REG_RUDDER__IDLE (uint16_t)((reg[34] & 65535U) >> 0)
#define REG_RUDDER__IDLE_Msk 65535U
#define REG_RUDDER__IDLE_Pos 0U
#define REG_RUDDER__RANGE (uint16_t)((reg[34] & 4294901760U) >> 16)
#define REG_RUDDER__RANGE_Msk 4294901760U
#define REG_RUDDER__RANGE_Pos 16U
#define REG_RUDDER_Addr 34
#define REG_AUX reg[35]
#define REG_AUX__IDLE (uint16_t)((reg[35] & 65535U) >> 0)
#define REG_AUX__IDLE_Msk 65535U
#define REG_AUX__IDLE_Pos 0U
#define REG_AUX__RANGE (uint16_t)((reg[35] & 4294901760U) >> 16)
#define REG_AUX__RANGE_Msk 4294901760U
#define REG_AUX__RANGE_Pos 16U
#define REG_AUX_Addr 35
#define REG_MPU_CFG reg[36]
#define REG_MPU_CFG__FILT (uint8_t)((reg[36] & 255U) >> 0)
#define REG_MPU_CFG__FILT_Msk 255U
#define REG_MPU_CFG__FILT_Pos 0U
#define REG_MPU_CFG__RATE (uint8_t)((reg[36] & 65280U) >> 8)
#define REG_MPU_CFG__RATE_Msk 65280U
#define REG_MPU_CFG__RATE_Pos 8U
#define REG_MPU_CFG_Addr 36
#define REG_DEBUG_REG reg[37]
#define REG_DEBUG_REG_Addr 37
