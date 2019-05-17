#define NB_REG 39

#define REG_VERSION reg[0]
#define REG_VERSION_Addr 0
#define REG_CTRL reg[1]
#define REG_CTRL__SENSOR_HOST_CTRL (uint8_t)((reg[1] & 1U) >> 0)
#define REG_CTRL__SENSOR_HOST_CTRL_Msk 1U
#define REG_CTRL__SENSOR_HOST_CTRL_Pos 0U
#define REG_CTRL__ARM_TEST (uint8_t)((reg[1] & 6U) >> 1)
#define REG_CTRL__ARM_TEST_Msk 6U
#define REG_CTRL__ARM_TEST_Pos 1U
#define REG_CTRL__BEEP_TEST (uint8_t)((reg[1] & 8U) >> 3)
#define REG_CTRL__BEEP_TEST_Msk 8U
#define REG_CTRL__BEEP_TEST_Pos 3U
#define REG_CTRL__TIME_MAXHOLD (uint8_t)((reg[1] & 16U) >> 4)
#define REG_CTRL__TIME_MAXHOLD_Msk 16U
#define REG_CTRL__TIME_MAXHOLD_Pos 4U
#define REG_CTRL__SENSOR_CAL (uint8_t)((reg[1] & 32U) >> 5)
#define REG_CTRL__SENSOR_CAL_Msk 32U
#define REG_CTRL__SENSOR_CAL_Pos 5U
#define REG_CTRL__BEEP_DISABLE (uint8_t)((reg[1] & 64U) >> 6)
#define REG_CTRL__BEEP_DISABLE_Msk 64U
#define REG_CTRL__BEEP_DISABLE_Pos 6U
#define REG_CTRL_Addr 1
#define REG_MOTOR_TEST reg[2]
#define REG_MOTOR_TEST__VALUE (uint16_t)((reg[2] & 65535U) >> 0)
#define REG_MOTOR_TEST__VALUE_Msk 65535U
#define REG_MOTOR_TEST__VALUE_Pos 0U
#define REG_MOTOR_TEST__SELECT (uint8_t)((reg[2] & 983040U) >> 16)
#define REG_MOTOR_TEST__SELECT_Msk 983040U
#define REG_MOTOR_TEST__SELECT_Pos 16U
#define REG_MOTOR_TEST__TELEMETRY (uint8_t)((reg[2] & 1048576U) >> 20)
#define REG_MOTOR_TEST__TELEMETRY_Msk 1048576U
#define REG_MOTOR_TEST__TELEMETRY_Pos 20U
#define REG_MOTOR_TEST_Addr 2
#define REG_DEBUG reg[3]
#define REG_DEBUG__CASE (uint8_t)((reg[3] & 255U) >> 0)
#define REG_DEBUG__CASE_Msk 255U
#define REG_DEBUG__CASE_Pos 0U
#define REG_DEBUG__MASK (uint16_t)((reg[3] & 16776960U) >> 8)
#define REG_DEBUG__MASK_Msk 16776960U
#define REG_DEBUG__MASK_Pos 8U
#define REG_DEBUG_Addr 3
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
#define REG_TIME reg[5]
#define REG_TIME__SENSOR (uint16_t)((reg[5] & 65535U) >> 0)
#define REG_TIME__SENSOR_Msk 65535U
#define REG_TIME__SENSOR_Pos 0U
#define REG_TIME__PROCESSING (uint16_t)((reg[5] & 4294901760U) >> 16)
#define REG_TIME__PROCESSING_Msk 4294901760U
#define REG_TIME__PROCESSING_Pos 16U
#define REG_TIME_Addr 5
#define REG_VBAT regf[6]
#define REG_VBAT_Addr 6
#define REG_VBAT_MIN regf[7]
#define REG_VBAT_MIN_Addr 7
#define REG_TIME_CONSTANT reg[8]
#define REG_TIME_CONSTANT__ACCEL (uint16_t)((reg[8] & 65535U) >> 0)
#define REG_TIME_CONSTANT__ACCEL_Msk 65535U
#define REG_TIME_CONSTANT__ACCEL_Pos 0U
#define REG_TIME_CONSTANT__VBAT (uint16_t)((reg[8] & 4294901760U) >> 16)
#define REG_TIME_CONSTANT__VBAT_Msk 4294901760U
#define REG_TIME_CONSTANT__VBAT_Pos 16U
#define REG_TIME_CONSTANT_Addr 8
#define REG_TIME_CONSTANT_RADIO reg[9]
#define REG_TIME_CONSTANT_RADIO_Addr 9
#define REG_EXPO_PITCH_ROLL regf[10]
#define REG_EXPO_PITCH_ROLL_Addr 10
#define REG_EXPO_YAW regf[11]
#define REG_EXPO_YAW_Addr 11
#define REG_MOTOR reg[12]
#define REG_MOTOR__START (uint16_t)((reg[12] & 1023U) >> 0)
#define REG_MOTOR__START_Msk 1023U
#define REG_MOTOR__START_Pos 0U
#define REG_MOTOR__ARMED (uint16_t)((reg[12] & 1047552U) >> 10)
#define REG_MOTOR__ARMED_Msk 1047552U
#define REG_MOTOR__ARMED_Pos 10U
#define REG_MOTOR__RANGE (uint16_t)((reg[12] & 4293918720U) >> 20)
#define REG_MOTOR__RANGE_Msk 4293918720U
#define REG_MOTOR__RANGE_Pos 20U
#define REG_MOTOR_Addr 12
#define REG_RATE reg[13]
#define REG_RATE__PITCH_ROLL (uint16_t)((reg[13] & 4095U) >> 0)
#define REG_RATE__PITCH_ROLL_Msk 4095U
#define REG_RATE__PITCH_ROLL_Pos 0U
#define REG_RATE__YAW (uint16_t)((reg[13] & 16773120U) >> 12)
#define REG_RATE__YAW_Msk 16773120U
#define REG_RATE__YAW_Pos 12U
#define REG_RATE__ANGLE (uint8_t)((reg[13] & 4278190080U) >> 24)
#define REG_RATE__ANGLE_Msk 4278190080U
#define REG_RATE__ANGLE_Pos 24U
#define REG_RATE_Addr 13
#define REG_P_PITCH regf[14]
#define REG_P_PITCH_Addr 14
#define REG_I_PITCH regf[15]
#define REG_I_PITCH_Addr 15
#define REG_D_PITCH regf[16]
#define REG_D_PITCH_Addr 16
#define REG_P_ROLL regf[17]
#define REG_P_ROLL_Addr 17
#define REG_I_ROLL regf[18]
#define REG_I_ROLL_Addr 18
#define REG_D_ROLL regf[19]
#define REG_D_ROLL_Addr 19
#define REG_P_YAW regf[20]
#define REG_P_YAW_Addr 20
#define REG_I_YAW regf[21]
#define REG_I_YAW_Addr 21
#define REG_D_YAW regf[22]
#define REG_D_YAW_Addr 22
#define REG_P_PITCH_ANGLE regf[23]
#define REG_P_PITCH_ANGLE_Addr 23
#define REG_I_PITCH_ANGLE regf[24]
#define REG_I_PITCH_ANGLE_Addr 24
#define REG_D_PITCH_ANGLE regf[25]
#define REG_D_PITCH_ANGLE_Addr 25
#define REG_P_ROLL_ANGLE regf[26]
#define REG_P_ROLL_ANGLE_Addr 26
#define REG_I_ROLL_ANGLE regf[27]
#define REG_I_ROLL_ANGLE_Addr 27
#define REG_D_ROLL_ANGLE regf[28]
#define REG_D_ROLL_ANGLE_Addr 28
#define REG_GYRO_DC_XY reg[29]
#define REG_GYRO_DC_XY__X (int16_t)((reg[29] & 65535U) >> 0)
#define REG_GYRO_DC_XY__X_Msk 65535U
#define REG_GYRO_DC_XY__X_Pos 0U
#define REG_GYRO_DC_XY__Y (int16_t)((reg[29] & 4294901760U) >> 16)
#define REG_GYRO_DC_XY__Y_Msk 4294901760U
#define REG_GYRO_DC_XY__Y_Pos 16U
#define REG_GYRO_DC_XY_Addr 29
#define REG_GYRO_DC_Z reg[30]
#define REG_GYRO_DC_Z_Addr 30
#define REG_ACCEL_DC_XY reg[31]
#define REG_ACCEL_DC_XY__X (int16_t)((reg[31] & 65535U) >> 0)
#define REG_ACCEL_DC_XY__X_Msk 65535U
#define REG_ACCEL_DC_XY__X_Pos 0U
#define REG_ACCEL_DC_XY__Y (int16_t)((reg[31] & 4294901760U) >> 16)
#define REG_ACCEL_DC_XY__Y_Msk 4294901760U
#define REG_ACCEL_DC_XY__Y_Pos 16U
#define REG_ACCEL_DC_XY_Addr 31
#define REG_ACCEL_DC_Z reg[32]
#define REG_ACCEL_DC_Z_Addr 32
#define REG_THROTTLE reg[33]
#define REG_THROTTLE__IDLE (uint16_t)((reg[33] & 65535U) >> 0)
#define REG_THROTTLE__IDLE_Msk 65535U
#define REG_THROTTLE__IDLE_Pos 0U
#define REG_THROTTLE__RANGE (uint16_t)((reg[33] & 4294901760U) >> 16)
#define REG_THROTTLE__RANGE_Msk 4294901760U
#define REG_THROTTLE__RANGE_Pos 16U
#define REG_THROTTLE_Addr 33
#define REG_AILERON reg[34]
#define REG_AILERON__IDLE (uint16_t)((reg[34] & 65535U) >> 0)
#define REG_AILERON__IDLE_Msk 65535U
#define REG_AILERON__IDLE_Pos 0U
#define REG_AILERON__RANGE (uint16_t)((reg[34] & 4294901760U) >> 16)
#define REG_AILERON__RANGE_Msk 4294901760U
#define REG_AILERON__RANGE_Pos 16U
#define REG_AILERON_Addr 34
#define REG_ELEVATOR reg[35]
#define REG_ELEVATOR__IDLE (uint16_t)((reg[35] & 65535U) >> 0)
#define REG_ELEVATOR__IDLE_Msk 65535U
#define REG_ELEVATOR__IDLE_Pos 0U
#define REG_ELEVATOR__RANGE (uint16_t)((reg[35] & 4294901760U) >> 16)
#define REG_ELEVATOR__RANGE_Msk 4294901760U
#define REG_ELEVATOR__RANGE_Pos 16U
#define REG_ELEVATOR_Addr 35
#define REG_RUDDER reg[36]
#define REG_RUDDER__IDLE (uint16_t)((reg[36] & 65535U) >> 0)
#define REG_RUDDER__IDLE_Msk 65535U
#define REG_RUDDER__IDLE_Pos 0U
#define REG_RUDDER__RANGE (uint16_t)((reg[36] & 4294901760U) >> 16)
#define REG_RUDDER__RANGE_Msk 4294901760U
#define REG_RUDDER__RANGE_Pos 16U
#define REG_RUDDER_Addr 36
#define REG_AUX reg[37]
#define REG_AUX__IDLE (uint16_t)((reg[37] & 65535U) >> 0)
#define REG_AUX__IDLE_Msk 65535U
#define REG_AUX__IDLE_Pos 0U
#define REG_AUX__RANGE (uint16_t)((reg[37] & 4294901760U) >> 16)
#define REG_AUX__RANGE_Msk 4294901760U
#define REG_AUX__RANGE_Pos 16U
#define REG_AUX_Addr 37
#define REG_MPU_CFG reg[38]
#define REG_MPU_CFG__FILT (uint8_t)((reg[38] & 255U) >> 0)
#define REG_MPU_CFG__FILT_Msk 255U
#define REG_MPU_CFG__FILT_Pos 0U
#define REG_MPU_CFG__RATE (uint8_t)((reg[38] & 65280U) >> 8)
#define REG_MPU_CFG__RATE_Msk 65280U
#define REG_MPU_CFG__RATE_Pos 8U
#define REG_MPU_CFG_Addr 38
