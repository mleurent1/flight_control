#define SELF_TST_X 13

#define SELF_TST_X__XG_TST(x) (((x) & 0x1F) << 0)
#define SELF_TST_X__XA_TST_MSB(x) (((x) & 0x7) << 5)

#define SELF_TST_Y 14

#define SELF_TST_Y__YG_TST(x) (((x) & 0x1F) << 0)
#define SELF_TST_Y__YA_TST_MSB(x) (((x) & 0x7) << 5)

#define SELF_TST_Z 15

#define SELF_TST_Z__ZG_TST(x) (((x) & 0x1F) << 0)
#define SELF_TST_Z__ZA_TST_MSB(x) (((x) & 0x7) << 5)

#define SELF_TST_A 16

#define SELF_TST_A__XA_TST_LSB(x) (((x) & 0x3) << 0)
#define SELF_TST_A__YA_TST_LSB(x) (((x) & 0x3) << 2)
#define SELF_TST_A__ZA_TST_LSB(x) (((x) & 0x3) << 4)

#define SMPLRT_DIV 25

#define CFG 26

#define CFG__DLPF_CFG(x) (((x) & 0x7) << 0)
#define CFG__EXT_SYNC_SET(x) (((x) & 0x7) << 3)

#define GYRO_CFG 27

#define GYRO_CFG__FS_SEL(x) (((x) & 0x3) << 3)
#define GYRO_CFG__ZG_ST (1 << 5)
#define GYRO_CFG__YG_ST (1 << 6)
#define GYRO_CFG__XG_ST (1 << 7)

#define INT_EN 56

#define INT_EN__DATA_RDY_EN (1 << 0)
#define INT_EN__I2C_MST_INT_EN (1 << 3)
#define INT_EN__FIFO_OFLOW_EN (1 << 4)

#define ACCEL_X_H 59
#define ACCEL_X_L 60
#define ACCEL_Y_H 61
#define ACCEL_Y_L 62
#define ACCEL_Z_H 63
#define ACCEL_Z_L 64
#define TEMP_H 65
#define TEMP_L 66
#define GYRO_X_H 67
#define GYRO_X_L 68
#define GYRO_Y_H 69
#define GYRO_Y_L 70
#define GYRO_Z_H 71
#define GYRO_Z_L 72

#define SIGNAL_PATH_RST 104

#define SIGNAL_PATH_RST__TEMP_RST (1 << 0)
#define SIGNAL_PATH_RST__ACCEL_RST (1 << 1)
#define SIGNAL_PATH_RST__GYRO_RST (1 << 2)

#define USER_CTRL 106

#define USER_CTRL__SIG_COND_RST (1 << 0)
#define USER_CTRL__I2C_MST_RST (1 << 1)
#define USER_CTRL__FIFO_RST (1 << 2)
#define USER_CTRL__I2C_IF_DIS (1 << 4)
#define USER_CTRL__I2C_MST_EN (1 << 5)
#define USER_CTRL__FIFO_EN (1 << 6)

#define PWR_MGMT_1 107

#define PWR_MGMT_1__CLKSEL(x) (((x) & 0x7) << 0)
#define PWR_MGMT_1__TEMP_DIS (1 << 3)
#define PWR_MGMT_1__CYCLE (1 << 5)
#define PWR_MGMT_1__SLEEP (1 << 6)
#define PWR_MGMT_1__DEVICE_RST (1 << 7)

#define WHO_AM_I 117
