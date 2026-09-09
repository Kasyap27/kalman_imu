#pragma once

// Register map for the ICM-20948 and its onboard AK09916 magnetometer.
// See ICM-20948 Datasheet, Register Map and Descriptions, Revision 1.3,
// https://www.invensense.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
// and AK09916 Datasheet and Register Map
// https://www.akm.com/akm/en/file/datasheet/AK09916C.pdf

// AK09916 magnetometer registers
#define AK09916_WHO_AM_I     0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1          0x10 // data ready status bit 0
#define AK09916_XOUT_L        0x11
#define AK09916_XOUT_H        0x12
#define AK09916_YOUT_L        0x13
#define AK09916_YOUT_H        0x14
#define AK09916_ZOUT_L        0x15
#define AK09916_ZOUT_H        0x16
#define AK09916_ST2           0x18 // overflow bit 3, data read error bit 2
#define AK09916_CNTL          0x30 // power down/single/self-test/fuse ROM modes, bits 3:0
#define AK09916_CNTL2         0x31 // normal (0), reset (1)

// USER BANK 0 REGISTER MAP
#define ICM20948_WHO_AM_I      0x00 // should return 0xEA
#define ICM20948_USER_CTRL     0x03 // bit 7 enable DMP, bit 3 reset DMP
#define ICM20948_LP_CONFIG     0x05
#define ICM20948_PWR_MGMT_1    0x06 // device defaults to the SLEEP mode
#define ICM20948_PWR_MGMT_2    0x07
#define ICM20948_INT_PIN_CFG   0x0F
#define ICM20948_INT_ENABLE    0x10
#define ICM20948_INT_ENABLE_1  0x11
#define ICM20948_INT_ENABLE_2  0x12
#define ICM20948_INT_ENABLE_3  0x13
#define ICM20948_I2C_MST_STATUS 0x17
#define ICM20948_INT_STATUS    0x19
#define ICM20948_INT_STATUS_1  0x1A
#define ICM20948_INT_STATUS_2  0x1B
#define ICM20948_INT_STATUS_3  0x1C
#define ICM20948_DELAY_TIMEH   0x28
#define ICM20948_DELAY_TIMEL   0x29
#define ICM20948_ACCEL_XOUT_H  0x2D
#define ICM20948_ACCEL_XOUT_L  0x2E
#define ICM20948_ACCEL_YOUT_H  0x2F
#define ICM20948_ACCEL_YOUT_L  0x30
#define ICM20948_ACCEL_ZOUT_H  0x31
#define ICM20948_ACCEL_ZOUT_L  0x32
#define ICM20948_GYRO_XOUT_H   0x33
#define ICM20948_GYRO_XOUT_L   0x34
#define ICM20948_GYRO_YOUT_H   0x35
#define ICM20948_GYRO_YOUT_L   0x36
#define ICM20948_GYRO_ZOUT_H   0x37
#define ICM20948_GYRO_ZOUT_L   0x38
#define ICM20948_TEMP_OUT_H    0x39
#define ICM20948_TEMP_OUT_L    0x3A
#define ICM20948_FIFO_EN_1     0x66
#define ICM20948_FIFO_EN_2     0x67
#define ICM20948_FIFO_RST      0x68
#define ICM20948_FIFO_MODE     0x69
#define ICM20948_FIFO_COUNTH   0x70
#define ICM20948_FIFO_COUNTL   0x71
#define ICM20948_FIFO_R_W      0x72
#define ICM20948_DATA_RDY_STATUS 0x74
#define ICM20948_FIFO_CFG      0x76
#define ICM20948_REG_BANK_SEL  0x7F

// USER BANK 1 REGISTER MAP
#define ICM20948_SELF_TEST_X_GYRO  0x02
#define ICM20948_SELF_TEST_Y_GYRO  0x03
#define ICM20948_SELF_TEST_Z_GYRO  0x04
#define ICM20948_SELF_TEST_X_ACCEL 0x0E
#define ICM20948_SELF_TEST_Y_ACCEL 0x0F
#define ICM20948_SELF_TEST_Z_ACCEL 0x10
#define ICM20948_XA_OFFSET_H       0x14
#define ICM20948_XA_OFFSET_L       0x15
#define ICM20948_YA_OFFSET_H       0x17
#define ICM20948_YA_OFFSET_L       0x18
#define ICM20948_ZA_OFFSET_H       0x1A
#define ICM20948_ZA_OFFSET_L       0x1B
#define ICM20948_TIMEBASE_CORRECTION_PLL 0x28

// USER BANK 2 REGISTER MAP
#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01
#define ICM20948_GYRO_CONFIG_2       0x02
#define ICM20948_XG_OFFSET_H         0x03
#define ICM20948_XG_OFFSET_L         0x04
#define ICM20948_YG_OFFSET_H         0x05
#define ICM20948_YG_OFFSET_L         0x06
#define ICM20948_ZG_OFFSET_H         0x07
#define ICM20948_ZG_OFFSET_L         0x08
#define ICM20948_ODR_ALIGN_EN        0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_ACCEL_INTEL_CTRL    0x12
#define ICM20948_ACCEL_WOM_THR       0x13
#define ICM20948_ACCEL_CONFIG        0x14
#define ICM20948_ACCEL_CONFIG_2      0x15
#define ICM20948_FSYNC_CONFIG        0x52
#define ICM20948_TEMP_CONFIG         0x53
#define ICM20948_MOD_CTRL_USR        0x54

// Default I2C addresses (AD0 pin low/high). Seven-bit address.
#define ICM20948_I2C_ADDR_AD0_LOW  0x68
#define ICM20948_I2C_ADDR_AD0_HIGH 0x69
#define AK09916_I2C_ADDR           0x0C

#define ICM20948_RESET_BIT 0x80
