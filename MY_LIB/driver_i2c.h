#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#include "stm32f4xx.h"

#define MPU6050_I2C_ADDR       0x68
#define MPU6050_I2C_READ       0x01
#define MPU6050_I2C_WRITE      0x00

/* MPU6050 register map */
#define MPU6050_REG_SELF_TEST_X        0x0D
#define MPU6050_REG_SELF_TEST_Y        0x0E
#define MPU6050_REG_SELF_TEST_Z        0x0F
#define MPU6050_REG_SELF_TEST_A        0x10
#define MPU6050_REG_SMPLRT_DIV         0x19
#define MPU6050_REG_CONFIG             0x1A
#define MPU6050_REG_GYRO_CONFIG        0x1B
#define MPU6050_REG_ACCEL_CONFIG       0x1C
#define MPU6050_REG_INT_PIN_CFG        0x37
#define MPU6050_REG_INT_ENABLE         0x38
#define MPU6050_REG_ACCEL_XOUT_H       0x3B
#define MPU6050_REG_ACCEL_XOUT_L       0x3C
#define MPU6050_REG_ACCEL_YOUT_H       0x3D
#define MPU6050_REG_ACCEL_YOUT_L       0x3E
#define MPU6050_REG_ACCEL_ZOUT_H       0x3F
#define MPU6050_REG_ACCEL_ZOUT_L       0x40
#define MPU6050_REG_TEMP_OUT_H         0x41
#define MPU6050_REG_TEMP_OUT_L         0x42
#define MPU6050_REG_GYRO_XOUT_H        0x43
#define MPU6050_REG_GYRO_XOUT_L        0x44
#define MPU6050_REG_GYRO_YOUT_H        0x45
#define MPU6050_REG_GYRO_YOUT_L        0x46
#define MPU6050_REG_GYRO_ZOUT_H        0x47
#define MPU6050_REG_GYRO_ZOUT_L        0x48
#define MPU6050_REG_PWR_MGMT_1         0x6B
#define MPU6050_REG_PWR_MGMT_2         0x6C
#define MPU6050_REG_WHO_AM_I           0x75

/* MPU6050 wake-up/reset values */
#define MPU6050_PWR_MGMT_1_WAKEUP      0x00
#define MPU6050_PWR_MGMT_1_RESET       0x80

/* Gyro and accel full scale config */
#define MPU6050_GYRO_FS_250_DPS        0x00
#define MPU6050_GYRO_FS_500_DPS        0x08
#define MPU6050_GYRO_FS_1000_DPS       0x10
#define MPU6050_GYRO_FS_2000_DPS       0x18

#define MPU6050_ACCEL_FS_2G            0x00
#define MPU6050_ACCEL_FS_4G            0x08
#define MPU6050_ACCEL_FS_8G            0x10
#define MPU6050_ACCEL_FS_16G           0x18

void I2C1_MPU6050_Init(void);
uint8_t I2C1_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C1_ReadReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
uint8_t I2C1_ReadRegs(uint8_t slaveAddr, uint8_t startReg, uint8_t *data, uint8_t len);

uint8_t MPU6050_ReadRawData(int16_t *accX, int16_t *accY, int16_t *accZ,
                             int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);
float MPU6050_GetRoll(void);
float MPU6050_GetPitch(void);
float MPU6050_GetYaw(void);
float MPU6050_GetHeading(void);

#endif /* __DRIVER_I2C_H */
