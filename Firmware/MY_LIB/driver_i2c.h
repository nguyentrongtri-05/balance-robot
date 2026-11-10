#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#include "stm32f4xx.h"

#define MPU6050_I2C_ADDR               0x68

/* MPU6050 register map */
#define MPU6050_REG_SMPLRT_DIV         0x19
#define MPU6050_REG_CONFIG             0x1A
#define MPU6050_REG_GYRO_CONFIG        0x1B
#define MPU6050_REG_ACCEL_CONFIG       0x1C
#define MPU6050_REG_ACCEL_XOUT_H       0x3B
#define MPU6050_REG_PWR_MGMT_1         0x6B

#define MPU6050_PWR_MGMT_1_WAKEUP      0x00
#define MPU6050_GYRO_FS_500_DPS        0x08
#define MPU6050_ACCEL_FS_2G            0x00

void    I2C1_MPU6050_Init(void);
uint8_t I2C1_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C1_ReadRegs(uint8_t slaveAddr, uint8_t startReg, uint8_t *data, uint8_t len);
uint8_t MPU6050_ReadRawData(int16_t *accX, int16_t *accY, int16_t *accZ,
                            int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

#endif /* __DRIVER_I2C_H */
