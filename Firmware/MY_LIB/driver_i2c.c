#include "driver_i2c.h"

#define I2C1_TIMEOUT     3000u

static void I2C1_Periph_Config(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    I2C_DeInit(I2C1);

    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
}

static void I2C1_Recover(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    I2C1_Periph_Config();
}

static uint8_t I2C1_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C1_TIMEOUT;

    while (I2C_CheckEvent(I2C1, event) == ERROR)
    {
        if (--timeout == 0u)
        {
            I2C1_Recover();
            return 1u;
        }
    }
    return 0u;
}

static void I2C1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
}

void I2C1_MPU6050_Init(void)
{
    I2C1_GPIO_Config();
    I2C1_Periph_Config();

    I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1,   MPU6050_PWR_MGMT_1_WAKEUP);
    I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV,   0x04);
    I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG,       0x03);
    I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_FS_500_DPS);
    I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
}

uint8_t I2C1_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 1u;

    I2C_Send7bitAddress(I2C1, (slaveAddr << 1), I2C_Direction_Transmitter);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 2u;

    I2C_SendData(I2C1, regAddr);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 3u;

    I2C_SendData(I2C1, data);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 4u;

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0u;
}

uint8_t I2C1_ReadRegs(uint8_t slaveAddr, uint8_t startReg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 || len == 0u) return 7u;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 1u;

    I2C_Send7bitAddress(I2C1, (slaveAddr << 1), I2C_Direction_Transmitter);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 2u;

    I2C_SendData(I2C1, startReg);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 3u;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 4u;

    I2C_Send7bitAddress(I2C1, (slaveAddr << 1), I2C_Direction_Receiver);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 5u;

    for (i = 0u; i < len; i++)
    {
        I2C_AcknowledgeConfig(I2C1, (i == (len - 1u)) ? DISABLE : ENABLE);
        if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 6u;
        data[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return 0u;
}

uint8_t MPU6050_ReadRawData(int16_t *accX, int16_t *accY, int16_t *accZ,
                            int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
    uint8_t buf[14];
    uint8_t ret;

    if (!accX || !accY || !accZ || !gyroX || !gyroY || !gyroZ) return 8u;

    ret = I2C1_ReadRegs(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != 0u) return ret;

    *accX  = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
    *accY  = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
    *accZ  = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
    *gyroX = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
    *gyroY = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    *gyroZ = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

    return 0u;
}
