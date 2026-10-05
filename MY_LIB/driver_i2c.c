#include "driver_i2c.h"

#include <math.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"

/* =====================================================================
 *  HẰNG SỐ NỘI BỘ
 * ===================================================================== */
#define I2C1_TIMEOUT_US              2000u   /* Timeout mỗi bước chờ (theo thời gian thực, không theo số vòng lặp) */
#define MPU6050_CALIB_MAX_RANGE_DPS  4.0f    /* Biên độ dao động gyro tối đa cho phép khi hiệu chuẩn */
#define MPU6050_SAMPLE_PERIOD_MS     5u
#define RAD2DEG                      57.29577951f

/* =====================================================================
 *  BIẾN NỘI BỘ
 * ===================================================================== */
static float    s_gyroOffset[3] = {0.0f, 0.0f, 0.0f};
static float    s_accOffset[3]  = {0.0f, 0.0f, 0.0f};
static float    s_accScale[3]   = {1.0f, 1.0f, 1.0f};
static uint8_t  s_whoAmI       = 0u;
static uint32_t s_errTotal     = 0u;
static uint32_t s_errConsec    = 0u;
static uint32_t s_recoverCnt   = 0u;

/* =====================================================================
 *  DWT CYCLE COUNTER - delay/timeout chính xác, không phụ thuộc RTOS
 * ===================================================================== */
void DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u)
    {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static uint32_t DWT_UsToCycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000u);
}

void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = DWT_UsToCycles(us);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

void DWT_DelayMs(uint32_t ms)
{
    while (ms--)
    {
        DWT_DelayUs(1000u);
    }
}

/* =====================================================================
 *  I2C1 - CÁC HÀM CHỜ CÓ TIMEOUT + PHÁT HIỆN NACK
 * ===================================================================== */

/* Chờ 1 event (đọc SR1+SR2). Thoát sớm nếu slave NACK (cờ AF). */
static uint8_t I2C1_WaitEvent(uint32_t event, uint8_t err)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tmo   = DWT_UsToCycles(I2C1_TIMEOUT_US);

    while (I2C_CheckEvent(I2C1, event) == ERROR)
    {
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            return err;
        }
        if ((DWT->CYCCNT - start) > tmo)
        {
            return err;
        }
    }
    return I2C_OK;
}

/* Chờ 1 cờ đạt trạng thái mong muốn (chỉ đọc SR tương ứng, KHÔNG xóa ADDR). */
static uint8_t I2C1_WaitFlag(uint32_t flag, FlagStatus state, uint8_t err)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tmo   = DWT_UsToCycles(I2C1_TIMEOUT_US);

    while (I2C_GetFlagStatus(I2C1, flag) != state)
    {
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            return err;
        }
        if ((DWT->CYCCNT - start) > tmo)
        {
            return err;
        }
    }
    return I2C_OK;
}

/* Kết thúc giao dịch lỗi: LUÔN phát STOP để nhả bus, khôi phục ACK/POS. */
static uint8_t I2C1_Abort(uint8_t err)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    return err;
}

static void I2C1_ClearAddr(void)
{
    volatile uint16_t tmp;
    tmp = I2C1->SR1;
    tmp = I2C1->SR2;
    (void)tmp;
}

/* =====================================================================
 *  I2C1 - KHỞI TẠO & PHỤC HỒI BUS
 * ===================================================================== */
static void I2C1_HwInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    DWT_DelayInit();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(I2C1_PORT, I2C1_SCL_PINSRC, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_PORT, I2C1_SDA_PINSRC, GPIO_AF_I2C1);

    I2C_DeInit(I2C1);   /* Reset ngoại vi qua RCC: xóa sạch trạng thái lỗi */

    I2C_InitStructure.I2C_ClockSpeed          = I2C1_SPEED_HZ;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

/**
 * @brief Giải phóng bus khi slave giữ SDA thấp (thường do MCU reset/nhiễu motor
 *        giữa lúc đang đọc). Tạo 9 xung SCL + STOP bằng GPIO, rồi khởi tạo lại I2C1.
 */
void I2C1_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    DWT_DelayInit();
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    I2C_Cmd(I2C1, DISABLE);

    /* Chuyển SCL/SDA sang GPIO open-drain để điều khiển tay */
    GPIO_SetBits(I2C1_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN);
    GPIO_InitStructure.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);
    DWT_DelayUs(5u);

    /* 9 xung clock: slave sẽ đẩy nốt bit đang dở và nhả SDA */
    for (i = 0u; i < 9u; i++)
    {
        GPIO_ResetBits(I2C1_PORT, I2C1_SCL_PIN);
        DWT_DelayUs(5u);
        GPIO_SetBits(I2C1_PORT, I2C1_SCL_PIN);
        DWT_DelayUs(5u);
    }

    /* Điều kiện STOP: SDA từ thấp lên cao khi SCL đang cao */
    GPIO_ResetBits(I2C1_PORT, I2C1_SCL_PIN);
    DWT_DelayUs(5u);
    GPIO_ResetBits(I2C1_PORT, I2C1_SDA_PIN);
    DWT_DelayUs(5u);
    GPIO_SetBits(I2C1_PORT, I2C1_SCL_PIN);
    DWT_DelayUs(5u);
    GPIO_SetBits(I2C1_PORT, I2C1_SDA_PIN);
    DWT_DelayUs(5u);

    s_recoverCnt++;
    I2C1_HwInit();
}

/* Đảm bảo bus rảnh trước START; nếu kẹt thì tự phục hồi 1 lần. */
static uint8_t I2C1_WaitIdle(void)
{
    if (I2C1_WaitFlag(I2C_FLAG_BUSY, RESET, I2C_ERR_BUSY) == I2C_OK)
    {
        return I2C_OK;
    }
    I2C1_BusRecover();
    return I2C1_WaitFlag(I2C_FLAG_BUSY, RESET, I2C_ERR_BUSY);
}

/* =====================================================================
 *  I2C1 - GHI / ĐỌC THANH GHI
 * ===================================================================== */
uint8_t I2C1_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    uint8_t st = I2C1_WaitIdle();
    if (st != I2C_OK)
    {
        return st;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_ERR_START))
    {
        return I2C1_Abort(I2C_ERR_START);
    }

    I2C_Send7bitAddress(I2C1, (uint8_t)(slaveAddr << 1), I2C_Direction_Transmitter);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_ERR_ADDR))
    {
        return I2C1_Abort(I2C_ERR_ADDR);
    }

    I2C_SendData(I2C1, regAddr);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING, I2C_ERR_TX))
    {
        return I2C1_Abort(I2C_ERR_TX);
    }

    I2C_SendData(I2C1, data);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_ERR_TX))
    {
        return I2C1_Abort(I2C_ERR_TX);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return I2C_OK;
}

uint8_t I2C1_ReadReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
    return I2C1_ReadRegs(slaveAddr, regAddr, data, 1u);
}

/**
 * @brief Đọc nhiều byte theo ĐÚNG trình tự trong Reference Manual RM0090
 *        (mục I2C master receiver) cho 3 trường hợp N=1, N=2, N>2.
 *        Trình tự cũ (tắt ACK ngay trước byte cuối) có thể bị lỡ nhịp nếu bị
 *        ngắt chen vào -> slave không nhận NACK -> treo bus.
 */
uint8_t I2C1_ReadRegs(uint8_t slaveAddr, uint8_t startReg, uint8_t *data, uint8_t len)
{
    uint8_t st;
    uint8_t idx = 0u;

    if ((data == 0) || (len == 0u))
    {
        return I2C_ERR_PARAM;
    }

    st = I2C1_WaitIdle();
    if (st != I2C_OK)
    {
        return st;
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

    /* ---- Pha 1: gửi địa chỉ thanh ghi ---- */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_ERR_START))
    {
        return I2C1_Abort(I2C_ERR_START);
    }

    I2C_Send7bitAddress(I2C1, (uint8_t)(slaveAddr << 1), I2C_Direction_Transmitter);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_ERR_ADDR))
    {
        return I2C1_Abort(I2C_ERR_ADDR);
    }

    I2C_SendData(I2C1, startReg);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_ERR_TX))
    {
        return I2C1_Abort(I2C_ERR_TX);
    }

    /* ---- Pha 2: Repeated START + đọc ---- */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_ERR_START))
    {
        return I2C1_Abort(I2C_ERR_START);
    }

    if (len == 1u)
    {
        I2C_AcknowledgeConfig(I2C1, DISABLE);           /* NACK phải set TRƯỚC khi xóa ADDR */
    }
    else if (len == 2u)
    {
        I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
    }

    I2C_Send7bitAddress(I2C1, (uint8_t)(slaveAddr << 1), I2C_Direction_Receiver);
    if (I2C1_WaitFlag(I2C_FLAG_ADDR, SET, I2C_ERR_ADDR))
    {
        return I2C1_Abort(I2C_ERR_ADDR);
    }

    if (len == 1u)
    {
        __disable_irq();
        I2C1_ClearAddr();
        I2C_GenerateSTOP(I2C1, ENABLE);
        __enable_irq();

        if (I2C1_WaitFlag(I2C_FLAG_RXNE, SET, I2C_ERR_RX))
        {
            return I2C1_Abort(I2C_ERR_RX);
        }
        data[0] = I2C_ReceiveData(I2C1);
    }
    else if (len == 2u)
    {
        __disable_irq();
        I2C1_ClearAddr();
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        __enable_irq();

        if (I2C1_WaitFlag(I2C_FLAG_BTF, SET, I2C_ERR_RX))
        {
            return I2C1_Abort(I2C_ERR_RX);
        }
        __disable_irq();
        I2C_GenerateSTOP(I2C1, ENABLE);
        data[0] = I2C_ReceiveData(I2C1);
        __enable_irq();
        data[1] = I2C_ReceiveData(I2C1);
    }
    else
    {
        I2C1_ClearAddr();

        /* Đọc bình thường cho tới khi còn 3 byte */
        while ((uint8_t)(len - idx) > 3u)
        {
            if (I2C1_WaitFlag(I2C_FLAG_RXNE, SET, I2C_ERR_RX))
            {
                return I2C1_Abort(I2C_ERR_RX);
            }
            data[idx++] = I2C_ReceiveData(I2C1);
        }

        /* Còn 3 byte: DataN-2 trong DR, DataN-1 trong shift register (BTF=1) */
        if (I2C1_WaitFlag(I2C_FLAG_BTF, SET, I2C_ERR_RX))
        {
            return I2C1_Abort(I2C_ERR_RX);
        }
        I2C_AcknowledgeConfig(I2C1, DISABLE);

        __disable_irq();
        data[idx++] = I2C_ReceiveData(I2C1);            /* DataN-2 */
        if (I2C1_WaitFlag(I2C_FLAG_BTF, SET, I2C_ERR_RX))
        {
            __enable_irq();
            return I2C1_Abort(I2C_ERR_RX);
        }
        I2C_GenerateSTOP(I2C1, ENABLE);
        data[idx++] = I2C_ReceiveData(I2C1);            /* DataN-1 */
        __enable_irq();

        if (I2C1_WaitFlag(I2C_FLAG_RXNE, SET, I2C_ERR_RX))
        {
            return I2C1_Abort(I2C_ERR_RX);
        }
        data[idx++] = I2C_ReceiveData(I2C1);            /* DataN */
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    return I2C_OK;
}

/* =====================================================================
 *  MPU6050
 * ===================================================================== */
uint8_t I2C1_MPU6050_Init(void)
{
    static const uint8_t cfg[][2] = {
        { MPU6050_REG_PWR_MGMT_1,   MPU6050_PWR_MGMT_1_CLK_PLL_X },
        { MPU6050_REG_SMPLRT_DIV,   MPU6050_SMPLRT_DIV_VAL       },
        { MPU6050_REG_CONFIG,       MPU6050_DLPF_CFG             },
        { MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_FS              },
        { MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS             },
    };
    uint8_t i;
    uint8_t st;
    uint8_t readback = 0u;

    /* Gỡ kẹt bus (nếu MCU vừa reset giữa giao dịch) đồng thời khởi tạo I2C1 */
    I2C1_BusRecover();
    s_recoverCnt = 0u;
    DWT_DelayMs(50u);

    /* 1. Kiểm tra cảm biến có trên bus không */
    st = I2C1_ReadReg(MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, &s_whoAmI);
    if ((st != I2C_OK) || (s_whoAmI == 0x00u) || (s_whoAmI == 0xFFu))
    {
        return MPU_ERR_WHOAMI;   /* 0x68 = MPU6050 thật; module clone có thể trả 0x70/0x72/0x98 */
    }

    /* 2. Reset toàn bộ thanh ghi về mặc định */
    st = I2C1_WriteReg(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);
    if (st != I2C_OK)
    {
        return st;
    }
    DWT_DelayMs(100u);

    /* 3. Ghi cấu hình và ĐỌC LẠI để chắc chắn dải đo khớp với hệ số chia */
    for (i = 0u; i < (uint8_t)(sizeof(cfg) / sizeof(cfg[0])); i++)
    {
        st = I2C1_WriteReg(MPU6050_I2C_ADDR, cfg[i][0], cfg[i][1]);
        if (st != I2C_OK)
        {
            return st;
        }
        st = I2C1_ReadReg(MPU6050_I2C_ADDR, cfg[i][0], &readback);
        if (st != I2C_OK)
        {
            return st;
        }
        if (readback != cfg[i][1])
        {
            return MPU_ERR_CONFIG;
        }
    }

    DWT_DelayMs(20u);   /* Chờ DLPF ổn định */
    s_errTotal  = 0u;
    s_errConsec = 0u;
    return I2C_OK;
}

uint8_t MPU6050_Read(MPU6050_Data_t *d)
{
    uint8_t buf[14];
    uint8_t st;

    if (d == 0)
    {
        return I2C_ERR_PARAM;
    }

    st = I2C1_ReadRegs(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, 14u);
    if (st != I2C_OK)
    {
        s_errTotal++;
        if (++s_errConsec >= MPU6050_MAX_CONSEC_ERR)
        {
            I2C1_BusRecover();
            s_errConsec = 0u;
        }
        return st;
    }
    s_errConsec = 0u;

    d->ax_raw   = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    d->ay_raw   = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    d->az_raw   = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    d->temp_raw = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    d->gx_raw   = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    d->gy_raw   = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    d->gz_raw   = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    d->ax = ((float)d->ax_raw / MPU6050_ACCEL_SENS - s_accOffset[0]) * s_accScale[0];
    d->ay = ((float)d->ay_raw / MPU6050_ACCEL_SENS - s_accOffset[1]) * s_accScale[1];
    d->az = ((float)d->az_raw / MPU6050_ACCEL_SENS - s_accOffset[2]) * s_accScale[2];

    d->gx = (float)d->gx_raw / MPU6050_GYRO_SENS - s_gyroOffset[0];
    d->gy = (float)d->gy_raw / MPU6050_GYRO_SENS - s_gyroOffset[1];
    d->gz = (float)d->gz_raw / MPU6050_GYRO_SENS - s_gyroOffset[2];

    d->temp = (float)d->temp_raw / 340.0f + 36.53f;

    return I2C_OK;
}

uint8_t MPU6050_CalibrateGyro(uint16_t samples)
{
    MPU6050_Data_t d;
    float sum[3]   = {0.0f, 0.0f, 0.0f};
    float vmin[3]  = {0.0f, 0.0f, 0.0f};
    float vmax[3]  = {0.0f, 0.0f, 0.0f};
    float saved[3];
    uint16_t i;
    uint16_t ok = 0u;
    uint8_t k;

    if (samples == 0u)
    {
        return I2C_ERR_PARAM;
    }

    for (k = 0u; k < 3u; k++)
    {
        saved[k] = s_gyroOffset[k];
        s_gyroOffset[k] = 0.0f;
    }

    for (i = 0u; i < samples; i++)
    {
        if (MPU6050_Read(&d) == I2C_OK)
        {
            float v[3];
            v[0] = d.gx;
            v[1] = d.gy;
            v[2] = d.gz;
            for (k = 0u; k < 3u; k++)
            {
                sum[k] += v[k];
                if ((ok == 0u) || (v[k] < vmin[k])) vmin[k] = v[k];
                if ((ok == 0u) || (v[k] > vmax[k])) vmax[k] = v[k];
            }
            ok++;
        }
        DWT_DelayMs(MPU6050_SAMPLE_PERIOD_MS);
    }

    if (ok < (samples / 2u))
    {
        for (k = 0u; k < 3u; k++) s_gyroOffset[k] = saved[k];
        return I2C_ERR_RX;
    }

    for (k = 0u; k < 3u; k++)
    {
        if ((vmax[k] - vmin[k]) > MPU6050_CALIB_MAX_RANGE_DPS)
        {
            for (k = 0u; k < 3u; k++) s_gyroOffset[k] = saved[k];
            return MPU_ERR_MOVING;
        }
    }

    for (k = 0u; k < 3u; k++)
    {
        s_gyroOffset[k] = sum[k] / (float)ok;
    }
    return I2C_OK;
}

void MPU6050_SetGyroOffset(float gx, float gy, float gz)
{
    s_gyroOffset[0] = gx;
    s_gyroOffset[1] = gy;
    s_gyroOffset[2] = gz;
}

void MPU6050_GetGyroOffset(float *gx, float *gy, float *gz)
{
    if (gx) *gx = s_gyroOffset[0];
    if (gy) *gy = s_gyroOffset[1];
    if (gz) *gz = s_gyroOffset[2];
}

void MPU6050_SetAccelCalib(const float offset[3], const float scale[3])
{
    uint8_t k;
    for (k = 0u; k < 3u; k++)
    {
        s_accOffset[k] = offset ? offset[k] : 0.0f;
        s_accScale[k]  = scale  ? scale[k]  : 1.0f;
    }
}

void MPU6050_GetAccelCalib(float offset[3], float scale[3])
{
    uint8_t k;
    for (k = 0u; k < 3u; k++)
    {
        if (offset) offset[k] = s_accOffset[k];
        if (scale)  scale[k]  = s_accScale[k];
    }
}

float MPU6050_AccelPitch(const MPU6050_Data_t *d)
{
    return atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD2DEG;
}

float MPU6050_AccelRoll(const MPU6050_Data_t *d)
{
    return atan2f(d->ay, d->az) * RAD2DEG;
}

uint8_t  MPU6050_GetWhoAmI(void)       { return s_whoAmI; }
uint32_t MPU6050_GetErrorCount(void)   { return s_errTotal; }
uint32_t MPU6050_GetRecoverCount(void) { return s_recoverCnt; }

/* =====================================================================
 *  API CŨ - giữ để tương thích
 * ===================================================================== */
uint8_t MPU6050_ReadRawData(int16_t *accX, int16_t *accY, int16_t *accZ,
                            int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
    MPU6050_Data_t d;
    uint8_t st;

    if (accX == 0 || accY == 0 || accZ == 0 || gyroX == 0 || gyroY == 0 || gyroZ == 0)
    {
        return I2C_ERR_PARAM;
    }

    st = MPU6050_Read(&d);
    if (st != I2C_OK)
    {
        return st;
    }

    *accX  = d.ax_raw;
    *accY  = d.ay_raw;
    *accZ  = d.az_raw;
    *gyroX = d.gx_raw;
    *gyroY = d.gy_raw;
    *gyroZ = d.gz_raw;
    return I2C_OK;
}

float MPU6050_GetRoll(void)
{
    MPU6050_Data_t d;
    if (MPU6050_Read(&d) != I2C_OK)
    {
        return 0.0f;
    }
    return MPU6050_AccelRoll(&d);
}

float MPU6050_GetPitch(void)
{
    MPU6050_Data_t d;
    if (MPU6050_Read(&d) != I2C_OK)
    {
        return 0.0f;
    }
    return MPU6050_AccelPitch(&d);
}

float MPU6050_GetYaw(void)
{
    static float yaw = 0.0f;
    MPU6050_Data_t d;

    if (MPU6050_Read(&d) != I2C_OK)
    {
        return yaw;
    }

    /* Tích phân gz (đã đúng hệ số độ nhạy). Giả định hàm được gọi mỗi 10ms. */
    yaw += d.gz * 0.01f;
    return yaw;
}

float MPU6050_GetHeading(void)
{
    /* On MPU6050, heading is derived from yaw around Z as a proxy. */
    return MPU6050_GetYaw();
}
