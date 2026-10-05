#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#include "stm32f4xx.h"

/* =====================================================================
 *  PHẦN CỨNG I2C1: PB8 = SCL, PB9 = SDA (Open-Drain, 400 kHz)
 * ===================================================================== */
#define I2C1_PORT               GPIOB
#define I2C1_SCL_PIN            GPIO_Pin_8
#define I2C1_SDA_PIN            GPIO_Pin_9
#define I2C1_SCL_PINSRC         GPIO_PinSource8
#define I2C1_SDA_PINSRC         GPIO_PinSource9
#define I2C1_SPEED_HZ           400000u

/* =====================================================================
 *  MÃ LỖI (0 = OK, giữ tương thích với code cũ so sánh == 0u)
 * ===================================================================== */
#define I2C_OK                  0u
#define I2C_ERR_BUSY            1u   /* Bus bận (SDA bị giữ thấp)        */
#define I2C_ERR_START           2u   /* Không tạo được START             */
#define I2C_ERR_ADDR            3u   /* Slave không ACK địa chỉ (NACK)   */
#define I2C_ERR_TX              4u   /* Lỗi khi gửi byte                 */
#define I2C_ERR_RX              5u   /* Lỗi khi nhận byte                */
#define I2C_ERR_PARAM           6u   /* Tham số không hợp lệ             */
#define MPU_ERR_WHOAMI          7u   /* Không thấy MPU6050 trên bus      */
#define MPU_ERR_CONFIG          8u   /* Ghi cấu hình xong đọc lại sai    */
#define MPU_ERR_MOVING          9u   /* Xe bị rung/di chuyển khi hiệu chuẩn */

/* =====================================================================
 *  MPU6050 REGISTER MAP
 * ===================================================================== */
#define MPU6050_I2C_ADDR       0x68
#define MPU6050_I2C_READ       0x01
#define MPU6050_I2C_WRITE      0x00

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
#define MPU6050_PWR_MGMT_1_CLK_PLL_X   0x01  /* Clock từ PLL gyro X: ổn định hơn RC nội 8MHz */

/* Gyro and accel full scale config */
#define MPU6050_GYRO_FS_250_DPS        0x00
#define MPU6050_GYRO_FS_500_DPS        0x08
#define MPU6050_GYRO_FS_1000_DPS       0x10
#define MPU6050_GYRO_FS_2000_DPS       0x18

#define MPU6050_ACCEL_FS_2G            0x00
#define MPU6050_ACCEL_FS_4G            0x08
#define MPU6050_ACCEL_FS_8G            0x10
#define MPU6050_ACCEL_FS_16G           0x18

/* =====================================================================
 *  CẤU HÌNH NGƯỜI DÙNG - CHỈ SỬA Ở ĐÂY
 *  Độ nhạy (LSB/đơn vị) được suy ra TỰ ĐỘNG từ dải đo => không bao giờ
 *  lệch giữa cấu hình thanh ghi và hệ số chia nữa.
 * ===================================================================== */
#define MPU6050_GYRO_FS         MPU6050_GYRO_FS_500_DPS
#define MPU6050_ACCEL_FS        MPU6050_ACCEL_FS_2G
#define MPU6050_DLPF_CFG        0x03   /* 44Hz BW, trễ ~4.9ms. 0x02 = 94Hz (trễ ~3ms) */
#define MPU6050_SMPLRT_DIV_VAL  0x04   /* 1kHz / (1+4) = 200Hz = 5ms, khớp chu kỳ task IMU */
#define MPU6050_MAX_CONSEC_ERR  3u     /* Lỗi liên tiếp bao nhiêu lần thì tự phục hồi bus */

#if   (MPU6050_GYRO_FS == MPU6050_GYRO_FS_250_DPS)
  #define MPU6050_GYRO_SENS     131.0f
#elif (MPU6050_GYRO_FS == MPU6050_GYRO_FS_500_DPS)
  #define MPU6050_GYRO_SENS     65.5f
#elif (MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000_DPS)
  #define MPU6050_GYRO_SENS     32.8f
#else
  #define MPU6050_GYRO_SENS     16.4f
#endif

#if   (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2G)
  #define MPU6050_ACCEL_SENS    16384.0f
#elif (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4G)
  #define MPU6050_ACCEL_SENS    8192.0f
#elif (MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8G)
  #define MPU6050_ACCEL_SENS    4096.0f
#else
  #define MPU6050_ACCEL_SENS    2048.0f
#endif

/* =====================================================================
 *  DỮ LIỆU ĐÃ QUY ĐỔI
 * ===================================================================== */
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t temp_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float ax, ay, az;   /* Gia tốc [g]                         */
    float gx, gy, gz;   /* Vận tốc góc [deg/s], đã trừ offset  */
    float temp;         /* Nhiệt độ [°C]                       */
} MPU6050_Data_t;

/* =====================================================================
 *  TIỆN ÍCH THỜI GIAN (DWT cycle counter - dùng được cả trước scheduler)
 * ===================================================================== */
void     DWT_DelayInit(void);
void     DWT_DelayUs(uint32_t us);
void     DWT_DelayMs(uint32_t ms);

/* =====================================================================
 *  I2C1 LOW-LEVEL
 * ===================================================================== */
void    I2C1_BusRecover(void);
uint8_t I2C1_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C1_ReadReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
uint8_t I2C1_ReadRegs(uint8_t slaveAddr, uint8_t startReg, uint8_t *data, uint8_t len);

/* =====================================================================
 *  MPU6050 API
 * ===================================================================== */
/**
 * @brief  Khởi tạo I2C1 + MPU6050 (reset, kiểm tra WHO_AM_I, ghi & đọc lại cấu hình).
 *         Gọi TRƯỚC vTaskStartScheduler (có dùng delay chặn).
 * @retval 0 nếu OK, ngược lại là mã lỗi I2C_ERR_x / MPU_ERR_x
 */
uint8_t I2C1_MPU6050_Init(void);

/**
 * @brief  Đọc 14 byte (accel + temp + gyro) trong MỘT giao dịch và quy đổi đơn vị.
 *         Tự động phục hồi bus nếu lỗi liên tiếp MPU6050_MAX_CONSEC_ERR lần.
 */
uint8_t MPU6050_Read(MPU6050_Data_t *d);

/**
 * @brief  Đo offset gyro khi xe ĐỨNG YÊN (chặn ~samples * 5ms).
 * @retval MPU_ERR_MOVING nếu phát hiện rung lắc, offset giữ nguyên giá trị cũ.
 */
uint8_t MPU6050_CalibrateGyro(uint16_t samples);
void    MPU6050_SetGyroOffset(float gx, float gy, float gz);
void    MPU6050_GetGyroOffset(float *gx, float *gy, float *gz);

/**
 * @brief  Hiệu chuẩn gia tốc kế (từ quy trình 6 mặt trong imu_calib.c):
 *         a_cal[k] = (a_raw[k] - offset[k]) * scale[k]     [g]
 *         Mặc định offset = 0, scale = 1.
 */
void    MPU6050_SetAccelCalib(const float offset[3], const float scale[3]);
void    MPU6050_GetAccelCalib(float offset[3], float scale[3]);

/** Góc tính từ gia tốc kế của mẫu ĐÃ đọc (không đọc lại I2C) [deg] */
float   MPU6050_AccelPitch(const MPU6050_Data_t *d);   /* quay quanh trục Y -> ghép với gy */
float   MPU6050_AccelRoll(const MPU6050_Data_t *d);    /* quay quanh trục X -> ghép với gx */

/** Chẩn đoán */
uint8_t  MPU6050_GetWhoAmI(void);
uint32_t MPU6050_GetErrorCount(void);
uint32_t MPU6050_GetRecoverCount(void);

/* ---- API cũ (giữ để tương thích). Mỗi hàm tự đọc I2C 1 lần ---- */
uint8_t MPU6050_ReadRawData(int16_t *accX, int16_t *accY, int16_t *accZ,
                            int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);
float MPU6050_GetRoll(void);
float MPU6050_GetPitch(void);
float MPU6050_GetYaw(void);
float MPU6050_GetHeading(void);

#endif /* __DRIVER_I2C_H */
