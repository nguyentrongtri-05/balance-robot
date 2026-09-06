#include "imu.h"
#include <math.h>

/* Khai báo các biến sẽ chứa dữ liệu RAW đọc từ thanh ghi I2C/SPI của IMU (VD: MPU6050/MPU9250) */
int16_t accel_x_raw = 0;
int16_t accel_y_raw = 0;
int16_t accel_z_raw = 0;

int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;

/* Các hệ số chuyển đổi tuỳ thuộc vào cấu hình tầm đo (Range) của cảm biến 
 * Ví dụ MPU6050: +-2g -> 16384 LSB/g, +-250deg/s -> 131 LSB/deg/s */
const float ACC_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;

void IMU_Config(void) {
    /* 1. Cấu hình I2C hoặc SPI ngoại vi ở đây */
    // I2C_Init(...);
    
    /* 2. Ghi vào các thanh ghi của IMU để đánh thức và cấu hình tầm đo */
    // I2C_WriteReg(IMU_ADDR, PWR_MGMT_1, 0x00); 
    // I2C_WriteReg(IMU_ADDR, ACCEL_CONFIG, 0x00); // +-2g
    // I2C_WriteReg(IMU_ADDR, GYRO_CONFIG, 0x00);  // +-250 deg/s
}

void IMU_Read(float *accAngle, float *gyroRate) {
    /* 1. Đọc dữ liệu thô từ các thanh ghi của IMU */
    // accel_x_raw = (I2C_ReadReg(IMU_ADDR, 0x3B) << 8) | I2C_ReadReg(IMU_ADDR, 0x3C);
    // accel_y_raw = (I2C_ReadReg(IMU_ADDR, 0x3D) << 8) | I2C_ReadReg(IMU_ADDR, 0x3E);
    // accel_z_raw = (I2C_ReadReg(IMU_ADDR, 0x3F) << 8) | I2C_ReadReg(IMU_ADDR, 0x40);
    // ... tương tự cho gyro ...

    /* 2. Chuyển đổi dữ liệu RAW sang đơn vị vật lý (g và độ/s) */
    float ay = (float)accel_y_raw / ACC_SCALE;
    float az = (float)accel_z_raw / ACC_SCALE;
    float gx = (float)gyro_x_raw / GYRO_SCALE; /* Giả sử trục X là trục nghiêng (Pitch) */

    /* 3. Tính toán góc tĩnh từ gia tốc kế (Dùng atan2) */
    /* Công thức tính góc nghiêng cơ bản từ trục Y và Z */
    float pitch_acc = atan2f(ay, az) * 180.0f / 3.14159265f;
    
    /* 4. Trả dữ liệu về */
    if (accAngle) *accAngle = pitch_acc;
    if (gyroRate) *gyroRate = gx; // Trả về vận tốc góc trục X (độ/s)
}
