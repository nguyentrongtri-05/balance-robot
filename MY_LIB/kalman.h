#ifndef __KALMAN_H
#define __KALMAN_H

/* Struct cho bộ lọc Kalman 1 chiều (1D Kalman Filter) */
typedef struct {
    float Q_angle;      /* Process noise variance cho góc */
    float Q_bias;       /* Process noise variance cho độ trôi (bias) của gyro */
    float R_measure;    /* Measurement noise variance (nhiễu đo lường từ gia tốc kế) */
    
    float angle;        /* Góc tính toán được từ bộ lọc */
    float bias;         /* Độ trôi (bias) của Gyro tính được */
    float rate;         /* Vận tốc góc sau khi đã khử bias */
    
    float P[2][2];      /* Ma trận hiệp phương sai sai số (Error covariance matrix) */
} Kalman_t;

/**
 * @brief Khởi tạo các thông số mặc định cho bộ lọc Kalman
 */
void Kalman_Init(Kalman_t *klm);

/**
 * @brief Cập nhật và tính toán góc nghiêng thông qua bộ lọc
 * @param klm: Con trỏ đến struct Kalman
 * @param newAngle: Góc tính được từ Gia tốc kế (Accelerometer)
 * @param newRate: Vận tốc góc đọc được từ Gyroscope (độ/s)
 * @param dt: Thời gian lấy mẫu (Delta Time, ví dụ: 0.005s cho 200Hz)
 * @retval Góc đã được lọc nhiễu (Angle)
 */
float Kalman_GetAngle(Kalman_t *klm, float newAngle, float newRate, float dt);

#endif /* __KALMAN_H */

