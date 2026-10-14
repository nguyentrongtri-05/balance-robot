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
void KalmanAngle_Init(Kalman_t *klm);

/**
 * @brief Cập nhật và tính toán góc nghiêng thông qua bộ lọc
 * @param klm: Con trỏ đến struct Kalman
 * @param newAngle: Góc tính được từ Gia tốc kế (Accelerometer)
 * @param newRate: Vận tốc góc đọc được từ Gyroscope (độ/s)
 * @param dt: Thời gian lấy mẫu (Delta Time, ví dụ: 0.005s cho 200Hz)
 * @retval Góc đã được lọc nhiễu (Angle)
 */
float Kalman_GetAngle(Kalman_t *klm, float newAngle, float newRate, float dt);

/**
 * @brief Đặt góc bắt đầu cho bộ lọc (First Sample) để triệt tiêu thời gian trễ quá độ
 * @param klm: Con trỏ đến struct Kalman
 * @param angle: Góc đo ban đầu từ Accelerometer
 */
void Kalman_SetAngle(Kalman_t *klm, float angle);

/* =====================================================================
 * STRUCT & HÀM CHO BỘ LỌC KALMAN VẬN TỐC 2 BÁNH XE (WHEEL SPEED KALMAN FILTER)
 * Mô hình không gian trạng thái 2 chiều:
 *   - Vector trạng thái: x = [speed, accel]^T (RPM và RPM/s)
 *   - Dự đoán (Predict):
 *       speed[k] = speed[k-1] + accel[k-1] * dt
 *       accel[k] = accel[k-1]
 *   - Đo lường (Update): z = rawSpeed từ Encoder
 * Ưu điểm:
 *   - Triệt tiêu trễ pha (Zero Phase Lag) khi tăng tốc hoặc hãm tốc
 *   - Lọc sạch nhiễu lượng tử hóa bước rời rạc (~4.55 RPM/pulse) từ Encoder
 *   - Khử hoàn toàn xung nhọn PWM tại ngõ ra của PID
 * ===================================================================== */
typedef struct {
    float Q_speed;      /* Process noise variance cho vận tốc */
    float Q_accel;      /* Process noise variance cho gia tốc */
    float R_measure;    /* Measurement noise variance (nhiễu lượng tử hóa từ encoder) */
    
    float speed;        /* Vận tốc ước lượng sau lọc (RPM) */
    float accel;        /* Gia tốc ước lượng (RPM/s) */
    
    float P[2][2];      /* Ma trận hiệp phương sai sai số */
} KalmanSpeed_t;

/**
 * @brief Khởi tạo thông số cho bộ lọc Kalman vận tốc bánh xe
 * @param klm: Con trỏ đến struct KalmanSpeed
 * @param q_speed: Phương sai nhiễu quá trình vận tốc (khuyến nghị: 0.05f)
 * @param q_accel: Phương sai nhiễu quá trình gia tốc (khuyến nghị: 50.0f)
 * @param r_measure: Phương sai nhiễu đo lường encoder (khuyến nghị: 5.0f)
 */
void KalmanSpeed_Init(KalmanSpeed_t *klm, float q_speed, float q_accel, float r_measure);

/**
 * @brief Khởi tạo bộ lọc Kalman vận tốc với bộ thông số tối ưu mặc định
 */
void KalmanSpeed_InitDefault(KalmanSpeed_t *klm);

/**
 * @brief Cập nhật và tính toán vận tốc bánh xe qua bộ lọc Kalman
 * @param klm: Con trỏ đến struct KalmanSpeed
 * @param rawSpeed: Vận tốc đo thô từ bộ đếm xung encoder (RPM)
 * @param dt: Chu kỳ trích mẫu (ví dụ: 0.010s)
 * @return Vận tốc đã lọc mượt (RPM)
 */
float KalmanSpeed_GetSpeed(KalmanSpeed_t *klm, float rawSpeed, float dt);

/**
 * @brief Đặt vận tốc ban đầu cho bộ lọc Kalman bánh xe (First Sample / Reset)
 * @param klm: Con trỏ đến struct KalmanSpeed
 * @param speed: Vận tốc khởi tạo (RPM)
 */
void KalmanSpeed_SetSpeed(KalmanSpeed_t *klm, float speed);

#endif /* __KALMAN_H */

