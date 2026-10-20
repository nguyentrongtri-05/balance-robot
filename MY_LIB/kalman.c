#include "kalman.h"

void KalmanAngle_Init(Kalman_t *klm) {
    /* Các giá trị phương sai (Variance) này có thể cần tinh chỉnh theo thực tế của IMU */
    klm->Q_angle = 0.001f;
    klm->Q_bias = 0.003f;
    klm->R_measure = 0.03f;

    klm->angle = 0.0f;
    klm->bias = 0.0f;

    klm->P[0][0] = 0.0f;
    klm->P[0][1] = 0.0f;
    klm->P[1][0] = 0.0f;
    klm->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *klm, float newAngle, float newRate, float dt) {
    /* 1. Predict (Dự đoán) */
    klm->rate = newRate - klm->bias;
    klm->angle += dt * klm->rate;

    /* Cập nhật ma trận hiệp phương sai P (Error covariance) */
    klm->P[0][0] += dt * (dt * klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
    klm->P[0][1] -= dt * klm->P[1][1];
    klm->P[1][0] -= dt * klm->P[1][1];
    klm->P[1][1] += klm->Q_bias * dt;

    /* 2. Correct (Cập nhật đo lường) */
    float S = klm->P[0][0] + klm->R_measure; // Sai số ước lượng
    float K[2]; // Hệ số Kalman (Kalman Gain)
    K[0] = klm->P[0][0] / S;
    K[1] = klm->P[1][0] / S;

    /* Tính toán sai lệch góc và cập nhật lại góc/bias */
    float y = newAngle - klm->angle;
    klm->angle += K[0] * y;
    klm->bias += K[1] * y;

    /* Cập nhật lại ma trận P */
    float P00_temp = klm->P[0][0];
    float P01_temp = klm->P[0][1];

    klm->P[0][0] -= K[0] * P00_temp;
    klm->P[0][1] -= K[0] * P01_temp;
    klm->P[1][0] -= K[1] * P00_temp;
    klm->P[1][1] -= K[1] * P01_temp;

    return klm->angle;
}

void Kalman_SetAngle(Kalman_t *klm, float angle) {
    klm->angle = angle;
}

/* =====================================================================
 * CÁC HÀM THỰC THI BỘ LỌC KALMAN CHO VẬN TỐC 2 BÁNH XE
 * ===================================================================== */

void KalmanSpeed_Init(KalmanSpeed_t *klm, float q_speed, float q_accel, float r_measure) {
    klm->Q_speed   = q_speed;
    klm->Q_accel   = q_accel;
    klm->R_measure = r_measure;

    klm->speed = 0.0f;
    klm->accel = 0.0f;

    klm->P[0][0] = 1.0f;
    klm->P[0][1] = 0.0f;
    klm->P[1][0] = 0.0f;
    klm->P[1][1] = 1.0f;
}

void KalmanSpeed_InitDefault(KalmanSpeed_t *klm) {
    /* Bộ tham số tối ưu cho động cơ JGB37-520 ở tần số lấy mẫu 200Hz (dt = 5ms):
     * Q_speed   = 0.08f  (Bám nhanh vận tốc thực ở 200Hz)
     * Q_accel   = 15.0f  (Giảm vọt lố đạo hàm gia tốc khi lượng tử hóa 1 xung = 9.09 RPM)
     * R_measure = 14.0f  (Làm mượt bậc thang lượng tử hóa xung ở chu kỳ 5ms)
     */
    /* Bộ số cho tần số cập nhật 50Hz (dt = 20ms, 1 xung = 2.27 RPM): chọn bằng mô phỏng so với
     * bộ cũ cho 200Hz (0.08, 15, 14). Giữ bộ cũ ở 50Hz sẽ làm tốc độ trễ ~120ms thay vì 50ms. */
    KalmanSpeed_Init(klm, 0.32f, 60.0f, 1.0f);
}

float KalmanSpeed_GetSpeed(KalmanSpeed_t *klm, float rawSpeed, float dt) {
    /* 1. Predict (Dự đoán trạng thái & ma trận hiệp phương sai với hệ số tắt dần gia tốc 0.90) */
    float accel_pred = klm->accel * 0.90f;
    float speed_pred = klm->speed + accel_pred * dt;

    /* P^- = F * P * F^T + Q
     * Ma trận F = [1  dt; 0  1]
     */
    float p00_temp = klm->P[0][0] + dt * (2.0f * klm->P[0][1] + dt * klm->P[1][1]) + klm->Q_speed;
    float p01_temp = klm->P[0][1] + dt * klm->P[1][1];
    float p11_temp = klm->P[1][1] + klm->Q_accel * dt;

    /* 2. Correct / Update (Cập nhật đo lường từ Encoder) */
    float S = p00_temp + klm->R_measure;
    float K0 = p00_temp / S;
    float K1 = p01_temp / S;

    /* Sai lệch đo lường (Innovation) */
    float y = rawSpeed - speed_pred;

    /* Cập nhật vector trạng thái ước lượng */
    klm->speed = speed_pred + K0 * y;
    klm->accel = accel_pred + K1 * y;

    /* Cập nhật ma trận hiệp phương sai sai số: P = (I - K*H) * P^- */
    klm->P[0][0] = (1.0f - K0) * p00_temp;
    klm->P[0][1] = (1.0f - K0) * p01_temp;
    klm->P[1][0] = klm->P[0][1];
    klm->P[1][1] = p11_temp - K1 * p01_temp;

    return klm->speed;
}

void KalmanSpeed_SetSpeed(KalmanSpeed_t *klm, float speed) {
    klm->speed = speed;
    klm->accel = 0.0f;

    klm->P[0][0] = 1.0f;
    klm->P[0][1] = 0.0f;
    klm->P[1][0] = 0.0f;
    klm->P[1][1] = 1.0f;
}

