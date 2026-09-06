#include "kalman.h"

void Kalman_Init(Kalman_t *klm) {
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

