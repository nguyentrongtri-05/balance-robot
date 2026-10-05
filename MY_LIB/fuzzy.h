#ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

/* Fuzzy logic states for 5 membership functions:
 * NB: Negative Big
 * NS: Negative Small
 * ZE: Zero
 * PS: Positive Small
 * PB: Positive Big
 */
typedef enum {
    NB = 0,
    NS,
    ZE,
    PS,
    PB
} FuzzyState_t;

/* Chọn phép AND trong suy luận:
 * 0 = PROD (tích)  -> mặt điều khiển trơn, nội suy song tuyến tính (khuyên dùng)
 * 1 = MIN          -> Mamdani cổ điển, mặt điều khiển có "gờ" */
#define FUZZY_USE_MIN_AND   0

/*
 * CẤU TRÚC BỘ ĐIỀU KHIỂN FUZZY-PD (Sugeno bậc 0, 5x5 luật)
 *
 *   e  ──x ke ──┐                      ┌─────────┐
 *               ├─> [-1,1] ─> Mờ hóa ─>│ 25 luật │─> Giải mờ ─> u_n ──x ku──> u (%PWM)
 *   de ──x kde ─┘                      └─────────┘
 *
 * Miền chuẩn hóa [-1, 1], đỉnh các tập mờ tại: -1, -in_small, 0, +in_small, +1
 * Singleton đầu ra tại:                         -1, -out_small, 0, +out_small, +1
 *
 * Vùng giữa (|e_n| < in_small, |de_n| < in_small) bộ điều khiển TƯƠNG ĐƯƠNG PD:
 *      Kp_eq = ku * ke  * (out_small / in_small_e)    [%PWM / deg]
 *      Kd_eq = ku * kde * (out_small / in_small_de)   [%PWM / (deg/s)]
 * Ngoài vùng giữa: độ lợi = ku*k*(1-out_small)/(1-in_small), rồi bão hòa ở ±ku.
 *
 * TẠO PHI TUYẾN (lý do dùng fuzzy thay vì PD):
 *   out_small < in_small  -> mềm quanh điểm cân bằng (ít rung), mạnh tay khi lệch lớn
 *   out_small > in_small  -> nhạy quanh 0, dịu lại khi lệch lớn
 *   out_small = in_small  -> tuyến tính (PD có bão hòa)
 */
typedef struct {
    /* Hệ số chuẩn hóa đầu vào / đầu ra */
    float ke;           /* [1/deg]      e_n  = ke  * e   (bão hòa ±1 khi |e|  >= 1/ke)  */
    float kde;          /* [1/(deg/s)]  de_n = kde * de  (bão hòa ±1 khi |de| >= 1/kde) */
    float ku;           /* [%PWM]       u    = ku  * u_n                                */
    float out_limit;    /* Giới hạn |u| cuối cùng [%PWM]                                 */

    /* Hình dạng tập mờ trên miền chuẩn hóa (0.05 .. 0.95) */
    float e_small;      /* Đỉnh NS/PS của e   */
    float de_small;     /* Đỉnh NS/PS của de  */
    float out_small;    /* Singleton NS/PS đầu ra */

    /* Giá trị nội bộ để quan sát trên Watch window của Keil */
    float dbg_e_n;
    float dbg_de_n;
    float dbg_u;
} FuzzyController_t;

/* Function Prototypes */

/**
 * @brief  Khởi tạo tham số mặc định:
 *         Kp_eq = 10 %PWM/deg, Kd_eq = 0.5 %PWM/(deg/s), bão hòa e = 10 deg, de = 200 deg/s
 * @param  fc: Pointer to FuzzyController_t struct
 */
void Fuzzy_Init(FuzzyController_t *fc);

/**
 * @brief  Compute the fuzzy logic output based on error and change in error
 * @param  fc: Pointer to FuzzyController_t struct
 * @param  error: The error value (e.g., Target Angle - Current Angle from IMU) [deg]
 * @param  dError: The change in error (e.g., Gyro rate or derivative of error) [deg/s]
 * @retval The control output (e.g., Motor PWM) [-out_limit, +out_limit]
 */
float Fuzzy_Compute(FuzzyController_t *fc, float error, float dError);

/**
 * @brief  Đặt trực tiếp 3 hệ số chuẩn hóa
 */
void Fuzzy_SetGains(FuzzyController_t *fc, float ke, float kde, float ku);

/**
 * @brief  Tune theo "ngôn ngữ PD": giữ nguyên ku, tự tính ke/kde để vùng giữa có
 *         độ lợi tương đương Kp [%PWM/deg] và Kd [%PWM/(deg/s)].
 *         Tiện khi đã có bộ PD chạy được và muốn chuyển sang fuzzy.
 */
void Fuzzy_SetFromPD(FuzzyController_t *fc, float Kp, float Kd);

#endif /* __FUZZY_H */

