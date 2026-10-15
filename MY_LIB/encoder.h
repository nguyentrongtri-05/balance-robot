#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

#define ENCODER_PULSES_PER_REV  330u
#define ENCODER_COUNTS_PER_REV  1320.0f     /* Chế độ TI12 x4: 330 * 4 = 1320 xung/vòng */
#define ENCODER_SAMPLE_TIME     0.010f      /* Chu kỳ lấy mẫu vTaskEncoder: 10ms (100Hz) */

/* Thông số vật lý bánh xe đường kính D = 65mm (6.5cm) & Khoảng cách 2 bánh W = 19.41cm (chuẩn hóa từ lan8.csv) */
#define WHEEL_DIAMETER_MM       65.0f       /* Đường kính bánh xe (mm) */
#define WHEEL_DIAMETER_CM       6.50f       /* Đường kính bánh xe (cm) */
#define WHEEL_RADIUS_CM         3.25f       /* Bán kính bánh xe R = D/2 (cm) */
#define WHEEL_CIRCUMFERENCE_CM  20.420352f  /* Chu vi 1 vòng bánh: pi * 6.5cm = 20.420352 cm */
#define WHEEL_CM_PER_PULSE      (WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REV) /* 0.015470 cm/xung */
#define WHEEL_RPM_TO_CMS        (WHEEL_CIRCUMFERENCE_CM / 60.0f)                  /* 0.340339 cm/s mỗi RPM */

#define ROBOT_MAX_SPEED_CMS     10.0f       /* Ràng buộc vận tốc dài tối đa của xe: < 10 cm/s */
#define ROBOT_MAX_SPEED_RPM     (ROBOT_MAX_SPEED_CMS / WHEEL_RPM_TO_CMS)          /* ~29.38 RPM ứng với 10 cm/s */

#define ROBOT_WHEELBASE_CM      19.21f      /* Khoảng cách hiệu dụng 2 bánh Trái - Phải (cm) chuẩn hóa thực nghiệm 180 deg */
#define YAW_DEG_PER_DIFF_CM     (57.2957795f / ROBOT_WHEELBASE_CM)                /* 2.98260 deg trên mỗi cm lệch 2 bánh */
#define YAW_DEG_PER_DIFF_PULSE  (WHEEL_CM_PER_PULSE * YAW_DEG_PER_DIFF_CM)        /* 0.04614 deg trên mỗi xung lệch 2 bánh */

typedef struct
{
    int32_t pulse;
    int32_t pre_pulse;
    int32_t delta_pulse;
    float speed;
} MOTOR;

/**
 * @brief Initialize Encoder timer interfaces on PB4/PB5 and PB6/PB7
 */
void Encoder_Config(void);

/**
 * @brief Read both wheel encoder speeds in one structure-based function
 * @param motorLeft: Pointer to left wheel encoder struct
 * @param motorRight: Pointer to right wheel encoder struct
 */
void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight);

#endif /* __ENCODER_H */

