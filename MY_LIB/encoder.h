#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

#define ENCODER_PULSES_PER_REV  330u
#define ENCODER_COUNTS_PER_REV  1320.0f     /* Chế độ TI12 x4: 330 * 4 = 1320 xung/vòng */
#define ENCODER_SAMPLE_TIME     0.005f      /* Chu kỳ lấy mẫu vTaskEncoder: 5ms (200Hz) */

/* Thông số vật lý bánh xe đường kính D = 65mm (6.5cm) & Khoảng cách 2 bánh W = 19.21cm */
#define WHEEL_CIRCUMFERENCE_CM  20.420352f  /* Chu vi 1 vòng bánh: pi * 6.5cm = 20.420352 cm */
#define WHEEL_CM_PER_PULSE      (WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REV)
#define WHEEL_RPM_TO_CMS        (WHEEL_CIRCUMFERENCE_CM / 60.0f)

#define ROBOT_MAX_SPEED_CMS     10.0f       /* Ràng buộc vận tốc dài tối đa của xe: < 10 cm/s */
#define ROBOT_MAX_SPEED_RPM     (ROBOT_MAX_SPEED_CMS / WHEEL_RPM_TO_CMS)

#define ROBOT_WHEELBASE_CM      19.21f      /* Khoảng cách hiệu dụng 2 bánh Trái - Phải (cm) */
#define YAW_DEG_PER_DIFF_CM     (57.2957795f / ROBOT_WHEELBASE_CM)
#define YAW_DEG_PER_DIFF_PULSE  (WHEEL_CM_PER_PULSE * YAW_DEG_PER_DIFF_CM)

typedef struct
{
    int32_t pulse;
    int32_t pre_pulse;
    int32_t delta_pulse;
    float   speed;
} MOTOR;

void Encoder_Config(void);
void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight);

#endif /* __ENCODER_H */
