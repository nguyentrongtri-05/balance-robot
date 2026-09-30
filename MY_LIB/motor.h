#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

/* Cấu hình chu kỳ và Prescaler cho PWM TIM1 (Timer 1 chạy trên APB2 - 168MHz) */
#define MOTOR_PWM_PERIOD        1000  /* ARR: 1000 mức, tương ứng độ phân giải 0.1% */
#define MOTOR_PWM_PRESCALER     8     /* 168MHz / (8 * 1000) = 21kHz (tần số siêu âm êm ái) */

/* Tùy chọn đảo chiều nếu động cơ đấu dây ngược (0: Bình thường, 1: Đảo chiều) */
#define MOTOR_LEFT_INVERT       0
#define MOTOR_RIGHT_INVERT      0

/**
 * @brief Khởi tạo PWM TIM1 (PA8 - Bánh trái, PA9 - Bánh phải) 
 *        và GPIO điều khiển chiều (PC8, PC6 - Trái | PC9, PC7 - Phải)
 */
void Motor_Config(void);

/**
 * @brief Điều khiển PWM cả 2 bánh cùng lúc
 * @param pwm: Giá trị từ -100.0f (lùi tối đa) đến 100.0f (tiến tối đa)
 */
void Motor_SetPWM(float pwm);

/**
 * @brief Điều khiển riêng bánh trái (PA8: PWM, PC8 & PC6: Chiều)
 * @param pwm: Giá trị từ -100.0f đến 100.0f
 */
void Motor_SetPWM_Left(float pwm);

/**
 * @brief Điều khiển riêng bánh phải (PA9: PWM, PC9 & PC7: Chiều)
 * @param pwm: Giá trị từ -100.0f đến 100.0f
 */
void Motor_SetPWM_Right(float pwm);

/**
 * @brief Dừng cả 2 bánh xe (PWM = 0)
 */
void Motor_Stop(void);

#endif /* __MOTOR_H */
