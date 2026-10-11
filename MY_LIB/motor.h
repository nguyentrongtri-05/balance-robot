#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

/* =====================================================================
 *  PHẦN CỨNG (TIM1 - Advanced timer, 168MHz)
 *   - Bánh trái : PWM PA8  (TIM1_CH1) | Chiều PC8 (IN1), PC6 (IN2)
 *   - Bánh phải : PWM PE11 (TIM1_CH2) | Chiều PC9 (IN1), PC7 (IN2)
 *   (Không dùng PA9 vì trên STM32F4-Discovery PA9 nối VBUS của USB OTG)
 * ===================================================================== */

/* Cấu hình chu kỳ và Prescaler cho PWM TIM1 (Timer 1 chạy trên APB2 - 168MHz) */
#define MOTOR_PWM_PERIOD        100  /* ARR: 100 mức, tương ứng độ phân giải 1% */
#define MOTOR_PWM_PRESCALER     167     /* 168MHz / (168 * 100) = 10kHz  */
/* Nếu dùng L298N (BJT, chuyển mạch chậm) nên hạ còn ~2-5kHz: PRESCALER = 42 -> 4kHz */

/* Tùy chọn đảo chiều nếu động cơ đấu dây ngược (0: Bình thường, 1: Đảo chiều) */
#define MOTOR_LEFT_INVERT       0
#define MOTOR_RIGHT_INVERT      0

/* Bù vùng chết từng motor [% PWM]: mức PWM nhỏ nhất làm bánh BẮT ĐẦU quay.
 * Cách đo: tăng dần Motor_SetPWM_Left(x) tới khi bánh vừa nhúc nhích -> DEADZONE = x.
 * Ánh xạ LIÊN TỤC (không có bước nhảy như kiểu cũ "u > 2 thì cộng 15"):
 *   |u| <  RAMP : out = |u| * (DZ + RAMP*(100-DZ)/100) / RAMP   (dốc tuyến tính qua 0)
 *   |u| >= RAMP : out = DZ + |u| * (100 - DZ) / 100
 * Để DEADZONE = 0 => tắt bù. */
#define MOTOR_LEFT_DEADZONE     15.0f   /* Giá trị cũ DEADBAND_OFFSET trong vTaskControl */
#define MOTOR_RIGHT_DEADZONE    15.0f
#define MOTOR_DZ_RAMP           1.0f    /* [%] độ rộng đoạn dốc quanh 0 */
#define MOTOR_CMD_EPSILON       0.01f   /* |u| nhỏ hơn mức này coi như 0 (dừng) */

/**
 * @brief Khởi tạo PWM TIM1 (PA8 - trái, PE11 - phải) và GPIO chiều.
 *        - Khi CPU dừng ở breakpoint, PWM TỰ TẮT (motor không chạy loạn)
 *        - Ngõ ra về mức thấp an toàn khi Motor_Enable(DISABLE)
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
 * @brief Điều khiển riêng bánh phải (PE11: PWM, PC9 & PC7: Chiều)
 * @param pwm: Giá trị từ -100.0f đến 100.0f
 */
void Motor_SetPWM_Right(float pwm);

/**
 * @brief Điều khiển 2 bánh với thành phần rẽ: trái = pwm + turn, phải = pwm - turn
 */
void Motor_SetPWM_Diff(float pwm, float turn);

/**
 * @brief Dừng cả 2 bánh xe (PWM = 0, thả trôi - coast)
 */
void Motor_Stop(void);

/**
 * @brief Phanh ngắn mạch cả 2 bánh (IN1 = IN2 = 1, PWM 100%) - dừng nhanh
 */
void Motor_Brake(void);

/**
 * @brief Bật/tắt TOÀN BỘ ngõ ra PWM bằng phần cứng (bit MOE).
 *        Dùng cho ngắt an toàn khi xe đổ: DISABLE => PWM ép về mức thấp ngay.
 */
void Motor_Enable(FunctionalState state);

/** Lệnh PWM cuối cùng đã áp (sau giới hạn, trước bù vùng chết) - để monitor */
float Motor_GetLastPWM_Left(void);
float Motor_GetLastPWM_Right(void);

#endif /* __MOTOR_H */
