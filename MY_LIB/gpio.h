#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"

/* Biến đếm số lần kích hoạt nút nhấn (phục vụ debug) */
extern volatile uint32_t exti_pc0_count;
extern volatile uint32_t exti_pc1_count;
extern volatile uint32_t exti_pc2_count;

/**
 * @brief Khởi tạo GPIO cơ bản (LEDs kit Discovery PD12..PD15 và 3 nút ngắt PC0..PC2)
 */
void GPIO_Config(void);

/**
 * @brief Cấu hình ngắt ngoài EXTI0..EXTI2 cho 3 nút nhấn PC0..PC2:
 *        - PC0: START (Chạy xe thăng bằng, kiểm tra góc an toàn)
 *        - PC1: STOP (Dừng khẩn cấp, ngắt toàn bộ động cơ)
 *        - PC2: CALIB THÔNG MINH TOÀN DIỆN (Tự nhận diện tĩnh -> Calib Gyro -> Nhận diện đứng thẳng -> Set Zero -> Lưu Flash)
 */
void EXTI_Buttons_Config(void);
#define EXTI_PC0_PC3_Config EXTI_Buttons_Config /* Alias tương thích */

#endif /* __GPIO_H */
