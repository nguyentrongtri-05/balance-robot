#include "motor.h"
#include <math.h>

void Motor_Config(void) {
    /* 1. Cấu hình GPIO điều hướng (Direction pins) */
    // GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Pin = DIR_LEFT_PIN | DIR_RIGHT_PIN;
    // ...
    
    /* 2. Cấu hình Timer xuất PWM (Ví dụ TIM4 Kênh 1 và 2) */
    // TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    // TIM_OCInitTypeDef TIM_OCInitStructure;
    // ... Khởi tạo TIM_TimeBaseStructure để chọn tần số băm (Ví dụ 20kHz) ...
    // ... Khởi tạo TIM_OCInitStructure chế độ PWM1 ...
    
    /* 3. Khởi động Timer PWM */
    // TIM_Cmd(TIM4, ENABLE);
}

void Motor_SetPWM(float pwm) {
    /* pwm là giá trị ngõ ra từ Fuzzy Logic. 
     * Dấu của pwm (+ hoặc -) sẽ quyết định chiều quay.
     * Độ lớn của pwm quyết định tốc độ (Duty cycle). */
    
    /* 1. Giới hạn giá trị PWM đầu ra (Bảo vệ phần cứng) */
    if (pwm > 100.0f) pwm = 100.0f;
    if (pwm < -100.0f) pwm = -100.0f;
    
    /* 2. Xác định chiều quay dựa vào dấu của lệnh PWM */
    if (pwm > 0) {
        // Xe chạy tới
        // GPIO_SetBits(GPIO_PORT_DIR, DIR_LEFT_PIN);
        // GPIO_SetBits(GPIO_PORT_DIR, DIR_RIGHT_PIN);
    } else {
        // Xe chạy lùi
        // GPIO_ResetBits(GPIO_PORT_DIR, DIR_LEFT_PIN);
        // GPIO_ResetBits(GPIO_PORT_DIR, DIR_RIGHT_PIN);
    }
    
    /* 3. Lấy giá trị tuyệt đối để nạp vào Duty Cycle */
    float duty_cycle = fabsf(pwm); 
    
    /* 4. Đưa dữ liệu Duty Cycle vào thanh ghi so sánh của Timer (CCR) 
     * Giả sử giá trị đếm (Period/ARR) của Timer là 1000 tương ứng 100% PWM */
    // uint16_t pulse = (uint16_t)((duty_cycle / 100.0f) * 1000.0f);
    
    // TIM4->CCR1 = pulse; // Động cơ trái
    // TIM4->CCR2 = pulse; // Động cơ phải
}
