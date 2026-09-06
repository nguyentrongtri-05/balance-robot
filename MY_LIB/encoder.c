#include "encoder.h"

/* Biến lưu trữ giá trị đếm xung Encoder của 2 bánh xe */
int32_t encoder_left_count = 0;
int32_t encoder_right_count = 0;

/* Các biến để tính toán sự thay đổi (Delta) và vận tốc */
int32_t prev_encoder_left = 0;
int32_t prev_encoder_right = 0;

void Encoder_Config(void) {
    /* 1. Cấu hình GPIO (Chân A, B của Encoder) thành Alternate Function */
    // GPIO_PinAFConfig(GPIOA, ...);
    
    /* 2. Cấu hình Timer ở chế độ Encoder Interface (Ví dụ TIM2 và TIM3) */
    // TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    // TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    /* 3. Khởi động Timer */
    // TIM_Cmd(TIM2, ENABLE);
    // TIM_Cmd(TIM3, ENABLE);
}

void Encoder_Read(float *speed) {
    /* 1. Đọc giá trị thanh ghi đếm (CNT) của Timer */
    // encoder_left_count = TIM2->CNT;
    // encoder_right_count = TIM3->CNT;
    
    /* 2. Tính số xung thay đổi (Delta xung) trong 1 chu kỳ lấy mẫu */
    int32_t delta_left = encoder_left_count - prev_encoder_left;
    int32_t delta_right = encoder_right_count - prev_encoder_right;
    
    /* Xử lý tràn (Overflow/Underflow) nếu Timer là 16-bit */
    if (delta_left > 32767) delta_left -= 65536;
    else if (delta_left < -32768) delta_left += 65536;
    
    if (delta_right > 32767) delta_right -= 65536;
    else if (delta_right < -32768) delta_right += 65536;
    
    prev_encoder_left = encoder_left_count;
    prev_encoder_right = encoder_right_count;
    
    /* 3. Chuyển đổi từ số xung (Pulses) sang vận tốc (Ví dụ: vòng/phút hoặc m/s) */
    /* Công thức: speed = (delta / Xung_1_vong) * (1 / dt) */
    // float dt = 0.010f; // 10ms
    // float speed_left = ((float)delta_left / 330.0f) / dt; // 330 là số xung 1 vòng của động cơ 
    // float speed_right = ((float)delta_right / 330.0f) / dt;
    
    /* Lấy tốc độ trung bình của xe làm đại diện trả về */
    // float average_speed = (speed_left + speed_right) / 2.0f;
    
    /* Code giữ chỗ (Placeholder) đợi tích hợp phần cứng */
    if (speed) *speed = 0.0f; // Sửa thành average_speed khi có code thật
}
