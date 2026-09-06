#ifndef __PID_H
#define __PID_H

/* Cấu trúc lưu trữ dữ liệu và thông số của bộ điều khiển PID */
typedef struct {
    float Kp;           /* Hệ số tỷ lệ */
    float Ki;           /* Hệ số tích phân */
    float Kd;           /* Hệ số vi phân */
    
    float integral;     /* Khâu tích phân (Cộng dồn sai số) */
    float prevError;    /* Sai số ở chu kỳ trước (Dùng cho khâu vi phân) */
    
    float outMin;       /* Giới hạn ngõ ra tối thiểu (Anti-Windup) */
    float outMax;       /* Giới hạn ngõ ra tối đa (Anti-Windup) */
} PID_t;

/**
 * @brief Khởi tạo các thông số của bộ điều khiển PID
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax);

/**
 * @brief Tính toán ngõ ra của PID
 * @param pid: Con trỏ đến struct PID
 * @param setpoint: Giá trị mong muốn (Target)
 * @param measurement: Giá trị đo đạc thực tế từ cảm biến (Feedback)
 * @param dt: Thời gian chu kỳ trích mẫu (Delta time)
 * @retval Tín hiệu điều khiển ngõ ra
 */
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);

#endif /* __PID_H */

