#ifndef __PID_H
#define __PID_H

/* Cấu trúc lưu trữ dữ liệu cho PID Gia số (Incremental PID) */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float pre_Error;      /* e(k-1) */
    float pre_pre_Error;  /* e(k-2) */
    float pre_Output;     /* u(k-1) */
    
    float outMin;
    float outMax;
} PID_t;

/**
 * @brief Khởi tạo các thông số của bộ điều khiển PID Gia số
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax);

/**
 * @brief Tính toán ngõ ra của PID (Sử dụng thuật toán Gia số)
 * @param pid: Con trỏ đến struct PID
 * @param setpoint: Giá trị mong muốn (Target)
 * @param measurement: Giá trị đo đạc thực tế từ cảm biến (Feedback)
 * @param dt: Thời gian chu kỳ trích mẫu (Delta time, T_Sample)
 * @retval Tín hiệu điều khiển ngõ ra u(k)
 */
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);

#endif /* __PID_H */
