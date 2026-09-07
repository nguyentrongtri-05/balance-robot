#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    pid->pre_Error = 0.0f;
    pid->pre_pre_Error = 0.0f;
    pid->pre_Output = 0.0f;
    
    pid->outMin = outMin;
    pid->outMax = outMax;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt) {
    float Error = setpoint - measurement;
    
    /* Tính toán các khâu gia số (Delta u) */
    
    /* 1. Khâu Tỷ lệ (P): 
     * Kp * (e(k) - e(k-1)) */
    float P_part = pid->Kp * (Error - pid->pre_Error);
    
    /* 2. Khâu Tích phân (I): Dùng phương pháp hình thang (Trapezoidal)
     * 0.5 * Ki * T * (e(k) + e(k-1)) */
    float I_part = 0.5f * pid->Ki * dt * (Error + pid->pre_Error);
    
    /* 3. Khâu Vi phân (D):
     * (Kd / T) * (e(k) - 2*e(k-1) + e(k-2)) */
    float D_part = (pid->Kd / dt) * (Error - 2.0f * pid->pre_Error + pid->pre_pre_Error);
    
    /* Cộng dồn lượng gia số vào Output cũ: u(k) = u(k-1) + delta_u */
    float Output = pid->pre_Output + P_part + I_part + D_part;
    
    /* Giới hạn ngõ ra (Clamping) */
    if (Output > pid->outMax) {
        Output = pid->outMax;
    } 
    else if (Output < pid->outMin) {
        Output = pid->outMin;
    }
    
    /* Cập nhật các biến trạng thái cho chu kỳ sau */
    pid->pre_pre_Error = pid->pre_Error;
    pid->pre_Error = Error;
    pid->pre_Output = Output;
    
    return Output;
}
