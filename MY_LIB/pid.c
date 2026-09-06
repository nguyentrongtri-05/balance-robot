#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    
    pid->outMin = outMin;
    pid->outMax = outMax;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt) {
    /* 1. Tính sai số (Error) */
    float error = setpoint - measurement;
    
    /* 2. Tính khâu tỷ lệ (Proportional) */
    float pOut = pid->Kp * error;
    
    /* 3. Tính khâu tích phân (Integral) */
    pid->integral += error * dt;
    float iOut = pid->Ki * pid->integral;
    
    /* 4. Tính khâu vi phân (Derivative) */
    float derivative = (error - pid->prevError) / dt;
    float dOut = pid->Kd * derivative;
    
    /* Cập nhật sai số cho lần chạy tiếp theo */
    pid->prevError = error;
    
    /* 5. Cộng dồn tín hiệu điều khiển */
    float output = pOut + iOut + dOut;
    
    /* 6. Giới hạn ngõ ra (Clamping / Anti-windup) */
    if (output > pid->outMax) {
        output = pid->outMax;
        /* Chống Windup cho khâu tích phân */
        pid->integral -= error * dt; 
    } 
    else if (output < pid->outMin) {
        output = pid->outMin;
        /* Chống Windup cho khâu tích phân */
        pid->integral -= error * dt;
    }
    
    return output;
}

