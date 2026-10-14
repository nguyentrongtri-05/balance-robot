#ifndef __MANAGETASK_H
#define __MANAGETASK_H

#include "stm32f4xx.h"

/* Trạng thái hoạt động của hệ thống (0: STOP, 1: RUN) */
extern volatile uint8_t g_systemEnable;

/* Các chế độ điều khiển Robot (Giữ Dual PID và Fuzzy) */
typedef enum {
    CTRL_MODE_DUAL_PID    = 0,  /* PID 2 vòng: Cân bằng PD + Vị trí/Tốc độ PI */
    CTRL_MODE_FUZZY       = 1   /* Điều khiển Mờ Fuzzy Logic */
} RobotControlMode_t;

extern volatile RobotControlMode_t g_controlMode;

/* Thông số PID 2 vòng (có thể tinh chỉnh trực tiếp hoặc qua UART) */
extern volatile float g_Kp_balance;
extern volatile float g_Kd_balance;
extern volatile float g_Kp_position;
extern volatile float g_Ki_position;
extern volatile float g_Kd_yaw;
extern volatile float g_Kp_yaw;
extern volatile float g_Kd_yaw_imu;

/* Các biến góc đo và điều khiển (Pitch & Yaw) */
extern volatile float g_filteredPitch;
extern volatile float g_targetAngle;
extern volatile float g_gyroOffset;
extern volatile float g_gyroZOffset;
extern volatile float g_rawYawRate;
extern volatile float g_rawYawAngle;
extern volatile float g_filteredYaw;
extern volatile float g_targetYaw;
extern volatile float g_targetSpeed;

void Robot_SetControlMode(RobotControlMode_t mode);
void System_Tasks_Init(void);
uint8_t Robot_Start(void);
void Robot_Stop(void);
uint8_t Robot_AutoCalibrate(void);

#endif /* __MANAGETASK_H */

