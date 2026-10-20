#ifndef __MANAGETASK_H
#define __MANAGETASK_H

#include "stm32f4xx.h"

/* Giới hạn góc cân bằng hoạt động của Robot: [-15.0° .. +15.0°] */
#define ROBOT_BALANCE_ANGLE_LIMIT       15.0f

/* ============================================================================
 * KIẾN TRÚC ĐIỀU KHIỂN VÒNG LỒNG (CASCADED CONTROL ARCHITECTURE):
 *  1. Vòng Ngoài Vận tốc (FuzzyPI / PID_Vel, chu kỳ 20ms):
 *     - Nhận vận tốc đặt và vận tốc đo từ Encoder -> tính góc nghiêng bù trừ
 *       tiltOffset thuộc [-2.5° .. +2.5°].
 *     - Góc đặt đưa vào Vòng Trong: targetAngle = g_targetAngle - tiltOffset.
 *  2. Vòng Trong Cân bằng Pitch (FuzzyPD / Balance PD, chu kỳ 5ms):
 *     - Nhận targetAngle và xuất trực tiếp 90% tổng công suất PWM (+-7560 / 8400).
 *  3. Vòng Độc lập Ngang (Yaw PID, chu kỳ 5ms):
 *     - Điều khiển góc hướng xoay Yaw với 10% tổng công suất PWM (+-840 / 8400).
 *     - Tổng công suất động cơ: 90% (7560) + 10% (840) = 100% (8400 PWM).
 * ============================================================================ */
#define PWM_LIMIT_BALANCE               7560.0f     /* 90% của 8400 PWM cho Vòng Trong Cân bằng */
#define PWM_LIMIT_YAW                    840.0f     /* 10% của 8400 PWM cho Vòng Hướng xoay Yaw */

#define CASCADE_CMD_TO_DEG              (1.0f / 420.0f) /* Hệ số quy đổi hiển thị góc bù vòng ngoài */
#define CASCADE_MAX_TOTAL_TILT_DEG      2.5f            /* Giới hạn góc nghiêng đặt bù trừ của Vòng Ngoài (+-2.5°) */

/* Trạng thái hoạt động của hệ thống (0: STOP, 1: RUN) */
extern volatile uint8_t g_systemEnable;

/* Các chế độ điều khiển Robot */
typedef enum {
    CTRL_MODE_DUAL_PID    = 0,  /* Vòng lồng Cascade với bộ PID_Vel + PD Cân bằng (90% PWM) + Yaw (10% PWM) */
    CTRL_MODE_FUZZY       = 1   /* Vòng lồng Cascade với bộ FuzzyPI + FuzzyPD (90% PWM) + Yaw (10% PWM) */
} RobotControlMode_t;

extern volatile RobotControlMode_t g_controlMode;

/* Thông số điều khiển Vòng Lồng (có thể tinh chỉnh trực tiếp qua UART) */
extern volatile float g_Kp_balance;
extern volatile float g_Kd_balance;
extern volatile float g_Kp_speed;
extern volatile float g_Ki_speed;
extern volatile float g_Kd_yaw;
extern volatile float g_Kp_yaw;
extern volatile float g_Kd_yaw_imu;

/* Các biến góc đo và điều khiển (Pitch & Yaw) */
extern volatile float g_filteredPitch;
extern volatile float g_targetAngle;
extern volatile float g_cascadeTargetAngle;
extern volatile float g_gyroOffset;
extern volatile float g_gyroZOffset;
extern volatile float g_rawYawRate;
extern volatile float g_rawYawAngle;
extern volatile float g_filteredYaw;
extern volatile float g_targetYaw;
extern volatile float g_targetSpeed;

/* Các biến giám sát Telemetry: OutFuzzy (90% PWM), Yaw (10% PWM) & Lượng bù Vòng Ngoài Vận tốc */
extern volatile float g_outFuzzy;
extern volatile float g_outSpeed;
extern volatile float g_outYaw;

void    Robot_SetControlMode(RobotControlMode_t mode);
void    System_Tasks_Init(void);
uint8_t Robot_Start(void);
void    Robot_Stop(void);
uint8_t Robot_AutoCalibrate(void);
void    Robot_RequestCalibrate(void);

#endif /* __MANAGETASK_H */
