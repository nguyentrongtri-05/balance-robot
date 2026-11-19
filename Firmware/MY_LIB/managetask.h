#ifndef __MANAGETASK_H
#define __MANAGETASK_H

#include "stm32f4xx.h"
#include "motor.h"

/* Giới hạn góc cân bằng hoạt động của Robot: [-15.0° .. +15.0°] */
#define ROBOT_BALANCE_ANGLE_LIMIT       15.0f

/* ============================================================================
 * KIẾN TRÚC ĐIỀU KHIỂN VÒNG LỒNG (CASCADED DUAL-FUZZYPD + PID YAW):
 *  1. Vòng Ngoài Vị trí / Vận tốc (FuzzyPD_Pos, chu kỳ 5ms / 200Hz):
 *     - Nhận sai số vị trí và sai số vận tốc đo từ Encoder -> tính góc nghiêng bù trừ
 *       tiltOffset thuộc [-2.40° .. +2.40°] (kẹp an toàn +-3.2°).
 *     - Góc đặt đưa vào Vòng Trong: targetAngle = g_targetAngle - tiltOffset.
 *  2. Vòng Trong Cân bằng Pitch (FuzzyPD_Pitch, chu kỳ 5ms / 200Hz):
 *     - Nhận targetAngle và xuất điện áp điều khiển u_pitch (V) với
 *       90% áp nguồn danh định (+-10.8V / 12.0V), quy đổi sang +-7560 / 8400 PWM.
 *  3. Vòng Độc lập Ngang (PID_Yaw từ thư viện pid.c, chu kỳ 5ms / 200Hz):
 *     - Điều khiển góc hướng xoay Yaw thuần IMU với 10% tổng công suất PWM (+-840 / 8400).
 *     - Tổng công suất động cơ: 90% (10.8V -> 7560 PWM) + 10% (1.2V -> 840 PWM) = 100% (8400 PWM).
 * ============================================================================ */
#define VOLTAGE_LIMIT_BALANCE           (0.90f * MOTOR_SUPPLY_VOLTAGE) /* 90% của 12.0V = +-10.8V cho FuzzyPD_Pitch */
#define PWM_LIMIT_BALANCE               7560.0f     /* 90% của 8400 PWM cho Vòng Trong Cân bằng */
#define PWM_LIMIT_YAW                    840.0f     /* 10% của 8400 PWM cho Vòng Hướng xoay Yaw */

#define CASCADE_MAX_TOTAL_TILT_DEG      3.2f        /* Giới hạn góc nghiêng đặt bù trừ tối đa của Vòng Ngoài (+-3.2°) */

/* Trạng thái hoạt động của hệ thống (0: STOP, 1: RUN) */
extern volatile uint8_t g_systemEnable;
extern volatile float   g_posError;

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

/* Các biến giám sát Telemetry: OutFuzzy (điện áp V của FuzzyPD_Pitch), Yaw (10% PWM) & Góc bù Vòng Ngoài (°) */
extern volatile float g_outFuzzy;
extern volatile float g_outSpeed;
extern volatile float g_outYaw;

void    System_Tasks_Init(void);
uint8_t Robot_Start(void);
void    Robot_Stop(void);
uint8_t Robot_AutoCalibrate(void);
void    Robot_RequestCalibrate(void);

#endif /* __MANAGETASK_H */
