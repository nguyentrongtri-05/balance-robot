#include "managetask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "driver_i2c.h"
#include "driver_uart.h"
#include "fuzzy.h"
#include "kalman.h"
#include "driver_flash.h"
#include "pid.h"
#include <math.h>
#include <string.h>

/* =====================================================================
 * 1. BIẾN & BỘ LỌC CỦA TASK IMU (Lấy Raw Pitch/Yaw & Kalman Pitch/Yaw)
 * ===================================================================== */
Kalman_t kalmanPitch;
Kalman_t kalmanYaw;

volatile float g_rawAccPitch   = 0.0f;
volatile float g_rawGyroRate   = 0.0f;
volatile float g_rawYawAngle   = 0.0f;
volatile float g_rawYawRate    = 0.0f;

volatile float g_filteredPitch = 0.0f;
volatile float g_filteredYaw   = 0.0f;

volatile float g_gyroOffset    = 0.0f;
volatile float g_gyroZOffset   = 0.0f;

#define YAW_GYRO_SIGN          (-1.0f)

/* =====================================================================
 * 2. BIẾN & BỘ LỌC CỦA TASK ENCODER (Raw Wheel, Kalman Wheel, Pos X, Y)
 * ===================================================================== */
KalmanSpeed_t kalmanSpeed_L;
KalmanSpeed_t kalmanSpeed_R;

volatile float g_motorSpeed_left          = 0.0f;
volatile float g_motorSpeed_right         = 0.0f;
volatile float g_motorSpeedfiltered_left  = 0.0f;
volatile float g_motorSpeedfiltered_right = 0.0f;

volatile float g_robotX          = 0.0f;
volatile float g_robotY          = 0.0f;
volatile float g_encoderYawAngle = 0.0f;
volatile float g_encoderYawRate  = 0.0f;

/* =====================================================================
 * 3. BIẾN CỦA TASK CONTROL (Trạng thái, Mục tiêu & Xung PWM xuất ra Motor)
 * ===================================================================== */
extern FuzzyController_t fuzzyControl;
volatile uint8_t g_systemEnable = 0;

volatile float g_targetAngle        = -0.73f;
volatile float g_cascadeTargetAngle = -0.73f;
volatile float g_targetYaw          = 0.0f;
volatile float g_targetSpeed        = 0.0f;
volatile float g_targetX            = 0.0f;
volatile float g_targetY            = 0.0f;

volatile float g_motorPWM_left  = 0.0f;
volatile float g_motorPWM_right = 0.0f;
volatile float g_outFuzzy       = 0.0f;
volatile float g_outSpeed       = 0.0f;
volatile float g_outPosition    = 0.0f;
volatile float g_outYaw         = 0.0f;

/* =====================================================================
 * CẤU HÌNH KIẾN TRÚC ĐIỀU KHIỂN VÒNG LỒNG (CASCADED CONTROL - CÁCH 1):
 *  - Vòng Ngoài (Vận tốc PI + Vị trí 2D PI):
 *      Sinh ra Góc nghiêng đặt bù trừ deltaTheta (kẹp an toàn +-5.0°)
 *      -> Góc đặt Vòng Trong: g_cascadeTargetAngle = g_targetAngle - deltaTheta
 *  - Vòng Trong (OutFuzzy 5 khâu / Balance PD):
 *      Bám theo g_cascadeTargetAngle, xuất 90% tổng công suất (+-7560 PWM)
 *  - Vòng Độc lập Ngang (Yaw PID):
 *      Điều khiển hướng xoay với 10% tổng công suất (+-840 PWM)
 * ===================================================================== */
volatile RobotControlMode_t g_controlMode = CTRL_MODE_FUZZY;
volatile float g_Kp_balance  = 440.0f;
volatile float g_Kd_balance  = 11.50f;
volatile float g_Kp_speed    = 18.00f;
volatile float g_Ki_speed    = 4.20f;
volatile float g_Kp_position = 12.60f;
volatile float g_Ki_position = 2.10f;
volatile float g_Kd_yaw      = 6.30f;
volatile float g_Kp_yaw      = 25.20f;
volatile float g_Kd_yaw_imu  = 3.78f;

volatile float g_speedIntegral = 0.0f;
volatile float g_posIntegral   = 0.0f;
volatile float g_yawIntegral   = 0.0f;
#define SPEED_INTEGRAL_MAX     60.0f
#define POS_INTEGRAL_MAX       100.0f
#define YAW_INTEGRAL_MAX       25.0f
static float s_gyroRateFiltered    = 0.0f;
static float s_cascadeTiltFiltered = 0.0f;
static float s_yawRateFiltered     = 0.0f;

/* Bù vùng chết động cơ (67/50 trên thang PWM 8400 bước) và giới hạn gia tốc PWM mượt */
#define MOTOR_DEADBAND_PWM_L   67.0f
#define MOTOR_DEADBAND_PWM_R   50.0f
#define BALANCE_MAX_PWM_DELTA  900.0f
static float s_prevBalancePWM_L = 0.0f;
static float s_prevBalancePWM_R = 0.0f;

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);
static void vTaskUART(void *pvParameters);

PID_t PID_Vel;

static float Clamp_Float(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

static void Reset_OdometryAndStates(void) {
    g_speedIntegral       = 0.0f;
    g_posIntegral         = 0.0f;
    g_yawIntegral         = 0.0f;
    g_robotX              = 0.0f;
    g_robotY              = 0.0f;
    g_targetX             = 0.0f;
    g_targetY             = 0.0f;
    g_encoderYawAngle     = 0.0f;
    g_encoderYawRate      = 0.0f;
    g_rawYawAngle         = 0.0f;
    g_filteredYaw         = 0.0f;
    g_targetYaw           = 0.0f;
    g_cascadeTargetAngle  = g_targetAngle;
    s_gyroRateFiltered    = 0.0f;
    s_cascadeTiltFiltered = 0.0f;
    s_yawRateFiltered     = 0.0f;
    s_prevBalancePWM_L    = 0.0f;
    s_prevBalancePWM_R    = 0.0f;
    PID_Reset(&PID_Vel);
    Kalman_SetAngle(&kalmanYaw, 0.0f);
}

void System_Tasks_Init(void) {
    I2C1_MPU6050_Init();
    Encoder_Config();
    UART3_Init(115200);
    KalmanAngle_Init(&kalmanPitch);
    KalmanAngle_Init(&kalmanYaw);
    KalmanSpeed_InitDefault(&kalmanSpeed_L);
    KalmanSpeed_InitDefault(&kalmanSpeed_R);

    /* Khởi tạo PID Vòng Ngoài Vận tốc: xuất trực tiếp góc nghiêng bù trừ deltaAngle thuộc [-2.5° .. +2.5°] */
    PID_Init(&PID_Vel, 0.035f, 0.010f, 0.0f, -CASCADE_MAX_TOTAL_TILT_DEG, CASCADE_MAX_TOTAL_TILT_DEG);
    int16_t init_ax = 0, init_ay = 0, init_az = 0, init_gx = 0, init_gy = 0, init_gz = 0;
    int32_t sumGyInit = 0, sumGzInit = 0;
    int16_t validInit = 0;
    for (int i = 0; i < 20; i++) {
        if (MPU6050_ReadRawData(&init_ax, &init_ay, &init_az, &init_gx, &init_gy, &init_gz) == 0u) {
            sumGyInit += init_gy;
            sumGzInit += init_gz;
            validInit++;
        }
    }
    if (validInit > 0) {
        g_gyroZOffset = (float)sumGzInit / (float)validInit;
    }

    RobotFlashConfig_t savedConfig;
    if (Flash_ReadConfig(&savedConfig)) {
        if (fabsf(savedConfig.targetAngle) > 0.1f && fabsf(savedConfig.targetAngle) < 12.0f) {
            g_targetAngle        = savedConfig.targetAngle;
            g_cascadeTargetAngle = g_targetAngle;
        }
        g_gyroOffset = savedConfig.gyroOffset;
        if (validInit > 0 && fabsf((float)init_gy) < 500.0f && fabsf((float)init_gy - g_gyroOffset) > 120.0f) {
            g_gyroOffset = (float)sumGyInit / (float)validInit;
        }

        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        for (volatile int d = 0; d < 500000; d++);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    }

    g_systemEnable = 0;
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15);

    xTaskCreate(vTaskIMU,     "Task_IMU",     512, NULL, 3, NULL);
    xTaskCreate(vTaskEncoder, "Task_Encoder", 512, NULL, 3, NULL);
    xTaskCreate(vTaskControl, "Task_Control", 512, NULL, 4, NULL);
    xTaskCreate(vTaskUART,    "Task_UART",    512, NULL, 2, NULL);
}

/* =====================================================================
 * CÁC HÀM ĐIỀU KHIỂN TRẠNG THÁI HỆ THỐNG (START / STOP / CALIB / MODE)
 * ===================================================================== */
static void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t d = 0; d < 35000; d++);
    }
}

uint8_t Robot_Start(void) {
    float diffAngle = fabsf(g_filteredPitch - g_targetAngle);

    /* Điều kiện an toàn: xe phải nằm trong vùng cân bằng [-15°, +15°] mới cho phép chạy */
    if (diffAngle > ROBOT_BALANCE_ANGLE_LIMIT) {
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
        return 0;
    }

    g_systemEnable = 1;
    Reset_OdometryAndStates();
    g_motorSpeedfiltered_left  = 0.0f;
    g_motorSpeedfiltered_right = 0.0f;

    KalmanSpeed_SetSpeed(&kalmanSpeed_L, 0.0f);
    KalmanSpeed_SetSpeed(&kalmanSpeed_R, 0.0f);
    PID_Reset(&PID_Vel);
    Fuzzy_Reset(&fuzzyControl);

    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    return 1;
}

void Robot_Stop(void) {
    g_systemEnable = 0;
    Motor_Stop();

    g_speedIntegral       = 0.0f;
    g_posIntegral         = 0.0f;
    g_yawIntegral         = 0.0f;
    s_gyroRateFiltered    = 0.0f;
    s_cascadeTiltFiltered = 0.0f;
    s_yawRateFiltered     = 0.0f;
    s_prevBalancePWM_L    = 0.0f;
    s_prevBalancePWM_R    = 0.0f;
    PID_Reset(&PID_Vel);
    Fuzzy_Reset(&fuzzyControl);

    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

void Robot_SetControlMode(RobotControlMode_t mode) {
    g_controlMode = mode;
    Reset_OdometryAndStates();
    PID_Reset(&PID_Vel);
    Fuzzy_Reset(&fuzzyControl);
    Motor_Stop();
}

static volatile uint8_t s_isCalibrating = 0;
static volatile uint8_t s_calibPending  = 0;

void Robot_RequestCalibrate(void) {
    s_calibPending = 1;
}

static void Calib_DelayMs(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && (__get_IPSR() == 0U)) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        Delay_ms(ms);
    }
}

uint8_t Robot_AutoCalibrate(void) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    if (g_systemEnable) {
        Robot_Stop();
    }

    s_isCalibrating = 1;
    Calib_DelayMs(20);

    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_SetBits(GPIOD, GPIO_Pin_13);

    uint8_t  still_batches   = 0;
    uint32_t timeout_counter = 0;
    float    prevAvgGy       = 99999.0f;
    float    prevAvgGz       = 99999.0f;
    float    sumStableGy     = 0.0f;
    float    sumStableGz     = 0.0f;

    while (still_batches < 3) {
        int16_t minGy = 32767, maxGy = -32768;
        int16_t minGz = 32767, maxGz = -32768;
        int32_t sumBatchGy = 0, sumBatchGz = 0;
        uint8_t valid = 0;

        for (int i = 0; i < 25; i++) {
            if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
                if (gy < minGy) minGy = gy;
                if (gy > maxGy) maxGy = gy;
                if (gz < minGz) minGz = gz;
                if (gz > maxGz) maxGz = gz;
                sumBatchGy += gy;
                sumBatchGz += gz;
                valid++;
            }
            Calib_DelayMs(8);
        }

        if (valid > 0) {
            float avgGy = (float)sumBatchGy / (float)valid;
            float avgGz = (float)sumBatchGz / (float)valid;

            if ((maxGy - minGy) < 900 && (maxGz - minGz) < 900) {
                if (still_batches == 0 ||
                    (fabsf(avgGy - prevAvgGy) < 150.0f && fabsf(avgGz - prevAvgGz) < 150.0f)) {
                    still_batches++;
                    sumStableGy += avgGy;
                    sumStableGz += avgGz;
                } else {
                    still_batches = 1;
                    sumStableGy   = avgGy;
                    sumStableGz   = avgGz;
                    GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
                }
                prevAvgGy = avgGy;
                prevAvgGz = avgGz;
            } else {
                still_batches = 0;
                sumStableGy   = 0.0f;
                sumStableGz   = 0.0f;
                prevAvgGy     = 99999.0f;
                prevAvgGz     = 99999.0f;
                GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
            }
        }

        if (++timeout_counter > 50) {
            GPIO_ResetBits(GPIOD, GPIO_Pin_13);
            for (int k = 0; k < 3; k++) {
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
            }
            GPIO_SetBits(GPIOD, GPIO_Pin_14);
            s_isCalibrating = 0;
            return 0;
        }
    }

    g_gyroOffset  = sumStableGy / 3.0f;
    g_gyroZOffset = sumStableGz / 3.0f;

    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    for (int k = 0; k < 2; k++) {
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(120);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(120);
    }

    uint8_t balance_batches = 0;
    timeout_counter = 0;
    float sumBatchPitch = 0.0f;
    float prevAvgP = 999.0f;

    while (balance_batches < 4) {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

        float minP = 999.0f, maxP = -999.0f;
        float currentSum = 0.0f;
        uint8_t validSamples = 0;

        for (int i = 0; i < 25; i++) {
            if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
                float axg = (float)ax / 16384.0f;
                float ayg = (float)ay / 16384.0f;
                float azg = (float)az / 16384.0f;
                float p = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);

                currentSum += p;
                validSamples++;
                if (p < minP) minP = p;
                if (p > maxP) maxP = p;
            }
            Calib_DelayMs(8);
        }

        if (validSamples > 0) {
            float avgP  = currentSum / (float)validSamples;
            float diffP = maxP - minP;

            if (avgP > -8.0f && avgP < 8.0f && diffP < 3.5f) {
                if (balance_batches == 0 || fabsf(avgP - prevAvgP) < 0.60f) {
                    balance_batches++;
                    sumBatchPitch += avgP;
                } else {
                    balance_batches = 1;
                    sumBatchPitch   = avgP;
                }
                prevAvgP = avgP;
            } else {
                balance_batches = 0;
                sumBatchPitch   = 0.0f;
                prevAvgP        = 999.0f;
            }
        }

        if (++timeout_counter > 80) {
            GPIO_ResetBits(GPIOD, GPIO_Pin_15);
            for (int k = 0; k < 3; k++) {
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
            }
            GPIO_SetBits(GPIOD, GPIO_Pin_14);
            s_isCalibrating = 0;
            return 0;
        }
    }

    g_targetAngle = sumBatchPitch / 4.0f;
    KalmanAngle_Init(&kalmanPitch);
    Kalman_SetAngle(&kalmanPitch, g_targetAngle);
    g_filteredPitch = g_targetAngle;

    KalmanAngle_Init(&kalmanYaw);
    Reset_OdometryAndStates();

    GPIO_SetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_13);
    Flash_SaveConfig(g_targetAngle, g_gyroOffset);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_13);

    for (int k = 0; k < 3; k++) {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        Calib_DelayMs(200);
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        Calib_DelayMs(200);
    }

    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    s_isCalibrating = 0;
    return 1;
}

/* =====================================================================
 * CÁC TASK FREERTOS
 * ===================================================================== */
static void vTaskIMU(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    uint8_t beat = 0;
    uint8_t isFirstSample = 1;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (s_isCalibrating) {
            continue;
        }

        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u)
        {
            float azSign = (az >= 0) ? 1.0f : -1.0f;

            float axg = (float)ax / 16384.0f;
            float ayg = (float)ay / 16384.0f;
            float azg = (float)az / 16384.0f;
            g_rawAccPitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);
            g_rawGyroRate = azSign * (((float)gy - g_gyroOffset) / 65.5f);

            if (fabsf(g_encoderYawRate) < 1.5f && fabsf((float)gz - g_gyroZOffset) < 150.0f) {
                g_gyroZOffset = 0.998f * g_gyroZOffset + 0.002f * (float)gz;
            }
            g_rawYawRate  = YAW_GYRO_SIGN * azSign * (((float)gz - g_gyroZOffset) / 65.5f);
            g_rawYawAngle = g_encoderYawAngle;

            if (isFirstSample) {
                Kalman_SetAngle(&kalmanPitch, g_rawAccPitch);
                g_filteredPitch = g_rawAccPitch;

                Kalman_SetAngle(&kalmanYaw, g_rawYawAngle);
                g_filteredYaw = g_rawYawAngle;
                isFirstSample = 0;
            } else {
                g_filteredPitch = Kalman_GetAngle(&kalmanPitch, g_rawAccPitch, g_rawGyroRate, 0.005f);
                g_filteredYaw   = Kalman_GetAngle(&kalmanYaw,   g_rawYawAngle, g_rawYawRate,  0.005f);
            }
        }

        if (g_systemEnable && (++beat == 20)) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
        }
    }
}

static void vTaskEncoder(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    MOTOR leftMotor  = {0};
    MOTOR rightMotor = {0};
    float prevYawRad = 0.0f;
    int8_t beat = 0;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Encoder_ReadStruct(&leftMotor, &rightMotor);
        g_motorSpeed_left  = leftMotor.speed;
        g_motorSpeed_right = rightMotor.speed;

        g_motorSpeedfiltered_left  = KalmanSpeed_GetSpeed(&kalmanSpeed_L, g_motorSpeed_left,  0.005f);
        g_motorSpeedfiltered_right = KalmanSpeed_GetSpeed(&kalmanSpeed_R, g_motorSpeed_right, 0.005f);

        float vDiff_cms      = (-g_motorSpeedfiltered_left - (-g_motorSpeedfiltered_right)) * WHEEL_RPM_TO_CMS;
        g_encoderYawRate     = vDiff_cms * YAW_DEG_PER_DIFF_CM;

        float dPulse_L       = -(float)leftMotor.delta_pulse;
        float dPulse_R       = -(float)rightMotor.delta_pulse;
        float deltaPulseAvg  = 0.5f * (dPulse_L + dPulse_R);
        float deltaPulseDiff = dPulse_L - dPulse_R;

        if (fabsf(g_encoderYawAngle) < 0.0001f && fabsf(g_robotX) < 0.0001f && fabsf(g_robotY) < 0.0001f) {
            prevYawRad = 0.0f;
        }

        g_encoderYawAngle += deltaPulseDiff * YAW_DEG_PER_DIFF_PULSE;

        float ds        = deltaPulseAvg * WHEEL_CM_PER_PULSE;
        float yawRad    = kalmanYaw.angle * (3.14159265359f / 180.0f);
        float yawMidRad = 0.5f * (prevYawRad + yawRad);
        prevYawRad      = yawRad;

        g_robotX += ds * cosf(yawMidRad);
        g_robotY += ds * sinf(yawMidRad);

        if (++beat >= 30) {
            beat = 0;
            if (!s_isCalibrating) {
                GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
            }
        }
    }
}

static void vTaskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    uint8_t beat = 0;

    float vel = 0.0f, vel_cmd = 0.0f;
    float deltaAngle = 0.0f, targetAngle = 0.0f;
    float pwm_fuzzy = 0.0f, pwm_yaw = 0.0f;
    float outpwm_L = 0.0f, outpwm_R = 0.0f;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (g_systemEnable)
        {
            /* 1. Bảo vệ an toàn: Ngắt động cơ nếu góc nghiêng lệch quá +-15.0 độ */
            if (fabsf(g_filteredPitch - g_targetAngle) > ROBOT_BALANCE_ANGLE_LIMIT) {
                Robot_Stop();
            }
            else
            {
                /* 2. Vòng Ngoài (PID Vận tốc -> Góc nghiêng đặt targetAngle) */
                vel        = (g_motorSpeedfiltered_left + g_motorSpeedfiltered_right) * 0.5f;
                vel_cmd    = -Clamp_Float(g_targetSpeed, -ROBOT_MAX_SPEED_RPM, ROBOT_MAX_SPEED_RPM);
                deltaAngle = PID_Compute(&PID_Vel, vel_cmd, vel, 0.005f);
                s_cascadeTiltFiltered = 0.85f * s_cascadeTiltFiltered + 0.15f * deltaAngle;
                targetAngle = g_targetAngle - s_cascadeTiltFiltered;

                /* 3. Vòng Trong (Fuzzy 5 khâu -> 90% OutPWM = +-7560) */
                if (g_controlMode == CTRL_MODE_FUZZY) {
                    pwm_fuzzy = Fuzzy_Compute(&fuzzyControl, targetAngle, g_filteredPitch, kalmanPitch.rate);
                } else {
                    s_gyroRateFiltered = 0.80f * s_gyroRateFiltered + 0.20f * kalmanPitch.rate;
                    pwm_fuzzy = g_Kp_balance * (g_filteredPitch - targetAngle) + g_Kd_balance * s_gyroRateFiltered;
                }
                pwm_fuzzy = Clamp_Float(pwm_fuzzy, -PWM_LIMIT_BALANCE, PWM_LIMIT_BALANCE);

                /* 4. Tính toán góc hướng xoay Yaw (10% OutPWM = +-840) */
                float yawError  = Clamp_Float(g_filteredYaw - g_targetYaw, -18.0f, 18.0f);
                float diffSpeed = (-g_motorSpeedfiltered_left) - (-g_motorSpeedfiltered_right);
                s_yawRateFiltered = 0.75f * s_yawRateFiltered + 0.25f * kalmanYaw.rate;

                if (fabsf(g_filteredYaw - g_targetYaw) < 15.0f) {
                    if (yawError * g_yawIntegral < 0.0f) {
                        g_yawIntegral *= 0.85f;
                    }
                    if (fabsf(yawError) > 0.35f) {
                        g_yawIntegral += yawError * 0.005f;
                    }
                    g_yawIntegral = Clamp_Float(g_yawIntegral, -YAW_INTEGRAL_MAX, YAW_INTEGRAL_MAX);
                } else {
                    g_yawIntegral = 0.0f;
                }

                pwm_yaw = (g_Kp_yaw * yawError)
                        + (4.20f * g_yawIntegral)
                        + (g_Kd_yaw * diffSpeed)
                        + (g_Kd_yaw_imu * s_yawRateFiltered);
                pwm_yaw = Clamp_Float(pwm_yaw, -PWM_LIMIT_YAW, PWM_LIMIT_YAW);

                /* 5. Tổng hợp xung PWM 2 bánh (Trái / Phải) & Bù vùng chết + Lọc mượt */
                outpwm_L = pwm_fuzzy - pwm_yaw;
                outpwm_R = pwm_fuzzy + pwm_yaw;

                if (outpwm_L > 252.0f) outpwm_L += MOTOR_DEADBAND_PWM_L;
                else if (outpwm_L < -252.0f) outpwm_L -= MOTOR_DEADBAND_PWM_L;
                else outpwm_L += (outpwm_L / 252.0f) * MOTOR_DEADBAND_PWM_L;

                if (outpwm_R > 252.0f) outpwm_R += MOTOR_DEADBAND_PWM_R;
                else if (outpwm_R < -252.0f) outpwm_R -= MOTOR_DEADBAND_PWM_R;
                else outpwm_R += (outpwm_R / 252.0f) * MOTOR_DEADBAND_PWM_R;

                outpwm_L = 0.65f * outpwm_L + 0.35f * s_prevBalancePWM_L;
                outpwm_L = Clamp_Float(outpwm_L, s_prevBalancePWM_L - BALANCE_MAX_PWM_DELTA, s_prevBalancePWM_L + BALANCE_MAX_PWM_DELTA);
                s_prevBalancePWM_L = outpwm_L;

                outpwm_R = 0.65f * outpwm_R + 0.35f * s_prevBalancePWM_R;
                outpwm_R = Clamp_Float(outpwm_R, s_prevBalancePWM_R - BALANCE_MAX_PWM_DELTA, s_prevBalancePWM_R + BALANCE_MAX_PWM_DELTA);
                s_prevBalancePWM_R = outpwm_R;

                outpwm_L = Clamp_Float(outpwm_L, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
                outpwm_R = Clamp_Float(outpwm_R, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);

                Motor_SetPWM_Left(outpwm_L);
                Motor_SetPWM_Right(outpwm_R);

                /* Cập nhật các biến Telemetry gửi lên App GUI */
                g_cascadeTargetAngle = targetAngle;
                g_outFuzzy           = pwm_fuzzy;
                g_outSpeed           = s_cascadeTiltFiltered / CASCADE_CMD_TO_DEG;
                g_outPosition        = 0.0f;
                g_outYaw             = pwm_yaw;
                g_motorPWM_left      = outpwm_L;
                g_motorPWM_right     = outpwm_R;
            }
        }
        else
        {
            Motor_Stop();
            PID_Reset(&PID_Vel);
            Fuzzy_Reset(&fuzzyControl);
            s_cascadeTiltFiltered = 0.0f;
            s_yawRateFiltered     = 0.0f;
            s_prevBalancePWM_L    = 0.0f;
            s_prevBalancePWM_R    = 0.0f;
            g_yawIntegral         = 0.0f;
            g_cascadeTargetAngle  = g_targetAngle;
            g_outFuzzy            = 0.0f;
            g_outSpeed            = 0.0f;
            g_outPosition         = 0.0f;
            g_outYaw              = 0.0f;
            g_motorPWM_left       = 0.0f;
            g_motorPWM_right      = 0.0f;
        }

        if (++beat == 40) {
            beat = 0;
            if (!s_isCalibrating) {
                if (!g_systemEnable) {
                    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
                } else {
                    GPIOD->ODR &= ~(1 << 14);
                }
            }
        }
    }
}

static void vTaskUART(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    uint8_t  rxState       = 0;
    uint8_t  rxId          = 0;
    uint8_t  rxLen         = 0;
    uint8_t  rxCount       = 0;
    uint8_t  rxChecksum    = 0;
    uint8_t  beat          = 0;
    uint8_t  txDivider     = 0;
    uint16_t bootCycle     = 0;
    uint8_t  sendConfigReq = 1;
    uint8_t  rxDataBuf[48];

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(200)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (s_calibPending) {
            s_calibPending = 0;
            Robot_AutoCalibrate();
            sendConfigReq = 1;
        }

        if (++beat >= 25) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
        }

        while (UART3_Available() > 0)
        {
            int16_t ch = UART3_ReadChar();
            if (ch < 0) break;
            uint8_t byte = (uint8_t)ch;

            switch (rxState)
            {
            case 0:
                if (byte == UART_FRAME_HEADER) {
                    rxState = 1;
                }
                break;

            case 1:
                rxId = byte;
                rxState = 2;
                break;

            case 2:
                rxLen   = byte;
                rxCount = 0;
                if (rxLen > sizeof(rxDataBuf)) {
                    rxState = 0;
                } else if (rxLen == 0) {
                    rxState = 4;
                } else {
                    rxState = 3;
                }
                break;

            case 3:
                rxDataBuf[rxCount++] = byte;
                if (rxCount >= rxLen) {
                    rxState = 4;
                }
                break;

            case 4:
                rxChecksum = byte;
                rxState = 5;
                break;

            case 5:
                if (byte == UART_FRAME_TAIL && UART_CalculateChecksum(rxId, rxLen, rxDataBuf) == rxChecksum) {
                    switch (rxId)
                    {
                    case 0x01:
                        Robot_Start();
                        break;
                    case 0x02:
                        Robot_Stop();
                        break;
                    case 0x03:
                        Robot_AutoCalibrate();
                        sendConfigReq = 1;
                        break;
                    case 0x04:
                        if (rxLen >= 4) {
                            memcpy((void *)&g_targetAngle, rxDataBuf, 4);
                            g_cascadeTargetAngle = g_targetAngle;
                        }
                        break;
                    case 0x05:
                        if (rxLen >= 8) {
                            float newSpeedL = 0.0f, newSpeedR = 0.0f;
                            memcpy(&newSpeedL, &rxDataBuf[0], 4);
                            memcpy(&newSpeedR, &rxDataBuf[4], 4);
                            g_targetSpeed = (newSpeedL + newSpeedR) * 0.5f;
                        } else if (rxLen >= 4) {
                            memcpy((void *)&g_targetSpeed, rxDataBuf, 4);
                        }
                        g_targetSpeed = Clamp_Float(g_targetSpeed, -ROBOT_MAX_SPEED_RPM, ROBOT_MAX_SPEED_RPM);
                        break;
                    case 0x06:
                        if (rxLen == 0) {
                            Reset_OdometryAndStates();
                        } else if (rxLen >= 12) {
                            memcpy((void *)&g_targetX,   &rxDataBuf[0], 4);
                            memcpy((void *)&g_targetY,   &rxDataBuf[4], 4);
                            memcpy((void *)&g_targetYaw, &rxDataBuf[8], 4);
                        } else if (rxLen >= 4) {
                            memcpy((void *)&g_targetYaw, &rxDataBuf[0], 4);
                        }
                        break;
                    case 0x07:
                        if (rxLen >= 1 && rxDataBuf[0] <= CTRL_MODE_FUZZY) {
                            Robot_SetControlMode((RobotControlMode_t)rxDataBuf[0]);
                        }
                        break;
                    case 0x08:
                        if (rxLen >= 16) {
                            memcpy((void *)&g_Kp_balance,  &rxDataBuf[0],  4);
                            memcpy((void *)&g_Kd_balance,  &rxDataBuf[4],  4);
                            memcpy((void *)&g_Kp_position, &rxDataBuf[8],  4);
                            memcpy((void *)&g_Ki_position, &rxDataBuf[12], 4);
                            if (rxLen >= 20) {
                                memcpy((void *)&g_Kd_yaw, &rxDataBuf[16], 4);
                            }
                            if (rxLen >= 28) {
                                memcpy((void *)&g_Kp_yaw,     &rxDataBuf[20], 4);
                                memcpy((void *)&g_Kd_yaw_imu, &rxDataBuf[24], 4);
                            }
                            if (rxLen >= 36) {
                                memcpy((void *)&g_Kp_speed,   &rxDataBuf[28], 4);
                                memcpy((void *)&g_Ki_speed,   &rxDataBuf[32], 4);
                                PID_Vel.Kp = (g_Kp_speed > 1.0f) ? (g_Kp_speed * CASCADE_CMD_TO_DEG) : g_Kp_speed;
                                PID_Vel.Ki = (g_Ki_speed > 0.5f) ? (g_Ki_speed * CASCADE_CMD_TO_DEG) : g_Ki_speed;
                            }
                            g_speedIntegral = 0.0f;
                            g_posIntegral   = 0.0f;
                            PID_Reset(&PID_Vel);
                        }
                        break;
                    case 0x09:
                        sendConfigReq = 1;
                        break;
                    default:
                        break;
                    }
                }
                rxState = 0;
                break;

            default:
                rxState = 0;
                break;
            }
        }

        if (bootCycle <= 160) {
            bootCycle++;
            if (bootCycle == 30 || bootCycle == 150) {
                sendConfigReq = 1;
            }
        }

        if (sendConfigReq) {
            RobotConfigSync_t cfgSync;
            cfgSync.kp_balance  = g_Kp_balance;
            cfgSync.kd_balance  = g_Kd_balance;
            cfgSync.kp_position = g_Kp_position;
            cfgSync.ki_position = g_Ki_position;
            cfgSync.kd_yaw      = g_Kd_yaw;
            cfgSync.kp_yaw      = g_Kp_yaw;
            cfgSync.kd_yaw_imu  = g_Kd_yaw_imu;
            cfgSync.targetAngle = g_targetAngle;
            cfgSync.targetX     = g_targetX;
            cfgSync.targetY     = g_targetY;
            cfgSync.targetYaw   = g_targetYaw;
            cfgSync.controlMode = (float)g_controlMode;
            cfgSync.kp_speed    = g_Kp_speed;
            cfgSync.ki_speed    = g_Ki_speed;

            if (UART3_SendPacketDMA(0x02, &cfgSync, (uint8_t)sizeof(RobotConfigSync_t))) {
                sendConfigReq = 0;
                txDivider = 0;
            }
        } else if (++txDivider >= 2u) {
            txDivider = 0;

            RobotTelemetry_t telem;
            telem.rawPitch                 = g_rawAccPitch;
            telem.filteredPitch            = g_filteredPitch;
            telem.gyroRate                 = kalmanPitch.rate;
            telem.rawYaw                   = g_rawYawAngle;
            telem.filteredYaw              = g_filteredYaw;
            telem.yawRate                  = kalmanYaw.rate;
            telem.motorSpeed_left          = g_motorSpeed_left;
            telem.motorSpeed_right         = g_motorSpeed_right;
            telem.motorSpeedFiltered_left  = g_motorSpeedfiltered_left;
            telem.motorSpeedFiltered_right = g_motorSpeedfiltered_right;
            telem.robotX                   = g_robotX;
            telem.robotY                   = g_robotY;
            telem.motorPWM_left            = g_motorPWM_left;
            telem.motorPWM_right           = g_motorPWM_right;
            telem.targetAngle              = g_systemEnable ? g_cascadeTargetAngle : g_targetAngle;
            telem.outFuzzy                 = g_outFuzzy;
            telem.outPosition              = g_outPosition;
            telem.outYaw                   = g_outYaw;
            telem.outSpeed                 = g_outSpeed;

            UART3_SendPacketDMA(0x01, &telem, (uint8_t)sizeof(RobotTelemetry_t));
        }
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Motor_Stop();
    while (1)
    {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
        for (volatile int i = 0; i < 500000; i++);
    }
}
