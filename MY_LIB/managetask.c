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
 * 2. BIẾN & BỘ LỌC CỦA TASK ENCODER (Raw Wheel, Kalman Wheel, Yaw)
 * ===================================================================== */
KalmanSpeed_t kalmanSpeed_L;
KalmanSpeed_t kalmanSpeed_R;

volatile float g_motorSpeed_left          = 0.0f;
volatile float g_motorSpeed_right         = 0.0f;
volatile float g_motorSpeedfiltered_left  = 0.0f;
volatile float g_motorSpeedfiltered_right = 0.0f;

volatile float g_encoderYawAngle = 0.0f;
volatile float g_encoderYawRate  = 0.0f;

/* =====================================================================
 * 3. BIẾN CỦA TASK CONTROL (Trạng thái, Mục tiêu & Xung PWM xuất ra Motor)
 * ===================================================================== */
extern FuzzyController_t FuzzyPD_Pitch;
extern FuzzyController_t FuzzyPD_Pos;
volatile uint8_t g_systemEnable = 0;

volatile float g_targetAngle        = -2.95f;
volatile float g_cascadeTargetAngle = -2.95f;
volatile float g_targetYaw          = 0.0f;
volatile float g_targetSpeed        = 0.0f;

volatile float g_motorPWM_left  = 0.0f;
volatile float g_motorPWM_right = 0.0f;
volatile float g_outFuzzy       = 0.0f;
volatile float g_outSpeed       = 0.0f;
volatile float g_outYaw         = 0.0f;
volatile float g_velCmd         = 0.0f;   /* debug: vận tốc đặt vòng ngoài (RPM) */
volatile float g_dVel           = 0.0f;   /* debug: gia tốc góc bánh xe trung bình (RPM/s) */
volatile float g_posError       = 0.0f;   /* debug: sai số vị trí vòng ngoài x_fw - xTarget (cm) */

/* =====================================================================
 * CẤU HÌNH KIẾN TRÚC ĐIỀU KHIỂN VÒNG LỒNG (CASCADED CONTROL):
 *  - Vòng Ngoài (Vị trí/Vận tốc FuzzyPD_Pos / PID_Vel):
 *      Sinh ra Góc nghiêng đặt bù trừ tiltOffset (kẹp an toàn +-2.2°)
 *      -> Góc đặt Vòng Trong: g_cascadeTargetAngle = g_targetAngle - tiltOffset
 *  - Vòng Trong (FuzzyPD / Balance PD):
 *      Bám theo g_cascadeTargetAngle, xuất 90% tổng công suất (+-7560 PWM)
 *  - Vòng Độc lập Ngang (Yaw PID):
 *      Điều khiển hướng xoay với 10% tổng công suất (+-840 PWM)
 * ===================================================================== */
volatile RobotControlMode_t g_controlMode = CTRL_MODE_FUZZY;
volatile float g_Kp_balance  = 580.0f;
volatile float g_Kd_balance  = 19.50f;
volatile float g_Kp_speed    = 18.00f;
volatile float g_Ki_speed    = 4.20f;
volatile float g_Kd_yaw      = 6.30f;
volatile float g_Kp_yaw      = 25.20f;
volatile float g_Ki_yaw      = 4.20f;
volatile float g_Kd_yaw_imu  = 3.78f;

volatile float g_yawIntegral   = 0.0f;
#define YAW_INTEGRAL_MAX       25.0f

/* Chu kỳ điều khiển đồng bộ toàn hệ thống (Vòng ngoài + Vòng trong + Yaw): 5ms (200Hz) */
#define CONTROL_LOOP_DT                  0.005f
#define POS_TARGET_LEAD_MAX_CM           15.0f   /* Giới hạn điểm đích lệch xa vị trí hiện tại tối đa (15 cm, không kẹp cứng ở ±6.5 cm như lan29) */
#define COM_PITCH_RATE_COMP_RPM_PER_DPS  0.42f   /* Bù vận tốc góc thân xe (RPM / (°/s)) để tách vận tốc tịnh tiến trọng tâm (chống cộng hưởng 4.8 Hz) */
#define COM_PITCH_POS_COMP_CM_PER_DEG    0.15f   /* Bù dịch chuyển bánh xe theo góc nghiêng thân xe (cm / °) */
#define MOTOR_BEMF_ASSIST_GAIN           8.00f   /* Bù sức phản điện động Back-EMF (PWM / RPM) giúp giảm độ sụt góc vòng trong khi bánh quay */
#define MOTOR_BEMF_ASSIST_MAX            750.0f  /* Giới hạn lượng bù Back-EMF tối đa (+-750 PWM) */
#define POS_INTEGRAL_KI                  0.004f  /* Hệ số tích phân tự động cân tâm góc tĩnh cực chậm (°/(cm*s), không gây trễ pha) */
#define POS_INTEGRAL_MAX_DEG             0.85f   /* Giới hạn lượng bù góc cân bằng tĩnh tối đa (+-0.85°) */

/* Bù vùng chết & độ lợi động học động cơ (giữ vùng đệm mềm 200 PWM & mức bù nhỏ để êm quanh 0,
 * đồng thời cân đối lại L = 110/85 và R = 155/160 theo dữ liệu lan29): */
#define MOTOR_DEADBAND_PWM_L_POS   110.0f
#define MOTOR_DEADBAND_PWM_L_NEG    85.0f
#define MOTOR_DEADBAND_PWM_R_POS   155.0f
#define MOTOR_DEADBAND_PWM_R_NEG   160.0f
#define MOTOR_DEADBAND_SOFT_RANGE  200.0f
#define MOTOR_GAIN_SCALE_L         0.94f
#define MOTOR_GAIN_SCALE_R         1.04f
#define BALANCE_MAX_PWM_DELTA      900.0f

static float s_prevBalancePWM_L   = 0.0f;
static float s_prevBalancePWM_R   = 0.0f;
static float s_posIntegralTilt    = 0.0f;   /* Dự phòng */
static float s_tiltOffsetFiltered = 0.0f;   /* Bộ lọc thông thấp góc nghiêng đặt vòng ngoài */
static float s_velCoMFiltered     = 0.0f;   /* Vận tốc tịnh tiến trọng tâm (RPM) đã lọc thông thấp, khử dao động lắc 4.8 Hz */

static volatile float    s_testPwmL       = 0.0f;
static volatile float    s_testPwmR       = 0.0f;
static volatile uint16_t s_testPwmTimeout = 0;   /* Timeout an toàn cho lệnh dò vùng chết 0x0B (~450ms) */

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

static volatile uint8_t s_posResetReq = 1;   /* 1: yêu cầu vòng vị trí lấy vị trí hiện tại làm gốc / điểm đích */
static void Reset_OdometryAndStates(void) {
    s_posResetReq          = 1;   /* vTaskControl sẽ đặt lại gốc vị trí & điểm đích tại vị trí hiện tại */
    g_yawIntegral          = 0.0f;
    g_encoderYawAngle      = 0.0f;
    g_encoderYawRate       = 0.0f;
    g_rawYawAngle          = 0.0f;
    g_filteredYaw          = 0.0f;
    g_targetYaw            = 0.0f;
    g_posError             = 0.0f;
    g_cascadeTargetAngle   = g_targetAngle;
    s_prevBalancePWM_L     = 0.0f;
    s_prevBalancePWM_R     = 0.0f;
    s_posIntegralTilt      = 0.0f;
    s_tiltOffsetFiltered   = 0.0f;
    s_velCoMFiltered       = 0.0f;
    PID_Reset(&PID_Vel);
    Fuzzy_Reset(&FuzzyPD_Pitch);
    Fuzzy_Reset(&FuzzyPD_Pos);
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
    xTaskCreate(vTaskControl, "Task_Control", 1024, NULL, 4, NULL);
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

    g_systemEnable   = 1;
    s_testPwmTimeout = 0;
    s_testPwmL       = 0.0f;
    s_testPwmR       = 0.0f;
    Reset_OdometryAndStates();
    g_motorSpeedfiltered_left  = 0.0f;
    g_motorSpeedfiltered_right = 0.0f;

    KalmanSpeed_SetSpeed(&kalmanSpeed_L, 0.0f);
    KalmanSpeed_SetSpeed(&kalmanSpeed_R, 0.0f);

    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    return 1;
}

void Robot_Stop(void) {
    g_systemEnable   = 0;
    s_testPwmTimeout = 0;
    s_testPwmL       = 0.0f;
    s_testPwmR       = 0.0f;
    Motor_Stop();

    g_yawIntegral          = 0.0f;
    s_prevBalancePWM_L     = 0.0f;
    s_prevBalancePWM_R     = 0.0f;
    s_posIntegralTilt      = 0.0f;
    s_tiltOffsetFiltered   = 0.0f;
    s_velCoMFiltered       = 0.0f;
    PID_Reset(&PID_Vel);
    Fuzzy_Reset(&FuzzyPD_Pitch);
    Fuzzy_Reset(&FuzzyPD_Pos);

    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

void Robot_SetControlMode(RobotControlMode_t mode) {
    g_controlMode = mode;
    Reset_OdometryAndStates();
    Motor_Stop();
}

static volatile uint8_t s_isCalibrating = 0;
static volatile int32_t s_encTotalSum   = 0;   /* Tổng xung encoder L+R tích lũy (cập nhật mỗi 5ms ở vTaskEncoder) */
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
    int8_t beat = 0;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Encoder_ReadStruct(&leftMotor, &rightMotor);

        /* Odometry yaw & tổng xung: cập nhật mỗi 5ms (200Hz) */
        float dPulse_L       = -(float)leftMotor.delta_pulse;
        float dPulse_R       = -(float)rightMotor.delta_pulse;
        float deltaPulseDiff = dPulse_L - dPulse_R;

        g_encoderYawAngle += deltaPulseDiff * YAW_DEG_PER_DIFF_PULSE;
        s_encTotalSum += (int32_t)(leftMotor.delta_pulse + rightMotor.delta_pulse);

        /* Tốc độ bánh & Kalman: cập nhật trực tiếp mỗi 5ms (200Hz) */
        g_motorSpeed_left  = leftMotor.speed;
        g_motorSpeed_right = rightMotor.speed;

        g_motorSpeedfiltered_left  = KalmanSpeed_GetSpeed(&kalmanSpeed_L, g_motorSpeed_left,  ENCODER_SAMPLE_TIME);
        g_motorSpeedfiltered_right = KalmanSpeed_GetSpeed(&kalmanSpeed_R, g_motorSpeed_right, ENCODER_SAMPLE_TIME);

        float vDiff_cms  = (-g_motorSpeedfiltered_left - (-g_motorSpeedfiltered_right)) * WHEEL_RPM_TO_CMS;
        g_encoderYawRate = vDiff_cms * YAW_DEG_PER_DIFF_CM;

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
    /* Vòng ngoài giữ vị trí: e = sai số vị trí (cm), de = sai số vận tốc (cm/s). Dấu theo firmware: tiến = âm. */
    int32_t posRefSum = 0;      /* tổng xung L+R tại gốc vị trí */
    float   xTarget   = 0.0f;   /* vị trí đích (cm), tự dịch chuyển theo vận tốc đặt */
    float tiltOffset = 0.0f, targetAngle = 0.0f;
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
                g_cascadeTargetAngle = g_targetAngle;
            }
            else
            {
                /* 2. Vòng Ngoài Vị trí / Vận tốc (chu kỳ 5ms / 200Hz): FuzzyPD_Pos hoặc PID_Vel -> Góc nghiêng bù trừ tiltOffset
                 *    targetAngle = g_targetAngle - tiltOffset  (tiltOffset > 0: ngả về sau) */
                vel      = (g_motorSpeedfiltered_left + g_motorSpeedfiltered_right) * 0.5f;
                vel_cmd  = -Clamp_Float(g_targetSpeed, -ROBOT_MAX_SPEED_RPM, ROBOT_MAX_SPEED_RPM);
                g_velCmd = vel_cmd;
                g_dVel   = (kalmanSpeed_L.accel + kalmanSpeed_R.accel) * 0.5f;

                /* Tách vận tốc tịnh tiến của trọng tâm (CoM): khử thành phần bánh xe giật ngược pha khi thân xe lắc ở 4.8 Hz */
                float pitchRateClamped = Clamp_Float(kalmanPitch.rate, -50.0f, 50.0f);
                float vel_com = vel - (COM_PITCH_RATE_COMP_RPM_PER_DPS * pitchRateClamped);
                s_velCoMFiltered += 0.16f * (vel_com - s_velCoMFiltered);

                if (g_controlMode == CTRL_MODE_FUZZY) {
                    /* FuzzyPD_Pos giữ vị trí theo tọa độ trọng tâm CoM:
                     *   e  = x_com - xTarget          (cm)
                     *   de = (e[k] - e[k-1]) / dt     (cm/s, có first_sample & lọc thông thấp) */
                    if (s_posResetReq) {
                        s_posResetReq        = 0;
                        posRefSum            = s_encTotalSum;
                        xTarget              = 0.0f;
                        s_posIntegralTilt    = 0.0f;
                        s_tiltOffsetFiltered = 0.0f;
                        s_velCoMFiltered     = 0.0f;
                        Fuzzy_Reset(&FuzzyPD_Pos);
                    }
                    float baseBalanceAngle = g_targetAngle - s_posIntegralTilt;
                    float x_fw     = (float)(s_encTotalSum - posRefSum) * 0.5f * WHEEL_CM_PER_PULSE;
                    float x_com    = x_fw - COM_PITCH_POS_COMP_CM_PER_DEG * (g_filteredPitch - baseBalanceAngle);
                    float v_cms    = s_velCoMFiltered * WHEEL_RPM_TO_CMS;
                    float vcmd_cms = vel_cmd * WHEEL_RPM_TO_CMS;
                    xTarget += vcmd_cms * CONTROL_LOOP_DT;                       /* điểm đích trôi theo vận tốc đặt */
                    xTarget  = Clamp_Float(xTarget, x_fw - POS_TARGET_LEAD_MAX_CM, x_fw + POS_TARGET_LEAD_MAX_CM);

                    float posErr = x_fw - xTarget;
                    g_posError   = posErr;

                    /* Tự động học độ lệch góc cân bằng tĩnh cực chậm khi xe đứng yên ổn định để đưa pos_err về 0 cm */
                    if (fabsf(vel_cmd) < 0.1f && fabsf(g_filteredPitch - baseBalanceAngle) < 1.5f &&
                        fabsf(kalmanPitch.rate) < 12.0f && fabsf(s_velCoMFiltered) < 10.0f) {
                        if ((posErr * v_cms) >= -1.0f) {
                            s_posIntegralTilt -= POS_INTEGRAL_KI * Clamp_Float(posErr, -6.0f, 6.0f) * CONTROL_LOOP_DT;
                            s_posIntegralTilt  = Clamp_Float(s_posIntegralTilt, -POS_INTEGRAL_MAX_DEG, POS_INTEGRAL_MAX_DEG);
                        }
                    }

                    /* Tính đạo hàm sai số (e[k] - e[k-1]) / dt kèm first_sample và lọc thông thấp trong FuzzyPD_Compute */
                    float rawTilt   = -FuzzyPD_Compute(&FuzzyPD_Pos, xTarget, x_com, CONTROL_LOOP_DT);
                    s_tiltOffsetFiltered += 0.25f * (rawTilt - s_tiltOffsetFiltered);
                    tiltOffset = Clamp_Float(s_posIntegralTilt + s_tiltOffsetFiltered, -CASCADE_MAX_TOTAL_TILT_DEG, CASCADE_MAX_TOTAL_TILT_DEG);
                } else {
                    g_posError = 0.0f;
                    tiltOffset = PID_Compute(&PID_Vel, vel_cmd, s_velCoMFiltered, CONTROL_LOOP_DT);
                }

                targetAngle = g_targetAngle - tiltOffset;

                /* 3. Vòng Trong Cân bằng Pitch + Bù Back-EMF theo vận tốc trọng tâm đã lọc (không làm suy giảm Kd_pitch ở 4.8 Hz) */
                float pwm_bemf = Clamp_Float(-MOTOR_BEMF_ASSIST_GAIN * (s_velCoMFiltered - vel_cmd), -MOTOR_BEMF_ASSIST_MAX, MOTOR_BEMF_ASSIST_MAX);
                if (g_controlMode == CTRL_MODE_FUZZY) {
                    pwm_fuzzy = FuzzyPD_Compute(&FuzzyPD_Pitch, targetAngle, g_filteredPitch, CONTROL_LOOP_DT) + pwm_bemf;
                } else {
                    pwm_fuzzy = g_Kp_balance * (g_filteredPitch - targetAngle) + g_Kd_balance * kalmanPitch.rate + pwm_bemf;
                }
                pwm_fuzzy = Clamp_Float(pwm_fuzzy, -PWM_LIMIT_BALANCE, PWM_LIMIT_BALANCE);

                /* 4. Tính toán góc hướng xoay Yaw (10% OutPWM = +-840) */
                float yawError  = Clamp_Float(g_filteredYaw - g_targetYaw, -18.0f, 18.0f);
                float diffSpeed = (-g_motorSpeedfiltered_left) - (-g_motorSpeedfiltered_right);

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
                        + (g_Ki_yaw * g_yawIntegral)
                        + (g_Kd_yaw * diffSpeed)
                        + (g_Kd_yaw_imu * kalmanYaw.rate);
                pwm_yaw = Clamp_Float(pwm_yaw, -PWM_LIMIT_YAW, PWM_LIMIT_YAW);

                /* 5. Tổng hợp xung PWM 2 bánh (Trái / Phải) & Bù độ lợi + Bù vùng chết + Giới hạn tốc độ thay đổi PWM */
                outpwm_L = (pwm_fuzzy * MOTOR_GAIN_SCALE_L) - pwm_yaw;
                outpwm_R = (pwm_fuzzy * MOTOR_GAIN_SCALE_R) + pwm_yaw;

                if (outpwm_L > MOTOR_DEADBAND_SOFT_RANGE) outpwm_L += MOTOR_DEADBAND_PWM_L_POS;
                else if (outpwm_L < -MOTOR_DEADBAND_SOFT_RANGE) outpwm_L -= MOTOR_DEADBAND_PWM_L_NEG;
                else if (outpwm_L >= 0.0f) outpwm_L += (outpwm_L / MOTOR_DEADBAND_SOFT_RANGE) * MOTOR_DEADBAND_PWM_L_POS;
                else outpwm_L += (outpwm_L / MOTOR_DEADBAND_SOFT_RANGE) * MOTOR_DEADBAND_PWM_L_NEG;

                if (outpwm_R > MOTOR_DEADBAND_SOFT_RANGE) outpwm_R += MOTOR_DEADBAND_PWM_R_POS;
                else if (outpwm_R < -MOTOR_DEADBAND_SOFT_RANGE) outpwm_R -= MOTOR_DEADBAND_PWM_R_NEG;
                else if (outpwm_R >= 0.0f) outpwm_R += (outpwm_R / MOTOR_DEADBAND_SOFT_RANGE) * MOTOR_DEADBAND_PWM_R_POS;
                else outpwm_R += (outpwm_R / MOTOR_DEADBAND_SOFT_RANGE) * MOTOR_DEADBAND_PWM_R_NEG;

                outpwm_L = Clamp_Float(outpwm_L, s_prevBalancePWM_L - BALANCE_MAX_PWM_DELTA, s_prevBalancePWM_L + BALANCE_MAX_PWM_DELTA);
                s_prevBalancePWM_L = outpwm_L;

                outpwm_R = Clamp_Float(outpwm_R, s_prevBalancePWM_R - BALANCE_MAX_PWM_DELTA, s_prevBalancePWM_R + BALANCE_MAX_PWM_DELTA);
                s_prevBalancePWM_R = outpwm_R;

                outpwm_L = Clamp_Float(outpwm_L, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
                outpwm_R = Clamp_Float(outpwm_R, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);

                Motor_SetPWM_Left(outpwm_L);
                Motor_SetPWM_Right(outpwm_R);

                /* Cập nhật các biến Telemetry gửi lên App GUI (outSpeed truyền trực tiếp độ nghiêng bù trừ °) */
                g_cascadeTargetAngle = targetAngle;
                g_outFuzzy           = pwm_fuzzy;
                g_outSpeed           = tiltOffset;
                g_outYaw             = pwm_yaw;
                g_motorPWM_left      = outpwm_L;
                g_motorPWM_right     = outpwm_R;
            }
        }
        else
        {
            if (s_testPwmTimeout > 0u) {
                s_testPwmTimeout--;
                Motor_SetPWM_Left(s_testPwmL);
                Motor_SetPWM_Right(s_testPwmR);
                g_motorPWM_left  = s_testPwmL;
                g_motorPWM_right = s_testPwmR;
            } else {
                s_testPwmL       = 0.0f;
                s_testPwmR       = 0.0f;
                Motor_Stop();
                g_motorPWM_left  = 0.0f;
                g_motorPWM_right = 0.0f;
            }
            PID_Reset(&PID_Vel);
            Fuzzy_Reset(&FuzzyPD_Pitch);
            Fuzzy_Reset(&FuzzyPD_Pos);
            tiltOffset            = 0.0f;
            s_posIntegralTilt     = 0.0f;
            s_tiltOffsetFiltered  = 0.0f;
            g_velCmd              = 0.0f;
            g_dVel                = 0.0f;
            g_posError            = 0.0f;
            s_prevBalancePWM_L    = 0.0f;
            s_prevBalancePWM_R    = 0.0f;
            g_yawIntegral         = 0.0f;
            g_cascadeTargetAngle  = g_targetAngle;
            g_outFuzzy            = 0.0f;
            g_outSpeed            = 0.0f;
            g_outYaw              = 0.0f;
        }

        if (++beat == 40) {
            beat = 0;
            if (!s_isCalibrating) {
                if (!g_systemEnable) {
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
            {
                uint8_t xorCs = UART_CalculateChecksum(rxId, rxLen, rxDataBuf);
                uint8_t orCs  = rxId | rxLen;
                for (uint8_t i = 0; i < rxLen; i++) {
                    orCs |= rxDataBuf[i];
                }
                if (byte == UART_FRAME_TAIL && (rxChecksum == xorCs || rxChecksum == orCs)) {
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
                            s_posIntegralTilt    = 0.0f;
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
                        if (rxLen >= 28) {
                            memcpy((void *)&g_Kp_balance, &rxDataBuf[0],  4);
                            memcpy((void *)&g_Kd_balance, &rxDataBuf[4],  4);
                            memcpy((void *)&g_Kd_yaw,     &rxDataBuf[8],  4);
                            memcpy((void *)&g_Kp_yaw,     &rxDataBuf[12], 4);
                            memcpy((void *)&g_Kd_yaw_imu, &rxDataBuf[16], 4);
                            memcpy((void *)&g_Kp_speed,   &rxDataBuf[20], 4);
                            memcpy((void *)&g_Ki_speed,   &rxDataBuf[24], 4);
                            g_Ki_yaw   = g_Ki_speed;
                            PID_Vel.Kp = (g_Kp_speed > 1.0f) ? (g_Kp_speed * CASCADE_CMD_TO_DEG) : g_Kp_speed;
                            PID_Vel.Ki = (g_Ki_speed > 0.5f) ? (g_Ki_speed * CASCADE_CMD_TO_DEG) : g_Ki_speed;
                            PID_Reset(&PID_Vel);
                            sendConfigReq = 1;
                        } else if (rxLen >= 12) {
                            /* Gói 3 float (12 byte) từ GUI App: [Kp_yaw, Ki_yaw, Kd_yaw] */
                            memcpy((void *)&g_Kp_yaw,     &rxDataBuf[0], 4);
                            memcpy((void *)&g_Ki_yaw,     &rxDataBuf[4], 4);
                            memcpy((void *)&g_Kd_yaw_imu, &rxDataBuf[8], 4);
                            g_Ki_speed    = g_Ki_yaw;
                            g_yawIntegral = 0.0f;
                            sendConfigReq = 1;
                        }
                        break;
                    case 0x09:
                        sendConfigReq = 1;
                        break;
                    case 0x0A:
                        /* SET FUZZY: 6 float = [Pitch ke, Pitch kde, Pitch ku, Pos ke, Pos kde, Pos ku]
                         * Chỉ nhận giá trị dương hữu hạn; giá trị <= 0 sẽ được giữ nguyên */
                        if (rxLen >= 24) {
                            float fz[6];
                            memcpy(fz, rxDataBuf, sizeof(fz));
                            if (fz[0] > 0.0f && fz[0] < 1.0e6f) FuzzyPD_Pitch.ke  = fz[0];
                            if (fz[1] > 0.0f && fz[1] < 1.0e6f) FuzzyPD_Pitch.kde = fz[1];
                            if (fz[2] > 0.0f && fz[2] < 1.0e6f) FuzzyPD_Pitch.ku  = fz[2];
                            if (fz[3] > 0.0f && fz[3] < 1.0e6f) FuzzyPD_Pos.ke    = fz[3];
                            if (fz[4] > 0.0f && fz[4] < 1.0e6f) FuzzyPD_Pos.kde   = fz[4];
                            if (fz[5] >= 0.0f && fz[5] < 1.0e6f) FuzzyPD_Pos.ku   = fz[5];  /* cho phép ku_Pos = 0 để tắt vòng ngoài */
                            sendConfigReq = 1;   /* gửi lại bộ số hiện tại để app xác nhận */
                        }
                        break;
                    case 0x0B:
                        /* CMD_TEST_PWM: 2 float = [pwm_L, pwm_R] dùng cho công cụ dò vùng chết trên PC */
                        if (rxLen >= 8 && !g_systemEnable) {
                            float testL = 0.0f, testR = 0.0f;
                            memcpy(&testL, &rxDataBuf[0], 4);
                            memcpy(&testR, &rxDataBuf[4], 4);
                            testL = Clamp_Float(testL, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
                            testR = Clamp_Float(testR, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
                            s_testPwmL = testL;
                            s_testPwmR = testR;
                            if (fabsf(testL) > 0.5f || fabsf(testR) > 0.5f) {
                                s_testPwmTimeout = 90u; /* ~450ms giữ sống */
                            } else {
                                s_testPwmTimeout = 0u;
                                Motor_Stop();
                            }
                        }
                        break;
                    default:
                        break;
                    }
                }
                rxState = 0;
                break;
            }

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
            cfgSync.kd_yaw      = g_Kd_yaw;
            cfgSync.kp_yaw      = g_Kp_yaw;
            cfgSync.kd_yaw_imu  = g_Kd_yaw_imu;
            cfgSync.targetAngle = g_targetAngle;
            cfgSync.targetYaw   = g_targetYaw;
            cfgSync.controlMode = (float)g_controlMode;
            cfgSync.kp_speed    = g_Kp_speed;
            cfgSync.ki_speed    = g_Ki_yaw;
            cfgSync.pitch_ke    = FuzzyPD_Pitch.ke;
            cfgSync.pitch_kde   = FuzzyPD_Pitch.kde;
            cfgSync.pitch_ku    = FuzzyPD_Pitch.ku;
            cfgSync.pos_ke      = FuzzyPD_Pos.ke;
            cfgSync.pos_kde     = FuzzyPD_Pos.kde;
            cfgSync.pos_ku      = FuzzyPD_Pos.ku;

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
            telem.motorPWM_left            = g_motorPWM_left;
            telem.motorPWM_right           = g_motorPWM_right;
            telem.targetAngle              = g_systemEnable ? g_cascadeTargetAngle : g_targetAngle;
            telem.outFuzzy                 = g_outFuzzy;
            telem.outYaw                   = g_outYaw;
            telem.outSpeed                 = g_outSpeed;
            telem.velCmd                   = g_velCmd;
            telem.dVel                     = g_posError;
            telem.pitchENorm               = FuzzyPD_Pitch.e_norm;
            telem.pitchDeNorm              = FuzzyPD_Pitch.de_norm;
            telem.posENorm                 = FuzzyPD_Pos.e_norm;
            telem.posDeNorm                = FuzzyPD_Pos.de_norm;

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
