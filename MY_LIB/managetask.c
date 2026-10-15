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
#include <math.h>
#include <string.h>

/* =====================================================================
 * 1. BIẾN & BỘ LỌC CỦA TASK IMU (Lấy Raw Pitch/Yaw & Kalman Pitch/Yaw)
 * ===================================================================== */
Kalman_t kalmanPitch;            /* Bộ lọc Kalman cho góc nghiêng Pitch */
Kalman_t kalmanYaw;              /* Bộ lọc Kalman cho góc hướng xoay Yaw */

volatile float g_rawAccPitch   = 0.0f;  /* Góc Pitch thô từ Accelerometer (deg) */
volatile float g_rawGyroRate   = 0.0f;  /* Vận tốc góc Pitch thô từ Gyro Y (deg/s) */
volatile float g_rawYawAngle   = 0.0f;  /* Góc Yaw thô tích phân từ Gyro Z (deg) */
volatile float g_rawYawRate    = 0.0f;  /* Vận tốc góc Yaw thô từ Gyro Z (deg/s) */

volatile float g_filteredPitch = 0.0f;  /* Góc Pitch sau lọc kalmanPitch (deg) */
volatile float g_filteredYaw   = 0.0f;  /* Góc Yaw sau lọc kalmanYaw (deg) */

volatile float g_gyroOffset    = 0.0f;  /* Offset tĩnh Gyro trục Y (Pitch) */
volatile float g_gyroZOffset   = 0.0f;  /* Offset tĩnh Gyro trục Z (Yaw) */

/* Chiều của trục Z Gyro IMU so với (wheelSpeed_L - wheelSpeed_R):
 * Nếu xoay xe sang phải (bánh trái tiến nhanh hơn bánh phải) mà g_rawYawRate bị âm,
 * hãy đổi YAW_GYRO_SIGN từ -1.0f thành 1.0f */
#define YAW_GYRO_SIGN          (-1.0f)

/* =====================================================================
 * 2. BIẾN & BỘ LỌC CỦA TASK ENCODER (Raw Wheel, Kalman Wheel, Pos X, Y)
 * ===================================================================== */
KalmanSpeed_t kalmanSpeed_L;     /* Bộ lọc Kalman cho Vận tốc Bánh Trái */
KalmanSpeed_t kalmanSpeed_R;     /* Bộ lọc Kalman cho Vận tốc Bánh Phải */

volatile float g_motorSpeed_left          = 0.0f;  /* Vận tốc Raw bánh trái (RPM) */
volatile float g_motorSpeed_right         = 0.0f;  /* Vận tốc Raw bánh phải (RPM) */
volatile float g_motorSpeedfiltered_left  = 0.0f;  /* Vận tốc Kalman bánh trái (RPM) */
volatile float g_motorSpeedfiltered_right = 0.0f;  /* Vận tốc Kalman bánh phải (RPM) */
volatile float g_motorSpeed               = 0.0f;  /* Vận tốc trung bình sau lọc Kalman (RPM) */

/* Tọa độ Pos (X, Y) & Góc Yaw chuẩn vi sai từ 2 Encoder bánh xe D=65mm (không bao giờ trôi) */
volatile float g_robotX          = 0.0f;  /* Tọa độ X trên mặt phẳng (cm) */
volatile float g_robotY          = 0.0f;  /* Tọa độ Y trên mặt phẳng (cm) */
volatile float g_robotDist       = 0.0f;  /* Quãng đường dịch chuyển tổng hợp (cm) */
volatile float g_encoderYawAngle = 0.0f;  /* Góc Yaw vi sai chuẩn từ xung 2 Encoder (deg) - neo chống trôi cho kalmanYaw */
volatile float g_encoderYawRate  = 0.0f;  /* Vận tốc góc Yaw vi sai từ 2 Encoder (deg/s) */

/* =====================================================================
 * 3. BIẾN CỦA TASK CONTROL (Trạng thái, Mục tiêu & Xung PWM xuất ra Motor)
 * ===================================================================== */
extern FuzzyController_t fuzzyControl;
volatile uint8_t g_systemEnable = 0;

volatile float g_targetAngle   = 0.0f; /* Điểm cân bằng thực tế đo từ lan5.csv (~2.85 độ) */
volatile float g_targetYaw     = 0.0f;
volatile float g_targetSpeed   = 0.0f;
volatile float g_targetX       = 0.0f;
volatile float g_targetY       = 0.0f;

volatile float g_motorPWM       = 0.0f;
volatile float g_motorPWM_left  = 0.0f;
volatile float g_motorPWM_right = 0.0f;

/* =====================================================================
 * CẤU HÌNH & THÔNG SỐ ĐIỀU KHIỂN CÂN BẰNG, VỊ TRÍ & KHÓA CỨNG GÓC YAW
 * ===================================================================== */
//volatile RobotControlMode_t g_controlMode = CTRL_MODE_DUAL_PID;
volatile RobotControlMode_t g_controlMode = CTRL_MODE_FUZZY;
volatile float g_Kp_balance  = 48.0f;
volatile float g_Kd_balance  = 1.20f;
volatile float g_Kp_position = 2.20f;
volatile float g_Ki_position = 0.65f;
volatile float g_Kd_yaw      = 1.00f;  /* Hệ số D ghìm lệch vận tốc 2 bánh Encoder */
volatile float g_Kp_yaw      = 5.00f;  /* Hệ số P khóa cứng góc Yaw (Encoder + IMU Kalman) */
volatile float g_Kd_yaw_imu  = 0.60f;  /* Hệ số D phanh vận tốc xoay ngang Gyro Z từ IMU */

/* Biến tích phân tọa độ (Coordinate Integral), tích phân khóa cứng Yaw & lọc vi phân Gyro */
volatile float g_posIntegral = 0.0f;
volatile float g_yawIntegral = 0.0f;
#define POS_INTEGRAL_MAX     500.0f
#define YAW_INTEGRAL_MAX     45.0f
static float s_gyroRateFiltered = 0.0f;
static float s_yawRateFiltered  = 0.0f;

/* Bù vùng chết động cơ riêng cho 2 bánh và giới hạn gia tốc PWM */
#define MOTOR_DEADBAND_PWM_L  50.0f
#define MOTOR_DEADBAND_PWM_R  40.0f
#define BALANCE_MAX_PWM_DELTA 250.0f
static float s_prevBalancePWM_L = 0.0f;
static float s_prevBalancePWM_R = 0.0f;

/* Task & Controller Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);
static void vTaskUART(void *pvParameters);
static void Robot_Controller_Update(float dt);

void System_Tasks_Init(void) {
    /* 1. Khởi tạo bus I2C, encoder, UART và các bộ lọc Kalman */
    I2C1_MPU6050_Init();
    Encoder_Config();
    UART3_Init(115200);
    KalmanAngle_Init(&kalmanPitch);
    KalmanAngle_Init(&kalmanYaw);
    KalmanSpeed_InitDefault(&kalmanSpeed_L);
    KalmanSpeed_InitDefault(&kalmanSpeed_R);

    /* Đọc First Sample từ Gia tốc kế để nạp ngay góc khởi đầu cho Kalman Pitch */
    int16_t init_ax = 0, init_ay = 0, init_az = 0, init_gx = 0, init_gy = 0, init_gz = 0;
    if (MPU6050_ReadRawData(&init_ax, &init_ay, &init_az, &init_gx, &init_gy, &init_gz) == 0u) {
        float axg = (float)init_ax / 16384.0f;
        float ayg = (float)init_ay / 16384.0f;
        float azg = (float)init_az / 16384.0f;
        float initPitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);
        Kalman_SetAngle(&kalmanPitch, initPitch);
        g_filteredPitch = initPitch;
    }
    Kalman_SetAngle(&kalmanYaw, 0.0f);

    /* Lấy mẫu nhanh 20 lần lúc khởi động để xác định Offset trục Z (Yaw) và kiểm tra trục Y */
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

    /* 2. Khởi tạo góc mục tiêu ban đầu (mặc định 0 độ theo lan5.csv) và đọc Flash (Sector 11) */
    g_targetAngle = -0.73f;
    RobotFlashConfig_t savedConfig;
    if (Flash_ReadConfig(&savedConfig)) {
        if (fabsf(savedConfig.targetAngle) > 0.1f && fabsf(savedConfig.targetAngle) < 12.0f) {
        //    g_targetAngle = savedConfig.targetAngle;
        }
        g_gyroOffset = savedConfig.gyroOffset;
        if (validInit > 0 && fabsf((float)init_gy) < 500.0f && fabsf((float)init_gy - g_gyroOffset) > 120.0f) {
            g_gyroOffset = (float)sumGyInit / (float)validInit;
        }

        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        for (volatile int d = 0; d < 500000; d++);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    }

    /* 3. Mặc định hệ thống ở trạng thái STOP: Bật LED Đỏ (PD14) báo Standby */
    g_systemEnable = 0;
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15);

    /* 4. Tạo các Task cho FreeRTOS */
    xTaskCreate(vTaskIMU, "Task_IMU", 512, NULL, 3, NULL);
    xTaskCreate(vTaskEncoder, "Task_Encoder", 512, NULL, 3, NULL);
    xTaskCreate(vTaskControl, "Task_Control", 512, NULL, 4, NULL);
    xTaskCreate(vTaskUART, "Task_UART", 512, NULL, 2, NULL);
}

/* =====================================================================
 * CÁC HÀM ĐIỀU KHIỂN TRẠNG THÁI HỆ THỐNG (START / STOP / CALIB / MODE)
 * ===================================================================== */

void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t d = 0; d < 35000; d++);
    }
}

uint8_t Robot_Start(void) {
    float diffAngle = fabsf(g_filteredPitch - g_targetAngle);

    /* Điều kiện an toàn: xe phải dựng thẳng (< 15 độ) mới cho chạy (không chặn cứng vTaskUART) */
    if (diffAngle > 15.0f) {
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
        return 0;
    }

    g_systemEnable = 1;
    g_posIntegral = 0.0f;
    g_yawIntegral = 0.0f;
    g_robotX = 0.0f;
    g_robotY = 0.0f;
    g_robotDist = 0.0f;
    g_targetX = 0.0f;
    g_targetY = 0.0f;
    g_encoderYawAngle = 0.0f;
    g_encoderYawRate  = 0.0f;
    g_rawYawAngle = 0.0f;
    g_filteredYaw = 0.0f;
    g_targetYaw = 0.0f;
    Kalman_SetAngle(&kalmanYaw, 0.0f);
    s_gyroRateFiltered = 0.0f;
    s_yawRateFiltered = 0.0f;
    s_prevBalancePWM_L = 0.0f;
    s_prevBalancePWM_R = 0.0f;
    g_motorSpeedfiltered_left = 0.0f;
    g_motorSpeedfiltered_right = 0.0f;

    KalmanSpeed_SetSpeed(&kalmanSpeed_L, 0.0f);
    KalmanSpeed_SetSpeed(&kalmanSpeed_R, 0.0f);

    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    return 1;
}

void Robot_Stop(void) {
    g_systemEnable = 0;

    Motor_SetPWM(0.0f);
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);

    g_posIntegral = 0.0f;
    g_yawIntegral = 0.0f;
    s_gyroRateFiltered = 0.0f;
    s_yawRateFiltered = 0.0f;
    s_prevBalancePWM_L = 0.0f;
    s_prevBalancePWM_R = 0.0f;

    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

void Robot_SetControlMode(RobotControlMode_t mode) {
    g_controlMode = mode;
    g_posIntegral = 0.0f;
    g_yawIntegral = 0.0f;
    g_robotX = 0.0f;
    g_robotY = 0.0f;
    g_robotDist = 0.0f;
    g_encoderYawAngle = 0.0f;
    g_encoderYawRate  = 0.0f;
    g_rawYawAngle = 0.0f;
    g_filteredYaw = 0.0f;
    Kalman_SetAngle(&kalmanYaw, 0.0f);
    s_prevBalancePWM_L = 0.0f;
    s_prevBalancePWM_R = 0.0f;
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);
}

static void Calib_DelayMs(uint32_t ms) {
    Delay_ms(ms);
}

uint8_t Robot_AutoCalibrate(void) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    uint8_t still_batches = 0;
    uint32_t timeout_counter = 0;

    if (g_systemEnable) {
        Robot_Stop();
    }

    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    /* BƯỚC 1: TỰ NHẬN DIỆN TRẠNG THÁI ĐỨNG YÊN & CALIB GYRO */
    GPIO_SetBits(GPIOD, GPIO_Pin_13);

    while (still_batches < 3) {
        int16_t minGx = 32767, maxGx = -32768;
        int16_t minGy = 32767, maxGy = -32768;
        int16_t minGz = 32767, maxGz = -32768;

        for (int i = 0; i < 25; i++) {
            if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
                if (gx < minGx) minGx = gx;
                if (gx > maxGx) maxGx = gx;
                if (gy < minGy) minGy = gy;
                if (gy > maxGy) maxGy = gy;
                if (gz < minGz) minGz = gz;
                if (gz > maxGz) maxGz = gz;
            }
            Calib_DelayMs(10);
        }

        if ((maxGx - minGx) < 200 && (maxGy - minGy) < 200 && (maxGz - minGz) < 200) {
            still_batches++;
        } else {
            still_batches = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
        }

        if (++timeout_counter > 60) {
            GPIO_ResetBits(GPIOD, GPIO_Pin_13);
            for (int k = 0; k < 3; k++) {
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
            }
            GPIO_SetBits(GPIOD, GPIO_Pin_14);
            return 0;
        }
    }

    int32_t sumGy = 0, sumGz = 0;
    for (int i = 0; i < 100; i++) {
        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
            sumGy += gy;
            sumGz += gz;
        }
        Calib_DelayMs(5);
    }
    g_gyroOffset  = (float)sumGy / 100.0f;
    g_gyroZOffset = (float)sumGz / 100.0f;

    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    for (int k = 0; k < 2; k++) {
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(150);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(150);
    }

    /* BƯỚC 2: TỰ NHẬN DIỆN TƯ THẾ DỰNG ĐỨNG CÂN BẰNG & CHỐT ZERO POINT */
    uint8_t balance_batches = 0;
    timeout_counter = 0;
    float sumBatchPitch = 0.0f;

    while (balance_batches < 4) {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

        float minP = 999.0f, maxP = -999.0f;
        float currentSum = 0.0f;
        uint8_t validSamples = 0;

        for (int i = 0; i < 20; i++) {
            if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
                float axg = (float)ax / 16384.0f;
                float ayg = (float)ay / 16384.0f;
                float azg = (float)az / 16384.0f;
                float p = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * 57.2957795f;

                currentSum += p;
                validSamples++;
                if (p < minP) minP = p;
                if (p > maxP) maxP = p;
            }
            Calib_DelayMs(10);
        }

        if (validSamples > 0) {
            float avgP = currentSum / (float)validSamples;
            float diffP = maxP - minP;

            if ((avgP > -12.0f && avgP < 12.0f) && (diffP < 0.6f)) {
                balance_batches++;
                sumBatchPitch += avgP;
            } else {
                balance_batches = 0;
                sumBatchPitch = 0.0f;
            }
        }

        if (++timeout_counter > 100) {
            GPIO_ResetBits(GPIOD, GPIO_Pin_15);
            for (int k = 0; k < 3; k++) {
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);
                Calib_DelayMs(150);
            }
            GPIO_SetBits(GPIOD, GPIO_Pin_14);
            return 0;
        }
    }

    g_targetAngle = sumBatchPitch / 4.0f;
    KalmanAngle_Init(&kalmanPitch);
    kalmanPitch.angle = g_targetAngle;
    g_filteredPitch = g_targetAngle;

    KalmanAngle_Init(&kalmanYaw);
    g_encoderYawAngle = 0.0f;
    g_encoderYawRate  = 0.0f;
    g_rawYawAngle     = 0.0f;
    g_filteredYaw     = 0.0f;
    g_yawIntegral     = 0.0f;

    /* BƯỚC 3: LƯU VÀO FLASH SECTOR 11 */
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
    return 1;
}

/* =====================================================================
 * CÁC BỘ ĐIỀU KHIỂN ROBOT CÂN BẰNG (DUAL PID & FUZZY LOGIC)
 * ===================================================================== */

/**
 * @brief BỘ ĐIỀU KHIỂN 1: PID 2 VÒNG + GIỮ TỌA ĐỘ ODOMETRY 2D (X, Y) & YAW IMU
 */
static void Controller_DualPID(float dt, float *out_pwmL, float *out_pwmR)
{
    /* VÒNG 1: CÂN BẰNG GÓC (Angle Balance PD Loop - 200Hz) */
    float angleError = g_filteredPitch - g_targetAngle;
    s_gyroRateFiltered = 0.7f * s_gyroRateFiltered + 0.3f * kalmanPitch.rate;
    float balancePWM = (g_Kp_balance * angleError) + (g_Kd_balance * s_gyroRateFiltered);

    /* VÒNG 2: GIỮ TỌA ĐỘ ODOMETRY (X, Y) & RÀNG BUỘC VẬN TỐC (< 10 cm/s) */
    float wheelSpeed_L = -g_motorSpeedfiltered_left;
    float wheelSpeed_R = -g_motorSpeedfiltered_right;
    float v_avg = (wheelSpeed_L + wheelSpeed_R) * 0.5f;   /* Vận tốc trung bình (RPM) */
    float v_cms = v_avg * WHEEL_RPM_TO_CMS;               /* Vận tốc dài của xe (cm/s) */

    /* Ràng buộc vận tốc mục tiêu g_targetSpeed luôn nằm trong [-10 cm/s, +10 cm/s] */
    float targetSpeed = g_targetSpeed;
    if (targetSpeed > ROBOT_MAX_SPEED_RPM) targetSpeed = ROBOT_MAX_SPEED_RPM;
    else if (targetSpeed < -ROBOT_MAX_SPEED_RPM) targetSpeed = -ROBOT_MAX_SPEED_RPM;

    float speedError = v_avg - targetSpeed;

    /* Chiếu sai lệch tọa độ (dx, dy) lên hướng đầu xe hiện tại để lấy độ lệch vị trí (cm) */
    float yawRad   = g_filteredYaw * (3.14159265359f / 180.0f);
    float dx       = g_robotX - g_targetX;
    float dy       = g_robotY - g_targetY;
    float posError = dx * cosf(yawRad) + dy * sinf(yawRad);

    /* Giới hạn vùng kéo tọa độ tối đa +-60 cm để xe bước về mượt, không chúi đầu quá gắt */
    if (posError > 60.0f) posError = 60.0f;
    else if (posError < -60.0f) posError = -60.0f;

    /* Giới hạn lực kéo vị trí tối đa ứng với vận tốc trần 10 cm/s (ROBOT_MAX_SPEED_RPM ~ 29.38 RPM) */
    float effKpPos   = (g_Kp_position > 0.5f) ? g_Kp_position : 0.5f;
    float maxPosPull = effKpPos * ROBOT_MAX_SPEED_RPM;

    /* Khâu I theo tọa độ:
     * - Khi posError đổi dấu (xe đi qua đích hoặc xoay đầu quá 90 độ), xả nhanh g_posIntegral để không đẩy ngược chiều
     * - Chỉ tích phân khi vận tốc xe < 10 cm/s và xe không đang xoay góc lớn (|yawError| < 15 độ) */
    float yawError = g_filteredYaw - g_targetYaw;
    if (posError * g_posIntegral < 0.0f) {
        g_posIntegral *= 0.80f;
    }
    if (fabsf(targetSpeed) < 0.5f) {
        if (fabsf(v_cms) < ROBOT_MAX_SPEED_CMS && fabsf(yawError) < 15.0f) {
            g_posIntegral += posError * dt;
            if (g_posIntegral > POS_INTEGRAL_MAX) g_posIntegral = POS_INTEGRAL_MAX;
            else if (g_posIntegral < -POS_INTEGRAL_MAX) g_posIntegral = -POS_INTEGRAL_MAX;
        }
    } else {
        g_targetX = g_robotX;
        g_targetY = g_robotY;
        g_posIntegral = 0.0f;
    }

    /* Tính lực kéo về điểm đích và kẹp trần để vận tốc di chuyển về đích không vượt quá 10 cm/s */
    float posPullPWM = (g_Ki_position * 3.0f * posError)
                     + (g_Ki_position * 0.8f * g_posIntegral);
    if (posPullPWM > maxPosPull) posPullPWM = maxPosPull;
    else if (posPullPWM < -maxPosPull) posPullPWM = -maxPosPull;

    /* Khi xe đang thực hiện lệnh xoay góc lớn (|yawError| > 15 độ), giảm lực kéo tịnh tiến để ưu tiên xoay êm tại chỗ */
    if (fabsf(yawError) > 15.0f) {
        posPullPWM *= 0.35f;
    }

    /* Phanh chủ động nếu vận tốc thực tế của xe vượt ngưỡng 10 cm/s (|v_cms| > 10 cm/s) */
    float overspeedBrakePWM = 0.0f;
    if (v_avg > ROBOT_MAX_SPEED_RPM) {
        overspeedBrakePWM = effKpPos * (v_avg - ROBOT_MAX_SPEED_RPM);
        if (posPullPWM < 0.0f) posPullPWM = 0.0f;
    } else if (v_avg < -ROBOT_MAX_SPEED_RPM) {
        overspeedBrakePWM = effKpPos * (v_avg + ROBOT_MAX_SPEED_RPM);
        if (posPullPWM > 0.0f) posPullPWM = 0.0f;
    }

    /* Kết hợp: Phanh vận tốc (D_pos) + Kéo về gốc tọa độ (< 10 cm/s) + Phanh chống vượt tốc */
    float positionPWM = (g_Kp_position * speedError) + posPullPWM + overspeedBrakePWM;
    if (positionPWM > 250.0f) positionPWM = 250.0f;
    else if (positionPWM < -250.0f) positionPWM = -250.0f;

    /* VÒNG 3: KHÓA CỨNG HƯỚNG XOAY YAW & RÀNG BUỘC TỐC ĐỘ XOAY ÊM */
    s_yawRateFiltered = 0.70f * s_yawRateFiltered + 0.30f * kalmanYaw.rate;
    float diffSpeed = wheelSpeed_L - wheelSpeed_R;

    /* Giới hạn sai số góc đưa vào khâu P ở +-18 độ để khi bấm xoay 45/90/180 độ xe xoay từ từ, không giật lệch bánh */
    float yawErrorClamped = yawError;
    if (yawErrorClamped > 18.0f) yawErrorClamped = 18.0f;
    else if (yawErrorClamped < -18.0f) yawErrorClamped = -18.0f;

    /* Tích phân khóa cứng góc Yaw khi xe đã gần hướng đích (|yawError| < 15 độ) */
    if (fabsf(yawError) < 15.0f) {
        if (yawError * g_yawIntegral < 0.0f) {
            g_yawIntegral *= 0.85f;
        }
        g_yawIntegral += yawError * dt;
        if (g_yawIntegral > YAW_INTEGRAL_MAX) g_yawIntegral = YAW_INTEGRAL_MAX;
        else if (g_yawIntegral < -YAW_INTEGRAL_MAX) g_yawIntegral = -YAW_INTEGRAL_MAX;
    } else {
        g_yawIntegral = 0.0f;
    }

    float yawPWM = (g_Kp_yaw * yawErrorClamped)
                 + (1.50f * g_yawIntegral)
                 + (g_Kd_yaw * diffSpeed)
                 + (g_Kd_yaw_imu * s_yawRateFiltered);

    if (yawPWM > 110.0f) yawPWM = 110.0f;
    else if (yawPWM < -110.0f) yawPWM = -110.0f;

    *out_pwmL = balancePWM + positionPWM - yawPWM;
    *out_pwmR = balancePWM + positionPWM + yawPWM;
}

/**
 * @brief BỘ ĐIỀU KHIỂN 2: FUZZY LOGIC (Điều khiển mờ cân bằng kết hợp giữ tọa độ X, Y & khóa cứng Yaw)
 */
static void Controller_Fuzzy(float dt, float *out_pwmL, float *out_pwmR)
{
    float errorAngle = g_filteredPitch - g_targetAngle;
    s_gyroRateFiltered = 0.70f * s_gyroRateFiltered + 0.30f * kalmanPitch.rate;
    float dErrorAngle = s_gyroRateFiltered;

    float wheelSpeed_L = -g_motorSpeedfiltered_left;
    float wheelSpeed_R = -g_motorSpeedfiltered_right;
    float v_avg = (wheelSpeed_L + wheelSpeed_R) * 0.5f;   /* Vận tốc trung bình (RPM) */
    float v_cms = v_avg * WHEEL_RPM_TO_CMS;               /* Vận tốc dài của xe (cm/s) */

    /* Ràng buộc vận tốc mục tiêu g_targetSpeed luôn nằm trong [-10 cm/s, +10 cm/s] */
    float targetSpeed = g_targetSpeed;
    if (targetSpeed > ROBOT_MAX_SPEED_RPM) targetSpeed = ROBOT_MAX_SPEED_RPM;
    else if (targetSpeed < -ROBOT_MAX_SPEED_RPM) targetSpeed = -ROBOT_MAX_SPEED_RPM;

    float speedError = v_avg - targetSpeed;

    float yawRad   = g_filteredYaw * (3.14159265359f / 180.0f);
    float dx       = g_robotX - g_targetX;
    float dy       = g_robotY - g_targetY;
    float posError = dx * cosf(yawRad) + dy * sinf(yawRad);

    if (posError > 60.0f) posError = 60.0f;
    else if (posError < -60.0f) posError = -60.0f;

    float effKpPos   = (g_Kp_position > 0.5f) ? g_Kp_position : 0.5f;
    float maxPosPull = effKpPos * ROBOT_MAX_SPEED_RPM;

    float yawError = g_filteredYaw - g_targetYaw;
    if (posError * g_posIntegral < 0.0f) {
        g_posIntegral *= 0.80f;
    }
    if (fabsf(targetSpeed) < 0.5f) {
        if (fabsf(v_cms) < ROBOT_MAX_SPEED_CMS && fabsf(yawError) < 15.0f) {
            g_posIntegral += posError * dt;
            if (g_posIntegral > POS_INTEGRAL_MAX) g_posIntegral = POS_INTEGRAL_MAX;
            else if (g_posIntegral < -POS_INTEGRAL_MAX) g_posIntegral = -POS_INTEGRAL_MAX;
        }
    } else {
        g_targetX = g_robotX;
        g_targetY = g_robotY;
        g_posIntegral = 0.0f;
    }

    /* Đồng bộ hệ số chuẩn hóa ke, kde theo g_Kp_balance & g_Kd_balance:
     * - Quanh điểm cân bằng 0 độ, độ lợi Fuzzy tương đương đúng (Kp_bal, Kd_bal) nên đứng êm không rung
     * - Khi xe nghiêng lớn hoặc đang đổ nhanh, luật mờ PS+PS -> PB tự động tăng lực đỡ phi tuyến */
    fuzzyControl.ke  = g_Kp_balance * 0.015f;
    fuzzyControl.kde = g_Kd_balance * 0.25f;
    float fuzzyPWM   = Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle) * 10.0f;

    float posPullPWM = (g_Ki_position * 3.0f * posError)
                     + (g_Ki_position * 0.8f * g_posIntegral);
    if (posPullPWM > maxPosPull) posPullPWM = maxPosPull;
    else if (posPullPWM < -maxPosPull) posPullPWM = -maxPosPull;

    if (fabsf(yawError) > 15.0f) {
        posPullPWM *= 0.35f;
    }

    float overspeedBrakePWM = 0.0f;
    if (v_avg > ROBOT_MAX_SPEED_RPM) {
        overspeedBrakePWM = effKpPos * (v_avg - ROBOT_MAX_SPEED_RPM);
        if (posPullPWM < 0.0f) posPullPWM = 0.0f;
    } else if (v_avg < -ROBOT_MAX_SPEED_RPM) {
        overspeedBrakePWM = effKpPos * (v_avg + ROBOT_MAX_SPEED_RPM);
        if (posPullPWM > 0.0f) posPullPWM = 0.0f;
    }

    float positionPWM = (g_Kp_position * speedError) + posPullPWM + overspeedBrakePWM;
    if (positionPWM > 250.0f) positionPWM = 250.0f;
    else if (positionPWM < -250.0f) positionPWM = -250.0f;

    /* VÒNG 3: KHÓA CỨNG HƯỚNG XOAY YAW & RÀNG BUỘC TỐC ĐỘ XOAY ÊM */
    s_yawRateFiltered = 0.70f * s_yawRateFiltered + 0.30f * kalmanYaw.rate;
    float diffSpeed = wheelSpeed_L - wheelSpeed_R;

    float yawErrorClamped = yawError;
    if (yawErrorClamped > 18.0f) yawErrorClamped = 18.0f;
    else if (yawErrorClamped < -18.0f) yawErrorClamped = -18.0f;

    if (fabsf(yawError) < 15.0f) {
        if (yawError * g_yawIntegral < 0.0f) {
            g_yawIntegral *= 0.85f;
        }
        g_yawIntegral += yawError * dt;
        if (g_yawIntegral > YAW_INTEGRAL_MAX) g_yawIntegral = YAW_INTEGRAL_MAX;
        else if (g_yawIntegral < -YAW_INTEGRAL_MAX) g_yawIntegral = -YAW_INTEGRAL_MAX;
    } else {
        g_yawIntegral = 0.0f;
    }

    float yawPWM = (g_Kp_yaw * yawErrorClamped)
                 + (1.50f * g_yawIntegral)
                 + (g_Kd_yaw * diffSpeed)
                 + (g_Kd_yaw_imu * s_yawRateFiltered);

    if (yawPWM > 110.0f) yawPWM = 110.0f;
    else if (yawPWM < -110.0f) yawPWM = -110.0f;

    *out_pwmL = fuzzyPWM + positionPWM - yawPWM;
    *out_pwmR = fuzzyPWM + positionPWM + yawPWM;
}

/**
 * @brief HÀM ĐIỀU PHỐI ĐIỀU KHIỂN ROBOT CHUNG (200Hz - 5ms)
 */
static void Robot_Controller_Update(float dt)
{
    if (!g_systemEnable) {
        Motor_SetPWM_Left(0.0f);
        Motor_SetPWM_Right(0.0f);
        g_motorPWM_left    = 0.0f;
        g_motorPWM_right   = 0.0f;
        g_motorPWM         = 0.0f;
        s_prevBalancePWM_L = 0.0f;
        s_prevBalancePWM_R = 0.0f;
        s_gyroRateFiltered = 0.0f;
        s_yawRateFiltered  = 0.0f;
        g_posIntegral      = 0.0f;
        g_yawIntegral      = 0.0f;
        return;
    }

    /* Bảo vệ chống ngã (Fall Protection: > 40 độ) */
    float diff = g_filteredPitch - g_targetAngle;
    if (diff > 40.0f || diff < -40.0f) {
        Robot_Stop();
        return;
    }

    float pwmL = 0.0f;
    float pwmR = 0.0f;

    if (g_controlMode == CTRL_MODE_FUZZY) {
        Controller_Fuzzy(dt, &pwmL, &pwmR);
    } else {
        Controller_DualPID(dt, &pwmL, &pwmR);
    }

    /* Bù vùng chết động cơ tuyến tính mượt riêng cho 2 bánh */
    if (pwmL > 20.0f) pwmL += MOTOR_DEADBAND_PWM_L;
    else if (pwmL < -20.0f) pwmL -= MOTOR_DEADBAND_PWM_L;
    else pwmL += (pwmL / 20.0f) * MOTOR_DEADBAND_PWM_L;

    if (pwmR > 20.0f) pwmR += MOTOR_DEADBAND_PWM_R;
    else if (pwmR < -20.0f) pwmR -= MOTOR_DEADBAND_PWM_R;
    else pwmR += (pwmR / 20.0f) * MOTOR_DEADBAND_PWM_R;

    /* Giới hạn gia tốc PWM (Slew-Rate Limiter) */
    float delta_L = pwmL - s_prevBalancePWM_L;
    if (delta_L > BALANCE_MAX_PWM_DELTA) pwmL = s_prevBalancePWM_L + BALANCE_MAX_PWM_DELTA;
    else if (delta_L < -BALANCE_MAX_PWM_DELTA) pwmL = s_prevBalancePWM_L - BALANCE_MAX_PWM_DELTA;
    s_prevBalancePWM_L = pwmL;

    float delta_R = pwmR - s_prevBalancePWM_R;
    if (delta_R > BALANCE_MAX_PWM_DELTA) pwmR = s_prevBalancePWM_R + BALANCE_MAX_PWM_DELTA;
    else if (delta_R < -BALANCE_MAX_PWM_DELTA) pwmR = s_prevBalancePWM_R - BALANCE_MAX_PWM_DELTA;
    s_prevBalancePWM_R = pwmR;

    /* Bão hòa đầu ra PWM (-1000.0f đến 1000.0f) */
    if (pwmL > 1000.0f) pwmL = 1000.0f;
    else if (pwmL < -1000.0f) pwmL = -1000.0f;

    if (pwmR > 1000.0f) pwmR = 1000.0f;
    else if (pwmR < -1000.0f) pwmR = -1000.0f;

    Motor_SetPWM_Left(pwmL);
    Motor_SetPWM_Right(pwmR);

    g_motorPWM_left  = pwmL;
    g_motorPWM_right = pwmR;
    g_motorPWM       = (pwmL + pwmR) * 0.5f;
}

/* =====================================================================
 * CÁC TASK FREERTOS ĐƯỢC PHÂN TÁCH RÕ RÀNG THEO CHỨC NĂNG
 * ===================================================================== */

/**
 * @brief TASK 1: vTaskIMU (Chu kỳ 5ms - 200Hz, Priority 3)
 *  - Bước 1: Đọc dữ liệu thô MPU6050 -> Tính góc Raw Pitch (g_rawAccPitch, g_rawGyroRate)
 *  - Bước 2: Khử trôi tĩnh Gyro Z khi 2 bánh không lệch tốc độ (ZUPT) & lấy g_rawYawRate,
 *            đồng thời lấy góc chuẩn không trôi từ vi sai xung Encoder (g_encoderYawAngle)
 *  - Bước 3: Chạy bộ lọc Kalman riêng cho Pitch (kalmanPitch: AccPitch + GyroY -> g_filteredPitch)
 *            và bộ lọc Kalman riêng cho Yaw (kalmanYaw: EncoderYaw + GyroZ -> g_filteredYaw)
 */
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

        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u)
        {
            float azSign = (az >= 0) ? 1.0f : -1.0f;

            /* -------------------------------------------------------------
             * [BƯỚC 1] LẤY GÓC RAW PITCH & VẬN TỐC GÓC PITCH (TRỤC Y)
             * ------------------------------------------------------------- */
            float axg = (float)ax / 16384.0f;
            float ayg = (float)ay / 16384.0f;
            float azg = (float)az / 16384.0f;
            g_rawAccPitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);
            g_rawGyroRate = azSign * (((float)gy - g_gyroOffset) / 65.5f);

            /* -------------------------------------------------------------
             * [BƯỚC 2] KHỬ TRÔI GYRO Z BẰNG ENCODER (ZUPT) & LẤY GÓC YAW VI SAI
             *  - Khi 2 bánh xe không xoay lệch nhau (|g_encoderYawRate| < 1.5 deg/s),
             *    mọi giá trị khác 0 của gz đều là trôi nhiệt / giao thoa trục Pitch -> tự bù g_gyroZOffset
             *  - Góc đo tham chiếu (g_rawYawAngle) lấy từ góc vi sai xung 2 Encoder (g_encoderYawAngle)
             *    nên tuyệt đối không bao giờ bị trôi theo thời gian!
             * ------------------------------------------------------------- */
            if (fabsf(g_encoderYawRate) < 1.5f && fabsf((float)gz - g_gyroZOffset) < 150.0f) {
                g_gyroZOffset = 0.998f * g_gyroZOffset + 0.002f * (float)gz;
            }
            g_rawYawRate  = YAW_GYRO_SIGN * azSign * (((float)gz - g_gyroZOffset) / 65.5f);
            g_rawYawAngle = g_encoderYawAngle;

            /* -------------------------------------------------------------
             * [BƯỚC 3] LỌC KALMAN CHO PITCH (kalmanPitch) & YAW (kalmanYaw)
             *  - kalmanPitch: kết hợp g_rawAccPitch (Gia tốc kế) + g_rawGyroRate (Gyro Y)
             *  - kalmanYaw  : kết hợp g_encoderYawAngle (Vi sai xung Encoder) + g_rawYawRate (Gyro Z)
             * ------------------------------------------------------------- */
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

/**
 * @brief TASK 2: vTaskEncoder (Chu kỳ 10ms - 100Hz, Priority 3)
 *  - Bước 1: Lấy số xung & vận tốc Raw của 2 bánh (g_motorSpeed_left, g_motorSpeed_right)
 *  - Bước 2: Lọc vận tốc 2 bánh qua Kalman (kalmanSpeed_L, kalmanSpeed_R)
 *  - Bước 3: Tính góc Yaw vi sai chuẩn từ xung 2 Encoder (g_encoderYawAngle) để khóa cứng Yaw,
 *            đồng thời Fusion số xung Encoder + góc kalmanYaw.angle ra tọa độ 2D (g_robotX, g_robotY)
 */
static void vTaskEncoder(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    MOTOR leftMotor = {0};
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

        /* -------------------------------------------------------------
         * [BƯỚC 1] ĐỌC XUNG ENCODER & LẤY VẬN TỐC RAW WHEEL LEFT / RIGHT
         * ------------------------------------------------------------- */
        Encoder_ReadStruct(&leftMotor, &rightMotor);
        g_motorSpeed_left  = leftMotor.speed;
        g_motorSpeed_right = rightMotor.speed;

        /* -------------------------------------------------------------
         * [BƯỚC 2] LỌC VẬN TỐC KALMAN WHEEL LEFT / RIGHT (kalmanSpeed_L/R)
         * ------------------------------------------------------------- */
        g_motorSpeedfiltered_left  = KalmanSpeed_GetSpeed(&kalmanSpeed_L, g_motorSpeed_left,  0.010f);
        g_motorSpeedfiltered_right = KalmanSpeed_GetSpeed(&kalmanSpeed_R, g_motorSpeed_right, 0.010f);
        g_motorSpeed               = (g_motorSpeedfiltered_left + g_motorSpeedfiltered_right) * 0.5f;

        /* -------------------------------------------------------------
         * [BƯỚC 3] KHÓA CỨNG GÓC YAW TỪ VI SAI XUNG ENCODER & FUSION TỌA ĐỘ 2D (X, Y)
         *  - Bánh xe D = 65mm (C = 20.420352 cm), Khoảng cách 2 bánh W = 17.0 cm
         *    + Mỗi xung dịch chuyển: WHEEL_CM_PER_PULSE = 0.015470 cm/xung
         *    + Mỗi xung lệch giữa bánh Trái và Phải xoay góc: YAW_DEG_PER_DIFF_PULSE = 0.05214 deg/xung
         *  - Tích lũy chính xác góc Yaw từ chênh lệch xung 2 bánh (g_encoderYawAngle):
         *    Khi 2 bánh lăn cùng số xung -> dPulse_L == dPulse_R -> góc Yaw đứng im tuyệt đối 0.00 deg!
         * ------------------------------------------------------------- */
        float vDiff_cms = (-g_motorSpeedfiltered_left - (-g_motorSpeedfiltered_right)) * WHEEL_RPM_TO_CMS;
        g_encoderYawRate = vDiff_cms * YAW_DEG_PER_DIFF_CM;

        /* Luôn cập nhật góc Yaw vi sai & tọa độ 2D (X, Y) ngay cả ở chế độ Standby (g_systemEnable == 0)
         * để có thể đẩy xe bằng tay kiểm tra trực tiếp góc Yaw và Bản đồ 2D trên GUI */
        float dPulse_L       = -(float)leftMotor.delta_pulse;
        float dPulse_R       = -(float)rightMotor.delta_pulse;
        float deltaPulseAvg  = 0.5f * (dPulse_L + dPulse_R);
        float deltaPulseDiff = dPulse_L - dPulse_R;

        if (fabsf(g_encoderYawAngle) < 0.0001f && fabsf(g_robotDist) < 0.0001f) {
            prevYawRad = 0.0f;
        }

        /* Cập nhật góc Yaw chuẩn từ phần cứng Encoder (neo chống trôi cho kalmanYaw) */
        g_encoderYawAngle += deltaPulseDiff * YAW_DEG_PER_DIFF_PULSE;

        /* Tính quãng đường ds (cm) và chiếu lên mặt phẳng 2D theo góc kalmanYaw */
        float ds        = deltaPulseAvg * WHEEL_CM_PER_PULSE;
        float yawRad    = kalmanYaw.angle * (3.14159265359f / 180.0f);
        float yawMidRad = 0.5f * (prevYawRad + yawRad);
        prevYawRad      = yawRad;

        g_robotDist += ds;
        g_robotX    += ds * cosf(yawMidRad);
        g_robotY    += ds * sinf(yawMidRad);

        if (++beat >= 15) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
        }
    }
}

/**
 * @brief TASK 3: vTaskControl (Chu kỳ 5ms - 200Hz, Priority 4)
 *  - Thực thi bộ điều khiển Dual PID hoặc Fuzzy dựa trên dữ liệu từ vTaskIMU & vTaskEncoder
 */
static void vTaskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    uint8_t beat = 0;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Robot_Controller_Update(0.005f);

        if (++beat == 40) {
            beat = 0;
            if (!g_systemEnable) {
                GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
            } else {
                GPIOD->ODR &= ~(1 << 14);
            }
        }
    }
}

static void vTaskUART(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); /* Chu kỳ quét lệnh RX 20ms (50Hz) giúp nhận lệnh START/STOP tức thì */

    uint8_t  rxState = 0;
    uint8_t  rxId = 0;
    uint8_t  rxLen = 0;
    uint8_t  rxCount = 0;
    uint8_t  rxChecksum = 0;
    uint8_t  beat = 0;
    uint8_t  txDivider = 0;
    uint16_t bootCycle = 0;
    uint8_t  sendConfigReq = 1; /* Gửi bộ số mặc định của STM32 lên PC khi vừa khởi động / Reset */
    uint8_t  rxDataBuf[32];

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(200)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (++beat >= 25) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
        }

        /* 1. Luôn ưu tiên đọc & giải mã gói lệnh từ PC trước ở tốc độ 50Hz (20ms) để không bao giờ trễ lệnh */
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
                rxLen = byte;
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
                if (byte == UART_FRAME_TAIL) {
                    if (UART_CalculateChecksum(rxId, rxLen, rxDataBuf) == rxChecksum) {
                        switch (rxId)
                        {
                        case 0x01: /* START */
                            Robot_Start();
                            break;
                        case 0x02: /* STOP */
                            Robot_Stop();
                            break;
                        case 0x03: /* CALIB */
                            Robot_AutoCalibrate();
                            sendConfigReq = 1; /* Gửi lại góc cân bằng mới sau khi Calib lên PC */
                            break;
                        case 0x04: /* Set Target Angle (4 byte float) */
                            if (rxLen >= 4) {
                                float newTarget = 0.0f;
                                memcpy(&newTarget, rxDataBuf, 4);
                                g_targetAngle = newTarget;
                            }
                            break;
                        case 0x05: /* Set Target Speed (8 byte L/R hoặc 4 byte chung, ràng buộc < 10 cm/s) */
                            if (rxLen >= 8) {
                                float newSpeedL = 0.0f, newSpeedR = 0.0f;
                                memcpy(&newSpeedL, &rxDataBuf[0], 4);
                                memcpy(&newSpeedR, &rxDataBuf[4], 4);
                                g_targetSpeed = (newSpeedL + newSpeedR) * 0.5f;
                            } else if (rxLen >= 4) {
                                float newSpeed = 0.0f;
                                memcpy(&newSpeed, rxDataBuf, 4);
                                g_targetSpeed = newSpeed;
                            }
                            if (g_targetSpeed > ROBOT_MAX_SPEED_RPM) g_targetSpeed = ROBOT_MAX_SPEED_RPM;
                            else if (g_targetSpeed < -ROBOT_MAX_SPEED_RPM) g_targetSpeed = -ROBOT_MAX_SPEED_RPM;
                            break;
                        case 0x06: /* Set Target Pos (X, Y, Yaw - 12B) hoặc Reset Odometry Origin (0B) */
                            if (rxLen == 0) {
                                g_robotX          = 0.0f;
                                g_robotY          = 0.0f;
                                g_robotDist       = 0.0f;
                                g_targetX         = 0.0f;
                                g_targetY         = 0.0f;
                                g_encoderYawAngle = 0.0f;
                                g_encoderYawRate  = 0.0f;
                                g_rawYawAngle     = 0.0f;
                                g_filteredYaw     = 0.0f;
                                g_targetYaw       = 0.0f;
                                g_posIntegral     = 0.0f;
                                g_yawIntegral     = 0.0f;
                                Kalman_SetAngle(&kalmanYaw, 0.0f);
                            } else if (rxLen >= 12) {
                                memcpy((void *)&g_targetX,   &rxDataBuf[0], 4);
                                memcpy((void *)&g_targetY,   &rxDataBuf[4], 4);
                                memcpy((void *)&g_targetYaw, &rxDataBuf[8], 4);
                            } else if (rxLen >= 4) {
                                memcpy((void *)&g_targetYaw, &rxDataBuf[0], 4);
                            }
                            break;
                        case 0x07: /* Set Control Mode (0 = Dual PID, 1 = Fuzzy) */
                            if (rxLen >= 1 && rxDataBuf[0] <= CTRL_MODE_FUZZY) {
                                Robot_SetControlMode((RobotControlMode_t)rxDataBuf[0]);
                            }
                            break;
                        case 0x08: /* Set PID 3 Vòng: Cân bằng (PD) + Vị trí (PI) + Hướng Yaw (Encoder + IMU) */
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
                                g_posIntegral = 0.0f;
                            }
                            break;
                        case 0x09: /* PC yêu cầu đọc bộ số cấu hình hiện tại từ STM32 */
                            sendConfigReq = 1;
                            break;
                        default:
                            break;
                        }
                    }
                }
                rxState = 0;
                break;

            default:
                rxState = 0;
                break;
            }
        }

        /* 2. Sau khi Reset STM32 (t = 300ms và 1.5s), phát gói đồng bộ cấu hình gốc (ID = 0x02) */
        if (bootCycle <= 80) {
            bootCycle++;
            if (bootCycle == 15 || bootCycle == 75) {
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

            if (UART3_SendPacketDMA(0x02, &cfgSync, (uint8_t)sizeof(RobotConfigSync_t))) {
                sendConfigReq = 0;
                txDivider = 0;
            }
        } else {
            /* 3. Giảm tần số gửi Telemetry xuống 5Hz (200ms) khi đang chạy và 3.3Hz (300ms) khi Standby
             *    để bộ đệm Bluetooth (COM18) luôn thông thoáng, không bao giờ bị tràn hoặc lag */
            uint8_t txPeriodTicks = g_systemEnable ? 10u : 15u; /* 10 * 20ms = 200ms (5Hz) | 15 * 20ms = 300ms (3.3Hz) */
            if (++txDivider >= txPeriodTicks) {
                txDivider = 0;

                RobotTelemetry_t telem;
                /* Dữ liệu từ vTaskIMU */
                telem.rawPitch                 = g_rawAccPitch;
                telem.filteredPitch            = g_filteredPitch;
                telem.gyroRate                 = kalmanPitch.rate;
                telem.rawYaw                   = g_rawYawAngle;
                telem.filteredYaw              = g_filteredYaw;
                telem.yawRate                  = kalmanYaw.rate;
                /* Dữ liệu từ vTaskEncoder */
                telem.motorSpeed_left          = g_motorSpeed_left;
                telem.motorSpeed_right         = g_motorSpeed_right;
                telem.motorSpeedFiltered_left  = g_motorSpeedfiltered_left;
                telem.motorSpeedFiltered_right = g_motorSpeedfiltered_right;
                telem.robotX                   = g_robotX;
                telem.robotY                   = g_robotY;
                /* Dữ liệu từ vTaskControl */
                telem.motorPWM_left            = g_motorPWM_left;
                telem.motorPWM_right           = g_motorPWM_right;
                telem.targetAngle              = g_targetAngle;

                UART3_SendPacketDMA(0x01, &telem, (uint8_t)sizeof(RobotTelemetry_t));
            }
        }
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Motor_SetPWM(0.0f);
    while (1)
    {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
        for (volatile int i = 0; i < 500000; i++);
    }
}

