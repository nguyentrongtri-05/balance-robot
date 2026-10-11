#include "managetask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "driver_i2c.h"
#include "driver_uart.h"
#include "fuzzy.h"
#include "pid.h"
#include "kalman.h"
#include "driver_flash.h"
#include <math.h>
#include <string.h>

/* Biến Controller */
extern FuzzyController_t fuzzyControl;
PID_t speedPID, speedPID_L, speedPID_R;
Kalman_t kalmanFilter;

/* Trạng thái hoạt động của hệ thống (0: STOP/Standby, 1: RUN) */
volatile uint8_t g_systemEnable = 0;

/* Các biến toàn cục chứa dữ liệu thực để tiện Monitor trên Keil C & UART */
volatile float g_rawAccPitch = 0.0f;
volatile float g_rawGyroRate = 0.0f;
volatile float g_filteredPitch = 0.0f; /* Góc thực sau đọc cảm biến và lọc Kalman */
volatile float g_motorSpeed = 0.0f;
volatile float g_motorSpeed_left = 0.0f;
volatile float g_motorSpeed_right = 0.0f;
volatile float g_motorSpeedTarget_left = 0.0f;
volatile float g_motorSpeedTarget_right = 0.0f;
volatile float g_motorPWM = 0.0f;

/* Các biến liên kết giữa 2 vòng lặp (Cascaded Loop) */
volatile float g_targetSpeed = 0.0f;
volatile float g_targetAngle = 0.0f; /* Điểm cân bằng mục tiêu */
volatile float g_gyroOffset = 0.0f;  /* Sai số tĩnh con quay hồi chuyển sau khi Calib */

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);
static void vTaskUART(void *pvParameters);

void System_Tasks_Init(void) {
    /* 1. Khởi tạo bus I2C, encoder, UART và các thuật toán */
    I2C1_MPU6050_Init();
    Encoder_Config();
    UART3_Init(115200);
    Kalman_Init(&kalmanFilter);

    /* Đọc First Sample từ Gia tốc kế để nạp ngay góc khởi đầu cho Kalman */
    int16_t init_ax, init_ay, init_az, init_gx, init_gy, init_gz;
    if (MPU6050_ReadRawData(&init_ax, &init_ay, &init_az, &init_gx, &init_gy, &init_gz) == 0u) {
        float axg = (float)init_ax / 16384.0f;
        float ayg = (float)init_ay / 16384.0f;
        float azg = (float)init_az / 16384.0f;
        float initPitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);
        Kalman_SetAngle(&kalmanFilter, initPitch);
        g_filteredPitch = initPitch;
    }

    /* 2. Cấu hình PID Tốc độ */
    PID_Init(&speedPID, 10.0f, 0.5f, 0.0f, -100.0f, 100.0f);
    PID_Init(&speedPID_R, 1.0f, 0.5f, 0.0f, -100.0f, 100.0f);
    PID_Init(&speedPID_L, 1.0f, 0.5f, 0.0f, -100.0f, 100.0f);

    /* 3. Tự động đọc lại điểm cân bằng & offset Gyro đã lưu trong Flash (Sector 11) */
    RobotFlashConfig_t savedConfig;
    if (Flash_ReadConfig(&savedConfig)) {
        g_targetAngle = savedConfig.targetAngle;
        g_gyroOffset  = savedConfig.gyroOffset;

        /* Nháy LED Xanh dương (PD15) 1 lần báo hiệu đã khôi phục cấu hình từ Flash */
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        for (volatile int d = 0; d < 500000; d++);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    }

    /* 4. Mặc định hệ thống ở trạng thái STOP: Bật LED Đỏ (PD14) báo Standby */
    g_systemEnable = 0;
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15);

    /* 5. Tạo các Task cho RTOS
     * Sửa theo Nguyên nhân số 2: Nâng Stack lên 512 words (2048 bytes) cho mỗi task
     * Đảm bảo không gian lưu trữ cho FPU (32 thanh ghi float) và ngắt lồng nhau
     */
    xTaskCreate(vTaskIMU, "Task_IMU", 512, NULL, 3, NULL);
    xTaskCreate(vTaskEncoder, "Task_Encoder", 512, NULL, 3, NULL);
    // xTaskCreate(vTaskControl, "Task_Control", 512, NULL, 4, NULL);
    xTaskCreate(vTaskUART, "Task_UART", 256, NULL, 1, NULL);
}

/* =====================================================================
 * CÁC HÀM XỬ LÝ 4 NÚT NHẤN ĐIỀU KHIỂN (PHƯƠNG ÁN A)
 * ===================================================================== */

/**
 * @brief Nút PC0: START hệ thống cân bằng
 *        - Kiểm tra an toàn: Chỉ cho chạy khi xe dựng thẳng (|Pitch - Target| < 15 độ)
 */
uint8_t Robot_Start(void) {
    float diffAngle = g_filteredPitch - g_targetAngle;
    if (diffAngle < 0.0f) diffAngle = -diffAngle;

    /* Điều kiện an toàn: xe phải dựng thẳng mới cho chạy */
    if (diffAngle > 15.0f) {
        /* Nháy nhanh LED Đỏ (PD14) 3 lần cảnh báo xe đang nghiêng quá mức */
        for (int i = 0; i < 3; i++) {
            GPIO_SetBits(GPIOD, GPIO_Pin_14);
            for (volatile int d = 0; d < 300000; d++);
            GPIO_ResetBits(GPIOD, GPIO_Pin_14);
            for (volatile int d = 0; d < 300000; d++);
        }
        GPIO_SetBits(GPIOD, GPIO_Pin_14); /* Giữ LED Đỏ báo STOP */
        return 0; /* Từ chối chạy */
    }

    g_systemEnable = 1;

    /* Reset các bộ tích phân và trạng thái PID */
    speedPID_L.pre_Error = 0.0f;
    speedPID_L.pre_pre_Error = 0.0f;
    speedPID_L.pre_Output = 0.0f;
    speedPID_L.Output = 0.0f;

    speedPID_R.pre_Error = 0.0f;
    speedPID_R.pre_pre_Error = 0.0f;
    speedPID_R.pre_Output = 0.0f;
    speedPID_R.Output = 0.0f;

    /* Chỉ thị LED: Bật Xanh lá (PD12), Tắt Đỏ (PD14) */
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    return 1;
}

/**
 * @brief Nút PC1: STOP / Dừng khẩn cấp
 *        - Ngắt PWM 2 động cơ ngay lập tức
 */
void Robot_Stop(void) {
    g_systemEnable = 0;

    Motor_SetPWM(0.0f);
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);

    speedPID_L.pre_Error = 0.0f;
    speedPID_L.pre_pre_Error = 0.0f;
    speedPID_L.pre_Output = 0.0f;
    speedPID_L.Output = 0.0f;

    speedPID_R.pre_Error = 0.0f;
    speedPID_R.pre_pre_Error = 0.0f;
    speedPID_R.pre_Output = 0.0f;
    speedPID_R.Output = 0.0f;

    /* Chỉ thị LED: Tắt Xanh lá (PD12), Bật Đỏ (PD14) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

static void Calib_DelayMs(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t d = 0; d < 35000; d++);
    }
}

/**
 * @brief Nút PC2: CALIB THÔNG MINH TOÀN DIỆN 
 *        - Bước 1: Tự nhận diện xe ĐỨNG YÊN HOÀN HẢO (chống rung tay) -> Calib Gyro Offset.
 *        - Bước 2: Tự nhận diện xe DỰNG THẲNG CÂN BẰNG -> Chốt Zero Point (Target Angle).
 *        - Bước 3: Tự động lưu vĩnh viễn cả 2 thông số vào Flash Sector 11.
 * @return 1 nếu calib thành công, 0 nếu timeout / hủy
 */
uint8_t Robot_AutoCalibrate(void) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    uint8_t still_batches = 0;
    uint32_t timeout_counter = 0;

    /* 0. Dừng động cơ ngay lập tức để an toàn tuyệt đối */
    if (g_systemEnable) {
        Robot_Stop();
    }

    /* Tắt tất cả các LED trước khi bắt đầu quy trình */
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    /* =========================================================================
     * BƯỚC 1: TỰ NHẬN DIỆN TRẠNG THÁI ĐỨNG YÊN (STILLNESS DETECTION) & CALIB GYRO
     * ========================================================================= */
    /* Bật LED Cam (PD13) báo hiệu đang trong Bước 1 */
    GPIO_SetBits(GPIOD, GPIO_Pin_13);

    while (still_batches < 3) {
        int16_t minGx = 32767, maxGx = -32768;
        int16_t minGy = 32767, maxGy = -32768;
        int16_t minGz = 32767, maxGz = -32768;

        /* Đọc 25 mẫu liên tiếp (~250ms) để kiểm tra độ rung lắc */
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

        /* Độ biến động con quay hồi chuyển (Peak-to-Peak Noise) */
        int16_t diffGx = maxGx - minGx;
        int16_t diffGy = maxGy - minGy;
        int16_t diffGz = maxGz - minGz;

        /* Ngưỡng tĩnh: Dưới 200 LSB (rung động góc < 1.5 deg/s) */
        if (diffGx < 200 && diffGy < 200 && diffGz < 200) {
            still_batches++;
        } else {
            /* Nếu phát hiện rung lắc: Nháy LED Cam cảnh báo xe chưa đứng yên */
            still_batches = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
        }

        timeout_counter++;
        if (timeout_counter > 60) {
            /* Quá 15 giây không thể đứng yên -> Báo lỗi nháy LED Đỏ 3 lần và thoát */
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

    /* Đã đạt trạng thái đứng yên tuyệt đối: Thu thập 100 mẫu tính gyroOffset */
    int32_t sumGx = 0;
    for (int i = 0; i < 100; i++) {
        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u) {
            sumGx += gx;
        }
        Calib_DelayMs(5);
    }
    g_gyroOffset = (float)sumGx / 100.0f;

    /* Tắt LED Cam (PD13), nháy LED Xanh dương (PD15) 2 lần báo xong Bước 1 */
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    for (int k = 0; k < 2; k++) {
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(150);
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        Calib_DelayMs(150);
    }

    /* =========================================================================
     * BƯỚC 2: TỰ NHẬN DIỆN TƯ THẾ DỰNG ĐỨNG CÂN BẰNG & CHỐT ZERO POINT
     * ========================================================================= */
    uint8_t balance_batches = 0;
    timeout_counter = 0;
    float sumBatchPitch = 0.0f;

    while (balance_batches < 4) {
        /* Nhấp nháy LED Xanh dương (PD15) nhắc người dùng giữ xe dựng thẳng đứng */
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

        float minP = 999.0f, maxP = -999.0f;
        float currentSum = 0.0f;
        uint8_t validSamples = 0;

        /* Đọc 20 mẫu (~200ms) tính góc Pitch và kiểm tra độ ổn định của tay giữ */
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

            /* Điều kiện nhận diện tư thế cân bằng:
             * 1. Xe đang dựng trong dải thẳng đứng (|avgP| < 12 độ, không nằm rạp sàn)
             * 2. Tay người giữ ổn định không bị lắc lư (diffP < 0.6 độ)
             */
            if ((avgP > -12.0f && avgP < 12.0f) && (diffP < 0.6f)) {
                balance_batches++;
                sumBatchPitch += avgP;
            } else {
                balance_batches = 0;
                sumBatchPitch = 0.0f;
            }
        }

        timeout_counter++;
        if (timeout_counter > 100) {
            /* Quá 20 giây người dùng không dựng xe -> Thoát và giữ nguyên cấu hình cũ */
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

    /* Đã nhận diện đúng tư thế dựng thẳng thăng bằng: Lấy trung bình góc làm điểm 0 */
    g_targetAngle = sumBatchPitch / 4.0f;

    /* Cập nhật lại bộ lọc Kalman với góc thăng bằng mới */
    Kalman_Init(&kalmanFilter);
    kalmanFilter.angle = g_targetAngle;
    g_filteredPitch = g_targetAngle;

    /* =========================================================================
     * BƯỚC 3: TỰ ĐỘNG LƯU VĨNH VIỄN CẢ 2 THÔNG SỐ VÀO FLASH SECTOR 11
     * ========================================================================= */
    /* Bật sáng cả LED Xanh dương và LED Cam trong lúc ghi Flash */
    GPIO_SetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_13);
    Flash_SaveConfig(g_targetAngle, g_gyroOffset);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15 | GPIO_Pin_13);

    /* Nháy 3 lần LED Xanh lá (PD12) báo hiệu toàn bộ quy trình hoàn tất mỹ mãn */
    for (int k = 0; k < 3; k++) {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        Calib_DelayMs(200);
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        Calib_DelayMs(200);
    }

    /* Quay lại trạng thái Standby an toàn: Bật LED Đỏ (PD14) */
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    return 1;
}

void Robot_SetBalancePoint(void) {
    /* Đổi sang gọi quy trình tự động */
    Robot_AutoCalibrate();
}

void Robot_CalibrateIMU(void) {
    /* Đổi sang gọi quy trình tự động */
    Robot_AutoCalibrate();
}

/* =====================================================================
 * CÁC TASK FREERTOS (Chu kỳ chuẩn xác với vTaskDelayUntil)
 * ===================================================================== */

static void vTaskIMU(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    uint8_t beat = 0;
    uint8_t isFirstSample = 1;

    while (1)
    {
        /* Bảo vệ mốc thời gian: Nếu bị trễ bất thường > 50ms, đồng bộ lại mốc */
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u)
        {
            /* Tính góc Accel Pitch trực tiếp từ ax, ay, az (không gọi lại I2C lần 2) */
            float axg = (float)ax / 16384.0f;
            float ayg = (float)ay / 16384.0f;
            float azg = (float)az / 16384.0f;
            g_rawAccPitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / 3.14159265359f);

            /* Vận tốc góc đã trừ sai số tĩnh sau khi Calib */
            g_rawGyroRate = ((float)gx - g_gyroOffset) / 131.0f;

            /* Lấy First Sample: Khởi tạo góc bắt đầu từ Accelerometer
             * Triệt tiêu hoàn toàn thời gian trễ quá độ (Zero Transient Lag) */
            if (isFirstSample) {
                Kalman_SetAngle(&kalmanFilter, g_rawAccPitch);
                g_filteredPitch = g_rawAccPitch;
                isFirstSample = 0;
            } else {
                /* Chạy qua bộ lọc Kalman (chu kỳ chuẩn xác dt = 0.005s) */
                g_filteredPitch = Kalman_GetAngle(&kalmanFilter, g_rawAccPitch, g_rawGyroRate, 0.005f);
            }
        }

        if (g_systemEnable && (++beat == 20)) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_12); /* Nháy LED Xanh lá khi đang RUN */
        }
    }
}

static void vTaskEncoder(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    MOTOR leftMotor = {0};
    MOTOR rightMotor = {0};
    int8_t beat = 0;
    float pwm_left = 0.0f;
    float pwm_right = 0.0f;

    while (1)
    {
        /* Chu kỳ lấy mẫu cố định chính xác 10ms (dt = 0.010s) cho bộ PID Tốc độ */
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  

        Encoder_ReadStruct(&leftMotor, &rightMotor);
        g_motorSpeed = (leftMotor.speed + rightMotor.speed) / 2.0f;
      
        g_motorSpeed_left = leftMotor.speed;
        g_motorSpeed_right = rightMotor.speed;

        pwm_left = PID_Compute(&speedPID_L, g_motorSpeedTarget_left, leftMotor.speed, 0.010f);
        pwm_right = PID_Compute(&speedPID_R, g_motorSpeedTarget_right, rightMotor.speed, 0.010f);

        /* Chỉ xuất PWM ra động cơ khi hệ thống đã được START */
        if (g_systemEnable) {
            Motor_SetPWM_Left(pwm_left);
            Motor_SetPWM_Right(pwm_right);
        } else {
            Motor_SetPWM_Left(0.0f);
            Motor_SetPWM_Right(0.0f);
        }
        
        if (++beat >= 15) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_13); /* Đèn Cam nháy báo nhịp Task Encoder */
        }
    }
}

static void vTaskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    const float DEADBAND_OFFSET = 15.0f;
    uint8_t beat = 0;

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(50)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* Tự động ngắt an toàn nếu xe bị ngã quá 45 độ */
        if (g_systemEnable) {
            float diff = g_filteredPitch - g_targetAngle;
            if (diff > 45.0f || diff < -45.0f) {
                Robot_Stop(); /* Tự ngắt động cơ để bảo vệ xe */
            }
        }

        if (g_systemEnable) {
            float errorAngle = g_targetAngle - g_filteredPitch;
            float dErrorAngle = -kalmanFilter.rate;

            float outputPWM = Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle);

            if (outputPWM > 2.0f)
            {
                outputPWM += DEADBAND_OFFSET;
            }
            else if (outputPWM < -2.0f)
            {
                outputPWM -= DEADBAND_OFFSET;
            }
            else
            {
                outputPWM = 0.0f;
            }

            Motor_SetPWM(outputPWM);
            g_motorPWM = outputPWM;
        } else {
            Motor_SetPWM(0.0f);
            g_motorPWM = 0.0f;
        }

        if (++beat == 40) {
            beat = 0;
            GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
        }
    }
}

/* ---------------------------------------------------------------------
 *  TASK UART - 20 Hz (50ms): Giao tiếp máy tính (PD8-TX, PD9-RX)
 *  Khung gói tin: [0xAA] [ID] [len] [Data...] [Checksum] [0xFF]
 *  Checksum = ID | len | Data
 * --------------------------------------------------------------------- */
static void vTaskUART(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); /* 20Hz (50ms) */

    /* Biến máy trạng thái giải mã gói tin nhận từ máy tính */
    uint8_t rxState = 0;
    uint8_t rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxCount = 0;
    uint8_t rxChecksum = 0;
    uint8_t rxDataBuf[32];

    while (1)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastWakeTime) > pdMS_TO_TICKS(150)) {
            xLastWakeTime = xCurrentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if (UART_TELEMETRY_MODE == 1)
        /* Chế độ ASCII CSV: Dễ dàng vẽ 2 đường đồ thị so sánh trực tiếp trên Arduino Serial Plotter / Teleplot */
        UART3_PrintfDMA("Raw_NoFilter:%.2f,Kalman_Filtered:%.2f\r\n", g_rawAccPitch, g_filteredPitch);
#else
        /* 1. Gửi gói tin Telemetry lên máy tính qua DMA (ID = 0x01, len = 24 byte)
         * Cấu trúc: 0xAA | 0x01 | 0x18 | Data (6 float) | Checksum | 0xFF
         * So sánh trực tiếp:
         *   - telem.rawPitch:      Góc KHÔNG BỘ LỌC (Gia tốc kế đo thô, nhạy với rung cơ học)
         *   - telem.filteredPitch: Góc CÓ BỘ LỌC (Kalman kết hợp Accel + Gyro mượt mà)
         */
        RobotTelemetry_t telem;
        telem.rawPitch      = g_rawAccPitch;
        telem.filteredPitch = g_filteredPitch;
        telem.gyroRate      = kalmanFilter.rate;
        telem.motorSpeed    = g_motorSpeed;
        telem.motorPWM      = g_motorPWM;
        telem.targetAngle   = g_targetAngle;

        UART3_SendPacketDMA(0x01, &telem, (uint8_t)sizeof(RobotTelemetry_t));
#endif

        /* 2. Nhận và giải mã gói tin từ máy tính: 0xAA ID len Data Checksum 0xFF */
        while (UART3_Available() > 0)
        {
            int16_t ch = UART3_ReadChar();
            if (ch < 0) break;
            uint8_t byte = (uint8_t)ch;

            switch (rxState)
            {
            case 0: /* Chờ Header 0xAA */
                if (byte == UART_FRAME_HEADER) {
                    rxState = 1;
                } else {
                    /* Hỗ trợ thêm ký tự ASCII trực tiếp nếu người dùng gõ từ Serial Terminal */
                    if (byte == 's' || byte == 'S') { Robot_Stop(); }
                    else if (byte == 'r' || byte == 'R') { Robot_Start(); }
                    else if (byte == 'c' || byte == 'C') { Robot_AutoCalibrate(); }
                }
                break;

            case 1: /* Nhận ID */
                rxId = byte;
                rxState = 2;
                break;

            case 2: /* Nhận len */
                rxLen = byte;
                rxCount = 0;
                if (rxLen > sizeof(rxDataBuf)) {
                    rxState = 0; /* Quá dung lượng buffer -> Hủy gói */
                } else if (rxLen == 0) {
                    rxState = 4; /* Không có Data -> Nhảy tới Checksum */
                } else {
                    rxState = 3; /* Nhận Data */
                }
                break;

            case 3: /* Đọc Data */
                rxDataBuf[rxCount++] = byte;
                if (rxCount >= rxLen) {
                    rxState = 4;
                }
                break;

            case 4: /* Đọc Checksum */
                rxChecksum = byte;
                rxState = 5;
                break;

            case 5: /* Chờ Tail 0xFF */
                if (byte == UART_FRAME_TAIL) {
                    /* Xác thực tính toàn vẹn: Checksum = ID | len | Data */
                    uint8_t calcCs = UART_CalculateChecksum(rxId, rxLen, rxDataBuf);
                    if (calcCs == rxChecksum) {
                        /* Gói tin hợp lệ 100%! Thực thi lệnh theo ID */
                        switch (rxId)
                        {
                        case 0x01: /* Lệnh START */
                            Robot_Start();
                            break;
                        case 0x02: /* Lệnh STOP */
                            Robot_Stop();
                            break;
                        case 0x03: /* Lệnh CALIB */
                            Robot_AutoCalibrate();
                            break;
                        case 0x04: /* Lệnh Set Target Angle (payload là float 4 byte) */
                            if (rxLen >= 4) {
                                float newTarget = 0.0f;
                                memcpy(&newTarget, rxDataBuf, 4);
                                g_targetAngle = newTarget;
                            }
                            break;
                        case 0x05: /* Lệnh Set Target Speed (payload là float 4 byte) */
                            if (rxLen >= 4) {
                                float newSpeed = 0.0f;
                                memcpy(&newSpeed, rxDataBuf, 4);
                                g_targetSpeed = newSpeed;
                            }
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
    }
}

/* Hook báo lỗi tràn Stack FreeRTOS (nếu có) */
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
