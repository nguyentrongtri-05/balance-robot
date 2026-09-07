#include "managetask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "motor.h"
#include "imu.h"
#include "encoder.h"
#include "fuzzy.h"
#include "kalman.h"
#include "pid.h"

/* Biến Controller */
extern FuzzyController_t fuzzyControl;
Kalman_t kalmanFilter;
PID_t speedPID;

/* Các biến toàn cục chứa dữ liệu thực để tiện Monitor trên Keil C */
volatile float g_rawAccPitch = 0.0f;
volatile float g_rawGyroRate = 0.0f;
volatile float g_filteredPitch = 0.0f; // Góc thực sau lọc Kalman
volatile float g_motorSpeed = 0.0f;
volatile float g_motorPWM = 0.0f;

/* Các biến liên kết giữa 2 vòng lặp (Cascaded Loop) */
volatile float g_targetSpeed = 0.0f; // Tốc độ người dùng mong muốn (Set = 0 để xe đứng yên)
volatile float g_targetAngle = 0.0f; // Góc nghiêng mục tiêu (Do PID vòng ngoài tính ra)

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);

void System_Tasks_Init(void) {
    /* 1. Khởi tạo thuật toán */
    Kalman_Init(&kalmanFilter);
    
    /* 2. Cấu hình PID Tốc độ (Vòng ngoài) 
     * Nhận sai số Tốc độ -> Xuất ra Góc nghiêng mục tiêu (Target Angle)
     * Giới hạn ngõ ra là +-10 độ để xe không bao giờ ra lệnh nghiêng quá giới hạn lật của xe */
    PID_Init(&speedPID, 0.05f, 0.005f, 0.0f, -10.0f, 10.0f); 

    /* 3. Tạo các Task cho RTOS */
    xTaskCreate(vTaskIMU, "Task_IMU", 256, NULL, 3, NULL);
    xTaskCreate(vTaskEncoder, "Task_Encoder", 256, NULL, 2, NULL); // Tăng Stack vì có xử lý PID
    xTaskCreate(vTaskControl, "Task_Control", 256, NULL, 4, NULL);
}

/* =========================================================================
 * Task 1: Read IMU 9 DOF (200Hz - 5ms)
 * ========================================================================= */
static void vTaskIMU(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); 

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // float accAngle, gyroRate;
        
        // /* Gọi hàm đọc dữ liệu thanh ghi từ IMU (I2C/SPI) */
        // IMU_Read(&accAngle, &gyroRate);
        
        // g_rawAccPitch = accAngle;
        // g_rawGyroRate = gyroRate;

        // /* Đưa qua bộ lọc Kalman khử nhiễu (dt = 0.005s) */
        // g_filteredPitch = Kalman_GetAngle(&kalmanFilter, accAngle, gyroRate, 0.005f);

        GPIO_ToggleBits(GPIOD, GPIO_Pin_12); 
    }
}

/* =========================================================================
 * Task 2: Read Encoder & PID Tốc Độ (100Hz - 10ms) - VÒNG NGOÀI (OUTER LOOP)
 * ========================================================================= */
static void vTaskEncoder(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); 

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // float speed;
        // Encoder_Read(&speed);
        // g_motorSpeed = speed;
        
        // /* Chạy khâu PID Vòng Ngoài: 
        //  * Lấy Tốc độ mong muốn (g_targetSpeed) trừ Tốc độ thực tế (g_motorSpeed)
        //  * -> Suy luận ra: Xe cần nghiêng bao nhiêu độ (g_targetAngle) để bám được tốc độ đó.
        //  * dt = 0.01s (100Hz) */
        // g_targetAngle = PID_Compute(&speedPID, g_targetSpeed, g_motorSpeed, 0.010f);
        
        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    }
}

/* =========================================================================
 * Task 3: Fuzzy Angle Control (200Hz - 5ms) - VÒNG TRONG (INNER LOOP)
 * ========================================================================= */
static void vTaskControl(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); 
    
    /* Vùng chết của động cơ (Deadband) - Tùy chỉnh theo phần cứng thực tế (Ví dụ 15%) */
    const float DEADBAND_OFFSET = 15.0f; 

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // /* 1. Tính toán sai số (Error) dựa trên Target Angle được cấp từ vòng PID Tốc độ */
        // float errorAngle = g_targetAngle - g_filteredPitch;
        // float dErrorAngle = -kalmanFilter.rate; 
        
        // /* 2. Đưa vào hệ Fuzzy Logic (Vòng trong phản ứng cực nhanh 200Hz xuất thẳng PWM) */
        // float outputPWM = Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle);

        // /* 3. Khâu bù Vùng Chết (Deadband Compensation) để chống hiện tượng Jitter (Lắc lư xe) */
        // if (outputPWM > 0.5f) {
        //     outputPWM += DEADBAND_OFFSET; // Xe có xu hướng tiến -> Cộng thêm bù chết phần tiến
        // } else if (outputPWM < -0.5f) {
        //     outputPWM -= DEADBAND_OFFSET; // Xe có xu hướng lùi -> Trừ đi bù chết phần lùi
        // } else {
        //     outputPWM = 0.0f;
        // }

        // /* 4. Đưa ra lệnh điều khiển động cơ qua PWM (TIMx->CCR) */
        // Motor_SetPWM(outputPWM);
        // g_motorPWM = outputPWM;
        
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    }
}
