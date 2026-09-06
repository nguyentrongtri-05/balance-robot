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

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);

void System_Tasks_Init(void) {
    /* 1. Khởi tạo thuật toán */
    Kalman_Init(&kalmanFilter);
    /* Giả sử Fuzzy trả về Vận Tốc mục tiêu, ta dùng PID để bám theo vận tốc đó */
    PID_Init(&speedPID, 2.0f, 0.5f, 0.01f, -100.0f, 100.0f); 

    /* 2. Tạo các Task cho RTOS */
    xTaskCreate(vTaskIMU, "Task_IMU", 256, NULL, 3, NULL);
    xTaskCreate(vTaskEncoder, "Task_Encoder", 128, NULL, 2, NULL);
    xTaskCreate(vTaskControl, "Task_Control", 256, NULL, 4, NULL);
}

/* =========================================================================
 * Task 1: Read IMU 9 DOF (200Hz - 5ms)
 * ========================================================================= */
static void vTaskIMU(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        float accAngle, gyroRate;
        
        /* 1. Gọi hàm đọc dữ liệu thanh ghi từ IMU (I2C/SPI) */
        IMU_Read(&accAngle, &gyroRate);
        
        g_rawAccPitch = accAngle;
        g_rawGyroRate = gyroRate;

        /* 2. Đưa qua bộ lọc Kalman khử nhiễu (dt = 0.005s) */
        g_filteredPitch = Kalman_GetAngle(&kalmanFilter, accAngle, gyroRate, 0.005f);

        GPIO_ToggleBits(GPIOD, GPIO_Pin_12); 
    }
}

/* =========================================================================
 * Task 2: Read Encoder (100Hz - 10ms)
 * ========================================================================= */
static void vTaskEncoder(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        float speed;
        
        /* Đọc xung thanh ghi CNT và chuyển sang vận tốc vật lý */
        Encoder_Read(&speed);
        g_motorSpeed = speed;
        
        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    }
}

/* =========================================================================
 * Task 3: Fuzzy Control Loop (200Hz - 5ms)
 * ========================================================================= */
static void vTaskControl(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 
    
    /* Mục tiêu (Setpoint) là xe đứng thẳng (0 độ) */
    float targetAngle = 0.0f;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* 1. Tính toán sai số (Error) và đạo hàm sai số (dError) */
        float errorAngle = targetAngle - g_filteredPitch;
        float dErrorAngle = -kalmanFilter.rate; 
        
        /* 2. Đưa vào hệ Fuzzy Logic */
        float outputPWM = Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle);
        /* 2. Đưa vào hệ Fuzzy Logic (Trả về vận tốc mục tiêu cho xe) */
        float targetSpeed = Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle);

        /* 3. Đưa ra lệnh điều khiển động cơ qua PWM (TIMx->CCR) */
        /* 3. Khâu PID vòng trong: Bám sát tốc độ mục tiêu bằng cách điều chỉnh PWM */
        float outputPWM = PID_Compute(&speedPID, targetSpeed, g_motorSpeed, 0.005f);

        /* 4. Đưa ra lệnh điều khiển động cơ qua PWM (TIMx->CCR) */
        Motor_SetPWM(outputPWM);
        g_motorPWM = outputPWM;
        
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    }
}
