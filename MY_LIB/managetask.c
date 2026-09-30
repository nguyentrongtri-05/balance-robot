#include "managetask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "driver_i2c.h"
#include "fuzzy.h"
#include "pid.h"
#include "kalman.h"

/* Biến Controller */
extern FuzzyController_t fuzzyControl;
PID_t speedPID;
Kalman_t kalmanFilter;

/* Các biến toàn cục chứa dữ liệu thực để tiện Monitor trên Keil C */
volatile float g_rawAccPitch = 0.0f;
volatile float g_rawGyroRate = 0.0f;
volatile float g_filteredPitch = 0.0f; /* Góc thực sau đọc cảm biến và lọc Kalman */
volatile float g_motorSpeed = 0.0f;
volatile float g_motorPWM = 0.0f;

/* Các biến liên kết giữa 2 vòng lặp (Cascaded Loop) */
volatile float g_targetSpeed = 0.0f;
volatile float g_targetAngle = 0.0f;

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskEncoder(void *pvParameters);
static void vTaskControl(void *pvParameters);

void System_Tasks_Init(void) {
    /* 1. Khởi tạo bus I2C, encoder và các thuật toán */
    I2C1_MPU6050_Init();
    Encoder_Config();
    Kalman_Init(&kalmanFilter);

    /* 2. Cấu hình PID Tốc độ */
    PID_Init(&speedPID, 10.0f, 0.5f, 0.0f, -100.0f, 100.0f);

    /* 3. Tạo các Task cho RTOS */
    xTaskCreate(vTaskIMU, "Task_IMU", 256, NULL, 3, NULL);
		xTaskCreate(vTaskEncoder, "Task_Encoder", 256, NULL, 2, NULL);
   // xTaskCreate(vTaskControl, "Task_Control", 256, NULL, 4, NULL);
}

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
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz) == 0u)
        {
            g_rawAccPitch = MPU6050_GetPitch();
            g_rawGyroRate = (float)gx / 131.0f; // Trục nghiêng thường là trục X hoặc Y tùy phần cứng. Đổi lại nếu sai trục.

            /* Chạy qua bộ lọc Kalman (dt = 0.005s) */
            g_filteredPitch = Kalman_GetAngle(&kalmanFilter, g_rawAccPitch, g_rawGyroRate, 0.005f);
        }

        // GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
}

static void vTaskEncoder(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
		MOTOR leftMotor = {0};
    MOTOR rightMotor = {0};
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  
        Encoder_ReadStruct(&leftMotor, &rightMotor);
        g_motorSpeed = (leftMotor.speed + rightMotor.speed) / 2.0f;
        g_targetAngle = PID_Compute(&speedPID, g_targetSpeed, g_motorSpeed, 0.010f);

        // GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    }
}

static void vTaskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    const float DEADBAND_OFFSET = 15.0f;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        float errorAngle = g_targetAngle - g_filteredPitch;
        float dErrorAngle = -kalmanFilter.rate; // Sử dụng vận tốc góc đã được Kalman khử drift/bias

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

        // GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    }
}
