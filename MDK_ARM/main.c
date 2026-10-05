#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* My libraries */
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "fuzzy.h"
#include "managetask.h"
#include "driver_i2c.h"

/* Chế độ chạy:
 * 0 = Chạy hệ cân bằng đầy đủ (RTOS + IMU + Fuzzy + PID tốc độ)
 * 1 = Test motor: quay 2 bánh với APP_TEST_PWM để kiểm tra chiều quay / vùng chết */
#define APP_MOTOR_TEST   0
#define APP_TEST_PWM     20.0f

/* Global Fuzzy Controller Instance */
FuzzyController_t fuzzyControl;

int main(void) {
    /* 1. REQUIRED FOR FREERTOS ON CORTEX-M: Priority Group 4 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Initialize Peripherals */
    GPIO_Config();
    Motor_Config();

#if APP_MOTOR_TEST
    /* Cả 2 bánh phải quay TIẾN. Nếu không: sửa MOTOR_x_INVERT trong motor.h
     * Lưu ý: lệnh đi qua bù vùng chết (MOTOR_x_DEADZONE) trước khi ra PWM */
    Motor_SetPWM(APP_TEST_PWM);
    while (1) {
    }
#else
    /* 3. Initialize Fuzzy Controller */
    Fuzzy_Init(&fuzzyControl);

    /* 4. Create RTOS Tasks (gồm khởi tạo + hiệu chuẩn IMU: GIỮ XE ĐỨNG YÊN ~1s) */
    System_Tasks_Init();

    /* 5. Start the FreeRTOS Scheduler */
    vTaskStartScheduler();

    /* CPU never reaches here if Heap is sufficient */
    Motor_Enable(DISABLE);
    while (1) {
    }
#endif
}