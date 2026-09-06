#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* My libraries */
#include "gpio.h"
#include "motor.h"
#include "imu.h"
#include "encoder.h"
#include "fuzzy.h"
#include "managetask.h"

/* Global Fuzzy Controller Instance */
FuzzyController_t fuzzyControl;

int main(void) {
    /* 1. REQUIRED FOR FREERTOS ON CORTEX-M: Priority Group 4 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Initialize Peripherals */
    GPIO_Config();
    Motor_Config();
    IMU_Config();
    Encoder_Config();
    
    /* 3. Initialize Fuzzy Controller */
    Fuzzy_Init(&fuzzyControl);

    /* 4. Create RTOS Tasks */
    System_Tasks_Init();

    /* 5. Start the FreeRTOS Scheduler */
    vTaskStartScheduler();

    /* CPU never reaches here if Heap is sufficient */
    while (1) {
    }
}