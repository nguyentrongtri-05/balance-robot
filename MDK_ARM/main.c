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

/* Global Fuzzy Controller Instance */
FuzzyController_t fuzzyControl;

int main(void) {
    /* 1. REQUIRED FOR FREERTOS ON CORTEX-M: Priority Group 4 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Initialize Peripherals */
    GPIO_Config();
    Motor_Config();
    /* 3. Initialize Fuzzy Controller */
   // Fuzzy_Init(&fuzzyControl);

    /* 4. Create RTOS Tasks */
	//	System_Tasks_Init();

    /* 5. Start the FreeRTOS Scheduler */
   // vTaskStartScheduler();

    /* CPU never reaches here if Heap is sufficient */
 Motor_SetPWM(20);
	// Motor_SetPWM_Right(30);
    while (1) {
    }
}