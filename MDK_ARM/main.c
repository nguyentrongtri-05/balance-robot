#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* My libraries */
#include "gpio.h"
#include "motor.h"
#include "fuzzy.h"
#include "managetask.h"

/* Global Fuzzy Controller Instance */
FuzzyController_t fuzzyControl;

int main(void) {
    /* 1. Cấu hình NVIC Priority Group 4 bắt buộc cho FreeRTOS trên Cortex-M */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khởi tạo ngoại vi cơ bản */
    GPIO_Config();
    Motor_Config();

    /* 3. Khởi tạo bộ điều khiển Fuzzy */
    Fuzzy_Init(&fuzzyControl);


    /* 5. Khởi tạo các Task RTOS & ngoại vi */
    System_Tasks_Init();

    /* 6. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();

    while (1) {
    }
}
