#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* My libraries */
#include "gpio.h"
#include "motor.h"
#include "fuzzy.h"
#include "managetask.h"

/* Global Fuzzy PD Controller Instances (Cascaded Dual-FuzzyPD) */
FuzzyController_t FuzzyPD_Pitch;   /* Khâu FuzzyPD Vòng Trong: Cân bằng góc chúc ngóc Pitch (5ms / 200Hz) */
FuzzyController_t FuzzyPD_Pos;     /* Khâu FuzzyPD Vòng Ngoài: Giữ vị trí & bám vận tốc (20ms / 50Hz) */

int main(void) {
    /* 1. Cấu hình NVIC Priority Group 4 bắt buộc cho FreeRTOS trên Cortex-M */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khởi tạo ngoại vi cơ bản */
    GPIO_Config();
    Motor_Config();

    /* 3. Khởi tạo 2 bộ điều khiển Fuzzy PD nối tiếp (Cascaded Dual-FuzzyPD):
     *    - FuzzyPD_Pitch (Vòng trong cân bằng Pitch, chu kỳ 5ms):
     *      + ke = 1/12 (e bão hòa ở ±12 độ), kde = 1/720 (de bão hòa ở ±720 độ/s), ku = 7560, out = [-7560, +7560] PWM
     *      + Dải e  : NB = 0.50, NS = 0.20, ZE_flat = 0.002
     *      + Dải de : NB = 0.50, NS = 0.25, ZE_flat = 0.002
     *      + Dải u  : Small(NS/PS) = 0.28, Med(NM/PM) = 0.64
     *    - FuzzyPD_Pos (Vòng ngoài giữ vị trí & vận tốc, chu kỳ 20ms):
     *      + e = sai số vị trí (bão hòa ±41.7 cm) -> ke = 0.024; de = sai số vận tốc (bão hòa ±29.4 cm/s) -> kde = 0.034; ku = 3.3 độ, out = [-3.5, +3.5] độ
     *      + Dải e  : NB = 0.39, NS = 0.09, ZE_flat = 0.001
     *      + Dải de : NB = 0.48, NS = 0.25, ZE_flat = 0.020
     *      + Dải u  : Small(NS/PS) = 0.28, Med(NM/PM) = 0.56
     */
    Fuzzy_Init(&FuzzyPD_Pitch,
               1.0f / 12.0f, 1.0f / 720.0f, 7560.0f, -7560.0f, 7560.0f,
               0.50f, 0.20f, 0.002f,
               0.50f, 0.25f, 0.002f,
               0.28f, 0.64f);
    Fuzzy_Init(&FuzzyPD_Pos,
               0.024f, 0.034f, 3.3f, -3.5f, 3.5f,   /* e: bao hoa +-41.7 cm, de: +-29.4 cm/s, ku ~ bien do goc dat bu tru (do) */
               0.39f, 0.09f, 0.001f,
               0.48f, 0.25f, 0.020f,
               0.28f, 0.56f);

    /* 4. Khởi tạo các Task RTOS & ngoại vi */
    System_Tasks_Init();

    /* 5. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();

    while (1) {
    }
}
