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
FuzzyController_t FuzzyPD_Pos;     /* Khâu FuzzyPD Vòng Ngoài: Giữ vị trí & bám vận tốc (5ms / 200Hz) */

int main(void) {
    /* 1. Cấu hình NVIC Priority Group 4 bắt buộc cho FreeRTOS trên Cortex-M */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khởi tạo ngoại vi cơ bản */
    GPIO_Config();
    Motor_Config();

    /* 3. Khởi tạo 2 bộ điều khiển Fuzzy PD nối tiếp (Cascaded Dual-FuzzyPD):
     *    - FuzzyPD_Pitch (Vòng trong cân bằng Pitch, chu kỳ 5ms / 200Hz):
     *      + ke = 1/11.5 (e bão hòa ở ±11.5 độ), kde = 1/320 (de bão hòa ở ±320 độ/s), ku = 7560 (±7560 PWM)
     *      + Dải e  : NB = 0.46, NS = 0.20, ZE_flat = 0.002  (Kp_pitch ≈ 920 PWM/°)
     *      + Dải de : NB = 0.46, NS = 0.20, ZE_flat = 0.002  (Kd_pitch ≈ 33.1 PWM/(°/s), bắt nhanh xung lực khi bị tác động nhẹ)
     *      + Dải u  : Small(NS/PS) = 0.28, Med(NM/PM) = 0.66
     *    - FuzzyPD_Pos (Vòng ngoài giữ vị trí & vận tốc trọng tâm CoM, chu kỳ 5ms / 200Hz):
     *      + e = sai số vị trí (bão hòa ±45.5 cm) -> ke = 0.022; de = sai số vận tốc CoM (bão hòa ±23.8 cm/s) -> kde = 0.042; ku = 2.20 độ
     *      + Dải e  : NB = 0.48, NS = 0.16, ZE_flat = 0.002  (Kp_pos ≈ 0.085 °/cm)
     *      + Dải de : NB = 0.50, NS = 0.20, ZE_flat = 0.008  (Kd_pos ≈ 0.129 °/(cm/s) > độ sụt góc vòng trong 0.080 °/(cm/s), đảm bảo phanh dương như lan28)
     *      + Dải u  : Small(NS/PS) = 0.28, Med(NM/PM) = 0.58
     */
    Fuzzy_Init(&FuzzyPD_Pitch,
               1.0f / 11.5f, 1.0f / 320.0f, 7560.0f,
               0.46f, 0.20f, 0.002f,
               0.46f, 0.20f, 0.002f,
               0.28f, 0.66f);
    FuzzyPD_Pitch.alpha_de = 0.50f;   /* Hệ số lọc thông thấp đạo hàm sai số góc nghiêng (200Hz) */

    Fuzzy_Init(&FuzzyPD_Pos,
               0.022f, 0.042f, 2.20f,   /* e: bao hoa +-45.5 cm, de: +-23.8 cm/s, ku: bien do goc dat (+-2.20 do) */
               0.48f, 0.16f, 0.002f,
               0.50f, 0.20f, 0.008f,
               0.28f, 0.58f);
    FuzzyPD_Pos.alpha_de   = 0.16f;   /* Hệ số lọc thông thấp đạo hàm sai số vị trí (200Hz) */

    /* 4. Khởi tạo các Task RTOS & ngoại vi */
    System_Tasks_Init();

    /* 5. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();	


    while (1) {
    }
}
