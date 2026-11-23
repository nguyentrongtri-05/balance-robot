#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* My libraries */
#include "gpio.h"
#include "motor.h"
#include "fuzzy.h"
#include "pid.h"
#include "managetask.h"

/* Global Controller Instances: Cascaded Dual-FuzzyPD + PID Yaw */
FuzzyController_t FuzzyPD_Pitch;   /* Khâu FuzzyPD Vòng Trong: Cân bằng góc Pitch -> xuất Điện áp V (5ms / 200Hz) */
FuzzyController_t FuzzyPD_Pos;     /* Khâu FuzzyPD Vòng Ngoài: Giữ vị trí & bám vận tốc -> xuất Góc ° (5ms / 200Hz) */
PID_t             PID_Yaw;         /* Bộ điều khiển PID Gia số giữ hướng xoay Yaw thuần IMU -> xuất PWM (5ms / 200Hz) */

int main(void) {
    /* 1. Cấu hình NVIC Priority Group 4 bắt buộc cho FreeRTOS trên Cortex-M */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khởi tạo ngoại vi cơ bản */
    GPIO_Config();
    Motor_Config();

    /* 3. Khởi tạo 2 bộ Fuzzy PD nối tiếp (cùng bảng luật FUZZY_RULE_PD) & bộ PID hướng xoay Yaw.
     *    Tham số Fuzzy_Init: ke, kde, ku | e: NB, NS | de: NB, NS | u: Small (NS/PS), Med (NM/PM) | bảng luật
     *
     *  - FuzzyPD_Pitch (vòng trong, đầu ra điện áp V):
     *      e  = pitch đo - góc đặt (°)  , ke  = 1/11.5 (bão hòa ±11.5°)
     *      de = vận tốc góc pitch (°/s) , kde = 1/320  (bão hòa ±320 °/s)
     *      ku = VOLTAGE_LIMIT_BALANCE = ±10.8V (90% của 12V ≈ ±7560 PWM)
     *  - FuzzyPD_Pos (vòng ngoài, đầu ra góc nghiêng bù °):
     *      e  = vị trí đo - vị trí đích (cm)   , ke  = 0.025 (kéo vị trí nhẹ)
     *      de = vận tốc đo - vận tốc đặt (cm/s), kde = 0.060 (ưu tiên phanh)
     *      ku = 2.40° (kẹp an toàn ±CASCADE_MAX_TOTAL_TILT_DEG)
     *  - PID_Yaw (giữ hướng thuần IMU, đầu ra ±840 PWM): Kp = 18.00, Ki = 8.00, Kd = 0.35
     */
    Fuzzy_Init(&FuzzyPD_Pitch, 1.0f / 11.5f, 1.0f / 320.0f, VOLTAGE_LIMIT_BALANCE,
               0.46f, 0.20f,  0.46f, 0.20f,  0.25f, 0.65f,  FUZZY_RULE_PD);
    Fuzzy_Init(&FuzzyPD_Pos, 0.025f, 0.060f, 2.40f,
               0.44f, 0.16f,  0.46f, 0.18f,  0.26f, 0.62f,  FUZZY_RULE_PD);
    PID_Init(&PID_Yaw, 18.00f, 8.00f, 0.35f, -PWM_LIMIT_YAW, PWM_LIMIT_YAW);

    /* 4. Khởi tạo các Task RTOS & ngoại vi */
    System_Tasks_Init();

    /* 5. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();

    while (1) {
    }
}
