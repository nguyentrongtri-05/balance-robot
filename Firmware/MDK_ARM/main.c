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

    /* 3. Khởi tạo 2 bộ điều khiển Fuzzy PD nối tiếp & bộ PID hướng xoay Yaw:
     *    - FuzzyPD_Pitch (Vòng trong cân bằng Pitch, chu kỳ 5ms / 200Hz -> Đầu ra Điện áp V):
     *      + ke = 1/11.5 (e bão hòa ±11.5°), kde = 1/320 (de bão hòa ±320 °/s),
     *        ku = VOLTAGE_LIMIT_BALANCE (±10.8V = 90% của 12V, tương đương ±7560 PWM)
     *      + Dải e  : NB = 0.46, NS = 0.20, ZE_flat = 0.002
     *      + Dải de : NB = 0.46, NS = 0.20, ZE_flat = 0.005
     *      + Dải u  : Small(NS/PS) = 0.25, Med(NM/PM) = 0.65
     *    - FuzzyPD_Pos (Vòng ngoài giữ vị trí & hãm vận tốc CoM, chu kỳ 5ms / 200Hz -> Đầu ra Góc nghiêng °):
     *      + ke = 0.025 (kéo vị trí nhẹ nhàng), kde = 0.060 (ưu tiên phanh vận tốc mạnh khi bị đẩy), ku = 2.40°
     *      + Dải e  : NB = 0.44, NS = 0.16, ZE_flat = 0.002
     *      + Dải de : NB = 0.46, NS = 0.18, ZE_flat = 0.010
     *      + Dải u  : Small(NS/PS) = 0.26, Med(NM/PM) = 0.62
     *    - PID_Yaw (Vòng giữ hướng xoay Yaw thuần IMU, chu kỳ 5ms / 200Hz -> Đầu ra ±840 PWM):
     *      + Kp = 18.00, Ki = 8.00, Kd = 0.35
     */
    Fuzzy_Init(&FuzzyPD_Pitch,
               1.0f / 11.5f, 1.0f / 320.0f, VOLTAGE_LIMIT_BALANCE,
               0.46f, 0.20f, 0.002f,
               0.46f, 0.20f, 0.005f,
               0.25f, 0.65f);
    Fuzzy_Init(&FuzzyPD_Pos,
               0.025f, 0.060f, 2.40f,
               0.44f, 0.16f, 0.002f,
               0.46f, 0.18f, 0.010f,
               0.26f, 0.62f);
    PID_Init(&PID_Yaw, 18.00f, 8.00f, 0.35f, -PWM_LIMIT_YAW, PWM_LIMIT_YAW);

    /* 4. Khởi tạo các Task RTOS & ngoại vi */
    System_Tasks_Init();

    /* 5. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();

    while (1) {
    }
}
