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

/* =====================================================================
 * CHẾ ĐỘ CHẠY (Chọn 1 trong các chế độ bên dưới):
 *  0 = Chạy hệ cân bằng đầy đủ (FreeRTOS + IMU + Fuzzy + PID tốc độ)
 *  1 = Test Motor   : quay 2 bánh với APP_TEST_PWM để kiểm tra chiều quay & vùng chết
 *  2 = Test Encoder : quay tay bánh xe hoặc chạy motor, quan sát t_spdL/t_spdR, t_posL/t_posR
 *  3 = Test IMU     : nghiêng xe, quan sát góc t_pitch, t_rate, t_ax, t_ay, t_az
 * ===================================================================== */
#define APP_MODE          0
#define APP_TEST_PWM      20.0f

/* Global Fuzzy Controller Instance */
FuzzyController_t fuzzyControl;

int main(void) {
    /* 1. Cấu hình NVIC Priority Group 4 bắt buộc cho FreeRTOS trên Cortex-M */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khởi tạo ngoại vi cơ bản */
    GPIO_Config();
    Motor_Config();

#if (APP_MODE == 1)
    /* -------------------------------------------------------------
     * TEST MOTOR: Cả 2 bánh phải quay TIẾN.
     * Nếu bánh nào quay lùi: đổi MOTOR_LEFT_INVERT / MOTOR_RIGHT_INVERT trong motor.h
     * ------------------------------------------------------------- */
    Motor_SetPWM(APP_TEST_PWM);
    while (1) {
    }
#else
    /* -------------------------------------------------------------
     * CHẾ ĐỘ 0: HỆ CÂN BẰNG TỰ ĐỘNG ĐẦY ĐỦ VỚI FREERTOS
     * ------------------------------------------------------------- */
    /* 3. Khởi tạo bộ điều khiển Fuzzy */
    Fuzzy_Init(&fuzzyControl);

    /* 4. Khởi tạo các Task RTOS & ngoại vi (giữ xe đứng yên ~1s khi cấp điện) */
    System_Tasks_Init();

    /* 5. Chạy bộ lập lịch FreeRTOS */
    vTaskStartScheduler();

    /* Nếu heap không đủ hoặc scheduler lỗi thì CPU mới rơi xuống đây */
    while (1) {
    }
#endif
}