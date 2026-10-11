#include "gpio.h"
#include "managetask.h"

/* Biến đếm số lần ngắt hợp lệ (sau khi đã lọc chống dội phím) */
volatile uint32_t exti_pc0_count = 0;
volatile uint32_t exti_pc1_count = 0;
volatile uint32_t exti_pc2_count = 0;

/**
 * @brief Chờ nhả phím, delay chống dội ở cuối vector và xóa cờ ngắt
 * @param GPIO_Pin Chân GPIO cần kiểm tra (GPIO_Pin_0 .. GPIO_Pin_2)
 * @param EXTI_Line Đường ngắt EXTI tương ứng (EXTI_Line0 .. EXTI_Line2)
 */
static void Button_Debounce_WaitRelease(uint16_t GPIO_Pin, uint32_t EXTI_Line) {
    /* 1. Đợi người dùng nhả nút (chân quay về mức HIGH = 1, timeout an toàn) */
    uint32_t timeout = 4000000;
    while ((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin) == Bit_RESET) && (--timeout));

    /* 2. Delay ở cuối vector để dập tắt dao động cơ học khi nhả nút */
    for (volatile uint32_t i = 0; i < 500000; i++);

    /* 3. Xóa cờ ngắt ở cuối sau khi tiếp điểm đã nhả hoàn toàn */
    EXTI_ClearITPendingBit(EXTI_Line);
}

/**
 * @brief Khởi tạo các chân GPIO cơ bản
 */
void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 1. Cấu hình LED trên kit Discovery: PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 2. Cấu hình ngắt ngoài cho 3 nút nhấn PC0..PC2 (đã bỏ chân PC3) */
    EXTI_Buttons_Config();
}

/**
 * @brief Cấu hình ngắt ngoài EXTI0..EXTI2 trên 3 chân PC0..PC2:
 *        - PC0: Nút START
 *        - PC1: Nút STOP
 *        - PC2: Nút CALIB THÔNG MINH TOÀN DIỆN (Phương án 2)
 */
void EXTI_Buttons_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. Cấp clock cho GPIOC và SYSCFG (SYSCFG nằm trên APB2) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* 2. Cấu hình PC0, PC1, PC2 là Input với điện trở kéo lên nội (Pull-up) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; /* Kéo lên 3.3V */
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 3. Ánh xạ các chân PC0..PC2 tới các đường ngắt EXTI0..EXTI2 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);

    /* 4. Cấu hình EXTI Line 0..2 kích hoạt sườn xuống (Falling Edge khi nhấn nút nối GND) */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0 | EXTI_Line1 | EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 5. Cấu hình NVIC cho EXTI0 -> EXTI2 (Priority = 6 an toàn cho FreeRTOS) */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;

    /* EXTI Line 0 (PC0 - Nút START) */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* EXTI Line 1 (PC1 - Nút STOP) */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* EXTI Line 2 (PC2 - Nút CALIB TOÀN DIỆN) */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

/* =====================================================================
 * CÁC HÀM PHỤC VỤ NGẮT NGOÀI (EXTI IRQ HANDLERS)
 * ===================================================================== */

/**
 * @brief NÚT 1 (PC0): START hệ thống cân bằng
 */
void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == Bit_RESET) {
            exti_pc0_count++;

            /* Kích hoạt lệnh START xe cân bằng */
            Robot_Start();

            /* Chờ nhả nút, delay chống dội ở cuối và xóa cờ ngắt */
            Button_Debounce_WaitRelease(GPIO_Pin_0, EXTI_Line0);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
    }
}

/**
 * @brief NÚT 2 (PC1): STOP / Dừng khẩn cấp động cơ
 */
void EXTI1_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == Bit_RESET) {
            exti_pc1_count++;

            /* Dừng khẩn cấp động cơ */
            Robot_Stop();

            Button_Debounce_WaitRelease(GPIO_Pin_1, EXTI_Line1);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line1);
        }
    }
}

/**
 * @brief NÚT 3 (PC2): CALIB THÔNG MINH TOÀN DIỆN (Phương án 2)
 *        - Bước 1: Tự nhận diện xe đứng yên -> Calib Gyro
 *        - Bước 2: Tự nhận diện xe dựng thẳng -> Chốt Zero Point
 *        - Bước 3: Lưu vào Flash Sector 11
 */
void EXTI2_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_RESET) {
            exti_pc2_count++;

            /* Chạy quy trình Calib tự động toàn diện */
            Robot_AutoCalibrate();

            Button_Debounce_WaitRelease(GPIO_Pin_2, EXTI_Line2);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line2);
        }
    }
}
