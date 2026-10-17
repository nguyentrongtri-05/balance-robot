#include "gpio.h"
#include "managetask.h"

static void Button_Debounce_WaitRelease(uint16_t GPIO_Pin, uint32_t EXTI_Line) {
    uint32_t timeout = 4000000;
    while ((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin) == Bit_RESET) && (--timeout));
    for (volatile uint32_t i = 0; i < 500000; i++);
    EXTI_ClearITPendingBit(EXTI_Line);
}

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

    /* 2. Cấu hình ngắt ngoài cho 3 nút nhấn PC0..PC2 */
    EXTI_Buttons_Config();
}

void EXTI_Buttons_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);

    EXTI_InitStructure.EXTI_Line    = EXTI_Line0 | EXTI_Line1 | EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

/* NÚT 1 (PC0): START */
void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == Bit_RESET) {
            Robot_Start();
            Button_Debounce_WaitRelease(GPIO_Pin_0, EXTI_Line0);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
    }
}

/* NÚT 2 (PC1): STOP */
void EXTI1_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == Bit_RESET) {
            Robot_Stop();
            Button_Debounce_WaitRelease(GPIO_Pin_1, EXTI_Line1);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line1);
        }
    }
}

/* NÚT 3 (PC2): CALIB (Đặt cờ yêu cầu để chạy Calib trong Task FreeRTOS, tránh treo/lỗi trong ngắt EXTI) */
void EXTI2_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_RESET) {
            Robot_RequestCalibrate();
            Button_Debounce_WaitRelease(GPIO_Pin_2, EXTI_Line2);
        } else {
            EXTI_ClearITPendingBit(EXTI_Line2);
        }
    }
}

