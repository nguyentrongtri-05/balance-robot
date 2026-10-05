#include "gpio.h"

#define BTN_PERIOD_MS      10u
#define BTN_DEBOUNCE_MS    50u
#define BTN_SHORT_MAX_MS   1500u
#define BTN_LONG_MS        2000u

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE);

    /* LED: PD12 xanh lá, PD13 cam, PD14 đỏ, PD15 xanh dương */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Nút USER B1: PA0, nhấn = 1 (board đã có điện trở kéo xuống) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

ButtonEvent_t Button_Poll(void) {
    static uint16_t heldMs = 0u;
    static uint8_t  longSent = 0u;
    ButtonEvent_t evt = BTN_EVT_NONE;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET)
    {
        if (heldMs < 0xFFFFu - BTN_PERIOD_MS) heldMs += BTN_PERIOD_MS;
        if ((heldMs >= BTN_LONG_MS) && !longSent)
        {
            longSent = 1u;
            evt = BTN_EVT_LONG;
        }
    }
    else
    {
        if (!longSent && (heldMs >= BTN_DEBOUNCE_MS) && (heldMs <= BTN_SHORT_MAX_MS))
        {
            evt = BTN_EVT_SHORT;
        }
        heldMs = 0u;
        longSent = 0u;
    }
    return evt;
}
