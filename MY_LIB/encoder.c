#include "encoder.h"

/* Đếm xung encoder của 2 bánh */
int32_t encoder_left_count = 0;
int32_t encoder_right_count = 0;

/* Lưu các giá trị trước đó để làm delta */
int32_t prev_encoder_left = 0;
int32_t prev_encoder_right = 0;

void Encoder_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* ----- PB6, PB7 -> TIM4 (right wheel) ----- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

    /* ----- PB4, PB5 -> TIM3 (left wheel) ----- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight)
{
    if (motorLeft)
    {
        motorLeft->pulse = TIM3->CNT;
        motorLeft->delta_pulse = motorLeft->pulse - motorLeft->pre_pulse;

        if (motorLeft->delta_pulse > 32767)
            motorLeft->delta_pulse -= 65536;
        else if (motorLeft->delta_pulse < -32768)
            motorLeft->delta_pulse += 65536;

        motorLeft->speed = 60.0f * (float)motorLeft->delta_pulse /
                           (ENCODER_PULSES_PER_REV * ENCODER_SAMPLE_TIME * 4.0f);

        motorLeft->pre_pulse = motorLeft->pulse;
    }

    if (motorRight)
    {
        motorRight->pulse = TIM4->CNT;
        motorRight->delta_pulse = motorRight->pulse - motorRight->pre_pulse;

        if (motorRight->delta_pulse > 32767)
            motorRight->delta_pulse -= 65536;
        else if (motorRight->delta_pulse < -32768)
            motorRight->delta_pulse += 65536;

        motorRight->speed = 60.0f * (float)motorRight->delta_pulse /
                            (ENCODER_PULSES_PER_REV * ENCODER_SAMPLE_TIME * 4.0f);

        motorRight->pre_pulse = motorRight->pulse;
    }
}
