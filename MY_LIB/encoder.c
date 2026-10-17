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
    /* Bật bộ lọc số phần cứng mức tối đa (IC1F = 0x0F, IC2F = 0x0F) triệt tiêu nhiễu gai EMI từ chổi than Motor */
    TIM4->CCMR1 |= (0x0Fu << 4) | (0x0Fu << 12);
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
    TIM3->CCMR1 |= (0x0Fu << 4) | (0x0Fu << 12);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight)
{
    if (motorLeft)
    {
        motorLeft->pulse = TIM3->CNT;
        int32_t dPulse = motorLeft->pulse - motorLeft->pre_pulse;

        if (dPulse > 32767)
            dPulse -= 65536;
        else if (dPulse < -32768)
            dPulse += 65536;

        /* Bảo vệ chống nhiễu vọt xung bất thường (> 16 xung/5ms ~ 145 RPM) */
        if (dPulse > 16 || dPulse < -16) {
            dPulse = motorLeft->delta_pulse;
        }
        motorLeft->delta_pulse = dPulse;

        motorLeft->speed = 60.0f * (float)motorLeft->delta_pulse /
                           (ENCODER_PULSES_PER_REV * ENCODER_SAMPLE_TIME * 4.0f);

        motorLeft->pre_pulse = motorLeft->pulse;
    }

    if (motorRight)
    {
        motorRight->pulse = TIM4->CNT;
        int32_t dPulse = motorRight->pulse - motorRight->pre_pulse;

        if (dPulse > 32767)
            dPulse -= 65536;
        else if (dPulse < -32768)
            dPulse += 65536;

        if (dPulse > 16 || dPulse < -16) {
            dPulse = motorRight->delta_pulse;
        }
        motorRight->delta_pulse = dPulse;

        motorRight->speed = 60.0f * (float)motorRight->delta_pulse /
                            (ENCODER_PULSES_PER_REV * ENCODER_SAMPLE_TIME * 4.0f);

        motorRight->pre_pulse = motorRight->pulse;
    }
}
