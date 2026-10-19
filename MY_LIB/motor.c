#include "motor.h"
#include <math.h>

/**
 * @brief Cấu hình PWM Timer 1 (20kHz, thang 8400 bước) và các chân GPIO điều khiển chiều
 * - Bánh trái:
 *    + PWM:   PA8  (TIM1_CH1)
 *    + Chiều: PC8, PC6
 * - Bánh phải:
 *    + PWM:   PE11 (TIM1_CH2)
 *    + Chiều: PC9, PC7
 */
void Motor_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* 1. Cấp clock cho GPIOA, GPIOC, GPIOE và TIM1 (APB2) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. Cấu hình chân PWM PA8 (TIM1_CH1 - Bánh trái) và PE11 (TIM1_CH2 - Bánh phải) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

    /* 3. Cấu hình các chân điều khiển chiều: Trái (PC8, PC6) | Phải (PC9, PC7) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);

    /* 4. Cấu hình TIM1 Time Base: 168MHz / (1 * 8400) = 20kHz */
    TIM_TimeBaseStructure.TIM_Period            = MOTOR_PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = MOTOR_PWM_PRESCALER - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 5. Cấu hình chế độ PWM1 cho Channel 1 (Bánh trái) và Channel 2 (Bánh phải) */
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void Motor_SetPWM_Left(float pwm) {
    if (pwm > (float)MOTOR_PWM_PERIOD)  pwm = (float)MOTOR_PWM_PERIOD;
    if (pwm < -(float)MOTOR_PWM_PERIOD) pwm = -(float)MOTOR_PWM_PERIOD;

#if MOTOR_LEFT_INVERT
    pwm = -pwm;
#endif

    if (pwm > 0.0f) {
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    } else if (pwm < 0.0f) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        GPIO_SetBits(GPIOC, GPIO_Pin_6);
    } else {
        GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_6);
    }

    uint16_t pulse = (uint16_t)(fabsf(pwm) + 0.5f);
    if (pulse > MOTOR_PWM_PERIOD) {
        pulse = MOTOR_PWM_PERIOD;
    }

    TIM1->CCR1 = pulse;
}

void Motor_SetPWM_Right(float pwm) {
    if (pwm > (float)MOTOR_PWM_PERIOD)  pwm = (float)MOTOR_PWM_PERIOD;
    if (pwm < -(float)MOTOR_PWM_PERIOD) pwm = -(float)MOTOR_PWM_PERIOD;

#if MOTOR_RIGHT_INVERT
    pwm = -pwm;
#endif

    if (pwm > 0.0f) {
        GPIO_SetBits(GPIOC, GPIO_Pin_9);
        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    } else if (pwm < 0.0f) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
    } else {
        GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_7);
    }

    uint16_t pulse = (uint16_t)(fabsf(pwm) + 0.5f);
    if (pulse > MOTOR_PWM_PERIOD) {
        pulse = MOTOR_PWM_PERIOD;
    }

    TIM1->CCR2 = pulse;
}

void Motor_SetPWM(float pwm) {
    Motor_SetPWM_Left(pwm);
    Motor_SetPWM_Right(pwm);
}

void Motor_Stop(void) {
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);
}
