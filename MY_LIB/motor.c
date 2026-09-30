#include "motor.h"
#include <math.h>

/**
 * @brief Cấu hình PWM Timer 1 và các chân GPIO điều khiển chiều
 * - Bánh trái:
 *    + PWM:   PA8 (TIM1_CH1)
 *    + Chiều: PC8, PC6
 * - Bánh phải:
 *    + PWM:   PA9 (TIM1_CH2)
 *    + Chiều: PC9, PC7
 */
void Motor_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* 1. Cấp clock cho GPIOA, GPIOC và TIM1 (TIM1 nằm trên APB2) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. Cấu hình chân PWM (PA8 - TIM1_CH1, PA9 - TIM1_CH2) chế độ Alternate Function */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
				// Cấp clock GPIOE
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

		// Cấu hình PE11
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		// Ánh xạ PE11 sang TIM1_CH2
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    /* Ánh xạ chân sang TIM1 Alternate Function */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

    /* 3. Cấu hình các chân điều khiển chiều: Output Push-Pull */
    /* Do PA8/PA9 làm PWM nên đẩy 2 chân chiều sang PC6/PC7 đang rảnh rỗi */
    /* Trái: PC8, PC6 | Phải: PC9, PC7 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Khởi tạo mức logic ban đầu: Tắt hết chân chiều */
    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);

    /* 4. Cấu hình TIM1 Time Base */
    /* Timer clock = APB2 * 2 = 168MHz (trên STM32F407)
     * Tần số PWM = 168MHz / (MOTOR_PWM_PRESCALER * MOTOR_PWM_PERIOD)
     *            = 168MHz / (8 * 1000) = 21kHz */
    TIM_TimeBaseStructure.TIM_Period        = MOTOR_PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = MOTOR_PWM_PRESCALER - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 5. Cấu hình chế độ PWM cho Channel 1 (Bánh trái) và Channel 2 (Bánh phải) */
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = 0; /* Khởi đầu Duty Cycle = 0 */
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* Kênh 1: PA8 (Bánh trái) */
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Kênh 2: PA9 (Bánh phải) */
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* CHÚ Ý: TIM1 là Advanced Timer nên bắt buộc phải gọi hàm này để xuất tín hiệu PWM */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* Cho phép tự động nạp lại thanh ghi ARR và bật Timer 1 */
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief Điều khiển bánh trái (PA8: PWM, PC8 & PC6: Chiều)
 */
void Motor_SetPWM_Left(float pwm) {
    /* Giới hạn giá trị PWM từ -100.0f đến 100.0f */
    if (pwm > 100.0f)  pwm = 100.0f;
    if (pwm < -100.0f) pwm = -100.0f;

#if MOTOR_LEFT_INVERT
    pwm = -pwm;
#endif

    /* Điều khiển chiều quay qua PC8 và PC6 */
    if (pwm > 0.0f) {
        /* Chạy tiến */
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    } else if (pwm < 0.0f) {
        /* Chạy lùi */
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        GPIO_SetBits(GPIOC, GPIO_Pin_6);
    } else {
        /* Dừng */
        GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_6);
    }

    /* Tính độ rộng xung tương ứng với Duty Cycle (0 - MOTOR_PWM_PERIOD) */
    uint16_t pulse = (uint16_t)((fabsf(pwm) / 100.0f) * (float)MOTOR_PWM_PERIOD);
    if (pulse > MOTOR_PWM_PERIOD) {
        pulse = MOTOR_PWM_PERIOD;
    }

    TIM1->CCR1 = pulse; /* Cập nhật PWM PA8 */
}

/**
 * @brief Điều khiển bánh phải (PA9: PWM, PC9 & PC7: Chiều)
 */
void Motor_SetPWM_Right(float pwm) {
    /* Giới hạn giá trị PWM từ -100.0f đến 100.0f */
    if (pwm > 100.0f)  pwm = 100.0f;
    if (pwm < -100.0f) pwm = -100.0f;

#if MOTOR_RIGHT_INVERT
    pwm = -pwm;
#endif

    /* Điều khiển chiều quay qua PC9 và PC7 */
    if (pwm > 0.0f) {
        /* Chạy tiến */
        GPIO_SetBits(GPIOC, GPIO_Pin_9);
        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    } else if (pwm < 0.0f) {
        /* Chạy lùi */
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
    } else {
        /* Dừng */
        GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_7);
    }

    /* Tính độ rộng xung tương ứng với Duty Cycle (0 - MOTOR_PWM_PERIOD) */
    uint16_t pulse = (uint16_t)((fabsf(pwm) / 100.0f) * (float)MOTOR_PWM_PERIOD);
    if (pulse > MOTOR_PWM_PERIOD) {
        pulse = MOTOR_PWM_PERIOD;
    }

    TIM1->CCR2 = pulse; /* Cập nhật PWM PA9 */
}

/**
 * @brief Điều khiển cả hai bánh xe cùng lúc
 */
void Motor_SetPWM(float pwm) {
    Motor_SetPWM_Left(pwm);
    Motor_SetPWM_Right(pwm);
}

/**
 * @brief Dừng cả hai bánh xe
 */
void Motor_Stop(void) {
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);
}
