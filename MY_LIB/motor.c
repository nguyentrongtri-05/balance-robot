#include "motor.h"
#include <math.h>

/* =====================================================================
 *  MÔ TẢ PHẦN CỨNG TỪNG MOTOR (gom lại để 2 bánh dùng chung 1 hàm)
 * ===================================================================== */
typedef struct
{
    uint8_t       channel;    /* 1 = TIM1_CH1, 2 = TIM1_CH2  */
    GPIO_TypeDef *dirPort;
    uint16_t      pinIn1;     /* Mức 1 khi chạy tiến         */
    uint16_t      pinIn2;     /* Mức 1 khi chạy lùi          */
    int8_t        invert;
    float         deadzone;
} MotorHw_t;

static const MotorHw_t s_motorLeft  = { 1u, GPIOC, GPIO_Pin_8, GPIO_Pin_6,
                                        MOTOR_LEFT_INVERT,  MOTOR_LEFT_DEADZONE  };
static const MotorHw_t s_motorRight = { 2u, GPIOC, GPIO_Pin_9, GPIO_Pin_7,
                                        MOTOR_RIGHT_INVERT, MOTOR_RIGHT_DEADZONE };

static volatile float s_lastLeft  = 0.0f;
static volatile float s_lastRight = 0.0f;

/**
 * @brief Cấu hình PWM Timer 1 và các chân GPIO điều khiển chiều
 * - Bánh trái:
 *    + PWM:   PA8  (TIM1_CH1)
 *    + Chiều: PC8, PC6
 * - Bánh phải:
 *    + PWM:   PE11 (TIM1_CH2)
 *    + Chiều: PC9, PC7
 */
void Motor_Config(void) {
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_BDTRInitTypeDef     TIM_BDTRInitStructure;

    /* 1. Cấp clock cho GPIOA, GPIOC, GPIOE và TIM1 (TIM1 nằm trên APB2) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. Chân chiều: Output Push-Pull, khởi tạo mức 0 (thả trôi) TRƯỚC khi bật PWM */
    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 3. TIM1 Time Base
     * Timer clock = APB2 * 2 = 168MHz (trên STM32F407)
     * Tần số PWM = 168MHz / (MOTOR_PWM_PRESCALER * MOTOR_PWM_PERIOD)
     *            = 168MHz / (8 * 1000) = 21kHz */
    TIM_TimeBaseStructure.TIM_Period            = MOTOR_PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = MOTOR_PWM_PRESCALER - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 4. PWM mode 1 cho CH1 (trái) và CH2 (phải), duty ban đầu = 0 */
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;   /* MOE=0 -> ngõ ra = 0 */
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* 5. BDTR: OSSI/OSSR = 1 => khi MOE = 0 ngõ ra bị ÉP về Idle (0) thay vì thả nổi.
     *    Code cũ để mặc định (OSSI=0) + pull-up trên chân PWM => khi MOE=0 hoặc
     *    debugger dừng CPU, chân EN có thể bị kéo lên 1 => motor chạy hết tốc. */
    TIM_BDTRStructInit(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime        = 0;
    TIM_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    /* 6. Dừng CPU ở breakpoint => TIM1 dừng và ngõ ra bị tắt (an toàn khi debug)
     *    Dùng bit 0 (DBG_TIM1_STOP) trực tiếp vì stm32f4xx.h của StdPeriph định nghĩa
     *    lại macro DBGMCU_APB2_FZ_DBG_TIM1_STOP thành một tên không tồn tại. */
    DBGMCU->APB2FZ |= (uint32_t)0x00000001;

    /* 7. Cho phép tự động nạp lại ARR, bật timer và ngõ ra chính (MOE) */
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* 8. Cuối cùng mới chuyển chân sang AF (timer đã chạy với duty 0 => không có xung rác)
     *    Pull-DOWN: nếu chân bị thả nổi, driver luôn nhận mức 0 (motor tắt) */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           /* PA8  -> TIM1_CH1 (trái) */
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;          /* PE11 -> TIM1_CH2 (phải) */
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
}

/* Đổi chiều bằng MỘT lần ghi BSRR => IN1/IN2 đổi đồng thời (code cũ ghi 2 lần,
 * giữa 2 lần ghi có thời điểm IN1 = IN2 = 1 gây phanh giật) */
static void Motor_SetDir(const MotorHw_t *m, int8_t dir)
{
    if (dir > 0)
    {
        m->dirPort->BSRR = (uint32_t)m->pinIn1 | ((uint32_t)m->pinIn2 << 16);
    }
    else if (dir < 0)
    {
        m->dirPort->BSRR = (uint32_t)m->pinIn2 | ((uint32_t)m->pinIn1 << 16);
    }
    else
    {
        m->dirPort->BSRR = ((uint32_t)(m->pinIn1 | m->pinIn2) << 16);
    }
}

static void Motor_SetCompare(const MotorHw_t *m, uint16_t pulse)
{
    if (m->channel == 1u)
    {
        TIM1->CCR1 = pulse;
    }
    else
    {
        TIM1->CCR2 = pulse;
    }
}

/* Áp lệnh cho 1 motor: giới hạn -> đảo chiều -> bù vùng chết -> PWM */
static float Motor_Apply(const MotorHw_t *m, float pwm)
{
    float    mag;
    uint16_t pulse;
    int8_t   dir;

    /* Chặn NaN (vd. bộ điều khiển chia 0) => dừng motor */
    if (pwm != pwm)
    {
        pwm = 0.0f;
    }

    /* Giới hạn giá trị PWM từ -100.0f đến 100.0f */
    if (pwm > 100.0f)  pwm = 100.0f;
    if (pwm < -100.0f) pwm = -100.0f;

    if (m->invert)
    {
        pwm = -pwm;
    }

    mag = fabsf(pwm);
    if (mag < MOTOR_CMD_EPSILON)
    {
        dir = 0;
        mag = 0.0f;
    }
    else
    {
        dir = (pwm > 0.0f) ? 1 : -1;
        if (m->deadzone > 0.0f)
        {
            /* Bù vùng chết LIÊN TỤC: dốc tuyến tính trong |u| < RAMP, sau đó tịnh tiến */
            float atRamp = m->deadzone + MOTOR_DZ_RAMP * (100.0f - m->deadzone) / 100.0f;
            if (mag < MOTOR_DZ_RAMP)
            {
                mag = mag * atRamp / MOTOR_DZ_RAMP;
            }
            else
            {
                mag = m->deadzone + mag * (100.0f - m->deadzone) / 100.0f;
            }
        }
    }

    /* Tính độ rộng xung tương ứng với Duty Cycle (0 - MOTOR_PWM_PERIOD) */
    pulse = (uint16_t)(mag * (float)MOTOR_PWM_PERIOD / 100.0f + 0.5f);
    if (pulse > MOTOR_PWM_PERIOD)
    {
        pulse = MOTOR_PWM_PERIOD;
    }

    Motor_SetDir(m, dir);
    Motor_SetCompare(m, pulse);

    return m->invert ? -pwm : pwm;
}

/**
 * @brief Điều khiển bánh trái (PA8: PWM, PC8 & PC6: Chiều)
 */
void Motor_SetPWM_Left(float pwm) {
    s_lastLeft = Motor_Apply(&s_motorLeft, pwm);
}

/**
 * @brief Điều khiển bánh phải (PE11: PWM, PC9 & PC7: Chiều)
 */
void Motor_SetPWM_Right(float pwm) {
    s_lastRight = Motor_Apply(&s_motorRight, pwm);
}

/**
 * @brief Điều khiển cả hai bánh xe cùng lúc
 */
void Motor_SetPWM(float pwm) {
    Motor_SetPWM_Left(pwm);
    Motor_SetPWM_Right(pwm);
}

void Motor_SetPWM_Diff(float pwm, float turn) {
    Motor_SetPWM_Left(pwm + turn);
    Motor_SetPWM_Right(pwm - turn);
}

/**
 * @brief Dừng cả hai bánh xe
 */
void Motor_Stop(void) {
    Motor_SetPWM_Left(0.0f);
    Motor_SetPWM_Right(0.0f);
}

void Motor_Brake(void) {
    GPIO_SetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
    TIM1->CCR1 = MOTOR_PWM_PERIOD;
    TIM1->CCR2 = MOTOR_PWM_PERIOD;
    s_lastLeft  = 0.0f;
    s_lastRight = 0.0f;
}

void Motor_Enable(FunctionalState state) {
    if (state == DISABLE)
    {
        TIM_CtrlPWMOutputs(TIM1, DISABLE);  /* Ngõ ra ép về 0 ngay lập tức (OSSI = 1) */
        Motor_Stop();
    }
    else
    {
        Motor_Stop();                       /* Bật lại luôn bắt đầu từ duty 0 */
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
    }
}

float Motor_GetLastPWM_Left(void)  { return s_lastLeft;  }
float Motor_GetLastPWM_Right(void) { return s_lastRight; }
