#include "encoder.h"

/* Dấu áp cho từng bánh (tính sẵn lúc biên dịch) */
#if ENCODER_LEFT_INVERT
  #define ENC_LEFT_SIGN   (-1)
#else
  #define ENC_LEFT_SIGN   (1)
#endif

#if ENCODER_RIGHT_INVERT
  #define ENC_RIGHT_SIGN  (-1)
#else
  #define ENC_RIGHT_SIGN  (1)
#endif

static void Encoder_TimerInit(TIM_TypeDef *TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_DeInit(TIMx);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* Encoder mode x4: đếm trên cả TI1 và TI2 */
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    /* Bộ lọc số cho IC1 (bit 7:4) và IC2 (bit 15:12) - code cũ để 0 (không lọc) */
    TIMx->CCMR1 &= (uint16_t)~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
    TIMx->CCMR1 |= (uint16_t)((ENCODER_INPUT_FILTER << 4) | (ENCODER_INPUT_FILTER << 12));

    TIM_SetCounter(TIMx, 0);
    TIM_Cmd(TIMx, ENABLE);
}

void Encoder_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    /* PB4, PB5 (TIM3 - trái) | PB6, PB7 (TIM4 - phải)
     * Lưu ý: PB4 mặc định là NJTRST -> chỉ dùng được khi nạp/debug bằng SWD */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;   /* Encoder Hall thường là open-collector */
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

    Encoder_TimerInit(TIM3);
    Encoder_TimerInit(TIM4);
}

static void Encoder_ResetOne(Encoder_t *e, TIM_TypeDef *TIMx)
{
    if (e == 0) return;
    e->last_cnt  = (uint16_t)TIMx->CNT;
    e->delta     = 0;
    e->position  = 0;
    e->speed_raw = 0.0f;
    e->speed     = 0.0f;
}

void Encoder_Reset(Encoder_t *left, Encoder_t *right)
{
    Encoder_ResetOne(left,  TIM3);
    Encoder_ResetOne(right, TIM4);
}

static void Encoder_UpdateOne(Encoder_t *e, TIM_TypeDef *TIMx, int32_t sign, float dt)
{
    uint16_t cnt;
    int16_t  d;

    if (e == 0) return;

    cnt = (uint16_t)TIMx->CNT;

    /* Phép trừ không dấu 16-bit rồi ép về có dấu => tự xử lý tràn 0xFFFF <-> 0
     * (đúng khi |delta| < 32768 count/chu kỳ, dư sức với motor này) */
    d = (int16_t)(uint16_t)(cnt - e->last_cnt);
    e->last_cnt = cnt;

    e->delta     = sign * (int32_t)d;
    e->position += e->delta;

    e->speed_raw = ((float)e->delta * 60.0f) / ((float)ENCODER_COUNTS_PER_REV * dt);
    e->speed    += ENCODER_SPEED_LPF_ALPHA * (e->speed_raw - e->speed);
}

void Encoder_Update(Encoder_t *left, Encoder_t *right, float dt)
{
    if (dt <= 0.0f) return;

    /* Đọc 2 timer sát nhau để 2 bánh cùng thời điểm lấy mẫu */
    Encoder_UpdateOne(left,  TIM3, ENC_LEFT_SIGN,  dt);
    Encoder_UpdateOne(right, TIM4, ENC_RIGHT_SIGN, dt);
}

void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight)
{
    Encoder_Update(motorLeft, motorRight, ENCODER_SAMPLE_TIME);
}

float Encoder_CountsToRev(int32_t counts)
{
    return (float)counts / (float)ENCODER_COUNTS_PER_REV;
}
