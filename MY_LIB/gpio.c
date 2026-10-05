#include "gpio.h"

#define BTN_PERIOD_MS      10u
#define BTN_DEBOUNCE_MS    50u
#define BTN_SHORT_MAX_MS   1500u
#define BTN_LONG_MS        2000u

/* ---- Phím ngắt ngoài PC0..PC3 ---- */
#define KEY_PORT           GPIOC
#define KEY_PINS           (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)
#define KEY_EXTI_MASK      0x0000000Fu   /* EXTI line 0..3                                   */
#define KEY_IRQ_PRIO       10u           /* Thấp hơn configMAX_SYSCALL (5) => an toàn với RTOS */
#define KEY_CONFIRM_MS     30u           /* Sau ngắt, phím phải còn giữ 30 ms mới tính là nhấn */
#define KEY_RELEASE_MS     50u           /* Phải nhả liên tục 50 ms mới mở lại ngắt           */

typedef enum {
    KS_ARMED = 0,      /* Đang chờ ngắt cạnh xuống                       */
    KS_CONFIRM,        /* Có ngắt: kiểm tra phím còn giữ (lọc dội/nhiễu) */
    KS_PRESSED,        /* Đã xác nhận nhấn: đo thời gian giữ             */
    KS_RELEASE         /* Chờ nhả hẳn rồi mở lại ngắt                    */
} KeyState_t;

typedef struct {
    uint8_t  state;
    uint8_t  longSent;
    uint16_t ms;
} KeyCtx_t;

/* Mỗi phím 1 byte riêng => ISR và task ghi không đè lên nhau (không cần bitmask RMW) */
static volatile uint8_t  s_keyPending[KEY_COUNT];
static volatile uint32_t s_keyIrqCnt[KEY_COUNT];
static KeyCtx_t          s_key[KEY_COUNT];

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

    /* Phím chức năng PC0..PC3 (ngắt ngoài) */
    Keys_Config();
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

/* =====================================================================
 *  PHÍM NGẮT NGOÀI PC0..PC3
 *  Ghi chú: stm32f4xx_exti.c không có trong project => thao tác thanh ghi EXTI trực tiếp.
 * ===================================================================== */
static uint8_t Key_IsDown(uint8_t k)
{
    return ((KEY_PORT->IDR & (1u << k)) == 0u) ? 1u : 0u;   /* Pull-up: nhấn = 0 */
}

/* Mở lại ngắt cho phím k. IMR bị ISR sửa (RMW) => khóa ngắt ngắn để không mất bit. */
static void Key_Arm(uint8_t k)
{
    const uint32_t line = 1u << k;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    EXTI->PR = line;               /* Xóa cạnh cũ (ghi 1 để xóa)       */
    s_keyPending[k] = 0u;
    EXTI->IMR |= line;
    if (Key_IsDown(k))
    {
        /* Bị nhấn đúng lúc đang mở ngắt => cạnh xuống đã qua, tự ghi nhận */
        EXTI->IMR &= ~line;
        s_keyPending[k] = 1u;
    }
    __set_PRIMASK(primask);
}

void Keys_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    const IRQn_Type irq[KEY_COUNT] = { EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn };
    uint8_t k;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* 1. PC0..PC3: input, kéo lên nội */
    GPIO_InitStructure.GPIO_Pin   = KEY_PINS;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(KEY_PORT, &GPIO_InitStructure);

    /* 2. Nối EXTI0..3 vào port C (=> PA0 KHÔNG dùng được EXTI0 nữa, nút USER vẫn đọc polling) */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);

    /* 3. Ngắt cạnh xuống; để MASK cho tới khi Keys_Poll() thấy phím đã nhả ổn định */
    EXTI->IMR  &= ~KEY_EXTI_MASK;
    EXTI->EMR  &= ~KEY_EXTI_MASK;
    EXTI->RTSR &= ~KEY_EXTI_MASK;
    EXTI->FTSR |=  KEY_EXTI_MASK;
    EXTI->PR    =  KEY_EXTI_MASK;

    for (k = 0u; k < KEY_COUNT; k++)
    {
        s_keyPending[k]  = 0u;
        s_keyIrqCnt[k]   = 0u;
        s_key[k].state    = KS_RELEASE;   /* Phải thấy nhả 50 ms rồi mới bật ngắt (chống kẹt phím lúc khởi động) */
        s_key[k].longSent = 0u;
        s_key[k].ms       = 0u;

        /* 4. NVIC (NVIC_PriorityGroup_4 => 4 bit preemption, 0..15) */
        NVIC_ClearPendingIRQ(irq[k]);
        NVIC_SetPriority(irq[k], KEY_IRQ_PRIO);
        NVIC_EnableIRQ(irq[k]);
    }
}

void Keys_Poll(ButtonEvent_t evt[KEY_COUNT])
{
    uint8_t k;

    for (k = 0u; k < KEY_COUNT; k++)
    {
        KeyCtx_t *c = &s_key[k];
        uint8_t down = Key_IsDown(k);

        evt[k] = BTN_EVT_NONE;

        switch (c->state)
        {
        case KS_ARMED:
            if (s_keyPending[k])
            {
                c->state = KS_CONFIRM;
                c->ms    = 0u;
            }
            break;

        case KS_CONFIRM:
            if (down)
            {
                c->ms += BTN_PERIOD_MS;
                if (c->ms >= KEY_CONFIRM_MS)
                {
                    c->state    = KS_PRESSED;
                    c->longSent = 0u;
                }
            }
            else
            {
                c->state = KS_RELEASE;   /* Chỉ là dội / nhiễu => bỏ qua */
                c->ms    = 0u;
            }
            break;

        case KS_PRESSED:
            if (down)
            {
                if (c->ms < (0xFFFFu - BTN_PERIOD_MS)) c->ms += BTN_PERIOD_MS;
                if (!c->longSent && (c->ms >= BTN_LONG_MS))
                {
                    c->longSent = 1u;
                    evt[k] = BTN_EVT_LONG;       /* Báo ngay khi đủ 2 s, không chờ nhả */
                }
            }
            else
            {
                if (!c->longSent) evt[k] = BTN_EVT_SHORT;
                c->state = KS_RELEASE;
                c->ms    = 0u;
            }
            break;

        case KS_RELEASE:
        default:
            if (!down)
            {
                c->ms += BTN_PERIOD_MS;
                if (c->ms >= KEY_RELEASE_MS)
                {
                    c->state = KS_ARMED;
                    c->ms    = 0u;
                    Key_Arm(k);
                }
            }
            else
            {
                c->ms = 0u;
            }
            break;
        }
    }
}

uint32_t Keys_GetIrqCount(KeyId_t k)
{
    return (k < KEY_COUNT) ? s_keyIrqCnt[k] : 0u;
}

/* ---- ISR: chỉ ghi nhận + khóa đường ngắt (không gọi API FreeRTOS) ---- */
static void Key_Isr(uint8_t k)
{
    const uint32_t line = 1u << k;

    if (EXTI->PR & line)
    {
        EXTI->PR   = line;        /* Xóa cờ, ghi 1 để xóa */
        EXTI->IMR &= ~line;       /* Khóa tới khi Keys_Poll() xử lý xong => dội không gây bão ngắt */
        s_keyPending[k] = 1u;
        s_keyIrqCnt[k]++;
    }
}

void EXTI0_IRQHandler(void) { Key_Isr(0u); }
void EXTI1_IRQHandler(void) { Key_Isr(1u); }
void EXTI2_IRQHandler(void) { Key_Isr(2u); }
void EXTI3_IRQHandler(void) { Key_Isr(3u); }
