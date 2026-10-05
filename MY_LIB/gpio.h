#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"

/* Sự kiện nút USER (B1 - PA0 trên STM32F4-Discovery, nhấn = mức 1) */
typedef enum {
    BTN_EVT_NONE  = 0,
    BTN_EVT_SHORT = 1,   /* Nhấn rồi nhả trong 50 ms .. 1.5 s       */
    BTN_EVT_LONG  = 2    /* Giữ >= 2 s (báo 1 lần ngay khi đủ 2 s)  */
} ButtonEvent_t;

/**
 * @brief Initialize basic GPIO pins (LED PD12..PD15 + nút USER PA0)
 */
void GPIO_Config(void);

/**
 * @brief Đọc nút có chống dội. Gọi ĐỀU ĐẶN mỗi 10 ms.
 */
ButtonEvent_t Button_Poll(void);

#endif /* __GPIO_H */
