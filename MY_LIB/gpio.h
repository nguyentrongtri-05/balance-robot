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

/* =====================================================================
 *  PHÍM NGẮT NGOÀI PC0..PC3 (EXTI0..EXTI3)
 *  Nối: PCx ── nút ── GND. Kéo lên nội (pull-up) => nhấn = mức 0, ngắt CẠNH XUỐNG.
 *  (Tùy chọn: thêm tụ 100 nF từ PCx xuống GND để lọc dội phần cứng)
 *
 *  ISR chỉ ghi nhận "có cạnh xuống" và TẠM KHÓA đường EXTI đó (không gọi API FreeRTOS,
 *  không bị dội nhấn liên tục). Keys_Poll() (gọi mỗi 10 ms trong task) xác nhận phím
 *  thật sự được giữ, đo thời gian giữ, chờ nhả hẳn rồi mới mở lại ngắt.
 * ===================================================================== */
typedef enum {
    KEY_CALIB  = 0,   /* PC0: nhấn = hiệu chuẩn ACCEL6, giữ 2 s = xóa hiệu chuẩn      */
    KEY_ZERO   = 1,   /* PC1: set zero (xe NẰM yên) - bias gyro + reset vị trí/target */
    KEY_TARGET = 2,   /* PC2: set target - lấy góc hiện tại làm góc cân bằng          */
    KEY_SPARE  = 3,   /* PC3: dự phòng                                                */
    KEY_COUNT  = 4
} KeyId_t;

/** Cấu hình PC0..PC3 + EXTI0..3 + NVIC. Được gọi sẵn trong GPIO_Config(). */
void Keys_Config(void);

/**
 * @brief Xử lý phím, gọi ĐỀU ĐẶN mỗi 10 ms.
 * @param evt: mảng KEY_COUNT phần tử, nhận BTN_EVT_NONE / SHORT / LONG cho từng phím
 */
void Keys_Poll(ButtonEvent_t evt[KEY_COUNT]);

/** Số lần ngắt của phím k (debug: đếm cả dội - so sánh với số lần nhấn thật) */
uint32_t Keys_GetIrqCount(KeyId_t k);

#endif /* __GPIO_H */
