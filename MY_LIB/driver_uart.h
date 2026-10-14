#ifndef __DRIVER_UART_H
#define __DRIVER_UART_H

#include "stm32f4xx.h"
#include <stdint.h>

/* =====================================================================
 * CẤU HÌNH UART3 KẾT NỐI MÁY TÍNH / BLUETOOTH
 *  - TX: PD8 (AF7) -> DMA1 Stream 3, Channel 4
 *  - RX: PD9 (AF7) -> Ngắt phần cứng RXNE + Ring Buffer
 * ===================================================================== */
#define UART3_TX_PIN            GPIO_Pin_8
#define UART3_RX_PIN            GPIO_Pin_9
#define UART3_PORT              GPIOD
#define UART3_GPIO_CLK          RCC_AHB1Periph_GPIOD
#define UART3_CLK               RCC_APB1Periph_USART3

#define UART3_RX_BUFFER_SIZE    128u
#define UART3_TX_DMA_BUF_SIZE   256u

/* =====================================================================
 * ĐỊNH NGHĨA KHUNG GÓI TIN UART (PACKET PROTOCOL)
 * Cấu trúc: [0xAA] [ID] [len] [Data...] [Checksum] [0xFF]
 * Checksum: ID | len | Data (bitwise OR)
 * ===================================================================== */
#define UART_FRAME_HEADER       0xAA
#define UART_FRAME_TAIL         0xFF

/* Cấu trúc gói dữ liệu Telemetry chuẩn gửi lên máy tính (ID = 0x01, len = 32 byte) */
typedef struct __attribute__((packed)) {
    float rawPitch;          /* 1. Góc thô trực tiếp từ Gia tốc kế */
    float filteredPitch;     /* 2. Góc sau lọc Kalman */
    float gyroRate;          /* 3. Vận tốc góc thực tế (độ/s) */
    float motorSpeed_left;   /* 4. Vận tốc thực tế Bánh Trái từ Encoder (qua Kalman) */
    float motorSpeed_right;  /* 5. Vận tốc thực tế Bánh Phải từ Encoder (qua Kalman) */
    float motorPWM_left;     /* 6. PWM xuất Bánh Trái (-1000 đến 1000) */
    float motorPWM_right;    /* 7. PWM xuất Bánh Phải (-1000 đến 1000) */
    float targetAngle;       /* 8. Điểm cân bằng mục tiêu (độ) */
} RobotTelemetry_t;

void UART3_Init(uint32_t baudrate);
void UART3_DMA_Init(void);
uint8_t UART3_SendDMA(const void *data, uint16_t len);
uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData);
uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len);
uint16_t UART3_Available(void);
int16_t UART3_ReadChar(void);

#endif /* __DRIVER_UART_H */

