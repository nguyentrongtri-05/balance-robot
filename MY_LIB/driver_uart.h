#ifndef __DRIVER_UART_H
#define __DRIVER_UART_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

/* =====================================================================
 * CẤU HÌNH UART3 KẾT NỐI MÁY TÍNH
 *  - TX: PD8 (AF7) -> DMA1 Stream 3, Channel 4
 *  - RX: PD9 (AF7) -> Ngắt phần cứng RXNE + Ring Buffer
 *  - Baudrate mặc định: 115200 bps, 8-bit data, 1 stop bit, no parity
 * ===================================================================== */
#define UART3_TX_PIN            GPIO_Pin_8
#define UART3_RX_PIN            GPIO_Pin_9
#define UART3_PORT              GPIOD
#define UART3_GPIO_CLK          RCC_AHB1Periph_GPIOD
#define UART3_CLK               RCC_APB1Periph_USART3
#define UART3_DEFAULT_BAUD      115200u

#define UART3_RX_BUFFER_SIZE    128u
#define UART3_TX_DMA_BUF_SIZE   256u

/* =====================================================================
 * ĐỊNH NGHĨA KHUNG GÓI TIN UART (PACKET PROTOCOL)
 * Cấu trúc: [0xAA] [ID] [len] [Data...] [Checksum] [0xFF]
 * Checksum: ID | len | Data (bitwise OR)
 * ===================================================================== */
#define UART_FRAME_HEADER       0xAA
#define UART_FRAME_TAIL         0xFF

/* Chế độ tính Checksum */
#define UART_CS_MODE_OR         1   /* ID | len | Data (bitwise OR theo yêu cầu) */
#define UART_CS_MODE_XOR        2   /* ID ^ len ^ Data (XOR) */
#define UART_CS_MODE_SUM        3   /* ID + len + Data (Cộng tổng byte) */

#define UART_CS_CURRENT_MODE    UART_CS_MODE_OR

/* Chế độ gửi dữ liệu Telemetry trong Task UART:
 * 0: Gói nhị phân đóng gói: 0xAA ID len Data Checksum 0xFF (dùng cho Python / App PC)
 * 1: Chuỗi ASCII CSV: "Raw:%.2f,Kalman:%.2f\r\n" (dùng cho Arduino Serial Plotter)
 */
#define UART_TELEMETRY_MODE     0   /* Mặc định: Gói tin nhị phân 0xAA...0xFF */

/* Cấu trúc gói dữ liệu Telemetry chuẩn gửi lên máy tính (ID = 0x01, len = 24 byte) */
typedef struct __attribute__((packed)) {
    float rawPitch;      /* 1. Góc thô trực tiếp từ Gia tốc kế (KHÔNG BỘ LỌC) */
    float filteredPitch; /* 2. Góc sau lọc Kalman (CÓ BỘ LỌC) */
    float gyroRate;      /* 3. Vận tốc góc thực tế (độ/s) */
    float motorSpeed;    /* 4. Vận tốc xe từ Encoder */
    float motorPWM;      /* 5. PWM động cơ đang xuất (-100 đến 100) */
    float targetAngle;   /* 6. Điểm cân bằng mục tiêu (độ) */
} RobotTelemetry_t;

/**
 * @brief Khởi tạo USART3 tại chân PD8 (TX) và PD9 (RX), kèm DMA TX và ngắt RX
 * @param baudrate: Tốc độ baud (ví dụ 115200)
 */
void UART3_Init(uint32_t baudrate);

/**
 * @brief Cấu hình DMA1 Stream 3 Channel 4 cho USART3 TX
 */
void UART3_DMA_Init(void);

/**
 * @brief Kiểm tra xem DMA TX có đang bận gửi dữ liệu không
 * @retval 1 nếu đang bận, 0 nếu rảnh
 */
uint8_t UART3_DMA_IsBusy(void);

/**
 * @brief Gửi dữ liệu qua DMA (Non-blocking: CPU giải phóng ngay lập tức)
 * @param data: Con trỏ vùng nhớ chứa dữ liệu
 * @param len: Số byte cần gửi
 * @retval 1 nếu bắt đầu gửi thành công, 0 nếu DMA đang bận gói trước
 */
uint8_t UART3_SendDMA(const void *data, uint16_t len);

/**
 * @brief In chuỗi có định dạng qua DMA (Non-blocking)
 */
uint8_t UART3_PrintfDMA(const char *fmt, ...);

/**
 * @brief Tính Checksum cho gói tin theo chế độ đã cấu hình
 * @param id: Mã ID gói
 * @param len: Chiều dài Data (số byte)
 * @param pData: Con trỏ mảng Data
 * @retval Giá trị Checksum 1 byte
 */
uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData);

/**
 * @brief Đóng gói và gửi qua DMA: [0xAA] [ID] [len] [Data] [Checksum] [0xFF]
 * @param id: Mã định danh gói (1 byte)
 * @param pData: Con trỏ dữ liệu payload
 * @param len: Chiều dài payload (byte)
 * @retval 1 nếu bắt đầu gửi thành công, 0 nếu DMA đang bận
 */
uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len);

/**
 * @brief Đóng gói và gửi trực tiếp (Blocking): [0xAA] [ID] [len] [Data] [Checksum] [0xFF]
 */
void UART3_SendPacket(uint8_t id, const void *pData, uint8_t len);

/**
 * @brief Gửi 1 ký tự qua UART (Blocking)
 */
void UART3_SendChar(char c);

/**
 * @brief Gửi chuỗi ký tự qua UART (Blocking)
 */
void UART3_SendString(const char *str);

/**
 * @brief Gửi mảng byte nhị phân qua UART (Blocking)
 */
void UART3_SendBytes(const uint8_t *data, uint16_t len);

/**
 * @brief In có định dạng gửi qua UART (Blocking)
 */
void UART3_Printf(const char *fmt, ...);

/**
 * @brief Kiểm tra số lượng byte đang có sẵn trong bộ đệm nhận RX
 * @retval Số byte có thể đọc
 */
uint16_t UART3_Available(void);

/**
 * @brief Đọc 1 byte từ bộ đệm nhận RX
 * @retval Byte nhận được (0-255), hoặc -1 nếu bộ đệm trống
 */
int16_t UART3_ReadChar(void);

/**
 * @brief Đọc 1 dòng từ bộ đệm nhận (kết thúc bằng '\n' hoặc '\r')
 * @param buf: Vùng nhớ lưu chuỗi nhận
 * @param maxLen: Kích thước tối đa của buf
 * @retval Số ký tự đọc được (0 nếu chưa có đủ 1 dòng)
 */
uint16_t UART3_ReadLine(char *buf, uint16_t maxLen);

#endif /* __DRIVER_UART_H */
