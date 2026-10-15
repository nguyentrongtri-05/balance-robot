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

/* Cấu trúc gói dữ liệu Telemetry chuẩn gửi lên máy tính (ID = 0x01, len = 60 byte = 15 float) */
typedef struct __attribute__((packed)) {
    /* 1. Dữ liệu từ vTaskIMU (Pitch & Yaw: Raw vs Kalman) */
    float rawPitch;                 /* 1. Góc Pitch thô từ Accelerometer (độ) */
    float filteredPitch;            /* 2. Góc Pitch sau lọc kalmanPitch (độ) */
    float gyroRate;                 /* 3. Vận tốc góc Pitch thực tế (độ/s) */
    float rawYaw;                   /* 4. Góc Yaw thô tích phân từ Gyro Z (độ) */
    float filteredYaw;              /* 5. Góc Yaw sau lọc kalmanYaw (độ) */
    float yawRate;                  /* 6. Vận tốc góc Yaw thực tế (độ/s) */

    /* 2. Dữ liệu từ vTaskEncoder (Raw Wheel, Kalman Wheel & Tọa độ Pos X, Y) */
    float motorSpeed_left;          /* 7. Vận tốc Raw Bánh Trái (RPM) */
    float motorSpeed_right;         /* 8. Vận tốc Raw Bánh Phải (RPM) */
    float motorSpeedFiltered_left;  /* 9. Vận tốc Kalman Bánh Trái (RPM) */
    float motorSpeedFiltered_right; /* 10. Vận tốc Kalman Bánh Phải (RPM) */
    float robotX;                   /* 11. Tọa độ Pos X trên mặt phẳng (cm) */
    float robotY;                   /* 12. Tọa độ Pos Y trên mặt phẳng (cm) */

    /* 3. Dữ liệu từ vTaskControl (PWM & Góc đặt) */
    float motorPWM_left;            /* 13. PWM xuất Bánh Trái (-1000 đến 1000) */
    float motorPWM_right;           /* 14. PWM xuất Bánh Phải (-1000 đến 1000) */
    float targetAngle;              /* 15. Điểm cân bằng mục tiêu (độ) */
} RobotTelemetry_t;

/* Cấu trúc gói đồng bộ Bộ Số Cấu Hình Gốc từ STM32 gửi lên máy tính khi khởi động / Reset (ID = 0x02, len = 48 byte = 12 float) */
typedef struct __attribute__((packed)) {
    float kp_balance;               /* 1. Hệ số Kp vòng cân bằng Pitch */
    float kd_balance;               /* 2. Hệ số Kd vòng cân bằng Pitch */
    float kp_position;              /* 3. Hệ số Kp vòng vị trí / vận tốc */
    float ki_position;              /* 4. Hệ số Ki vòng tọa độ 2D (X, Y) */
    float kd_yaw;                   /* 5. Hệ số Kd ghìm lệch vận tốc 2 bánh Encoder */
    float kp_yaw;                   /* 6. Hệ số Kp khóa cứng góc Yaw */
    float kd_yaw_imu;               /* 7. Hệ số Kd phanh vận tốc xoay ngang Gyro Z IMU */
    float targetAngle;              /* 8. Góc cân bằng mục tiêu (từ Code / Flash) */
    float targetX;                  /* 9. Tọa độ đích X (cm) */
    float targetY;                  /* 10. Tọa độ đích Y (cm) */
    float targetYaw;                /* 11. Góc hướng đích Yaw (độ) */
    float controlMode;              /* 12. Chế độ điều khiển (0.0f = Dual PID, 1.0f = Fuzzy) */
} RobotConfigSync_t;

void UART3_Init(uint32_t baudrate);
void UART3_DMA_Init(void);
uint8_t UART3_SendDMA(const void *data, uint16_t len);
uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData);
uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len);
uint16_t UART3_Available(void);
int16_t UART3_ReadChar(void);

#endif /* __DRIVER_UART_H */

