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
 * Checksum: ID ^ len ^ Data (bitwise XOR)
 * ===================================================================== */
#define UART_FRAME_HEADER       0xAA
#define UART_FRAME_TAIL         0xFF

/* Cấu trúc gói dữ liệu Telemetry chuẩn gửi lên máy tính (ID = 0x01, len = 88 byte = 22 float) */
typedef struct __attribute__((packed)) {
    /* 1. Dữ liệu từ vTaskIMU (Pitch & Yaw: Raw vs Kalman) */
    float rawPitch;                 /* 1. Góc Pitch thô từ Accelerometer (độ) */
    float filteredPitch;            /* 2. Góc Pitch sau lọc kalmanPitch (độ) */
    float gyroRate;                 /* 3. Vận tốc góc Pitch thực tế (độ/s) */
    float rawYaw;                   /* 4. Góc Yaw tích phân trực tiếp từ Gyro Z IMU (độ) */
    float filteredYaw;              /* 5. Góc Yaw sau lọc kalmanYaw thuần IMU (độ) */
    float yawRate;                  /* 6. Vận tốc góc Yaw thực tế từ IMU (độ/s) */

    /* 2. Dữ liệu từ vTaskEncoder (Raw Wheel & Kalman Wheel) */
    float motorSpeed_left;          /* 7. Vận tốc Raw Bánh Trái (RPM) */
    float motorSpeed_right;         /* 8. Vận tốc Raw Bánh Phải (RPM) */
    float motorSpeedFiltered_left;  /* 9. Vận tốc Kalman Bánh Trái (RPM) */
    float motorSpeedFiltered_right; /* 10. Vận tốc Kalman Bánh Phải (RPM) */

    /* 3. Dữ liệu từ vTaskControl (Kiến trúc Vòng Lồng Cascade: 90% OutFuzzy + 10% Yaw) */
    float motorPWM_left;            /* 11. PWM xuất Bánh Trái (-8400 đến +8400) */
    float motorPWM_right;           /* 12. PWM xuất Bánh Phải (-8400 đến +8400) */
    float targetAngle;              /* 13. Góc cân bằng đặt thực tế sau Vòng Lồng Cascade (độ) */
    float outFuzzy;                 /* 14. Ngõ ra điện áp Vòng Trong Cân bằng FuzzyPD_Pitch (tối đa 90% = +-10.8V) */
    float outYaw;                   /* 15. Ngõ ra Vòng Góc Yaw (tối đa 10% = +-840 PWM) */
    float outSpeed;                 /* 16. Góc bù Vòng Ngoài Vị trí FuzzyPD_Pos (độ, giới hạn +-3.2°) */

    /* 4. Dữ liệu debug 2 khâu FuzzyPD nối tiếp (FuzzyPD_Pitch & FuzzyPD_Pos) */
    float velCmd;                   /* 17. Vận tốc đặt đưa vào vòng ngoài (RPM, dấu theo encoder) */
    float dVel;                     /* 18. Gia tốc góc bánh dVel = d(vel)/dt (RPM/s) */
    float pitchENorm;               /* 19. FuzzyPD_Pitch (Vòng trong): e đã chuẩn hóa & bão hòa [-1..+1] */
    float pitchDeNorm;              /* 20. FuzzyPD_Pitch (Vòng trong): de đã chuẩn hóa & bão hòa [-1..+1] */
    float posENorm;                 /* 21. FuzzyPD_Pos (Vòng ngoài): e đã chuẩn hóa & bão hòa [-1..+1] */
    float posDeNorm;                /* 22. FuzzyPD_Pos (Vòng ngoài): de đã chuẩn hóa & bão hòa [-1..+1] */
} RobotTelemetry_t;

/* Cấu trúc gói đồng bộ Bộ Số Cấu Hình Gốc từ STM32 gửi lên máy tính khi khởi động / Reset (ID = 0x02, len = 48 byte = 12 float) */
typedef struct __attribute__((packed)) {
    float kp_yaw;                   /* 1. Hệ số Kp khóa hướng xoay PID_Yaw (10% PWM)     */
    float ki_yaw;                   /* 2. Hệ số Ki tích phân hướng xoay PID_Yaw          */
    float kd_yaw;                   /* 3. Hệ số Kd giảm chấn hướng xoay PID_Yaw          */
    float targetAngle;              /* 4. Góc cân bằng tĩnh gốc (từ Code / Flash)        */
    float targetYaw;                /* 5. Góc hướng đích Yaw (độ)                        */
    float controlMode;              /* 6. Kiến trúc điều khiển (1.0f = Dual FuzzyPD)     */
    float pitch_ke;                 /* 7. FuzzyPD_Pitch ke  (1/deg)                      */
    float pitch_kde;                /* 8. FuzzyPD_Pitch kde (1/(deg/s))                  */
    float pitch_ku;                 /* 9. FuzzyPD_Pitch ku  (Volt)                       */
    float pos_ke;                   /* 10. FuzzyPD_Pos ke   (1/cm)                       */
    float pos_kde;                  /* 11. FuzzyPD_Pos kde  (1/(cm/s))                   */
    float pos_ku;                   /* 12. FuzzyPD_Pos ku   (deg)                        */
} RobotConfigSync_t;

void UART3_Init(uint32_t baudrate);
void UART3_DMA_Init(void);
uint8_t UART3_SendDMA(const void *data, uint16_t len);
uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData);
uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len);
uint16_t UART3_Available(void);
int16_t UART3_ReadChar(void);

#endif /* __DRIVER_UART_H */
