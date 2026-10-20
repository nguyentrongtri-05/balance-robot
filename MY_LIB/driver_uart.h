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

/* Cấu trúc gói dữ liệu Telemetry chuẩn gửi lên máy tính (ID = 0x01, len = 88 byte = 22 float) */
typedef struct __attribute__((packed)) {
    /* 1. Dữ liệu từ vTaskIMU (Pitch & Yaw: Raw vs Kalman) */
    float rawPitch;                 /* 1. Góc Pitch thô từ Accelerometer (độ) */
    float filteredPitch;            /* 2. Góc Pitch sau lọc kalmanPitch (độ) */
    float gyroRate;                 /* 3. Vận tốc góc Pitch thực tế (độ/s) */
    float rawYaw;                   /* 4. Góc Yaw thô từ vi sai Encoder (độ) */
    float filteredYaw;              /* 5. Góc Yaw sau lọc kalmanYaw (độ) */
    float yawRate;                  /* 6. Vận tốc góc Yaw thực tế (độ/s) */

    /* 2. Dữ liệu từ vTaskEncoder (Raw Wheel & Kalman Wheel) */
    float motorSpeed_left;          /* 7. Vận tốc Raw Bánh Trái (RPM) */
    float motorSpeed_right;         /* 8. Vận tốc Raw Bánh Phải (RPM) */
    float motorSpeedFiltered_left;  /* 9. Vận tốc Kalman Bánh Trái (RPM) */
    float motorSpeedFiltered_right; /* 10. Vận tốc Kalman Bánh Phải (RPM) */

    /* 3. Dữ liệu từ vTaskControl (Kiến trúc Vòng Lồng Cascade: 90% OutFuzzy + 10% Yaw) */
    float motorPWM_left;            /* 11. PWM xuất Bánh Trái (-8400 đến +8400) */
    float motorPWM_right;           /* 12. PWM xuất Bánh Phải (-8400 đến +8400) */
    float targetAngle;              /* 13. Góc cân bằng đặt thực tế sau Vòng Lồng Cascade (độ) */
    float outFuzzy;                 /* 14. Ngõ ra Vòng Trong Cân bằng FuzzyPD_Pitch (tối đa 90% = +-7560 PWM) */
    float outYaw;                   /* 15. Ngõ ra Vòng Góc Yaw (tối đa 10% = +-840 PWM) */
    float outSpeed;                 /* 16. Góc bù Vòng Ngoài Vị trí FuzzyPD_Pos x 420 (chia 420 ra độ, giới hạn +-3.5°) */

    /* 4. Dữ liệu debug 2 khâu FuzzyPD nối tiếp (FuzzyPD_Pitch & FuzzyPD_Pos) */
    float velCmd;                   /* 17. Vận tốc đặt đưa vào vòng ngoài (RPM, dấu theo encoder) */
    float dVel;                     /* 18. Gia tốc góc bánh dVel = d(vel)/dt (RPM/s) */
    float pitchENorm;               /* 19. FuzzyPD_Pitch (Vòng trong): e đã chuẩn hóa & bão hòa [-1..+1] */
    float pitchDeNorm;              /* 20. FuzzyPD_Pitch (Vòng trong): de đã chuẩn hóa & bão hòa [-1..+1] */
    float posENorm;                 /* 21. FuzzyPD_Pos (Vòng ngoài): e đã chuẩn hóa & bão hòa [-1..+1] */
    float posDeNorm;                /* 22. FuzzyPD_Pos (Vòng ngoài): de đã chuẩn hóa & bão hòa [-1..+1] */
} RobotTelemetry_t;

/* Cấu trúc gói đồng bộ Bộ Số Cấu Hình Gốc từ STM32 gửi lên máy tính khi khởi động / Reset (ID = 0x02, len = 64 byte = 16 float) */
typedef struct __attribute__((packed)) {
    float kp_balance;               /* 1. Hệ số Kp vòng trong cân bằng Pitch (90% PWM) */
    float kd_balance;               /* 2. Hệ số Kd vòng trong cân bằng Pitch (90% PWM) */
    float kd_yaw;                   /* 3. Hệ số Kd ghìm lệch vận tốc 2 bánh Encoder (10% PWM) */
    float kp_yaw;                   /* 4. Hệ số Kp khóa cứng góc Yaw (10% PWM) */
    float kd_yaw_imu;               /* 5. Hệ số Kd phanh vận tốc xoay ngang Gyro Z IMU (10% PWM) */
    float targetAngle;              /* 6. Góc cân bằng tĩnh gốc (từ Code / Flash) */
    float targetYaw;                /* 7. Góc hướng đích Yaw (độ) */
    float controlMode;              /* 8. Chế độ điều khiển (0.0f = Dual PID Cascade, 1.0f = Dual FuzzyPD Cascade) */
    float kp_speed;                 /* 9. Hệ số Kp vòng ngoài vận tốc đặt */
    float ki_speed;                 /* 10. Hệ số Ki vòng ngoài vận tốc đặt */
    float pitch_ke;                 /* 11. FuzzyPD_Pitch ke  */
    float pitch_kde;                /* 12. FuzzyPD_Pitch kde */
    float pitch_ku;                 /* 13. FuzzyPD_Pitch ku  */
    float pos_ke;                   /* 14. FuzzyPD_Pos ke    */
    float pos_kde;                  /* 15. FuzzyPD_Pos kde   */
    float pos_ku;                   /* 16. FuzzyPD_Pos ku    */
} RobotConfigSync_t;

void UART3_Init(uint32_t baudrate);
void UART3_DMA_Init(void);
uint8_t UART3_SendDMA(const void *data, uint16_t len);
uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData);
uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len);
uint16_t UART3_Available(void);
int16_t UART3_ReadChar(void);

#endif /* __DRIVER_UART_H */
