#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

/* =====================================================================
 *  PHẦN CỨNG
 *   - Bánh trái : TIM3 (PB4 = CH1, PB5 = CH2)
 *   - Bánh phải : TIM4 (PB6 = CH1, PB7 = CH2)
 *   Chế độ Encoder x4 (đếm cả 2 sườn của 2 kênh)
 * ===================================================================== */

/* =====================================================================
 *  CẤU HÌNH NGƯỜI DÙNG
 * ===================================================================== */
#define ENCODER_PULSES_PER_REV   330u    /* Xung / vòng TRỤC RA trên 1 kênh (11 PPR x hộp số 1:30) */
#define ENCODER_COUNTS_PER_REV   (ENCODER_PULSES_PER_REV * 4u)  /* x4 = 1320 count/vòng */
#define ENCODER_SAMPLE_TIME      0.010f  /* Chu kỳ mặc định cho Encoder_ReadStruct() [s] */

/* Đảo dấu khi 2 motor lắp đối xứng gương: xe chạy TIẾN thì cả 2 speed phải DƯƠNG */
#define ENCODER_LEFT_INVERT      0
#define ENCODER_RIGHT_INVERT     0

/* Bộ lọc số trên chân vào (IC1F/IC2F, 0..15). Clock timer 84MHz:
 * 0x0A = fDTS/16, N=8  => bỏ qua xung nhiễu < ~1.5us (nhiễu chuyển mạch PWM motor) */
#define ENCODER_INPUT_FILTER     0x0Au

/* Lọc thông thấp bậc 1 cho tốc độ: speed += ALPHA * (raw - speed)
 * 1.0 = không lọc; 0.2..0.4 hợp lý với chu kỳ 10ms.
 * Lý do: 1 count trong 10ms = 60/(1320*0.01) ≈ 4.5 RPM -> tín hiệu thô rất "bậc thang". */
#define ENCODER_SPEED_LPF_ALPHA  0.3f

/* =====================================================================
 *  DỮ LIỆU
 * ===================================================================== */
typedef struct
{
    uint16_t last_cnt;   /* Giá trị CNT ở lần đọc trước                  */
    int32_t  delta;      /* Số count trong chu kỳ vừa rồi (đã áp INVERT) */
    int32_t  position;   /* Tổng count tích lũy (dùng cho giữ vị trí)    */
    float    speed_raw;  /* Tốc độ chưa lọc [RPM]                         */
    float    speed;      /* Tốc độ đã lọc   [RPM]                         */
} Encoder_t;

typedef Encoder_t MOTOR;   /* Giữ tên cũ để tương thích code hiện có */

/* =====================================================================
 *  API
 * ===================================================================== */
/**
 * @brief Khởi tạo TIM3/TIM4 ở chế độ Encoder x4 có bộ lọc nhiễu đầu vào
 */
void Encoder_Config(void);

/**
 * @brief Đồng bộ last_cnt với CNT hiện tại, xóa position/speed.
 *        Gọi 1 lần trước vòng lặp đọc để tránh "cú nhảy" ở mẫu đầu tiên.
 */
void Encoder_Reset(Encoder_t *left, Encoder_t *right);

/**
 * @brief Cập nhật delta, position, speed của 2 bánh
 * @param dt: Thời gian thực giữa 2 lần gọi [s]
 */
void Encoder_Update(Encoder_t *left, Encoder_t *right, float dt);

/**
 * @brief API cũ: tương đương Encoder_Update(left, right, ENCODER_SAMPLE_TIME)
 */
void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight);

/** Đổi count sang số vòng bánh xe */
float Encoder_CountsToRev(int32_t counts);

#endif /* __ENCODER_H */
