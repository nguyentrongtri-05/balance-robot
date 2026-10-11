#ifndef __MANAGETASK_H
#define __MANAGETASK_H

#include "stm32f4xx.h"

/* Trạng thái hoạt động của hệ thống (0: STOP, 1: RUN) */
extern volatile uint8_t g_systemEnable;

/* Các biến góc đo và điều khiển */
extern volatile float g_filteredPitch;
extern volatile float g_targetAngle;
extern volatile float g_gyroOffset;

/**
 * @brief Khởi tạo toàn bộ task RTOS của hệ thống
 */
void System_Tasks_Init(void);

/**
 * @brief Nút PC0: START hệ thống cân bằng (kèm kiểm tra góc dựng an toàn < 15 độ)
 * @return 1 nếu khởi động thành công, 0 nếu góc nghiêng quá lớn (từ chối chạy)
 */
uint8_t Robot_Start(void);

/**
 * @brief Nút PC1: STOP / Dừng khẩn cấp, cắt PWM cả 2 động cơ ngay lập tức
 */
void Robot_Stop(void);

/**
 * @brief Nút PC2: CALIB THÔNG MINH TOÀN DIỆN (Phương án 2)
 *        - Bước 1: Tự nhận diện xe ĐỨNG YÊN TĨNH (chống rung tay) -> Calib Gyro Offset
 *        - Bước 2: Tự nhận diện xe DỰNG THẲNG CÂN BẰNG -> Chốt Zero Point
 *        - Bước 3: Tự động lưu cả 2 giá trị vào Flash Sector 11
 * @return 1 nếu calib thành công, 0 nếu bị timeout
 */
uint8_t Robot_AutoCalibrate(void);

/**
 * @brief Các hàm đơn lẻ (vẫn giữ để tương thích lệnh UART nếu cần)
 */
void Robot_SetBalancePoint(void);
void Robot_CalibrateIMU(void);

#endif /* __MANAGETASK_H */
