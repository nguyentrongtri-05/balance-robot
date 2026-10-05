#ifndef __MANAGETASK_H
#define __MANAGETASK_H

/* Trạng thái hệ cân bằng (xem biến g_balanceState trên Watch window) */
typedef enum {
    BAL_WAIT_UPRIGHT = 0,   /* Motor tắt, chờ dựng xe thẳng đứng & đứng yên */
    BAL_RUNNING      = 1,   /* Đang điều khiển cân bằng                       */
    BAL_FAULT        = 2,   /* Lỗi không tự phục hồi (IMU khởi tạo hỏng, ...) */
    BAL_CALIB        = 3    /* Đang hiệu chuẩn IMU (motor tắt)                */
} BalanceState_t;

/* Yêu cầu hiệu chuẩn: ghi vào biến g_calibRequest trên Watch window, hoặc dùng nút USER */
#define CALIB_REQ_NONE      0u
#define CALIB_REQ_LEVEL     1u   /* = nhấn ngắn nút USER  */
#define CALIB_REQ_ACCEL6    2u   /* = giữ nút USER >= 2 s */
#define CALIB_REQ_ERASE     3u   /* Xóa hiệu chuẩn trong Flash */

/**
 * @brief Initialize all system RTOS tasks
 */
void System_Tasks_Init(void);

#endif /* __MANAGETASK_H */
