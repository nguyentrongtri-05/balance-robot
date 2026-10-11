#ifndef __DRIVER_FLASH_H
#define __DRIVER_FLASH_H

#include "stm32f4xx.h"

/* Địa chỉ Sector 11 (Sector cuối cùng 128KB của STM32F407 1MB Flash)
 * Vùng nhớ: 0x080E0000 - 0x080FFFFF (An toàn tuyệt đối, không trùng với code chương trình) */
#define FLASH_CONFIG_START_ADDR     0x080E0000
#define FLASH_MAGIC_KEY             0x55AA2026  /* Khóa nhận diện tính hợp lệ của dữ liệu */

/* Cấu trúc lưu trữ các thông số cần nhớ của xe tự cân bằng */
typedef struct {
    uint32_t magic;          /* Khóa nhận diện dữ liệu hợp lệ */
    float    targetAngle;    /* Điểm cân bằng mục tiêu (do nút Set Zero lưu) */
    float    gyroOffset;     /* Sai số tĩnh con quay hồi chuyển (do nút Calib lưu) */
    uint32_t checksum;       /* Checksum kiểm tra toàn vẹn dữ liệu */
} RobotFlashConfig_t;

/**
 * @brief Đọc thông số đã lưu từ Flash lúc khởi động
 * @param config Con trỏ tới struct để nhận dữ liệu
 * @return 1 nếu dữ liệu hợp lệ, 0 nếu Flash trống (chưa từng lưu cấu hình)
 */
uint8_t Flash_ReadConfig(RobotFlashConfig_t *config);

/**
 * @brief Lưu các thông số (targetAngle, gyroOffset) vào Flash Sector 11
 * @param targetAngle Điểm cân bằng
 * @param gyroOffset Sai số gyro
 * @return 1 nếu lưu thành công, 0 nếu có lỗi
 */
uint8_t Flash_SaveConfig(float targetAngle, float gyroOffset);

#endif /* __DRIVER_FLASH_H */
