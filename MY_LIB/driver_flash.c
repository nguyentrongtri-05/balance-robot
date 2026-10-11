#include "driver_flash.h"
#include "stm32f4xx_flash.h"

static uint32_t CalculateChecksum(float targetAngle, float gyroOffset) {
    uint32_t sum = FLASH_MAGIC_KEY;
    union {
        float f;
        uint32_t u;
    } conv;

    conv.f = targetAngle;
    sum ^= conv.u;

    conv.f = gyroOffset;
    sum ^= conv.u;

    return sum;
}

uint8_t Flash_ReadConfig(RobotFlashConfig_t *config) {
    if (config == 0) return 0;

    /* Đọc trực tiếp từ địa chỉ Flash */
    volatile uint32_t *flash_ptr = (volatile uint32_t *)FLASH_CONFIG_START_ADDR;

    uint32_t magic = flash_ptr[0];
    if (magic != FLASH_MAGIC_KEY) {
        return 0; /* Chưa từng lưu cấu hình (Flash đang là 0xFFFFFFFF) */
    }

    union {
        uint32_t u;
        float f;
    } conv;

    conv.u = flash_ptr[1];
    float targetAngle = conv.f;

    conv.u = flash_ptr[2];
    float gyroOffset = conv.f;

    uint32_t checksum = flash_ptr[3];

    /* Kiểm tra tính toàn vẹn của dữ liệu */
    if (checksum != CalculateChecksum(targetAngle, gyroOffset)) {
        return 0; /* Dữ liệu bị lỗi hoặc hỏng */
    }

    config->magic = magic;
    config->targetAngle = targetAngle;
    config->gyroOffset = gyroOffset;
    config->checksum = checksum;

    return 1; /* Đọc thành công */
}

uint8_t Flash_SaveConfig(float targetAngle, float gyroOffset) {
    FLASH_Status status;
    uint32_t checksum = CalculateChecksum(targetAngle, gyroOffset);

    union {
        float f;
        uint32_t u;
    } convAngle, convOffset;

    convAngle.f = targetAngle;
    convOffset.f = gyroOffset;

    /* 1. Mở khóa bộ điều khiển Flash */
    FLASH_Unlock();

    /* 2. Xóa các cờ lỗi cũ */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* 3. Xóa Sector 11 (Điện áp kit Discovery 3.3V -> VoltageRange_3) */
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }

    /* 4. Ghi 4 Word (16 bytes) dữ liệu */
    status = FLASH_ProgramWord(FLASH_CONFIG_START_ADDR, FLASH_MAGIC_KEY);
    if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }

    status = FLASH_ProgramWord(FLASH_CONFIG_START_ADDR + 4, convAngle.u);
    if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }

    status = FLASH_ProgramWord(FLASH_CONFIG_START_ADDR + 8, convOffset.u);
    if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }

    status = FLASH_ProgramWord(FLASH_CONFIG_START_ADDR + 12, checksum);
    if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }

    /* 5. Khóa lại Flash để bảo vệ an toàn */
    FLASH_Lock();

    return 1; /* Lưu thành công */
}
