#ifndef __IMU_CALIB_H
#define __IMU_CALIB_H

#include "stm32f4xx.h"
#include "driver_i2c.h"

/* =====================================================================
 *  LƯU TRỮ: Sector 11 của Flash (128 KB cuối, 0x080E0000 trên STM32F407VG)
 *  -> Project Keil đã giới hạn IROM1 = 0xE0000 để code KHÔNG BAO GIỜ nằm ở đây.
 * ===================================================================== */
#define IMU_CALIB_FLASH_ADDR     0x080E0000u
#define IMU_CALIB_FLASH_SECTOR   FLASH_Sector_11
#define IMU_CALIB_MAGIC          0x494D5543u   /* 'IMUC' */
#define IMU_CALIB_VERSION        1u

/* Cờ cho biết phần nào trong dữ liệu đã được hiệu chuẩn */
#define IMU_CALIB_HAS_GYRO       (1u << 0)
#define IMU_CALIB_HAS_ACCEL      (1u << 1)
#define IMU_CALIB_HAS_ANGLE      (1u << 2)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t flags;            /* IMU_CALIB_HAS_x                         */
    float    gyro_offset[3];   /* [deg/s]  bias gyro                      */
    float    accel_offset[3];  /* [g]      a_cal = (a - offset) * scale   */
    float    accel_scale[3];
    float    angle_offset;     /* [deg]    góc cân bằng cơ khí            */
    uint32_t crc;              /* CRC32 của mọi trường phía trên          */
} ImuCalibData_t;

/* Quy trình hiệu chuẩn */
typedef enum {
    CALIB_PROC_NONE   = 0,
    CALIB_PROC_LEVEL  = 1,   /* Gyro bias + góc cân bằng (giữ xe ở điểm cân bằng, đứng yên) */
    CALIB_PROC_ACCEL6 = 2,   /* Gia tốc kế 6 mặt (đặt xe/board lần lượt 6 mặt, thứ tự tùy ý) */
    CALIB_PROC_ERASE  = 3,   /* Xóa dữ liệu hiệu chuẩn, về mặc định                        */
    CALIB_PROC_ZERO   = 4,   /* CHỈ bias gyro (xe nằm yên, tư thế bất kỳ)                  */
    CALIB_PROC_TARGET = 5    /* CHỈ góc cân bằng (giữ xe ở điểm cân bằng)                   */
} CalibProc_t;

typedef enum {
    CALIB_STATUS_IDLE    = 0,
    CALIB_STATUS_RUNNING = 1,
    CALIB_STATUS_OK      = 2,
    CALIB_STATUS_FAIL    = 3
} CalibStatus_t;

/* Mã lỗi (ImuCalib_GetError) */
#define CALIB_ERR_NONE      0u
#define CALIB_ERR_TIMEOUT   1u   /* Không đứng yên đủ lâu / chưa đủ 6 mặt trước khi hết giờ */
#define CALIB_ERR_RANGE     2u   /* Kết quả vô lý (cảm biến hỏng / đặt sai tư thế)          */
#define CALIB_ERR_FLASH     3u   /* Ghi Flash thất bại                                       */

/**
 * @brief Đọc dữ liệu từ Flash (nếu hợp lệ) và áp hiệu chuẩn gia tốc kế cho driver.
 *        Gọi SAU I2C1_MPU6050_Init(), TRƯỚC khi hiệu chuẩn gyro lúc khởi động.
 * @retval flags IMU_CALIB_HAS_x của dữ liệu đã nạp (0 = chưa từng hiệu chuẩn)
 */
uint32_t ImuCalib_Init(void);

/** Bản dữ liệu hiệu chuẩn đang dùng (chỉ đọc) */
const ImuCalibData_t *ImuCalib_Get(void);

/**
 * @brief Bắt đầu 1 quy trình. Trả về 0 nếu đang bận.
 *        Gọi từ task Control (motor phải đang TẮT).
 */
uint8_t ImuCalib_Start(CalibProc_t proc);
void    ImuCalib_Abort(void);

/**
 * @brief Cấp 1 mẫu IMU cho quy trình đang chạy. Gọi trong task IMU sau mỗi lần đọc (5 ms).
 * @param d     : mẫu vừa đọc (dùng giá trị RAW để không phụ thuộc hiệu chuẩn cũ)
 * @param pitch : góc pitch tính từ gia tốc kế ĐÃ hiệu chuẩn, đúng quy ước dấu của hệ
 */
void    ImuCalib_Feed(const MPU6050_Data_t *d, float pitch);

CalibStatus_t ImuCalib_GetStatus(void);
CalibProc_t   ImuCalib_GetProc(void);
uint8_t       ImuCalib_GetError(void);
uint8_t       ImuCalib_GetFaces(void);        /* bitmask 6 mặt đã thu: bit0 +X, bit1 -X, ... bit5 -Z */
uint32_t      ImuCalib_GetCaptureCount(void); /* tăng 1 mỗi khi thu xong 1 mặt (để nháy LED)          */
void          ImuCalib_ClearStatus(void);     /* OK/FAIL -> IDLE                                     */

/**
 * @brief Đặt góc cân bằng NGAY trong RAM (không ghi Flash) - dùng khi xe ĐANG CHẠY,
 *        vì xóa Flash làm CPU treo 1-2 s => xe đổ. Đánh dấu "chưa lưu".
 * @retval CALIB_ERR_NONE hoặc CALIB_ERR_RANGE (|góc| quá lớn, không áp dụng)
 */
uint8_t ImuCalib_SetAngleOffset(float angle);

/** Có dữ liệu đặt bằng ImuCalib_SetAngleOffset() chưa ghi xuống Flash? */
uint8_t ImuCalib_IsDirty(void);

/**
 * @brief Ghi Flash nếu đang "chưa lưu". CHỈ gọi khi motor TẮT (CPU treo 1-2 s).
 * @retval CALIB_ERR_NONE (đã lưu / không cần lưu) hoặc CALIB_ERR_FLASH
 */
uint8_t ImuCalib_SaveIfDirty(void);

#endif /* __IMU_CALIB_H */
