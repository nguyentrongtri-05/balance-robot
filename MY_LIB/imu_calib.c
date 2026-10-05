#include "imu_calib.h"
#include "stm32f4xx_flash.h"
#include <stddef.h>
#include <string.h>

/* =====================================================================
 *  THÔNG SỐ QUY TRÌNH (đơn vị: số mẫu, IMU chạy 200 Hz)
 * ===================================================================== */
#define CALIB_RATE_HZ            200u
#define LEVEL_START_DELAY        (1u * CALIB_RATE_HZ)    /* 1 s để buông tay sau khi bấm nút */
#define LEVEL_SAMPLES            (2u * CALIB_RATE_HZ)    /* Lấy trung bình 2 s               */
#define LEVEL_TIMEOUT            (15u * CALIB_RATE_HZ)
#define ACC6_SETTLE              (CALIB_RATE_HZ / 2u)    /* Mỗi mặt: đứng yên 0.5 s ...      */
#define ACC6_SAMPLES             (1u * CALIB_RATE_HZ)    /* ... rồi lấy trung bình 1 s       */
#define ACC6_TIMEOUT             (120u * CALIB_RATE_HZ)  /* 2 phút cho đủ 6 mặt              */

#define STILL_GYRO_RANGE_DPS     4.0f    /* Dao động gyro trong cửa sổ  > mức này = đang rung */
#define STILL_GYRO_HAND_DPS      15.0f   /* TARGET: xe thường được GIỮ BẰNG TAY -> nới ngưỡng  */
#define STILL_ACC_RANGE_G        0.04f
#define FACE_DOM_MIN_G           0.80f   /* Trục hướng thẳng đứng phải đọc > 0.8 g            */
#define FACE_OTHER_MAX_G         0.30f   /* 2 trục còn lại phải < 0.3 g (đặt đủ thẳng)         */

#define MAX_GYRO_BIAS_DPS        20.0f   /* Datasheet MPU6050: ±20 deg/s                       */
#define MAX_ANGLE_OFFSET_DEG     30.0f
#define MAX_ACC_OFFSET_G         0.25f
#define MIN_ACC_SCALE            0.85f
#define MAX_ACC_SCALE            1.15f

/* =====================================================================
 *  TRẠNG THÁI
 * ===================================================================== */
static ImuCalibData_t s_cal;

static volatile CalibProc_t   s_proc     = CALIB_PROC_NONE;
static volatile CalibProc_t   s_lastProc = CALIB_PROC_NONE;
static volatile CalibStatus_t s_status   = CALIB_STATUS_IDLE;
static volatile uint8_t       s_err      = CALIB_ERR_NONE;
static volatile uint8_t       s_faces    = 0u;
static volatile uint32_t      s_captures = 0u;
static volatile uint8_t       s_dirty    = 0u;   /* s_cal khác Flash (chờ lưu khi motor tắt) */

static uint32_t s_tick;          /* Số mẫu từ lúc bắt đầu quy trình   */
static uint32_t s_n;             /* Số mẫu trong cửa sổ trung bình    */
static uint32_t s_settle;
static int8_t   s_curFace;
static float    s_sum[4];
static float    s_min[4];
static float    s_max[4];
static float    s_faceVal[6];

/* =====================================================================
 *  CRC32 (đa thức 0xEDB88320) - phát hiện dữ liệu Flash hỏng / phiên bản lạ
 * ===================================================================== */
static uint32_t Crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    uint8_t b;

    while (len--)
    {
        crc ^= *p++;
        for (b = 0u; b < 8u; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static void Calib_Defaults(ImuCalibData_t *c)
{
    uint8_t k;
    c->magic   = IMU_CALIB_MAGIC;
    c->version = IMU_CALIB_VERSION;
    c->flags   = 0u;
    for (k = 0u; k < 3u; k++)
    {
        c->gyro_offset[k]  = 0.0f;
        c->accel_offset[k] = 0.0f;
        c->accel_scale[k]  = 1.0f;
    }
    c->angle_offset = 0.0f;
    c->crc = 0u;
}

/* =====================================================================
 *  FLASH
 *  Lưu ý: khi xóa sector (~1-2 s) CPU bị treo vì đang chạy code từ cùng bank Flash.
 *  Chỉ làm khi motor đã TẮT (quy trình hiệu chuẩn đảm bảo điều này).
 * ===================================================================== */
static void Flash_FlushDataCache(void)
{
    /* ART accelerator có thể còn giữ dữ liệu cũ của sector vừa ghi */
    FLASH_DataCacheCmd(DISABLE);
    FLASH_DataCacheReset();
    FLASH_DataCacheCmd(ENABLE);
}

static void Flash_Begin(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

static uint8_t Flash_EraseCalib(void)
{
    FLASH_Status st;

    Flash_Begin();
    st = FLASH_EraseSector(IMU_CALIB_FLASH_SECTOR, VoltageRange_3);
    FLASH_Lock();
    Flash_FlushDataCache();

    return (st == FLASH_COMPLETE) ? CALIB_ERR_NONE : CALIB_ERR_FLASH;
}

static uint8_t Flash_SaveCalib(ImuCalibData_t *c)
{
    const uint32_t *w = (const uint32_t *)c;
    uint32_t i;
    FLASH_Status st;

    c->magic   = IMU_CALIB_MAGIC;
    c->version = IMU_CALIB_VERSION;
    c->crc     = Crc32(c, (uint32_t)offsetof(ImuCalibData_t, crc));

    Flash_Begin();
    st = FLASH_EraseSector(IMU_CALIB_FLASH_SECTOR, VoltageRange_3);
    for (i = 0u; (st == FLASH_COMPLETE) && (i < sizeof(ImuCalibData_t) / 4u); i++)
    {
        st = FLASH_ProgramWord(IMU_CALIB_FLASH_ADDR + 4u * i, w[i]);
    }
    FLASH_Lock();
    Flash_FlushDataCache();

    if (st != FLASH_COMPLETE)
    {
        return CALIB_ERR_FLASH;
    }
    /* Đọc lại để chắc chắn */
    if (memcmp((const void *)IMU_CALIB_FLASH_ADDR, c, sizeof(ImuCalibData_t)) != 0)
    {
        return CALIB_ERR_FLASH;
    }
    s_dirty = 0u;
    return CALIB_ERR_NONE;
}

/* =====================================================================
 *  API CHUNG
 * ===================================================================== */
uint32_t ImuCalib_Init(void)
{
    const ImuCalibData_t *f = (const ImuCalibData_t *)IMU_CALIB_FLASH_ADDR;

    Calib_Defaults(&s_cal);

    /* Flash trắng = 0xFFFFFFFF -> magic sai -> dùng mặc định */
    if ((f->magic == IMU_CALIB_MAGIC) &&
        (f->version == IMU_CALIB_VERSION) &&
        (Crc32(f, (uint32_t)offsetof(ImuCalibData_t, crc)) == f->crc))
    {
        s_cal = *f;
    }

    if (s_cal.flags & IMU_CALIB_HAS_ACCEL)
    {
        MPU6050_SetAccelCalib(s_cal.accel_offset, s_cal.accel_scale);
    }
    else
    {
        MPU6050_SetAccelCalib(NULL, NULL);
    }

    /* Bias gyro từ Flash chỉ là giá trị dự phòng: hiệu chuẩn lúc khởi động
     * (MPU6050_CalibrateGyro) sẽ ghi đè nếu xe đứng yên. */
    if (s_cal.flags & IMU_CALIB_HAS_GYRO)
    {
        MPU6050_SetGyroOffset(s_cal.gyro_offset[0], s_cal.gyro_offset[1], s_cal.gyro_offset[2]);
    }

    s_proc   = CALIB_PROC_NONE;
    s_status = CALIB_STATUS_IDLE;
    return s_cal.flags;
}

const ImuCalibData_t *ImuCalib_Get(void)
{
    return &s_cal;
}

uint8_t ImuCalib_Start(CalibProc_t proc)
{
    if ((s_proc != CALIB_PROC_NONE) || (proc == CALIB_PROC_NONE))
    {
        return 0u;
    }

    s_tick    = 0u;
    s_n       = 0u;
    s_settle  = 0u;
    s_curFace = -1;
    s_err     = CALIB_ERR_NONE;
    if (proc == CALIB_PROC_ACCEL6)
    {
        s_faces = 0u;
    }
    s_lastProc = proc;
    s_status   = CALIB_STATUS_RUNNING;

    /* Task IMU (ưu tiên cao hơn) chỉ bắt đầu xử lý khi thấy s_proc != NONE
     * => mọi biến ở trên phải được ghi xong trước */
    __DMB();
    s_proc = proc;
    return 1u;
}

void ImuCalib_Abort(void)
{
    s_proc   = CALIB_PROC_NONE;
    s_status = CALIB_STATUS_IDLE;
}

static void Calib_Finish(uint8_t err)
{
    s_err    = err;
    s_status = (err == CALIB_ERR_NONE) ? CALIB_STATUS_OK : CALIB_STATUS_FAIL;
    s_proc   = CALIB_PROC_NONE;
}

CalibStatus_t ImuCalib_GetStatus(void)       { return s_status; }
CalibProc_t   ImuCalib_GetProc(void)         { return s_lastProc; }
uint8_t       ImuCalib_GetError(void)        { return s_err; }
uint8_t       ImuCalib_GetFaces(void)        { return s_faces; }
uint32_t      ImuCalib_GetCaptureCount(void) { return s_captures; }

void ImuCalib_ClearStatus(void)
{
    if (s_proc == CALIB_PROC_NONE)
    {
        s_status = CALIB_STATUS_IDLE;
    }
}

/* =====================================================================
 *  QUY TRÌNH 1: LEVEL - bias gyro + góc cân bằng
 *  Giữ xe ở đúng tư thế "tự đứng được" (hoặc kê lên giá), không chạm/rung.
 *  Dùng chung cho:
 *    ZERO   : chỉ lưu bias gyro  (xe NẰM yên, tư thế bất kỳ)
 *    TARGET : chỉ lưu góc cân bằng (xe được giữ tay ở điểm cân bằng -> nới ngưỡng rung)
 * ===================================================================== */
static void Feed_Level(const MPU6050_Data_t *d, float pitch)
{
    const uint8_t wantGyro  = (s_proc != CALIB_PROC_TARGET) ? 1u : 0u;
    const uint8_t wantAngle = (s_proc != CALIB_PROC_ZERO)   ? 1u : 0u;
    const float   stillMax  = (s_proc == CALIB_PROC_TARGET) ? STILL_GYRO_HAND_DPS
                                                            : STILL_GYRO_RANGE_DPS;
    float g[3];
    uint8_t k;

    if (++s_tick > LEVEL_TIMEOUT)
    {
        Calib_Finish(CALIB_ERR_TIMEOUT);
        return;
    }
    if (s_tick <= LEVEL_START_DELAY)
    {
        return;   /* Chờ hết rung do bấm nút */
    }

    /* Dùng giá trị RAW (chưa trừ offset cũ) vì đang đi đo chính offset đó */
    g[0] = (float)d->gx_raw / MPU6050_GYRO_SENS;
    g[1] = (float)d->gy_raw / MPU6050_GYRO_SENS;
    g[2] = (float)d->gz_raw / MPU6050_GYRO_SENS;

    if (s_n == 0u)
    {
        for (k = 0u; k < 4u; k++) s_sum[k] = 0.0f;
        for (k = 0u; k < 3u; k++) { s_min[k] = g[k]; s_max[k] = g[k]; }
    }

    for (k = 0u; k < 3u; k++)
    {
        if (g[k] < s_min[k]) s_min[k] = g[k];
        if (g[k] > s_max[k]) s_max[k] = g[k];
        if ((s_max[k] - s_min[k]) > stillMax)
        {
            s_n = 0u;     /* Bị rung -> đo lại từ đầu (vẫn trong thời gian timeout) */
            return;
        }
    }

    for (k = 0u; k < 3u; k++) s_sum[k] += g[k];
    s_sum[3] += pitch;
    s_n++;

    if (s_n >= LEVEL_SAMPLES)
    {
        float gb[3];
        float ang = s_sum[3] / (float)s_n;

        for (k = 0u; k < 3u; k++)
        {
            gb[k] = s_sum[k] / (float)s_n;
            if (wantGyro && ((gb[k] > MAX_GYRO_BIAS_DPS) || (gb[k] < -MAX_GYRO_BIAS_DPS)))
            {
                Calib_Finish(CALIB_ERR_RANGE);
                return;
            }
        }
        if (wantAngle && ((ang > MAX_ANGLE_OFFSET_DEG) || (ang < -MAX_ANGLE_OFFSET_DEG)))
        {
            Calib_Finish(CALIB_ERR_RANGE);
            return;
        }

        if (wantGyro)
        {
            for (k = 0u; k < 3u; k++) s_cal.gyro_offset[k] = gb[k];
            s_cal.flags |= IMU_CALIB_HAS_GYRO;
            MPU6050_SetGyroOffset(gb[0], gb[1], gb[2]);
        }
        if (wantAngle)
        {
            s_cal.angle_offset = ang;
            s_cal.flags |= IMU_CALIB_HAS_ANGLE;
        }

        Calib_Finish(Flash_SaveCalib(&s_cal));
    }
}

/* =====================================================================
 *  QUY TRÌNH 2: ACCEL6 - gia tốc kế 6 mặt
 *  Đặt cảm biến lần lượt sao cho +X, -X, +Y, -Y, +Z, -Z hướng LÊN (thứ tự tùy ý).
 *  Mỗi mặt: giữ yên ~1.5 s -> LED xanh lá nháy báo đã thu -> xoay sang mặt khác.
 *  Với mỗi trục k: offset = (a+ + a-)/2 ; scale = 2 / (a+ - a-)
 * ===================================================================== */
static void Accel6_Compute(void)
{
    float off[3];
    float sc[3];
    uint8_t k;

    for (k = 0u; k < 3u; k++)
    {
        float plus  = s_faceVal[2u * k];        /* trục k hướng lên:   ~ +1 g */
        float minus = s_faceVal[2u * k + 1u];   /* trục k hướng xuống: ~ -1 g */
        float span  = plus - minus;             /* ~ 2 g */

        if (span < 1.5f)
        {
            Calib_Finish(CALIB_ERR_RANGE);
            return;
        }
        off[k] = 0.5f * (plus + minus);
        sc[k]  = 2.0f / span;

        if ((off[k] > MAX_ACC_OFFSET_G) || (off[k] < -MAX_ACC_OFFSET_G) ||
            (sc[k] < MIN_ACC_SCALE) || (sc[k] > MAX_ACC_SCALE))
        {
            Calib_Finish(CALIB_ERR_RANGE);
            return;
        }
    }

    for (k = 0u; k < 3u; k++)
    {
        s_cal.accel_offset[k] = off[k];
        s_cal.accel_scale[k]  = sc[k];
    }
    s_cal.flags |= IMU_CALIB_HAS_ACCEL;
    /* Gia tốc kế thay đổi => góc pitch đổi => góc cân bằng cũ không còn đúng, phải đo lại */
    s_cal.flags &= ~IMU_CALIB_HAS_ANGLE;
    MPU6050_SetAccelCalib(off, sc);

    Calib_Finish(Flash_SaveCalib(&s_cal));
}

static void Feed_Accel6(const MPU6050_Data_t *d)
{
    float a[3];
    float g[3];
    float absA[3];
    uint8_t k;
    uint8_t dom = 0u;
    int8_t face = -1;

    if (++s_tick > ACC6_TIMEOUT)
    {
        Calib_Finish(CALIB_ERR_TIMEOUT);
        return;
    }

    /* RAW: không phụ thuộc hiệu chuẩn gia tốc cũ */
    a[0] = (float)d->ax_raw / MPU6050_ACCEL_SENS;
    a[1] = (float)d->ay_raw / MPU6050_ACCEL_SENS;
    a[2] = (float)d->az_raw / MPU6050_ACCEL_SENS;
    g[0] = (float)d->gx_raw / MPU6050_GYRO_SENS;
    g[1] = (float)d->gy_raw / MPU6050_GYRO_SENS;
    g[2] = (float)d->gz_raw / MPU6050_GYRO_SENS;

    /* 1. Xác định mặt đang hướng lên */
    for (k = 0u; k < 3u; k++)
    {
        absA[k] = (a[k] < 0.0f) ? -a[k] : a[k];
        if (absA[k] > absA[dom]) dom = k;
    }
    if (absA[dom] > FACE_DOM_MIN_G)
    {
        uint8_t okOthers = 1u;
        for (k = 0u; k < 3u; k++)
        {
            if ((k != dom) && (absA[k] > FACE_OTHER_MAX_G)) okOthers = 0u;
        }
        if (okOthers)
        {
            face = (int8_t)(2u * dom + ((a[dom] < 0.0f) ? 1u : 0u));
        }
    }

    if ((face < 0) || (face != s_curFace))
    {
        s_curFace = face;     /* Đang xoay / đặt nghiêng -> chờ */
        s_settle  = 0u;
        s_n       = 0u;
        return;
    }
    if (s_faces & (uint8_t)(1u << face))
    {
        return;               /* Mặt này thu rồi -> chờ người dùng xoay sang mặt khác */
    }
    if (s_settle < ACC6_SETTLE)
    {
        s_settle++;
        s_n = 0u;
        return;
    }

    /* 2. Lấy trung bình khi đứng yên */
    if (s_n == 0u)
    {
        s_sum[0] = 0.0f;
        for (k = 0u; k < 3u; k++) { s_min[k] = g[k]; s_max[k] = g[k]; }
        s_min[3] = a[dom];
        s_max[3] = a[dom];
    }
    for (k = 0u; k < 3u; k++)
    {
        if (g[k] < s_min[k]) s_min[k] = g[k];
        if (g[k] > s_max[k]) s_max[k] = g[k];
    }
    if (a[dom] < s_min[3]) s_min[3] = a[dom];
    if (a[dom] > s_max[3]) s_max[3] = a[dom];

    for (k = 0u; k < 3u; k++)
    {
        if ((s_max[k] - s_min[k]) > STILL_GYRO_RANGE_DPS)
        {
            s_n = 0u;
            s_settle = 0u;
            return;
        }
    }
    if ((s_max[3] - s_min[3]) > STILL_ACC_RANGE_G)
    {
        s_n = 0u;
        s_settle = 0u;
        return;
    }

    s_sum[0] += a[dom];
    s_n++;

    if (s_n >= ACC6_SAMPLES)
    {
        s_faceVal[face] = s_sum[0] / (float)s_n;
        s_faces |= (uint8_t)(1u << face);
        s_captures++;
        s_n = 0u;
        s_settle = 0u;

        if (s_faces == 0x3Fu)
        {
            Accel6_Compute();
        }
    }
}

/* =====================================================================
 *  QUY TRÌNH 3: ERASE - về mặc định
 * ===================================================================== */
static void Do_Erase(void)
{
    uint8_t err = Flash_EraseCalib();
    Calib_Defaults(&s_cal);
    MPU6050_SetAccelCalib(NULL, NULL);
    s_dirty = 0u;
    /* Giữ nguyên bias gyro hiện tại trong driver (đo lúc khởi động, vẫn đúng) */
    Calib_Finish(err);
}

void ImuCalib_Feed(const MPU6050_Data_t *d, float pitch)
{
    if (d == NULL) return;

    switch (s_proc)
    {
    case CALIB_PROC_LEVEL:
    case CALIB_PROC_ZERO:
    case CALIB_PROC_TARGET: Feed_Level(d, pitch); break;
    case CALIB_PROC_ACCEL6: Feed_Accel6(d);       break;
    case CALIB_PROC_ERASE:  Do_Erase();           break;
    default:                                      break;
    }
}

/* =====================================================================
 *  ĐẶT GÓC CÂN BẰNG KHI XE ĐANG CHẠY (không ghi Flash ngay)
 * ===================================================================== */
uint8_t ImuCalib_SetAngleOffset(float angle)
{
    if ((angle > MAX_ANGLE_OFFSET_DEG) || (angle < -MAX_ANGLE_OFFSET_DEG))
    {
        return CALIB_ERR_RANGE;
    }
    s_cal.angle_offset = angle;
    s_cal.flags |= IMU_CALIB_HAS_ANGLE;
    s_dirty = 1u;
    return CALIB_ERR_NONE;
}

uint8_t ImuCalib_IsDirty(void)
{
    return s_dirty;
}

uint8_t ImuCalib_SaveIfDirty(void)
{
    if (!s_dirty || (s_proc != CALIB_PROC_NONE))
    {
        return CALIB_ERR_NONE;   /* Không cần lưu / quy trình khác đang chạy sẽ tự lưu */
    }
    return Flash_SaveCalib(&s_cal);
}
