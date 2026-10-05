#include "managetask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "driver_i2c.h"
#include "fuzzy.h"
#include "pid.h"
#include "kalman.h"
#include "imu_calib.h"

/* =====================================================================
 *  QUY ƯỚC DẤU (phải kiểm tra trên phần cứng TRƯỚC khi cho xe tự cân bằng)
 *   - Pitch  > 0 : xe nghiêng về PHÍA TRƯỚC
 *   - PWM    > 0 : bánh quay TIẾN          (chỉnh MOTOR_x_INVERT trong motor.h)
 *   - Speed  > 0 : xe chạy TIẾN            (chỉnh ENCODER_x_INVERT trong encoder.h)
 *  Khi 3 quy ước trên đúng, toàn bộ dấu bên dưới tự đúng:
 *   - Vòng góc : xe nghiêng tới  -> e > 0 -> PWM > 0 -> bánh chạy tới đỡ trọng tâm
 *   - Vòng tốc độ: xe chạy tới nhanh hơn đặt -> góc đặt < 0 (ngả ra sau) -> xe chậm lại
 * ===================================================================== */

/* ---- IMU ---- */
/* MPU6050_AccelPitch() = atan2(-ax, sqrt(ay²+az²)) là góc quay quanh trục Y => vận tốc góc là gy.
 * Kiểm tra: nghiêng xe từ từ về phía trước, g_rawAccPitch và g_rawGyroRate phải CÙNG DẤU và DƯƠNG.
 * Nếu cảm biến gắn xoay 90° (trục X dọc trục bánh xe) thì đổi sang AccelRoll() + gx. */
#define IMU_PITCH_SIGN          (1.0f)    /* Đảo nếu nghiêng tới mà pitch âm          */
#define IMU_GYRO_SIGN           (1.0f)    /* Đảo nếu pitch và rate ngược dấu          */
#define IMU_DT                  (0.005f)  /* 200 Hz                                    */

/* ---- Vòng góc (Fuzzy-PD) ---- */
#define BAL_ANGLE_OFFSET        (0.0f)    /* Góc cân bằng cơ khí MẶC ĐỊNH [deg]. Giá trị thực dùng
                                             là g_angleOffset: tự nạp từ Flash nếu đã chạy
                                             hiệu chuẩn LEVEL (nhấn ngắn nút USER)          */
#define BAL_OUTPUT_SIGN         (1.0f)    /* Chỉ đảo nếu đã kiểm 3 quy ước mà vẫn ngược */

/* ---- An toàn ---- */
#define BAL_FALL_ANGLE          (35.0f)   /* |góc| vượt mức này => coi như đổ, tắt motor */
#define BAL_UPRIGHT_ANGLE       (5.0f)    /* |góc| dưới mức này mới cho phép chạy        */
#define BAL_UPRIGHT_RATE        (20.0f)   /* ... và đang đứng yên (|rate| < 20 deg/s)    */
#define BAL_UPRIGHT_TIME_MS     (500u)    /* ... giữ liên tục 0.5 s                      */
#define BAL_IMU_TIMEOUT_MS      (20u)     /* Mất dữ liệu IMU quá 20 ms (4 mẫu) => tắt    */

/* ---- Vòng tốc độ (PID gia số): RPM -> góc đặt [deg] ---- */
#define SPEED_DT                (0.010f)
#define SPEED_KP                (0.05f)   /* 60 RPM sai lệch -> 3 deg                    */
#define SPEED_KI                (0.02f)   /* deg / (RPM.s)                               */
#define SPEED_KD                (0.0f)
#define SPEED_ANGLE_LIMIT       (6.0f)    /* Code cũ ±100 deg => chắc chắn đổ            */

/* ---- Đèn LED trên STM32F4-Discovery ---- */
#define LED_GREEN               GPIO_Pin_12  /* RUNNING                     */
#define LED_ORANGE              GPIO_Pin_13  /* WAIT_UPRIGHT                */
#define LED_RED                 GPIO_Pin_14  /* FAULT / stack overflow      */
#define LED_BLUE                GPIO_Pin_15  /* Nhịp tim IMU (nháy ~1 Hz)   */

/* Biến Controller */
extern FuzzyController_t fuzzyControl;
PID_t speedPID;
Kalman_t kalmanFilter;

/* Các biến toàn cục chứa dữ liệu thực để tiện Monitor trên Keil C */
volatile float g_rawAccPitch = 0.0f;
volatile float g_rawGyroRate = 0.0f;
volatile float g_filteredPitch = 0.0f; /* Góc thực sau đọc cảm biến và lọc Kalman */
volatile float g_filteredRate = 0.0f;  /* Vận tốc góc đã khử bias (Kalman)       */
volatile float g_motorSpeed = 0.0f;
volatile float g_motorPWM = 0.0f;
volatile int32_t g_wheelPosition = 0;  /* Tổng count trung bình 2 bánh (cho giữ vị trí sau này) */

volatile uint8_t  g_imuStatus = 0xFF;      /* 0 = OK, khác 0 = mã lỗi MPU_ERR_x / I2C_ERR_x */
volatile uint8_t  g_imuCalibStatus = 0xFF;
volatile BalanceState_t g_balanceState = BAL_WAIT_UPRIGHT;
volatile uint32_t g_fallCount = 0;         /* Số lần xe đổ                    */
volatile uint32_t g_imuTimeoutCount = 0;   /* Số lần mất dữ liệu IMU          */

/* ---- Hiệu chuẩn IMU (xem/ghi trên Watch window) ---- */
volatile uint8_t  g_calibRequest = CALIB_REQ_NONE; /* Ghi 1/2/3 để yêu cầu (xem managetask.h) */
volatile uint8_t  g_calibStatus  = 0;      /* CalibStatus_t: 0 idle, 1 đang chạy, 2 OK, 3 lỗi */
volatile uint8_t  g_calibError   = 0;      /* CALIB_ERR_x của lần gần nhất                  */
volatile uint8_t  g_calibFaces   = 0;      /* Bitmask 6 mặt đã thu (0x3F = đủ)              */
volatile uint32_t g_calibFlags   = 0;      /* IMU_CALIB_HAS_x đang có trong Flash           */
volatile float    g_angleOffset  = BAL_ANGLE_OFFSET;  /* Góc cân bằng đang dùng [deg]       */

/* Các biến liên kết giữa 2 vòng lặp (Cascaded Loop) */
volatile float g_targetSpeed = 0.0f;   /* [RPM] đặt từ remote sau này      */
volatile float g_targetAngle = 0.0f;   /* [deg] = offset + đầu ra vòng tốc độ */
volatile float g_turn = 0.0f;          /* [%PWM] thành phần rẽ             */

static TaskHandle_t s_controlTask = NULL;

/* Task Prototypes */
static void vTaskIMU(void *pvParameters);
static void vTaskSpeed(void *pvParameters);
static void vTaskControl(void *pvParameters);

static void LED_Show(BalanceState_t st)
{
    GPIO_ResetBits(GPIOD, LED_GREEN | LED_ORANGE | LED_RED);
    if (st == BAL_RUNNING)            GPIO_SetBits(GPIOD, LED_GREEN);
    else if (st == BAL_WAIT_UPRIGHT)  GPIO_SetBits(GPIOD, LED_ORANGE);
    else                              GPIO_SetBits(GPIOD, LED_RED);
}

static float absf(float x) { return (x < 0.0f) ? -x : x; }

void System_Tasks_Init(void) {
    uint8_t retry;
    BaseType_t ok = pdPASS;

    /* 0. Motor tắt cứng cho tới khi xe được dựng thẳng */
    Motor_Enable(DISABLE);

    /* 1. Khởi tạo bus I2C, encoder và các thuật toán */
    g_imuStatus = I2C1_MPU6050_Init();
    if (g_imuStatus == I2C_OK)
    {
        /* Nạp hiệu chuẩn đã lưu (accel offset/scale, góc cân bằng, bias gyro dự phòng) */
        g_calibFlags = ImuCalib_Init();
        if (g_calibFlags & IMU_CALIB_HAS_ANGLE)
        {
            g_angleOffset = ImuCalib_Get()->angle_offset;
        }

        /* Đo offset gyro: GIỮ XE ĐỨNG YÊN ~1s sau khi cấp điện.
         * Nếu thất bại cả 3 lần (xe bị rung) thì giữ bias gyro từ Flash (nếu có). */
        for (retry = 0u; retry < 3u; retry++)
        {
            g_imuCalibStatus = MPU6050_CalibrateGyro(200u);
            if (g_imuCalibStatus == I2C_OK) break;
        }
    }
    Encoder_Config();
    Kalman_Init(&kalmanFilter);

    /* 2. Cấu hình PID Tốc độ */
    PID_Init(&speedPID, SPEED_KP, SPEED_KI, SPEED_KD, -SPEED_ANGLE_LIMIT, SPEED_ANGLE_LIMIT);

    /* 3. Tạo các Task cho RTOS
     *    IMU (4) đọc + lọc xong thì ĐÁNH THỨC Control (3) ngay => không trễ thêm 1 chu kỳ.
     *    Control tạo TRƯỚC để IMU có handle hợp lệ. */
    if (xTaskCreate(vTaskControl, "Task_Control", 256, NULL, 3, &s_controlTask) != pdPASS) ok = pdFAIL;
    if (xTaskCreate(vTaskSpeed,   "Task_Speed",   256, NULL, 2, NULL)           != pdPASS) ok = pdFAIL;
    if (g_imuStatus == I2C_OK)
    {
        if (xTaskCreate(vTaskIMU, "Task_IMU", 256, NULL, 4, NULL) != pdPASS) ok = pdFAIL;
    }

    g_balanceState = ((g_imuStatus == I2C_OK) && (ok == pdPASS)) ? BAL_WAIT_UPRIGHT : BAL_FAULT;
    LED_Show(g_balanceState);
}

/* ---------------------------------------------------------------------
 *  TASK IMU - 200 Hz: đọc MPU6050 -> Kalman -> đánh thức Control
 * --------------------------------------------------------------------- */
static void vTaskIMU(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    MPU6050_Data_t imu;
    uint8_t firstSample = 1u;
    uint16_t beat = 0u;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* Đọc accel + gyro trong CÙNG MỘT giao dịch I2C (cùng thời điểm lấy mẫu).
         * Nếu lỗi: bỏ qua chu kỳ này, KHÔNG đưa góc 0 giả vào Kalman, KHÔNG đánh thức
         * Control => Control tự tắt motor nếu lỗi kéo dài quá BAL_IMU_TIMEOUT_MS. */
        if (MPU6050_Read(&imu) == I2C_OK)
        {
            g_rawAccPitch = IMU_PITCH_SIGN * MPU6050_AccelPitch(&imu);
            g_rawGyroRate = IMU_PITCH_SIGN * IMU_GYRO_SIGN * imu.gy;

            if (firstSample)
            {
                /* Khởi tạo Kalman bằng góc gia tốc kế để không phải chờ hội tụ từ 0 */
                kalmanFilter.angle = g_rawAccPitch;
                firstSample = 0u;
            }

            /* Chạy qua bộ lọc Kalman (dt = 0.005s) */
            g_filteredPitch = Kalman_GetAngle(&kalmanFilter, g_rawAccPitch, g_rawGyroRate, IMU_DT);
            g_filteredRate  = kalmanFilter.rate;

            /* Đang hiệu chuẩn: cấp mẫu cho quy trình (có thể ghi Flash ~1-2 s ở mẫu cuối,
             * lúc đó motor đã tắt vì Control đang ở BAL_CALIB) */
            if (ImuCalib_GetStatus() == CALIB_STATUS_RUNNING)
            {
                ImuCalib_Feed(&imu, g_rawAccPitch);
            }

            if (s_controlTask != NULL)
            {
                xTaskNotifyGive(s_controlTask);
            }

            if (++beat >= 100u)   /* 100 x 5ms = 0.5s */
            {
                beat = 0u;
                GPIO_ToggleBits(GPIOD, LED_BLUE);
            }
        }
    }
}

/* ---------------------------------------------------------------------
 *  TASK SPEED - 100 Hz: encoder -> PID tốc độ -> góc đặt cho vòng trong
 * --------------------------------------------------------------------- */
static void vTaskSpeed(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    Encoder_t leftMotor;
    Encoder_t rightMotor;
    float speedOut;

    Encoder_Reset(&leftMotor, &rightMotor);   /* Lấy mốc CNT hiện tại, tránh nhảy ở mẫu đầu */

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        Encoder_Update(&leftMotor, &rightMotor, SPEED_DT);
        g_motorSpeed    = (leftMotor.speed + rightMotor.speed) / 2.0f;     /* RPM đã lọc */
        g_wheelPosition = (leftMotor.position + rightMotor.position) / 2;

        if (g_balanceState == BAL_RUNNING)
        {
            speedOut = PID_Compute(&speedPID, g_targetSpeed, g_motorSpeed, SPEED_DT);
        }
        else
        {
            /* Không chạy => xóa trạng thái để lần bật sau bắt đầu sạch */
            PID_Reset(&speedPID);
            speedOut = 0.0f;
        }

        g_targetAngle = g_angleOffset + speedOut;

        /* Nút USER: nhấn ngắn = LEVEL, giữ 2 s = ACCEL6 (Control quyết định có nhận hay không) */
        {
            ButtonEvent_t evt = Button_Poll();
            if (g_calibRequest == CALIB_REQ_NONE)
            {
                if (evt == BTN_EVT_SHORT)      g_calibRequest = CALIB_REQ_LEVEL;
                else if (evt == BTN_EVT_LONG)  g_calibRequest = CALIB_REQ_ACCEL6;
            }
        }
    }
}

/* ---------------------------------------------------------------------
 *  TASK CONTROL - chạy ngay sau mỗi mẫu IMU: máy trạng thái + Fuzzy-PD
 * --------------------------------------------------------------------- */
static void vTaskControl(void *pvParameters)
{
    (void)pvParameters;
    const uint32_t uprightNeed = BAL_UPRIGHT_TIME_MS / 5u;   /* số chu kỳ 5ms */
    uint32_t uprightCnt = 0u;
    BalanceState_t shown = (BalanceState_t)0xFF;
    uint8_t    showingResult = 0u;
    TickType_t resultUntil = 0u;
    TickType_t captureFlashUntil = 0u;
    uint32_t   lastCaptures = ImuCalib_GetCaptureCount();

    while (1)
    {
        /* Chờ IMU báo có mẫu mới (có timeout để phát hiện IMU chết) */
        uint32_t   got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BAL_IMU_TIMEOUT_MS));
        TickType_t now = xTaskGetTickCount();

        /* ---- Yêu cầu hiệu chuẩn: CHỈ nhận khi motor đang tắt (WAIT_UPRIGHT) ---- */
        if (g_calibRequest != CALIB_REQ_NONE)
        {
            uint8_t req = g_calibRequest;
            CalibProc_t proc = CALIB_PROC_NONE;
            g_calibRequest = CALIB_REQ_NONE;

            if (req == CALIB_REQ_LEVEL)       proc = CALIB_PROC_LEVEL;
            else if (req == CALIB_REQ_ACCEL6) proc = CALIB_PROC_ACCEL6;
            else if (req == CALIB_REQ_ERASE)  proc = CALIB_PROC_ERASE;

            if ((g_balanceState == BAL_WAIT_UPRIGHT) && (proc != CALIB_PROC_NONE) &&
                ImuCalib_Start(proc))
            {
                Motor_Enable(DISABLE);
                g_motorPWM    = 0.0f;
                uprightCnt    = 0u;
                showingResult = 0u;
                lastCaptures  = ImuCalib_GetCaptureCount();
                g_balanceState = BAL_CALIB;
            }
        }

        if (got == 0u)
        {
            g_imuTimeoutCount++;
            Motor_Enable(DISABLE);
            g_motorPWM = 0.0f;
            uprightCnt = 0u;
            /* CALIB: IMU có thể đang ghi Flash (CPU treo ~1-2 s) -> giữ nguyên trạng thái */
            if (g_balanceState == BAL_RUNNING)
            {
                g_balanceState = BAL_WAIT_UPRIGHT;
            }
        }
        else
        {
            float pitch = g_filteredPitch;
            float rate  = g_filteredRate;
            float tilt  = absf(pitch - g_angleOffset);

            switch (g_balanceState)
            {
            case BAL_WAIT_UPRIGHT:
                /* Chỉ cho chạy khi xe được dựng thẳng VÀ đứng yên đủ lâu */
                if ((tilt < BAL_UPRIGHT_ANGLE) && (absf(rate) < BAL_UPRIGHT_RATE))
                {
                    if (++uprightCnt >= uprightNeed)
                    {
                        uprightCnt = 0u;
                        Motor_Enable(ENABLE);
                        g_balanceState = BAL_RUNNING;
                    }
                }
                else
                {
                    uprightCnt = 0u;
                }
                g_motorPWM = 0.0f;
                break;

            case BAL_RUNNING:
                if (tilt > BAL_FALL_ANGLE)
                {
                    /* Xe đổ: cắt PWM bằng phần cứng ngay lập tức */
                    Motor_Enable(DISABLE);
                    g_motorPWM = 0.0f;
                    g_fallCount++;
                    g_balanceState = BAL_WAIT_UPRIGHT;
                }
                else
                {
                    /* e > 0 khi xe nghiêng tới so với góc đặt; de = de/dt = rate
                     * (bỏ qua đạo hàm của g_targetAngle vì nó thay đổi chậm) */
                    float errorAngle  = pitch - g_targetAngle;
                    float dErrorAngle = rate;
                    float outputPWM   = BAL_OUTPUT_SIGN *
                                        Fuzzy_Compute(&fuzzyControl, errorAngle, dErrorAngle);

                    /* Bù vùng chết đã chuyển vào driver (MOTOR_x_DEADZONE, ánh xạ liên tục)
                     * thay cho kiểu cũ "u > 2 thì cộng 15" gây nhảy bậc 0 -> 17% */
                    Motor_SetPWM_Diff(outputPWM, g_turn);
                    g_motorPWM = outputPWM;
                }
                break;

            case BAL_CALIB:
                break;   /* Xử lý bên dưới (chạy cả khi IMU timeout) */

            case BAL_FAULT:
            default:
                Motor_Enable(DISABLE);
                g_motorPWM = 0.0f;
                break;
            }
        }

        /* ---- Theo dõi quy trình hiệu chuẩn + LED ---- */
        if (g_balanceState == BAL_CALIB)
        {
            CalibStatus_t cs = ImuCalib_GetStatus();
            g_calibStatus = (uint8_t)cs;
            g_calibFaces  = ImuCalib_GetFaces();

            if (cs == CALIB_STATUS_RUNNING)
            {
                /* Cam nháy 4 Hz = đang đo; xanh lá sáng 0.4 s mỗi khi thu xong 1 mặt (ACCEL6) */
                if (ImuCalib_GetCaptureCount() != lastCaptures)
                {
                    lastCaptures = ImuCalib_GetCaptureCount();
                    captureFlashUntil = now + pdMS_TO_TICKS(400);
                }
                GPIO_ResetBits(GPIOD, LED_RED | LED_GREEN | LED_ORANGE);
                if (((now / pdMS_TO_TICKS(125)) & 1u) != 0u) GPIO_SetBits(GPIOD, LED_ORANGE);
                if ((int32_t)(captureFlashUntil - now) > 0)  GPIO_SetBits(GPIOD, LED_GREEN);
            }
            else if ((cs == CALIB_STATUS_OK) || (cs == CALIB_STATUS_FAIL))
            {
                if (!showingResult)
                {
                    showingResult = 1u;
                    resultUntil   = now + pdMS_TO_TICKS(1500);
                    g_calibError  = ImuCalib_GetError();
                    g_calibFlags  = ImuCalib_Get()->flags;
                    /* Áp góc cân bằng mới (ACCEL6/ERASE xóa góc cũ -> về mặc định) */
                    g_angleOffset = (g_calibFlags & IMU_CALIB_HAS_ANGLE) ?
                                    ImuCalib_Get()->angle_offset : BAL_ANGLE_OFFSET;
                }

                /* Kết quả 1.5 s: OK = xanh lá + cam sáng; LỖI = đỏ nháy */
                GPIO_ResetBits(GPIOD, LED_RED | LED_GREEN | LED_ORANGE);
                if (cs == CALIB_STATUS_OK)
                {
                    GPIO_SetBits(GPIOD, LED_GREEN | LED_ORANGE);
                }
                else if (((now / pdMS_TO_TICKS(125)) & 1u) != 0u)
                {
                    GPIO_SetBits(GPIOD, LED_RED);
                }

                if ((int32_t)(now - resultUntil) >= 0)
                {
                    ImuCalib_ClearStatus();
                    showingResult  = 0u;
                    uprightCnt     = 0u;
                    g_balanceState = BAL_WAIT_UPRIGHT;
                }
            }
            else
            {
                g_balanceState = BAL_WAIT_UPRIGHT;
            }
            shown = (BalanceState_t)0xFF;   /* Ép cập nhật LED khi rời CALIB */
        }
        else if (shown != g_balanceState)
        {
            shown = g_balanceState;
            LED_Show(shown);
        }
    }
}

/* ---------------------------------------------------------------------
 *  FreeRTOS hooks: lỗi nghiêm trọng => tắt motor, bật LED đỏ, dừng hẳn
 * --------------------------------------------------------------------- */
static void System_Halt(void)
{
    taskDISABLE_INTERRUPTS();
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    GPIO_ResetBits(GPIOD, LED_GREEN | LED_ORANGE | LED_BLUE);
    GPIO_SetBits(GPIOD, LED_RED);
    while (1)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;   /* Đặt breakpoint ở đây để xem task nào tràn stack */
    System_Halt();
}

void vApplicationMallocFailedHook(void)
{
    System_Halt();
}
