    #ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

/* ============================================================================
 * 1. CẤU HÌNH #DEFINE KHÂU KHUẾCH ĐẠI TIỀN XỬ LÝ (PRE-PROCESSING AMPLIFICATION)
 *  - Khuếch đại / chuẩn hóa tín hiệu vật lý (error góc, dError vận tốc góc)
 *    để đưa vào miền chuẩn hóa [-1.0f .. +1.0f] của bộ Mờ:
 *      + e_norm  = clamp(error  * FUZZY_PRE_KE,  -1.0f, +1.0f)
 *      + de_norm = clamp(dError * FUZZY_PRE_KDE, -1.0f, +1.0f)
 * ============================================================================ */
#define FUZZY_PRE_KE                0.0500f     /* Hệ số khuếch đại tiền xử lý sai số góc e (ứng với dải ~ +-20 độ -> +-1.0) */
#define FUZZY_PRE_KDE               0.0013f     /* Hệ số khuếch đại tiền xử lý đạo hàm góc de (độ/giây -> miền [-1.0 .. +1.0]) */

#define FUZZY_PRE_E_MIN            -1.0f        /* Giới hạn dưới chuẩn hóa của đầu vào e_norm (-1.0) */
#define FUZZY_PRE_E_MAX             1.0f        /* Giới hạn trên chuẩn hóa của đầu vào e_norm (+1.0) */
#define FUZZY_PRE_DE_MIN           -1.0f        /* Giới hạn dưới chuẩn hóa của đầu vào de_norm (-1.0) */
#define FUZZY_PRE_DE_MAX            1.0f        /* Giới hạn trên chuẩn hóa của đầu vào de_norm (+1.0) */

#define FUZZY_PRE_E_DEADZONE        0.00f       /* Vùng chết lọc nhiễu vi mô sai số góc e trước khi khuếch đại (độ) */
#define FUZZY_PRE_DE_LPF_ALPHA      0.25f       /* Hệ số lọc thông thấp EMA cho đạo hàm de ở tiền xử lý (0.25 mẫu mới + 0.75 mẫu cũ) */

/* ============================================================================
 * 2. CẤU HÌNH #DEFINE MIỀN HÀM LIÊN THUỘC ĐẦU VÀO CHUẨN HÓA [-1.0 .. +1.0]
 *  - Tọa độ các đỉnh tập mờ {NB, NS, ZE, PS, PB} của e_norm và de_norm trong [-1.0 .. +1.0]
 * ============================================================================ */
/* Miền hàm liên thuộc cho Sai số góc đã chuẩn hóa e_norm thuộc [-1.0 .. +1.0] */
#define FUZZY_MF_E_NB_NS           -1.00f       /* Đỉnh tập NB của e_norm (-1.0) */
#define FUZZY_MF_E_NS_ZE           -0.40f       /* Đỉnh tập NS / chân trái tập ZE của e_norm */
#define FUZZY_MF_E_ZE_CENTER        0.00f       /* Tâm tập ZE của e_norm (0.0) */
#define FUZZY_MF_E_ZE_PS            0.40f       /* Đỉnh tập PS / chân phải tập ZE của e_norm */
#define FUZZY_MF_E_PS_PB            1.00f       /* Đỉnh tập PB của e_norm (+1.0) */

/* Miền hàm liên thuộc cho Đạo hàm sai số góc đã chuẩn hóa de_norm thuộc [-1.0 .. +1.0] */
#define FUZZY_MF_DE_NB_NS          -1.00f       /* Đỉnh tập NB của de_norm (-1.0) */
#define FUZZY_MF_DE_NS_ZE          -0.40f       /* Đỉnh tập NS / chân trái tập ZE của de_norm */
#define FUZZY_MF_DE_ZE_CENTER       0.00f       /* Tâm tập ZE của de_norm (0.0) */
#define FUZZY_MF_DE_ZE_PS           0.40f       /* Đỉnh tập PS / chân phải tập ZE của de_norm */
#define FUZZY_MF_DE_PS_PB           1.00f       /* Đỉnh tập PB của de_norm (+1.0) */

/* ============================================================================
 * 3. CẤU HÌNH #DEFINE DẢI GIÁ TRỊ KHÂU GIẢI MỜ CHUẨN HÓA [-1.0 .. +1.0]
 *  - Đầu ra của khâu giải mờ (u_defuzz) nằm trong dải [-1.0f .. +1.0f]
 *  - Điều chỉnh được biên [FUZZY_DEFUZZ_RANGE_MIN, FUZZY_DEFUZZ_RANGE_MAX]
 *    và vị trí các Singleton NB, NS, ZE, PS, PB trong [-1.0 .. +1.0]
 * ============================================================================ */
#define FUZZY_DEFUZZ_RANGE_MIN     -1.00f       /* Biên âm của khâu giải mờ (-1.0) */
#define FUZZY_DEFUZZ_RANGE_MAX      1.00f       /* Biên dương của khâu giải mờ (+1.0) */
#define FUZZY_DEFUZZ_SMALL_RATIO    0.30f       /* Tỉ lệ vùng nhỏ NS/PS trong [-1..1] (0.30 -> NS = -0.30, PS = +0.30 giúp êm quanh 0) */

#define FUZZY_DEFUZZ_OUT_NB        (FUZZY_DEFUZZ_RANGE_MIN)                             /* -1.00f */
#define FUZZY_DEFUZZ_OUT_NS        (FUZZY_DEFUZZ_RANGE_MIN * FUZZY_DEFUZZ_SMALL_RATIO)  /* -0.30f */
#define FUZZY_DEFUZZ_OUT_ZE        (0.00f)                                              /*  0.00f */
#define FUZZY_DEFUZZ_OUT_PS        (FUZZY_DEFUZZ_RANGE_MAX * FUZZY_DEFUZZ_SMALL_RATIO)  /* +0.30f */
#define FUZZY_DEFUZZ_OUT_PB        (FUZZY_DEFUZZ_RANGE_MAX)                             /* +1.00f */

/* ============================================================================
 * 4. CẤU HÌNH #DEFINE KHÂU KHUẾCH ĐẠI HẬU XỬ LÝ (POST-PROCESSING AMPLIFICATION)
 *  - Khuếch đại tín hiệu giải mờ u_defuzz thuộc [-1.0 .. +1.0] ra công suất PWM:
 *      + u_pwm = u_defuzz * FUZZY_POST_KU
 * ============================================================================ */
#define FUZZY_POST_KU            1000.0f        /* Hệ số khuếch đại hậu xử lý: nhân [-1.0 .. +1.0] ra dải PWM [-1000 .. +1000] */
#define FUZZY_POST_OUT_MIN      -1000.0f        /* Ngưỡng bão hòa dưới của tín hiệu sau hậu xử lý */
#define FUZZY_POST_OUT_MAX       1000.0f        /* Ngưỡng bão hòa trên của tín hiệu sau hậu xử lý */
#define FUZZY_POST_EMA_ALPHA        0.65f       /* Hệ số lọc mượt EMA hậu xử lý (1.0f = không lọc, 0.5~0.7 = lọc êm xung) */
#define FUZZY_POST_MAX_DELTA       60.0f        /* Giới hạn bước nhảy đầu ra tối đa mỗi chu kỳ ở khâu hậu xử lý */

/* Số lượng tập mờ ngôn ngữ (NB, NS, ZE, PS, PB) */
#define FUZZY_NUM_STATES             5u

/* Các nhãn ngôn ngữ mờ (Linguistic States):
 *  - NB: Negative Big   (Âm lớn)
 *  - NS: Negative Small (Âm nhỏ)
 *  - ZE: Zero           (Bằng không)
 *  - PS: Positive Small (Dương nhỏ)
 *  - PB: Positive Big   (Dương lớn)
 */
typedef enum {
    NB = 0,
    NS = 1,
    ZE = 2,
    PS = 3,
    PB = 4
} FuzzyState_t;

/* Cấu trúc lưu trữ toàn bộ tham số và trạng thái trung gian của các khâu Fuzzy */
typedef struct {
    /* --- 1. Tham số Khâu Tiền Xử Lý (Pre-processing) --- */
    float ke;               /* Hệ số tỉ lệ chuẩn hóa sai số e */
    float kde;              /* Hệ số tỉ lệ chuẩn hóa đạo hàm sai số de */
    float e_min;            /* Ngưỡng kẹp dưới cho e sau tiền xử lý */
    float e_max;            /* Ngưỡng kẹp trên cho e sau tiền xử lý */
    float de_min;           /* Ngưỡng kẹp dưới cho de sau tiền xử lý */
    float de_max;           /* Ngưỡng kẹp trên cho de sau tiền xử lý */
    float e_deadzone;       /* Vùng chết lọc nhiễu vi mô sai số e */
    float de_lpf_alpha;     /* Hệ số lọc thông thấp EMA cho đạo hàm de */
    float de_filtered_state;/* Biến trạng thái lưu giá trị lọc đạo hàm de */

    /* --- 2. Tham số Khâu Mờ Hóa (Fuzzification Membership Limits) --- */
    float e_limit_nb_ns;
    float e_limit_ns_ze;
    float e_limit_ze_ps;
    float e_limit_ps_pb;

    float de_limit_nb_ns;
    float de_limit_ns_ze;
    float de_limit_ze_ps;
    float de_limit_ps_pb;

    /* --- 3. Tham số Dải Giá Trị Khâu Giải Mờ (Defuzzification) --- */
    float defuzz_min;       /* Giới hạn dưới của dải giải mờ */
    float defuzz_max;       /* Giới hạn trên của dải giải mờ */
    float out_nb;           /* Singleton mức NB */
    float out_ns;           /* Singleton mức NS */
    float out_ze;           /* Singleton mức ZE */
    float out_ps;           /* Singleton mức PS */
    float out_pb;           /* Singleton mức PB */

    /* --- 4. Tham số Khâu Hậu Xử Lý (Post-processing) --- */
    float ku;               /* Hệ số khuếch đại hậu xử lý */
    float post_out_min;     /* Giới hạn bão hòa dưới của ngõ ra cuối cùng */
    float post_out_max;     /* Giới hạn bão hòa trên của ngõ ra cuối cùng */
    float post_ema_alpha;   /* Hệ số lọc mượt EMA ngõ ra hậu xử lý */
    float post_max_delta;   /* Giới hạn bước nhảy tối đa mỗi chu kỳ */
    float post_prev_output; /* Trạng thái đầu ra chu kỳ trước của khâu hậu xử lý */

    /* --- 5. Biến trung gian lưu kết quả từng khâu (phục vụ quan sát/debug) --- */
    float setpoint;                                 /* Giá trị đặt mục tiêu (Setpoint, ví dụ g_targetAngle) */
    float current_val;                              /* Giá trị đo hiện tại (Process Variable, ví dụ g_filteredPitch) */
    float last_raw_error;                           /* Sai số vật lý chưa chuẩn hóa: e = current_val - setpoint */
    float prev_raw_error;                           /* Sai số vật lý chu kỳ trước (dùng khi tính đạo hàm theo dt) */
    float last_raw_derror;                          /* Đạo hàm sai số vật lý chưa chuẩn hóa: de */
    float last_e_norm;                              /* Ngõ ra khâu Tiền xử lý kênh e thuộc [-1.0 .. +1.0] */
    float last_de_norm;                             /* Ngõ ra khâu Tiền xử lý kênh de thuộc [-1.0 .. +1.0] */
    float mu_e[FUZZY_NUM_STATES];                   /* Độ phụ thuộc mờ hóa của e */
    float mu_de[FUZZY_NUM_STATES];                  /* Độ phụ thuộc mờ hóa của de */
    float rule_weights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]; /* Trọng số kích hoạt 25 luật */
    float mu_out[FUZZY_NUM_STATES];                 /* Độ phụ thuộc hợp thành của 5 tập đầu ra */
    float last_defuzz_out;                          /* Ngõ ra rõ của khâu Giải mờ thuộc [-1.0 .. +1.0] */
    float last_final_out;                           /* Ngõ ra cuối cùng của khâu Hậu xử lý (PWM) */

} FuzzyController_t;

/* ============================================================================
 * CÁC HÀM TÍNH TOÁN CƠ SỞ & TỪNG KHÂU CỦA THUẬT TOÁN FUZZY
 * ============================================================================ */

/**
 * @brief Khởi tạo bộ điều khiển Fuzzy với toàn bộ thông số từ các #define mặc định
 */
void Fuzzy_Init(FuzzyController_t *fc);

/**
 * @brief Xóa trạng thái lọc nội bộ (tiền xử lý & hậu xử lý) khi dừng xe
 */
void Fuzzy_Reset(FuzzyController_t *fc);

/* --- Nhóm hàm tính toán Hàm Liên Thuộc (Membership Function Primitives) --- */
float Fuzzy_MF_Triangle(float x, float a, float b, float c);
float Fuzzy_MF_TrapezoidLeft(float x, float a, float b);
float Fuzzy_MF_TrapezoidRight(float x, float a, float b);
float Fuzzy_MF_Trapezoid(float x, float a, float b, float c, float d);

/* --- Khâu 1: Tiền xử lý (Pre-processing: tính sai số từ Setpoint, lọc, khuếch đại ke/kde và chuẩn hóa về [-1..1]) --- */
void Fuzzy_PreProcess(FuzzyController_t *fc,
                      float setpoint,
                      float currentVal,
                      float dCurrentVal,
                      float *out_eNorm,
                      float *out_deNorm);

/* --- Khâu 2: Mờ hóa (Fuzzification trên miền chuẩn hóa [-1.0 .. +1.0]) --- */
void Fuzzy_FuzzifyChannel(float x, float lim_nb_ns, float lim_ns_ze, float lim_ze_ps, float lim_ps_pb, float mu[FUZZY_NUM_STATES]);
void Fuzzy_Fuzzify(FuzzyController_t *fc, float eNorm, float deNorm, float mu_e[FUZZY_NUM_STATES], float mu_de[FUZZY_NUM_STATES]);

/* --- Khâu 3: Suy luận mờ trên bảng luật 5x5 (Fuzzy Inference) --- */
void Fuzzy_Inference(FuzzyController_t *fc,
                     const float mu_e[FUZZY_NUM_STATES],
                     const float mu_de[FUZZY_NUM_STATES],
                     float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES],
                     float outMembership[FUZZY_NUM_STATES]);

/* --- Khâu 4: Giải mờ (Defuzzification ra miền chuẩn hóa [-1.0 .. +1.0]) --- */
float Fuzzy_Defuzzify(FuzzyController_t *fc, const float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]);

/* --- Khâu 5: Hậu xử lý (Post-processing: khuếch đại ku từ [-1..1] ra PWM, lọc mượt & kẹp bão hòa) --- */
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput);

/**
 * @brief Hàm điều khiển tổng hợp chạy trọn vẹn 5 khâu (dùng khi đã có đạo hàm từ cảm biến như Gyro)
 * @param fc: Con trỏ cấu trúc FuzzyController_t
 * @param setpoint: Giá trị đặt mục tiêu (ví dụ: g_targetAngle)
 * @param currentVal: Giá trị đo thực tế hiện tại (ví dụ: g_filteredPitch)
 * @param dCurrentVal: Vận tốc biến thiên thực tế (ví dụ: kalmanPitch.rate từ Gyro)
 * @retval Tín hiệu điều khiển sau hậu xử lý (ví dụ: xung PWM [-1000 .. +1000])
 */
float Fuzzy_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal);

/**
 * @brief Hàm điều khiển tổng hợp tự tính đạo hàm sai số theo chu kỳ lấy mẫu dt (dùng khi không có Gyro)
 */
float Fuzzy_ComputeWithDt(FuzzyController_t *fc, float setpoint, float currentVal, float dt);

/* ============================================================================
 * CÁC HÀM HIỆU CHỈNH THÔNG SỐ TIỀN XỬ LÝ, DẢI GIẢI MỜ & HẬU XỬ LÝ
 * ============================================================================ */

/**
 * @brief Chỉnh sửa hệ số tiền xử lý (ke, kde)
 */
void Fuzzy_SetPreProcessGains(FuzzyController_t *fc, float ke, float kde);

/**
 * @brief Chỉnh sửa giới hạn kẹp miền xác định ở khâu tiền xử lý
 */
void Fuzzy_SetPreProcessLimits(FuzzyController_t *fc, float eMin, float eMax, float deMin, float deMax);

/**
 * @brief Điều chỉnh miền hàm liên thuộc đầu vào của e và de
 */
void Fuzzy_SetMembershipLimits(FuzzyController_t *fc,
                               float e_nb_ns, float e_ns_ze, float e_ze_ps, float e_ps_pb,
                               float de_nb_ns, float de_ns_ze, float de_ze_ps, float de_ps_pb);

/**
 * @brief Điều chỉnh dải giá trị trong khâu giải mờ [minVal, maxVal] và tỉ lệ vùng nhỏ smallRatio
 * @param minVal: Biên âm tối đa (gán cho NB, ví dụ -100.0f)
 * @param maxVal: Biên dương tối đa (gán cho PB, ví dụ +100.0f)
 * @param smallRatio: Tỉ lệ của NS/PS so với biên (0.05f .. 0.95f, ví dụ 0.28f -> NS = -28, PS = +28)
 */
void Fuzzy_SetDefuzzRange(FuzzyController_t *fc, float minVal, float maxVal, float smallRatio);

/**
 * @brief Điều chỉnh trực tiếp 5 giá trị Singleton trong khâu giải mờ (NB, NS, ZE, PS, PB)
 */
void Fuzzy_SetDefuzzSingletons(FuzzyController_t *fc, float out_nb, float out_ns, float out_ze, float out_ps, float out_pb);

/**
 * @brief Chỉnh sửa hệ số hậu xử lý (ku, dải bão hòa outMin/outMax, lọc mượt emaAlpha, giới hạn bước nhảy maxDelta)
 */
void Fuzzy_SetPostProcessConfig(FuzzyController_t *fc, float ku, float outMin, float outMax, float emaAlpha, float maxDelta);

#endif /* __FUZZY_H */
