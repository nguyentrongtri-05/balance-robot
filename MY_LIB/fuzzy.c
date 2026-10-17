#include "fuzzy.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * BẢNG LUẬT MỜ 5x5 (FUZZY RULE BASE MATRIX)
 *  - Hàng (Rows i = 0..4): Sai số góc nghiêng e   {NB, NS, ZE, PS, PB}
 *  - Cột  (Cols j = 0..4): Vận tốc góc nghiêng de {NB, NS, ZE, PS, PB}
 * ============================================================================ */
static const FuzzyState_t s_FuzzyRuleBase[FUZZY_NUM_STATES][FUZZY_NUM_STATES] = {
    /* de =       NB,  NS,  ZE,  PS,  PB */
    /* e = NB */ { NB,  NB,  NB,  NS,  ZE },
    /* e = NS */ { NB,  NB,  NS,  ZE,  PS },
    /* e = ZE */ { NB,  NS,  ZE,  PS,  PB },
    /* e = PS */ { NS,  ZE,  PS,  PB,  PB },
    /* e = PB */ { ZE,  PS,  PB,  PB,  PB }
};

/* Hàm tiện ích kẹp bão hòa giá trị thực trong đoạn [minVal, maxVal] */
static float Fuzzy_Clamp(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

/* ============================================================================
 * KHỞI TẠO & CẤU HÌNH THÔNG SỐ BỘ ĐIỀU KHIỂN MỜ
 * ============================================================================ */

void Fuzzy_Reset(FuzzyController_t *fc) {
    if (!fc) return;

    fc->de_filtered_state = 0.0f;
    fc->post_prev_output  = 0.0f;
    fc->setpoint          = 0.0f;
    fc->current_val       = 0.0f;
    fc->last_raw_error    = 0.0f;
    fc->prev_raw_error    = 0.0f;
    fc->last_raw_derror   = 0.0f;
    fc->last_e_norm       = 0.0f;
    fc->last_de_norm      = 0.0f;
    fc->last_defuzz_out   = 0.0f;
    fc->last_final_out    = 0.0f;

    memset(fc->mu_e,         0, sizeof(fc->mu_e));
    memset(fc->mu_de,        0, sizeof(fc->mu_de));
    memset(fc->rule_weights, 0, sizeof(fc->rule_weights));
    memset(fc->mu_out,       0, sizeof(fc->mu_out));
}

void Fuzzy_Init(FuzzyController_t *fc) {
    if (!fc) return;

    /* 1. Khởi tạo tham số Khâu Tiền Xử Lý từ #define */
    fc->ke                = FUZZY_PRE_KE;
    fc->kde               = FUZZY_PRE_KDE;
    fc->e_min             = FUZZY_PRE_E_MIN;
    fc->e_max             = FUZZY_PRE_E_MAX;
    fc->de_min            = FUZZY_PRE_DE_MIN;
    fc->de_max            = FUZZY_PRE_DE_MAX;
    fc->e_deadzone        = FUZZY_PRE_E_DEADZONE;
    fc->de_lpf_alpha      = FUZZY_PRE_DE_LPF_ALPHA;

    /* 2. Khởi tạo miền Hàm Liên Thuộc Khâu Mờ Hóa [-1.0 .. +1.0] từ #define */
    fc->e_limit_nb_ns     = FUZZY_MF_E_NB_NS;
    fc->e_limit_ns_ze     = FUZZY_MF_E_NS_ZE;
    fc->e_limit_ze_ps     = FUZZY_MF_E_ZE_PS;
    fc->e_limit_ps_pb     = FUZZY_MF_E_PS_PB;

    fc->de_limit_nb_ns    = FUZZY_MF_DE_NB_NS;
    fc->de_limit_ns_ze    = FUZZY_MF_DE_NS_ZE;
    fc->de_limit_ze_ps    = FUZZY_MF_DE_ZE_PS;
    fc->de_limit_ps_pb    = FUZZY_MF_DE_PS_PB;

    /* 3. Khởi tạo dải giá trị & Singleton Khâu Giải Mờ [-1.0 .. +1.0] từ #define */
    fc->defuzz_min        = FUZZY_DEFUZZ_RANGE_MIN;
    fc->defuzz_max        = FUZZY_DEFUZZ_RANGE_MAX;
    fc->out_nb            = FUZZY_DEFUZZ_OUT_NB;
    fc->out_ns            = FUZZY_DEFUZZ_OUT_NS;
    fc->out_ze            = FUZZY_DEFUZZ_OUT_ZE;
    fc->out_ps            = FUZZY_DEFUZZ_OUT_PS;
    fc->out_pb            = FUZZY_DEFUZZ_OUT_PB;

    /* 4. Khởi tạo tham số Khâu Hậu Xử Lý từ #define */
    fc->ku                = FUZZY_POST_KU;
    fc->post_out_min      = FUZZY_POST_OUT_MIN;
    fc->post_out_max      = FUZZY_POST_OUT_MAX;
    fc->post_ema_alpha    = FUZZY_POST_EMA_ALPHA;
    fc->post_max_delta    = FUZZY_POST_MAX_DELTA;

    /* 5. Xóa các biến trạng thái động */
    Fuzzy_Reset(fc);
}

void Fuzzy_SetPreProcessGains(FuzzyController_t *fc, float ke, float kde) {
    if (!fc) return;
    fc->ke  = ke;
    fc->kde = kde;
}

void Fuzzy_SetPreProcessLimits(FuzzyController_t *fc, float eMin, float eMax, float deMin, float deMax) {
    if (!fc) return;
    if (eMin < eMax) {
        fc->e_min = eMin;
        fc->e_max = eMax;
    }
    if (deMin < deMax) {
        fc->de_min = deMin;
        fc->de_max = deMax;
    }
}

void Fuzzy_SetMembershipLimits(FuzzyController_t *fc,
                               float e_nb_ns, float e_ns_ze, float e_ze_ps, float e_ps_pb,
                               float de_nb_ns, float de_ns_ze, float de_ze_ps, float de_ps_pb) {
    if (!fc) return;
    fc->e_limit_nb_ns  = e_nb_ns;
    fc->e_limit_ns_ze  = e_ns_ze;
    fc->e_limit_ze_ps  = e_ze_ps;
    fc->e_limit_ps_pb  = e_ps_pb;

    fc->de_limit_nb_ns = de_nb_ns;
    fc->de_limit_ns_ze = de_ns_ze;
    fc->de_limit_ze_ps = de_ze_ps;
    fc->de_limit_ps_pb = de_ps_pb;
}

void Fuzzy_SetDefuzzRange(FuzzyController_t *fc, float minVal, float maxVal, float smallRatio) {
    if (!fc || minVal >= maxVal) return;

    float ratio = Fuzzy_Clamp(smallRatio, 0.05f, 0.95f);
    float center = 0.5f * (minVal + maxVal);

    fc->defuzz_min = minVal;
    fc->defuzz_max = maxVal;
    fc->out_nb     = minVal;
    fc->out_ns     = center + (minVal - center) * ratio;
    fc->out_ze     = center;
    fc->out_ps     = center + (maxVal - center) * ratio;
    fc->out_pb     = maxVal;
}

void Fuzzy_SetDefuzzSingletons(FuzzyController_t *fc, float out_nb, float out_ns, float out_ze, float out_ps, float out_pb) {
    if (!fc) return;
    fc->out_nb     = out_nb;
    fc->out_ns     = out_ns;
    fc->out_ze     = out_ze;
    fc->out_ps     = out_ps;
    fc->out_pb     = out_pb;
    fc->defuzz_min = out_nb;
    fc->defuzz_max = out_pb;
}

void Fuzzy_SetPostProcessConfig(FuzzyController_t *fc, float ku, float outMin, float outMax, float emaAlpha, float maxDelta) {
    if (!fc) return;
    fc->ku             = ku;
    if (outMin < outMax) {
        fc->post_out_min = outMin;
        fc->post_out_max = outMax;
    }
    fc->post_ema_alpha = Fuzzy_Clamp(emaAlpha, 0.05f, 1.0f);
    fc->post_max_delta = (maxDelta > 0.0f) ? maxDelta : FUZZY_POST_MAX_DELTA;
}

/* ============================================================================
 * NHÓM HÀM TÍNH TOÁN HÀM LIÊN THUỘC CƠ SỞ (MEMBERSHIP FUNCTION PRIMITIVES)
 * ============================================================================ */

/**
 * @brief Hàm liên thuộc dạng Tam giác (Triangular Membership Function)
 *        Đạt đỉnh 1.0 tại x = b, bằng 0.0 khi x <= a hoặc x >= c
 */
float Fuzzy_MF_Triangle(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) {
        return (b > a) ? ((x - a) / (b - a)) : 0.0f;
    }
    return (c > b) ? ((c - x) / (c - b)) : 0.0f;
}

/**
 * @brief Hàm liên thuộc dạng Hình thang vai trái (Left-Shoulder Trapezoid - tập NB)
 *        Bằng 1.0 khi x <= a, giảm tuyến tính về 0.0 trên đoạn [a, b]
 */
float Fuzzy_MF_TrapezoidLeft(float x, float a, float b) {
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    return (b > a) ? ((b - x) / (b - a)) : 0.0f;
}

/**
 * @brief Hàm liên thuộc dạng Hình thang vai phải (Right-Shoulder Trapezoid - tập PB)
 *        Bằng 0.0 khi x <= a, tăng tuyến tính lên 1.0 trên đoạn [a, b]
 */
float Fuzzy_MF_TrapezoidRight(float x, float a, float b) {
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (b > a) ? ((x - a) / (b - a)) : 0.0f;
}

/**
 * @brief Hàm liên thuộc dạng Hình thang tổng quát 4 điểm (a <= b <= c <= d)
 */
float Fuzzy_MF_Trapezoid(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) {
        return (b > a) ? ((x - a) / (b - a)) : 1.0f;
    }
    return (d > c) ? ((d - x) / (d - c)) : 1.0f;
}

/* ============================================================================
 * KHÂU 1: TIỀN XỬ LÝ (PRE-PROCESSING STAGE)
 *  - Tính sai số vật lý từ giá trị đặt Setpoint: rawError = currentVal - setpoint
 *  - Lọc vùng chết sai số góc e_deadzone (nếu có)
 *  - Lọc thông thấp EMA vận tốc góc de_filtered_state giảm nhiễu rung Gyro
 *  - Khuếch đại chuẩn hóa qua ke, kde và kẹp vào miền [-1.0f .. +1.0f]
 * ============================================================================ */
void Fuzzy_PreProcess(FuzzyController_t *fc,
                      float setpoint,
                      float currentVal,
                      float dCurrentVal,
                      float *out_eNorm,
                      float *out_deNorm) {
    if (!fc) return;

    /* 1. Tính sai số thực tế từ Setpoint và lưu trạng thái */
    float rawError  = currentVal - setpoint;
    float rawDError = dCurrentVal;

    fc->setpoint        = setpoint;
    fc->current_val     = currentVal;
    fc->prev_raw_error  = fc->last_raw_error;
    fc->last_raw_error  = rawError;
    fc->last_raw_derror = rawDError;

    /* 2. Xử lý vùng chết nhỏ cho sai số e (nếu cấu hình e_deadzone > 0) */
    float e_clean = rawError;
    if (fc->e_deadzone > 0.0f) {
        if (fabsf(e_clean) <= fc->e_deadzone) {
            e_clean = 0.0f;
        } else if (e_clean > 0.0f) {
            e_clean -= fc->e_deadzone;
        } else {
            e_clean += fc->e_deadzone;
        }
    }

    /* 3. Lọc thông thấp EMA cho đạo hàm sai số de */
    float alpha = Fuzzy_Clamp(fc->de_lpf_alpha, 0.05f, 1.0f);
    fc->de_filtered_state = (1.0f - alpha) * fc->de_filtered_state + alpha * rawDError;

    /* 4. Khuếch đại tiền xử lý (ke, kde) và chuẩn hóa vào miền [-1.0 .. +1.0] */
    float e_scaled  = e_clean * fc->ke;
    float de_scaled = fc->de_filtered_state * fc->kde;

    e_scaled  = Fuzzy_Clamp(e_scaled,  fc->e_min,  fc->e_max);
    de_scaled = Fuzzy_Clamp(de_scaled, fc->de_min, fc->de_max);

    fc->last_e_norm  = e_scaled;
    fc->last_de_norm = de_scaled;

    if (out_eNorm)  *out_eNorm  = e_scaled;
    if (out_deNorm) *out_deNorm = de_scaled;
}

/* ============================================================================
 * KHÂU 2: MỜ HÓA (FUZZIFICATION STAGE TRÊN MIỀN [-1.0 .. +1.0])
 *  - Ánh xạ giá trị rõ đã tiền xử lý sang 5 tập mờ {NB, NS, ZE, PS, PB}
 *  - Đảm bảo phân hoạch đơn vị (Partition of Unity: tổng mu[i] = 1.0 tại mọi điểm)
 * ============================================================================ */
void Fuzzy_FuzzifyChannel(float x,
                          float lim_nb_ns,
                          float lim_ns_ze,
                          float lim_ze_ps,
                          float lim_ps_pb,
                          float mu[FUZZY_NUM_STATES]) {
    if (!mu) return;

    mu[NB] = Fuzzy_MF_TrapezoidLeft(x, lim_nb_ns, lim_ns_ze);
    mu[NS] = Fuzzy_MF_Triangle(x, lim_nb_ns, lim_ns_ze, 0.0f);
    mu[ZE] = Fuzzy_MF_Triangle(x, lim_ns_ze, 0.0f, lim_ze_ps);
    mu[PS] = Fuzzy_MF_Triangle(x, 0.0f, lim_ze_ps, lim_ps_pb);
    mu[PB] = Fuzzy_MF_TrapezoidRight(x, lim_ze_ps, lim_ps_pb);
}

void Fuzzy_Fuzzify(FuzzyController_t *fc,
                   float eNorm,
                   float deNorm,
                   float mu_e[FUZZY_NUM_STATES],
                   float mu_de[FUZZY_NUM_STATES]) {
    if (!fc) return;

    Fuzzy_FuzzifyChannel(eNorm,
                         fc->e_limit_nb_ns,
                         fc->e_limit_ns_ze,
                         fc->e_limit_ze_ps,
                         fc->e_limit_ps_pb,
                         mu_e);

    Fuzzy_FuzzifyChannel(deNorm,
                         fc->de_limit_nb_ns,
                         fc->de_limit_ns_ze,
                         fc->de_limit_ze_ps,
                         fc->de_limit_ps_pb,
                         mu_de);
}

/* ============================================================================
 * KHÂU 3: SUY LUẬN MỜ & HỢP THÀNH (FUZZY INFERENCE & AGGREGATION STAGE)
 *  - Duyệt ma trận 25 luật s_FuzzyRuleBase[5][5]
 *  - Toán tử suy luận AND dùng Tích đại số (Larsen Product: w_ij = mu_e[i] * mu_de[j])
 *    giúp mặt điều khiển liên tục, trơn mượt, không bị gãy khúc bậc thang
 *  - Hợp thành độ phụ thuộc cho từng nhãn đầu ra outMembership[NB..PB]
 * ============================================================================ */
void Fuzzy_Inference(FuzzyController_t *fc,
                     const float mu_e[FUZZY_NUM_STATES],
                     const float mu_de[FUZZY_NUM_STATES],
                     float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES],
                     float outMembership[FUZZY_NUM_STATES]) {
    uint8_t i, j;
    (void)fc;

    if (outMembership) {
        for (i = 0; i < FUZZY_NUM_STATES; i++) {
            outMembership[i] = 0.0f;
        }
    }

    for (i = 0; i < FUZZY_NUM_STATES; i++) {
        for (j = 0; j < FUZZY_NUM_STATES; j++) {
            float w = 0.0f;
            if (mu_e[i] > 0.0f && mu_de[j] > 0.0f) {
                w = mu_e[i] * mu_de[j];
            }

            if (ruleWeights) {
                ruleWeights[i][j] = w;
            }

            if (w > 0.0f && outMembership) {
                FuzzyState_t consequent = s_FuzzyRuleBase[i][j];
                outMembership[consequent] += w;
            }
        }
    }
}

/* ============================================================================
 * KHÂU 4: GIẢI MỜ (DEFUZZIFICATION STAGE RA MIỀN CHUẨN HÓA [-1.0 .. +1.0])
 *  - Giải mờ theo phương pháp Trung bình trọng số / Trọng tâm Singleton:
 *      u_defuzz = (Tổng w_ij * Singleton[RuleBase[i][j]]) / (Tổng w_ij)
 *  - Kết quả giải mờ thuộc [-1.0 .. +1.0] (hoặc dải [defuzz_min, defuzz_max] tùy chỉnh)
 * ============================================================================ */
float Fuzzy_Defuzzify(FuzzyController_t *fc, const float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]) {
    if (!fc || !ruleWeights) return 0.0f;

    float singletons[FUZZY_NUM_STATES] = {
        fc->out_nb,
        fc->out_ns,
        fc->out_ze,
        fc->out_ps,
        fc->out_pb
    };

    float numerator   = 0.0f;
    float denominator = 0.0f;
    uint8_t i, j;

    for (i = 0; i < FUZZY_NUM_STATES; i++) {
        for (j = 0; j < FUZZY_NUM_STATES; j++) {
            float w = ruleWeights[i][j];
            if (w > 0.0f) {
                FuzzyState_t outState = s_FuzzyRuleBase[i][j];
                numerator   += w * singletons[outState];
                denominator += w;
            }
        }
    }

    float defuzzOut = 0.0f;
    if (denominator > 1e-6f) {
        defuzzOut = numerator / denominator;
    }

    /* Kẹp trong dải giá trị giải mờ [defuzz_min, defuzz_max] (mặc định [-1.0 .. +1.0]) */
    defuzzOut = Fuzzy_Clamp(defuzzOut, fc->defuzz_min, fc->defuzz_max);
    fc->last_defuzz_out = defuzzOut;

    return defuzzOut;
}

/* ============================================================================
 * KHÂU 5: HẬU XỬ LÝ (POST-PROCESSING STAGE)
 *  - Khuếch đại giá trị giải mờ [-1.0 .. +1.0] qua hệ số hậu xử lý ku ra PWM
 *  - Lọc thông thấp EMA (post_ema_alpha) làm mượt xung điều khiển
 *  - Giới hạn tốc độ biến thiên mỗi chu kỳ (post_max_delta)
 *  - Kẹp bão hòa đầu ra trong dải [post_out_min, post_out_max]
 * ============================================================================ */
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput) {
    if (!fc) return 0.0f;

    /* 1. Khuếch đại hậu xử lý bằng hệ số ku */
    float scaledOut = defuzzOutput * fc->ku;

    /* 2. Lọc thông thấp EMA làm mượt tín hiệu điều khiển */
    float alpha = Fuzzy_Clamp(fc->post_ema_alpha, 0.05f, 1.0f);
    float smoothedOut = alpha * scaledOut + (1.0f - alpha) * fc->post_prev_output;

    /* 3. Giới hạn bước nhảy tối đa mỗi chu kỳ (Slew-Rate Limiter hậu xử lý) */
    if (fc->post_max_delta > 0.0f) {
        float delta = smoothedOut - fc->post_prev_output;
        if (delta > fc->post_max_delta) {
            smoothedOut = fc->post_prev_output + fc->post_max_delta;
        } else if (delta < -fc->post_max_delta) {
            smoothedOut = fc->post_prev_output - fc->post_max_delta;
        }
    }

    /* 4. Bão hòa đầu ra trong ngưỡng [post_out_min, post_out_max] */
    float finalOut = Fuzzy_Clamp(smoothedOut, fc->post_out_min, fc->post_out_max);

    fc->post_prev_output = finalOut;
    fc->last_final_out   = finalOut;

    return finalOut;
}

/* ============================================================================
 * HÀM ĐIỀU KHIỂN FUZZY TỔNG HỢP (NHẬN SETPOINT, GIÁ TRỊ HIỆN TẠI & VẬN TỐC GÓC)
 * ============================================================================ */
float Fuzzy_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal) {
    if (!fc) return 0.0f;

    float eNorm  = 0.0f;
    float deNorm = 0.0f;

    /* Bước 1: Khâu Tiền Xử Lý (Tính e = currentVal - setpoint, lọc, khuếch đại ke/kde -> [-1..1]) */
    Fuzzy_PreProcess(fc, setpoint, currentVal, dCurrentVal, &eNorm, &deNorm);

    /* Bước 2: Khâu Mờ Hóa trên miền [-1.0 .. +1.0] (Fuzzification) */
    Fuzzy_Fuzzify(fc, eNorm, deNorm, fc->mu_e, fc->mu_de);

    /* Bước 3: Khâu Suy Luận Mờ trên Bảng Luật 5x5 (Inference) */
    Fuzzy_Inference(fc, fc->mu_e, fc->mu_de, fc->rule_weights, fc->mu_out);

    /* Bước 4: Khâu Giải Mờ ra miền [-1.0 .. +1.0] (Defuzzification) */
    float defuzzOut = Fuzzy_Defuzzify(fc, fc->rule_weights);

    /* Bước 5: Khâu Khuếch Đại Hậu Xử Lý (Nhân ku -> PWM, lọc mượt & bão hòa) */
    float finalOut  = Fuzzy_PostProcess(fc, defuzzOut);

    return finalOut;
}

float Fuzzy_ComputeWithDt(FuzzyController_t *fc, float setpoint, float currentVal, float dt) {
    if (!fc) return 0.0f;

    float rawError = currentVal - setpoint;
    float dError   = (dt > 1e-6f) ? ((rawError - fc->last_raw_error) / dt) : 0.0f;

    return Fuzzy_Compute(fc, setpoint, currentVal, dError);
}
