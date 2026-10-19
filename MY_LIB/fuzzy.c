#include "fuzzy.h"
#include <string.h>

/* ============================================================================
 * BẢNG LUẬT MỜ 5x5 VỚI 7 BIẾN NGÔN NGỮ ĐẦU RA u (FUZZY RULE BASE MATRIX)
 *  - Hàng (Rows i = 0..4): Sai số góc nghiêng e   {NB, NS, ZE, PS, PB}
 *  - Cột  (Cols j = 0..4): Vận tốc góc nghiêng de {NB, NS, ZE, PS, PB}
 *  - Đầu ra u (7 mức)    : {NB, NM, NS, ZE, PS, PM, PB}
 * ============================================================================ */
static const FuzzyState_t s_FuzzyRuleBase[FUZZY_NUM_STATES][FUZZY_NUM_STATES] = {
    /* de =       NB,  NS,  ZE,  PS,  PB */
    /* e = NB */ { NB,  NB,  NM,  NS,  ZE },
    /* e = NS */ { NB,  NM,  NS,  ZE,  PS },
    /* e = ZE */ { NM,  NS,  ZE,  PS,  PM },
    /* e = PS */ { NS,  ZE,  PS,  PM,  PB },
    /* e = PB */ { ZE,  PS,  PM,  PB,  PB }
};

/* Hàm tiện ích kẹp bão hòa giá trị thực trong đoạn [minVal, maxVal] */
static float Fuzzy_Clamp(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

/* ============================================================================
 * KHỞI TẠO & RESET BỘ ĐIỀU KHIỂN MỜ
 * ============================================================================ */
void Fuzzy_Reset(FuzzyController_t *fc) {
    if (!fc) return;

    fc->de_filtered_state = 0.0f;
    fc->post_prev_output  = 0.0f;

    memset(fc->mu_e,         0, sizeof(fc->mu_e));
    memset(fc->mu_de,        0, sizeof(fc->mu_de));
    memset(fc->rule_weights, 0, sizeof(fc->rule_weights));
}

void Fuzzy_Init(FuzzyController_t *fc) {
    if (!fc) return;

    /* 1. Khởi tạo tham số Khâu Tiền Xử Lý (Góc cân bằng [-15° .. +15°]) */
    fc->ke                = FUZZY_PRE_KE;
    fc->kde               = FUZZY_PRE_KDE;
    fc->e_min             = FUZZY_PRE_E_MIN;
    fc->e_max             = FUZZY_PRE_E_MAX;
    fc->de_min            = FUZZY_PRE_DE_MIN;
    fc->de_max            = FUZZY_PRE_DE_MAX;
    fc->de_lpf_alpha      = FUZZY_PRE_DE_LPF_ALPHA;

    /* 2. Khởi tạo miền Hàm Liên Thuộc Khâu Mờ Hóa [-1.0 .. +1.0] */
    fc->e_limit_nb_ns     = FUZZY_MF_E_NB_NS;
    fc->e_limit_ns_ze     = FUZZY_MF_E_NS_ZE;
    fc->e_ze_flat_min     = FUZZY_MF_E_ZE_FLAT_MIN;
    fc->e_ze_flat_max     = FUZZY_MF_E_ZE_FLAT_MAX;
    fc->e_limit_ze_ps     = FUZZY_MF_E_ZE_PS;
    fc->e_limit_ps_pb     = FUZZY_MF_E_PS_PB;

    fc->de_limit_nb_ns    = FUZZY_MF_DE_NB_NS;
    fc->de_limit_ns_ze    = FUZZY_MF_DE_NS_ZE;
    fc->de_ze_flat_min    = FUZZY_MF_DE_ZE_FLAT_MIN;
    fc->de_ze_flat_max    = FUZZY_MF_DE_ZE_FLAT_MAX;
    fc->de_limit_ze_ps    = FUZZY_MF_DE_ZE_PS;
    fc->de_limit_ps_pb    = FUZZY_MF_DE_PS_PB;

    /* 3. Khởi tạo dải giá trị & 7 Singleton Khâu Giải Mờ [-1.0 .. +1.0] */
    fc->defuzz_min        = FUZZY_DEFUZZ_RANGE_MIN;
    fc->defuzz_max        = FUZZY_DEFUZZ_RANGE_MAX;
    fc->out_nb            = FUZZY_DEFUZZ_OUT_NB;
    fc->out_nm            = FUZZY_DEFUZZ_OUT_NM;
    fc->out_ns            = FUZZY_DEFUZZ_OUT_NS;
    fc->out_ze            = FUZZY_DEFUZZ_OUT_ZE;
    fc->out_ps            = FUZZY_DEFUZZ_OUT_PS;
    fc->out_pm            = FUZZY_DEFUZZ_OUT_PM;
    fc->out_pb            = FUZZY_DEFUZZ_OUT_PB;

    /* 4. Khởi tạo tham số Khâu Hậu Xử Lý (Vòng trong Cascade 90% PWM = +-7560.0f) */
    fc->ku                = FUZZY_POST_KU;
    fc->post_out_min      = FUZZY_POST_OUT_MIN;
    fc->post_out_max      = FUZZY_POST_OUT_MAX;
    fc->post_ema_alpha    = FUZZY_POST_EMA_ALPHA;
    fc->post_max_delta    = FUZZY_POST_MAX_DELTA;

    Fuzzy_Reset(fc);
}

/* ============================================================================
 * NHÓM HÀM TÍNH TOÁN HÀM LIÊN THUỘC CƠ SỞ (MEMBERSHIP FUNCTION PRIMITIVES)
 * ============================================================================ */
float Fuzzy_MF_Triangle(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) {
        return (b > a) ? ((x - a) / (b - a)) : 0.0f;
    }
    return (c > b) ? ((c - x) / (c - b)) : 0.0f;
}

float Fuzzy_MF_TrapezoidLeft(float x, float a, float b) {
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    return (b > a) ? ((b - x) / (b - a)) : 0.0f;
}

float Fuzzy_MF_TrapezoidRight(float x, float a, float b) {
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (b > a) ? ((x - a) / (b - a)) : 0.0f;
}

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
 *  - Tính sai số góc: rawError = currentVal - setpoint
 *  - Lọc thông thấp EMA vận tốc góc de_filtered_state
 *  - Chuẩn hóa qua ke (1/15), kde và kẹp vào miền [-1.0f .. +1.0f]
 * ============================================================================ */
void Fuzzy_PreProcess(FuzzyController_t *fc,
                      float setpoint,
                      float currentVal,
                      float dCurrentVal,
                      float *out_eNorm,
                      float *out_deNorm) {
    if (!fc) return;

    float rawError = currentVal - setpoint;
    float alpha    = Fuzzy_Clamp(fc->de_lpf_alpha, 0.05f, 1.0f);
    fc->de_filtered_state = (1.0f - alpha) * fc->de_filtered_state + alpha * dCurrentVal;

    float e_scaled  = Fuzzy_Clamp(rawError * fc->ke,                fc->e_min,  fc->e_max);
    float de_scaled = Fuzzy_Clamp(fc->de_filtered_state * fc->kde,  fc->de_min, fc->de_max);

    if (out_eNorm)  *out_eNorm  = e_scaled;
    if (out_deNorm) *out_deNorm = de_scaled;
}

/* ============================================================================
 * KHÂU 2: MỜ HÓA (FUZZIFICATION STAGE TRÊN MIỀN [-1.0 .. +1.0])
 *  - Cả hai kênh e_norm và de_norm đều dùng cùng cấu trúc hàm liên thuộc:
 *      + Tập ZE là hình thang (Fuzzy_MF_Trapezoid) có khoảng nằm ngang [-0.002 .. +0.002].
 *      + Tập NS / PS là tam giác có đỉnh tại +-0.25.
 *      + Tập NB / PB là hình thang biên cố định bằng 1.0 từ +-0.50 đến +-1.00.
 * ============================================================================ */
void Fuzzy_Fuzzify(FuzzyController_t *fc,
                   float eNorm,
                   float deNorm,
                   float mu_e[FUZZY_NUM_STATES],
                   float mu_de[FUZZY_NUM_STATES]) {
    if (!fc || !mu_e || !mu_de) return;

    /* Mờ hóa sai số góc e_norm (5 tập đầu vào: IN_NB, IN_NS, IN_ZE, IN_PS, IN_PB) */
    mu_e[IN_NB] = Fuzzy_MF_TrapezoidLeft(eNorm, fc->e_limit_nb_ns, fc->e_limit_ns_ze);
    mu_e[IN_NS] = Fuzzy_MF_Triangle(eNorm, fc->e_limit_nb_ns, fc->e_limit_ns_ze, fc->e_ze_flat_min);
    mu_e[IN_ZE] = Fuzzy_MF_Trapezoid(eNorm, fc->e_limit_ns_ze, fc->e_ze_flat_min, fc->e_ze_flat_max, fc->e_limit_ze_ps);
    mu_e[IN_PS] = Fuzzy_MF_Triangle(eNorm, fc->e_ze_flat_max, fc->e_limit_ze_ps, fc->e_limit_ps_pb);
    mu_e[IN_PB] = Fuzzy_MF_TrapezoidRight(eNorm, fc->e_limit_ze_ps, fc->e_limit_ps_pb);

    /* Mờ hóa đạo hàm sai số góc de_norm (Giống hệt cấu trúc của e_norm) */
    mu_de[IN_NB] = Fuzzy_MF_TrapezoidLeft(deNorm, fc->de_limit_nb_ns, fc->de_limit_ns_ze);
    mu_de[IN_NS] = Fuzzy_MF_Triangle(deNorm, fc->de_limit_nb_ns, fc->de_limit_ns_ze, fc->de_ze_flat_min);
    mu_de[IN_ZE] = Fuzzy_MF_Trapezoid(deNorm, fc->de_limit_ns_ze, fc->de_ze_flat_min, fc->de_ze_flat_max, fc->de_limit_ze_ps);
    mu_de[IN_PS] = Fuzzy_MF_Triangle(deNorm, fc->de_ze_flat_max, fc->de_limit_ze_ps, fc->de_limit_ps_pb);
    mu_de[IN_PB] = Fuzzy_MF_TrapezoidRight(deNorm, fc->de_limit_ze_ps, fc->de_limit_ps_pb);
}

/* ============================================================================
 * KHÂU 3: SUY LUẬN MỜ (FUZZY INFERENCE STAGE - BẢNG LUẬT 5x5)
 *  - Toán tử suy luận AND dùng Tích đại số (Larsen Product: w_ij = mu_e[i] * mu_de[j])
 * ============================================================================ */
void Fuzzy_Inference(const float mu_e[FUZZY_NUM_STATES],
                     const float mu_de[FUZZY_NUM_STATES],
                     float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]) {
    uint8_t i, j;
    if (!mu_e || !mu_de || !ruleWeights) return;

    for (i = 0; i < FUZZY_NUM_STATES; i++) {
        for (j = 0; j < FUZZY_NUM_STATES; j++) {
            ruleWeights[i][j] = (mu_e[i] > 0.0f && mu_de[j] > 0.0f) ? (mu_e[i] * mu_de[j]) : 0.0f;
        }
    }
}

/* ============================================================================
 * KHÂU 4: GIẢI MỜ (DEFUZZIFICATION STAGE RA MIỀN CHUẨN HÓA [-1.0 .. +1.0])
 *  - Trung bình trọng số 7 Singleton {NB, NM, NS, ZE, PS, PM, PB}
 * ============================================================================ */
float Fuzzy_Defuzzify(FuzzyController_t *fc, const float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]) {
    if (!fc || !ruleWeights) return 0.0f;

    const float singletons[FUZZY_NUM_OUT_STATES] = {
        fc->out_nb,
        fc->out_nm,
        fc->out_ns,
        fc->out_ze,
        fc->out_ps,
        fc->out_pm,
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

    float defuzzOut = (denominator > 1e-6f) ? (numerator / denominator) : 0.0f;
    return Fuzzy_Clamp(defuzzOut, fc->defuzz_min, fc->defuzz_max);
}

/* ============================================================================
 * KHÂU 5: HẬU XỬ LÝ (POST-PROCESSING STAGE: 90% OUTPWM = +-7560 PWM)
 * ============================================================================ */
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput) {
    if (!fc) return 0.0f;

    float scaledOut   = defuzzOutput * fc->ku;
    float alpha       = Fuzzy_Clamp(fc->post_ema_alpha, 0.05f, 1.0f);
    float smoothedOut = alpha * scaledOut + (1.0f - alpha) * fc->post_prev_output;

    if (fc->post_max_delta > 0.0f) {
        float delta = smoothedOut - fc->post_prev_output;
        if (delta > fc->post_max_delta) {
            smoothedOut = fc->post_prev_output + fc->post_max_delta;
        } else if (delta < -fc->post_max_delta) {
            smoothedOut = fc->post_prev_output - fc->post_max_delta;
        }
    }

    float finalOut = Fuzzy_Clamp(smoothedOut, fc->post_out_min, fc->post_out_max);
    fc->post_prev_output = finalOut;

    return finalOut;
}

/* ============================================================================
 * HÀM ĐIỀU KHIỂN FUZZY TỔNG HỢP CHẠY TRỌN VẸN 5 KHÂU
 * ============================================================================ */
float Fuzzy_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal) {
    if (!fc) return 0.0f;

    float eNorm  = 0.0f;
    float deNorm = 0.0f;

    Fuzzy_PreProcess(fc, setpoint, currentVal, dCurrentVal, &eNorm, &deNorm);
    Fuzzy_Fuzzify(fc, eNorm, deNorm, fc->mu_e, fc->mu_de);
    Fuzzy_Inference(fc->mu_e, fc->mu_de, fc->rule_weights);
    float defuzzOut = Fuzzy_Defuzzify(fc, fc->rule_weights);

    return Fuzzy_PostProcess(fc, defuzzOut);
}
