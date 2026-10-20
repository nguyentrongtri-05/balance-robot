#include "fuzzy.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * BẢNG LUẬT MỜ 5x5 VỚI 7 BIẾN NGÔN NGỮ ĐẦU RA u (FUZZY RULE BASE MATRIX)
 *  - Hàng (Rows i = 0..4): Sai số chuẩn hóa e_norm         {NB, NS, ZE, PS, PB}
 *  - Cột  (Cols j = 0..4): Đạo hàm sai số chuẩn hóa de_norm {NB, NS, ZE, PS, PB}
 *  - Đầu ra u (7 mức)    : {NB, NM, NS, ZE, PS, PM, PB}
 * ============================================================================ */
static const FuzzyState_t s_FuzzyRuleBase[FUZZY_NUM_STATES][FUZZY_NUM_STATES] = {
    /*    u   de = NB,  NS,  ZE,  PS,  PB */
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
 * KHỞI TẠO, CẤU HÌNH THAM SỐ TỔNG QUÁT & RESET BỘ ĐIỀU KHIỂN MỜ FUZZY PD
 * ============================================================================ */
void Fuzzy_Reset(FuzzyController_t *fc) {
    if (!fc) return;

    fc->e_norm  = 0.0f;
    fc->de_norm = 0.0f;

    memset(fc->mu_e,         0, sizeof(fc->mu_e));
    memset(fc->mu_de,        0, sizeof(fc->mu_de));
    memset(fc->rule_weights, 0, sizeof(fc->rule_weights));
}

void Fuzzy_Init(FuzzyController_t *fc,
                float ke, float kde, float ku, float outMin, float outMax,
                float e_nb, float e_ns, float e_ze,
                float de_nb, float de_ns, float de_ze,
                float out_small, float out_med) {
    if (!fc) return;

    /* 1. Tham số Khâu Tiền Xử Lý & Bão hòa đầu vào [-1.0 .. +1.0] */
    fc->ke                = ke;
    fc->kde               = kde;
    fc->e_min             = -1.0f;
    fc->e_max             =  1.0f;
    fc->de_min            = -1.0f;
    fc->de_max            =  1.0f;

    /* 2. Cấu hình dải biến ngôn ngữ đầu vào e và de (chuẩn hóa đối xứng quanh 0) */
    float e_nb_val  = fabsf(e_nb);
    float e_ns_val  = fabsf(e_ns);
    float e_ze_val  = fabsf(e_ze);
    float de_nb_val = fabsf(de_nb);
    float de_ns_val = fabsf(de_ns);
    float de_ze_val = fabsf(de_ze);

    fc->e_limit_nb_ns     = -e_nb_val;
    fc->e_limit_ns_ze     = -e_ns_val;
    fc->e_ze_flat_min     = -e_ze_val;
    fc->e_ze_flat_max     =  e_ze_val;
    fc->e_limit_ze_ps     =  e_ns_val;
    fc->e_limit_ps_pb     =  e_nb_val;

    fc->de_limit_nb_ns    = -de_nb_val;
    fc->de_limit_ns_ze    = -de_ns_val;
    fc->de_ze_flat_min    = -de_ze_val;
    fc->de_ze_flat_max    =  de_ze_val;
    fc->de_limit_ze_ps    =  de_ns_val;
    fc->de_limit_ps_pb    =  de_nb_val;

    /* 3. Cấu hình dải 7 mức Singleton Khâu Giải Mờ đầu ra u & Bão hòa [-1.0 .. +1.0] trước Ku */
    float out_s_val = fabsf(out_small);
    float out_m_val = fabsf(out_med);

    fc->defuzz_min        = -1.00f;
    fc->defuzz_max        =  1.00f;
    fc->out_nb            = -1.00f;
    fc->out_nm            = -out_m_val;
    fc->out_ns            = -out_s_val;
    fc->out_ze            =  0.00f;
    fc->out_ps            =  out_s_val;
    fc->out_pm            =  out_m_val;
    fc->out_pb            =  1.00f;

    /* 4. Tham số Khâu Hậu Xử Lý (nhập trực tiếp từ hàm Init) */
    fc->ku                = ku;
    fc->post_out_min      = outMin;
    fc->post_out_max      = outMax;

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
 * KHÂU 1: TIỀN XỬ LÝ TỔNG QUÁT (PRE-PROCESSING STAGE)
 *  - Tính sai số: rawError = currentVal - setpoint
 *  - Chuẩn hóa e_norm = rawError * ke, de_norm = dCurrentVal * kde
 *  (Khâu bão hòa [-1.0 .. +1.0] được thực hiện ngay sau bước Tiền xử lý)
 * ============================================================================ */
void Fuzzy_PreProcess(FuzzyController_t *fc,
                      float setpoint,
                      float currentVal,
                      float dCurrentVal,
                      float *out_eNorm,
                      float *out_deNorm) {
    if (!fc) return;

    float rawError = currentVal - setpoint;

    float e_scaled  = rawError    * fc->ke;
    float de_scaled = dCurrentVal * fc->kde;

    if (out_eNorm)  *out_eNorm  = e_scaled;
    if (out_deNorm) *out_deNorm = de_scaled;
}

/* ============================================================================
 * KHÂU 2: MỜ HÓA TỔNG QUÁT (FUZZIFICATION STAGE TRÊN MIỀN [-1.0 .. +1.0])
 *  - Cả hai kênh e_norm và de_norm đều dùng cùng cấu trúc 5 hàm liên thuộc:
 *      + Tập ZE: Hình thang cân (Fuzzy_MF_Trapezoid)
 *      + Tập NS / PS: Tam giác (Fuzzy_MF_Triangle)
 *      + Tập NB / PB: Hình thang biên (Fuzzy_MF_TrapezoidLeft / Right)
 * ============================================================================ */
void Fuzzy_Fuzzify(FuzzyController_t *fc,
                   float eNorm,
                   float deNorm,
                   float mu_e[FUZZY_NUM_STATES],
                   float mu_de[FUZZY_NUM_STATES]) {
    if (!fc || !mu_e || !mu_de) return;

    /* Mờ hóa sai số e_norm (5 tập đầu vào: IN_NB, IN_NS, IN_ZE, IN_PS, IN_PB) */
    mu_e[IN_NB] = Fuzzy_MF_TrapezoidLeft(eNorm, fc->e_limit_nb_ns, fc->e_limit_ns_ze);
    mu_e[IN_NS] = Fuzzy_MF_Triangle(eNorm, fc->e_limit_nb_ns, fc->e_limit_ns_ze, fc->e_ze_flat_min);
    mu_e[IN_ZE] = Fuzzy_MF_Trapezoid(eNorm, fc->e_limit_ns_ze, fc->e_ze_flat_min, fc->e_ze_flat_max, fc->e_limit_ze_ps);
    mu_e[IN_PS] = Fuzzy_MF_Triangle(eNorm, fc->e_ze_flat_max, fc->e_limit_ze_ps, fc->e_limit_ps_pb);
    mu_e[IN_PB] = Fuzzy_MF_TrapezoidRight(eNorm, fc->e_limit_ze_ps, fc->e_limit_ps_pb);

    /* Mờ hóa đạo hàm sai số de_norm (5 tập đầu vào: IN_NB, IN_NS, IN_ZE, IN_PS, IN_PB) */
    mu_de[IN_NB] = Fuzzy_MF_TrapezoidLeft(deNorm, fc->de_limit_nb_ns, fc->de_limit_ns_ze);
    mu_de[IN_NS] = Fuzzy_MF_Triangle(deNorm, fc->de_limit_nb_ns, fc->de_limit_ns_ze, fc->de_ze_flat_min);
    mu_de[IN_ZE] = Fuzzy_MF_Trapezoid(deNorm, fc->de_limit_ns_ze, fc->de_ze_flat_min, fc->de_ze_flat_max, fc->de_limit_ze_ps);
    mu_de[IN_PS] = Fuzzy_MF_Triangle(deNorm, fc->de_ze_flat_max, fc->de_limit_ze_ps, fc->de_limit_ps_pb);
    mu_de[IN_PB] = Fuzzy_MF_TrapezoidRight(deNorm, fc->de_limit_ze_ps, fc->de_limit_ps_pb);
}

/* ============================================================================
 * KHÂU 3: SUY DIỄN MỜ TỔNG QUÁT (FUZZY INFERENCE STAGE - BẢNG LUẬT 5x5)
 *  - Toán tử suy diễn AND dùng Tích đại số (Larsen Product: w_ij = mu_e[i] * mu_de[j])
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
 * KHÂU 4: GIẢI MỜ TỔNG QUÁT (DEFUZZIFICATION STAGE)
 *  - Trung bình trọng số 7 Singleton {NB, NM, NS, ZE, PS, PM, PB}
 *  (Khâu bão hòa [-1.0 .. +1.0] được đặt ngay sau Giải mờ, trước Hậu xử lý Ku)
 * ============================================================================ */
float Fuzzy_Defuzzify(FuzzyController_t *fc, float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]) {
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

    return (denominator > 1e-6f) ? (numerator / denominator) : 0.0f;
}

/* ============================================================================
 * KHÂU 5: HẬU XỬ LÝ TỔNG QUÁT (POST-PROCESSING STAGE)
 *  - Khuếch đại tín hiệu giải mờ (đã bão hòa [-1.0 .. +1.0]) qua hệ số Ku
 *    và giới hạn trong dải vật lý [post_out_min, post_out_max]
 * ============================================================================ */
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput) {
    if (!fc) return 0.0f;

    return Fuzzy_Clamp(defuzzOutput * fc->ku, fc->post_out_min, fc->post_out_max);
}

/* ============================================================================
 * HÀM ĐIỀU KHIỂN FUZZY PD TỔNG HỢP
 *  - Khâu 1: Tiền xử lý (chuẩn hóa e_norm = e * ke, de_norm = de * kde)
 *  - Bão hòa đầu vào: Kẹp e_norm, de_norm vào đoạn [-1.0 .. +1.0] sau Tiền xử lý
 *  - Khâu 2: Mờ hóa (Fuzzification)
 *  - Khâu 3: Suy diễn mờ (Fuzzy Inference 5x5)
 *  - Khâu 4: Giải mờ (Defuzzification)
 *  - Bão hòa trước Hậu xử lý: Kẹp u_defuzz vào đoạn [-1.0 .. +1.0] trước khi nhân Ku
 *  - Khâu 5: Hậu xử lý (Post-processing Ku)
 * ============================================================================ */
float FuzzyPD_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal) {
    if (!fc) return 0.0f;

    float eNorm  = 0.0f;
    float deNorm = 0.0f;

    /* 1. Khâu Tiền xử lý: chuẩn hóa e_norm và de_norm */
    Fuzzy_PreProcess(fc, setpoint, currentVal, dCurrentVal, &eNorm, &deNorm);

    /* Bão hòa [-1.0 .. +1.0] ngay sau khâu Tiền xử lý */
    eNorm       = Fuzzy_Clamp(eNorm,  fc->e_min,  fc->e_max);
    deNorm      = Fuzzy_Clamp(deNorm, fc->de_min, fc->de_max);
    fc->e_norm  = eNorm;
    fc->de_norm = deNorm;

    /* 2. Khâu Mờ hóa */
    Fuzzy_Fuzzify(fc, eNorm, deNorm, fc->mu_e, fc->mu_de);

    /* 3. Khâu Suy diễn mờ */
    Fuzzy_Inference(fc->mu_e, fc->mu_de, fc->rule_weights);

    /* 4. Khâu Giải mờ */
    float defuzzOut = Fuzzy_Defuzzify(fc, fc->rule_weights);

    /* Bão hòa [-1.0 .. +1.0] trước khâu Hậu xử lý Ku */
    defuzzOut = Fuzzy_Clamp(defuzzOut, fc->defuzz_min, fc->defuzz_max);

    /* 5. Khâu Hậu xử lý Ku */
    return Fuzzy_PostProcess(fc, defuzzOut);
}
