#ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

/* ============================================================================
 * 1. CẤU HÌNH #DEFINE MẶC ĐỊNH CHO KHÂU TIỀN XỬ LÝ (PRE-PROCESSING DEFAULTS)
 *  - Chuẩn hóa tín hiệu đầu vào vật lý (sai số e, đạo hàm sai số de) về [-1.0f .. +1.0f]:
 *      + e_norm  = clamp(e  * ke,  e_min,  e_max)
 *      + de_norm = clamp(de * kde, de_min, de_max)
 * ============================================================================ */
#define FUZZY_BALANCE_ANGLE_MAX     15.0f                             /* Biên độ sai số chuẩn hóa mặc định */
#define FUZZY_PRE_KE                (1.0f / FUZZY_BALANCE_ANGLE_MAX)  /* Hệ số chuẩn hóa sai số e mặc định (1/15) */
#define FUZZY_PRE_KDE               0.0020f                           /* Hệ số chuẩn hóa đạo hàm sai số de mặc định */

#define FUZZY_PRE_E_MIN            -1.0f        /* Giới hạn dưới miền chuẩn hóa của e_norm (-1.0) */
#define FUZZY_PRE_E_MAX             1.0f        /* Giới hạn trên miền chuẩn hóa của e_norm (+1.0) */
#define FUZZY_PRE_DE_MIN           -1.0f        /* Giới hạn dưới miền chuẩn hóa của de_norm (-1.0) */
#define FUZZY_PRE_DE_MAX            1.0f        /* Giới hạn trên miền chuẩn hóa của de_norm (+1.0) */
#define FUZZY_PRE_DE_LPF_ALPHA      0.38f       /* Hệ số lọc thông thấp EMA cho đạo hàm de ở tiền xử lý */

/* ============================================================================
 * 2. CẤU HÌNH #DEFINE MIỀN HÀM LIÊN THUỘC ĐẦU VÀO CHUẨN HÓA [-1.0 .. +1.0]
 *  - Áp dụng đồng nhất cho cả 2 kênh đầu vào e_norm và de_norm:
 *      + Tập ZE: Hình thang cân (Trapezoid) có đỉnh phẳng bằng 1.0 từ -0.002 đến +0.002
 *      + Tập NS / PS: Tam giác (Triangle) có đỉnh tại +-0.25
 *      + Tập NB / PB: Hình thang biên (TrapezoidLeft/Right) bằng 1.0 từ +-0.50 đến +-1.00
 * ============================================================================ */
/* Miền hàm liên thuộc cho Sai số chuẩn hóa e_norm thuộc [-1.0 .. +1.0] */
#define FUZZY_MF_E_NB_NS           -0.50f       /* Điểm bắt đầu cố định tập NB = 1.0 trên [-1.0 .. -0.50] */
#define FUZZY_MF_E_NS_ZE           -0.25f       /* Đỉnh tập NS / chân trái hình thang ZE của e_norm (-0.25) */
#define FUZZY_MF_E_ZE_FLAT_MIN     -0.002f      /* Điểm đầu khoảng nằm ngang tập hình thang ZE của e_norm (-0.002) */
#define FUZZY_MF_E_ZE_FLAT_MAX      0.002f      /* Điểm cuối khoảng nằm ngang tập hình thang ZE của e_norm (+0.002) */
#define FUZZY_MF_E_ZE_PS            0.25f       /* Đỉnh tập PS / chân phải hình thang ZE của e_norm (+0.25) */
#define FUZZY_MF_E_PS_PB            0.50f       /* Điểm bắt đầu cố định tập PB = 1.0 trên [+0.50 .. +1.0] */

/* Miền hàm liên thuộc cho Đạo hàm sai số chuẩn hóa de_norm thuộc [-1.0 .. +1.0] */
#define FUZZY_MF_DE_NB_NS          -0.50f       /* Điểm bắt đầu cố định tập NB = 1.0 trên [-1.0 .. -0.50] */
#define FUZZY_MF_DE_NS_ZE          -0.25f       /* Đỉnh tập NS / chân trái hình thang ZE của de_norm (-0.25) */
#define FUZZY_MF_DE_ZE_FLAT_MIN    -0.002f      /* Điểm đầu khoảng nằm ngang tập hình thang ZE của de_norm (-0.002) */
#define FUZZY_MF_DE_ZE_FLAT_MAX     0.002f      /* Điểm cuối khoảng nằm ngang tập hình thang ZE của de_norm (+0.002) */
#define FUZZY_MF_DE_ZE_PS           0.25f       /* Đỉnh tập PS / chân phải hình thang ZE của de_norm (+0.25) */
#define FUZZY_MF_DE_PS_PB           0.50f       /* Điểm bắt đầu cố định tập PB = 1.0 trên [+0.50 .. +1.0] */

/* ============================================================================
 * 3. CẤU HÌNH #DEFINE DẢI GIÁ TRỊ KHÂU GIẢI MỜ CHUẨN HÓA [-1.0 .. +1.0]
 *  - Đầu ra của khâu giải mờ (u_defuzz) gồm 7 biến ngôn ngữ:
 *      { NB, NM, NS, ZE, PS, PM, PB } nằm trong dải [-1.0f .. +1.0f]
 * ============================================================================ */
#define FUZZY_DEFUZZ_RANGE_MIN     -1.00f       /* Biên âm của khâu giải mờ (-1.0) */
#define FUZZY_DEFUZZ_RANGE_MAX      1.00f       /* Biên dương của khâu giải mờ (+1.0) */
#define FUZZY_DEFUZZ_SMALL_RATIO    0.28f       /* Tỉ lệ mức nhỏ NS/PS (+-0.28) */
#define FUZZY_DEFUZZ_MEDIUM_RATIO   0.64f       /* Tỉ lệ mức vừa NM/PM (+-0.64) */

#define FUZZY_DEFUZZ_OUT_NB        (FUZZY_DEFUZZ_RANGE_MIN)                              /* -1.00f */
#define FUZZY_DEFUZZ_OUT_NM        (FUZZY_DEFUZZ_RANGE_MIN * FUZZY_DEFUZZ_MEDIUM_RATIO)  /* -0.64f */
#define FUZZY_DEFUZZ_OUT_NS        (FUZZY_DEFUZZ_RANGE_MIN * FUZZY_DEFUZZ_SMALL_RATIO)   /* -0.28f */
#define FUZZY_DEFUZZ_OUT_ZE        (0.00f)                                               /*  0.00f */
#define FUZZY_DEFUZZ_OUT_PS        (FUZZY_DEFUZZ_RANGE_MAX * FUZZY_DEFUZZ_SMALL_RATIO)   /* +0.28f */
#define FUZZY_DEFUZZ_OUT_PM        (FUZZY_DEFUZZ_RANGE_MAX * FUZZY_DEFUZZ_MEDIUM_RATIO)  /* +0.64f */
#define FUZZY_DEFUZZ_OUT_PB        (FUZZY_DEFUZZ_RANGE_MAX)                              /* +1.00f */

/* ============================================================================
 * 4. CẤU HÌNH #DEFINE MẶC ĐỊNH CHO KHÂU HẬU XỬ LÝ (POST-PROCESSING DEFAULTS)
 * ============================================================================ */
#define FUZZY_POST_KU            7560.0f        /* Hệ số khuếch đại đầu ra hậu xử lý mặc định */
#define FUZZY_POST_OUT_MIN      -7560.0f        /* Ngưỡng bão hòa dưới mặc định */
#define FUZZY_POST_OUT_MAX       7560.0f        /* Ngưỡng bão hòa trên mặc định */
#define FUZZY_POST_EMA_ALPHA        0.78f       /* Hệ số lọc thông thấp EMA hậu xử lý */
#define FUZZY_POST_MAX_DELTA     1200.0f        /* Giới hạn bước nhảy đầu ra tối đa mỗi chu kỳ */

/* Số lượng tập mờ đầu vào (5 tập: NB, NS, ZE, PS, PB) & đầu ra (7 tập: NB, NM, NS, ZE, PS, PM, PB) */
#define FUZZY_NUM_STATES             5u
#define FUZZY_NUM_OUT_STATES         7u

/* 5 biến ngôn ngữ đầu vào (e, de) */
typedef enum {
    IN_NB = 0,
    IN_NS = 1,
    IN_ZE = 2,
    IN_PS = 3,
    IN_PB = 4
} FuzzyInputState_t;

/* 7 biến ngôn ngữ đầu ra u: { NB, NM, NS, ZE, PS, PM, PB } */
typedef enum {
    NB = 0,
    NM = 1,
    NS = 2,
    ZE = 3,
    PS = 4,
    PM = 5,
    PB = 6
} FuzzyState_t;

/* Cấu trúc dữ liệu tổng quát của bộ điều khiển Fuzzy (Dùng chung cho cả FuzzyPD và FuzzyPI) */
typedef struct {
    /* 1. Tham số Khâu Tiền Xử Lý (Pre-processing) */
    float ke;
    float kde;
    float e_min;
    float e_max;
    float de_min;
    float de_max;
    float de_lpf_alpha;
    float de_filtered_state;

    /* 2. Tham số Khâu Mờ Hóa (Fuzzification Membership Limits) */
    float e_limit_nb_ns;
    float e_limit_ns_ze;
    float e_ze_flat_min;
    float e_ze_flat_max;
    float e_limit_ze_ps;
    float e_limit_ps_pb;

    float de_limit_nb_ns;
    float de_limit_ns_ze;
    float de_ze_flat_min;
    float de_ze_flat_max;
    float de_limit_ze_ps;
    float de_limit_ps_pb;

    /* 3. Tham số Khâu Giải Mờ (Defuzzification - 7 biến ngôn ngữ đầu ra u) */
    float defuzz_min;
    float defuzz_max;
    float out_nb;
    float out_nm;
    float out_ns;
    float out_ze;
    float out_ps;
    float out_pm;
    float out_pb;

    /* 4. Tham số Khâu Hậu Xử Lý (Post-processing & Trạng thái tích phân cho FuzzyPI) */
    float   ku;
    float   post_out_min;
    float   post_out_max;
    float   post_ema_alpha;
    float   post_max_delta;
    float   post_prev_output;
    float   pi_prev_error;
    float   pi_integral_output;
    uint8_t pi_is_first_sample;

    /* 5. Mảng trạng thái mờ phục vụ tính toán 5 khâu */
    float mu_e[FUZZY_NUM_STATES];
    float mu_de[FUZZY_NUM_STATES];
    float rule_weights[FUZZY_NUM_STATES][FUZZY_NUM_STATES];
} FuzzyController_t;

/* ============================================================================
 * CÁC HÀM KHỞI TẠO, HÀM LIÊN THUỘC CƠ SỞ & 5 KHÂU TỔNG QUÁT CỦA FUZZY
 * ============================================================================ */
void  Fuzzy_Init(FuzzyController_t *fc);
void  Fuzzy_SetParams(FuzzyController_t *fc, float ke, float kde, float ku, float outMin, float outMax);
void  Fuzzy_Reset(FuzzyController_t *fc);

/* Nhóm hàm tính toán Hàm Liên Thuộc (Membership Function Primitives) */
float Fuzzy_MF_Triangle(float x, float a, float b, float c);
float Fuzzy_MF_TrapezoidLeft(float x, float a, float b);
float Fuzzy_MF_TrapezoidRight(float x, float a, float b);
float Fuzzy_MF_Trapezoid(float x, float a, float b, float c, float d);

/* 5 khâu xử lý tổng quát của bộ điều khiển Fuzzy */
void  Fuzzy_PreProcess(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal, float *out_eNorm, float *out_deNorm);
void  Fuzzy_Fuzzify(FuzzyController_t *fc, float eNorm, float deNorm, float mu_e[FUZZY_NUM_STATES], float mu_de[FUZZY_NUM_STATES]);
void  Fuzzy_Inference(const float mu_e[FUZZY_NUM_STATES], const float mu_de[FUZZY_NUM_STATES], float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]);
float Fuzzy_Defuzzify(FuzzyController_t *fc, const float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]);
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput);

/* 2 hàm thực thi bộ điều khiển Fuzzy PD và Fuzzy PI */
float FuzzyPD_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal);
float FuzzyPI_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dt);

#endif /* __FUZZY_H */
