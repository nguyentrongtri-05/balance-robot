#ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

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

/* Cấu trúc dữ liệu tổng quát của bộ điều khiển Fuzzy PD (Dùng chung cho cả 2 khâu FuzzyPD_Pitch và FuzzyPD_Pos nối tiếp) */
typedef struct {
    /* 1. Tham số Khâu Tiền Xử Lý (Pre-processing) & Bão hòa đầu vào [-1..+1] */
    float ke;
    float kde;
    float e_min;
    float e_max;
    float de_min;
    float de_max;

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

    /* 3. Tham số Khâu Giải Mờ (Defuzzification - 7 biến ngôn ngữ đầu ra u) & Bão hòa trước Ku [-1..+1] */
    float defuzz_min;
    float defuzz_max;
    float out_nb;
    float out_nm;
    float out_ns;
    float out_ze;
    float out_ps;
    float out_pm;
    float out_pb;

    /* 4. Tham số Khâu Hậu Xử Lý (Post-processing Ku) */
    float ku;
    float post_out_min;
    float post_out_max;

    /* Giá trị e, de sau chuẩn hóa và bão hòa [-1..+1] của lần tính gần nhất (phục vụ debug / telemetry) */
    float e_norm;
    float de_norm;

    /* 5. Mảng trạng thái mờ phục vụ tính toán các khâu */
    float mu_e[FUZZY_NUM_STATES];
    float mu_de[FUZZY_NUM_STATES];
    float rule_weights[FUZZY_NUM_STATES][FUZZY_NUM_STATES];
} FuzzyController_t;

/* ============================================================================
 * CÁC HÀM KHỞI TẠO, HÀM LIÊN THUỘC CƠ SỞ & CÁC KHÂU CỦA BỘ ĐIỀU KHIỂN FUZZY PD
 * ============================================================================ */
void  Fuzzy_Init(FuzzyController_t *fc,
                 float ke, float kde, float ku, float outMin, float outMax,
                 float e_nb, float e_ns, float e_ze,
                 float de_nb, float de_ns, float de_ze,
                 float out_small, float out_med);
void  Fuzzy_Reset(FuzzyController_t *fc);

/* Nhóm hàm tính toán Hàm Liên Thuộc (Membership Function Primitives) */
float Fuzzy_MF_Triangle(float x, float a, float b, float c);
float Fuzzy_MF_TrapezoidLeft(float x, float a, float b);
float Fuzzy_MF_TrapezoidRight(float x, float a, float b);
float Fuzzy_MF_Trapezoid(float x, float a, float b, float c, float d);

/* Các khâu xử lý tổng quát của bộ điều khiển Fuzzy PD */
void  Fuzzy_PreProcess(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal, float *out_eNorm, float *out_deNorm);
void  Fuzzy_Fuzzify(FuzzyController_t *fc, float eNorm, float deNorm, float mu_e[FUZZY_NUM_STATES], float mu_de[FUZZY_NUM_STATES]);
void  Fuzzy_Inference(const float mu_e[FUZZY_NUM_STATES], const float mu_de[FUZZY_NUM_STATES], float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]);
float Fuzzy_Defuzzify(FuzzyController_t *fc, float ruleWeights[FUZZY_NUM_STATES][FUZZY_NUM_STATES]);
float Fuzzy_PostProcess(FuzzyController_t *fc, float defuzzOutput);

/* Hàm thực thi bộ điều khiển Fuzzy PD (dùng chung cho cả vòng ngoài FuzzyPD_Pos và vòng trong FuzzyPD_Pitch) */
float FuzzyPD_Compute(FuzzyController_t *fc, float setpoint, float currentVal, float dCurrentVal);

#endif /* __FUZZY_H */
