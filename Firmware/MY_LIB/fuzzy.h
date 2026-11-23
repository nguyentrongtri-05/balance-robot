#ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

/* Bộ điều khiển Fuzzy PD kiểu Sugeno bậc 0: 2 đầu vào (e, de) x 5 tập -> đầu ra u gồm 7 singleton */
#define FUZZY_N_IN    5u   /* Tập mờ đầu vào e, de : NB NS ZE PS PB         */
#define FUZZY_N_OUT   7u   /* Tập mờ đầu ra u      : NB NM NS ZE PS PM PB   */

/* Nhãn 7 tập đầu ra u (dùng để viết bảng luật) */
typedef enum { NB = 0, NM, NS, ZE, PS, PM, PB } FuzzyState_t;

typedef struct {
    float ke, kde, ku;          /* Tiền xử lý: e_n = e*ke, de_n = de*kde | Hậu xử lý: u = u_n*ku       */
    float e_ns, e_nb;           /* Đỉnh tập NS/PS và NB/PB của e_n  (yêu cầu 0 < NS < NB <= 1)         */
    float de_ns, de_nb;         /* Đỉnh tập NS/PS và NB/PB của de_n                                    */
    float out[FUZZY_N_OUT];     /* 7 singleton đầu ra: {-1, -med, -small, 0, +small, +med, +1}         */
    const FuzzyState_t (*rule)[FUZZY_N_IN];  /* Bảng luật 5x5: rule[e][de] = tập đầu ra             */
    float e_norm, de_norm;      /* e_n, de_n (đã bão hòa [-1, 1]) của lần tính gần nhất -> telemetry   */
} FuzzyController_t;

/* Bảng luật PD chuẩn 5x5 (đối xứng lẻ qua tâm) */
extern const FuzzyState_t FUZZY_RULE_PD[FUZZY_N_IN][FUZZY_N_IN];

/* rule = NULL -> dùng FUZZY_RULE_PD */
void  Fuzzy_Init(FuzzyController_t *fc, float ke, float kde, float ku,
                 float e_nb, float e_ns, float de_nb, float de_ns,
                 float out_small, float out_med,
                 const FuzzyState_t rule[FUZZY_N_IN][FUZZY_N_IN]);
void  Fuzzy_Reset(FuzzyController_t *fc);

/* e = measured - setpoint, de = dMeasured (đạo hàm của đại lượng đo). Trả về u ∈ [-ku, +ku] */
float FuzzyPD_Compute(FuzzyController_t *fc, float setpoint, float measured, float dMeasured);

#endif /* __FUZZY_H */
