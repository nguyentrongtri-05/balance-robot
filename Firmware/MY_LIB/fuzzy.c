#include "fuzzy.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * BẢNG LUẬT PD 5x5: hàng = tập của e_n, cột = tập của de_n, ô = tập đầu ra u
 *  - Đối xứng lẻ qua tâm: u(-e, -de) = -u(e, de)
 *  - Đường chéo phụ (e_n = -de_n) cho u = ZE: sai số đang tự giảm đúng tốc độ thì không can thiệp
 * ============================================================================ */
const FuzzyState_t FUZZY_RULE_PD[FUZZY_N_IN][FUZZY_N_IN] = {
    /*  u          de = NB  NS  ZE  PS  PB */
    /* e = NB */      { NB, NB, NM, NS, ZE },
    /* e = NS */      { NB, NM, NS, ZE, PS },
    /* e = ZE */      { NM, NS, ZE, PS, PM },
    /* e = PS */      { NS, ZE, PS, PM, PB },
    /* e = PB */      { ZE, PS, PM, PB, PB }
};

static float Sat1(float x) { return (x < -1.0f) ? -1.0f : ((x > 1.0f) ? 1.0f : x); }

void Fuzzy_Init(FuzzyController_t *fc, float ke, float kde, float ku,
                float e_nb, float e_ns, float de_nb, float de_ns,
                float out_small, float out_med,
                const FuzzyState_t rule[FUZZY_N_IN][FUZZY_N_IN]) {
    const float s = fabsf(out_small), m = fabsf(out_med);
    const float out[FUZZY_N_OUT] = { -1.0f, -m, -s, 0.0f, s, m, 1.0f };

    fc->ke    = ke;            fc->kde   = kde;          fc->ku = ku;
    fc->e_nb  = fabsf(e_nb);   fc->e_ns  = fabsf(e_ns);
    fc->de_nb = fabsf(de_nb);  fc->de_ns = fabsf(de_ns);
    memcpy(fc->out, out, sizeof(out));
    fc->rule  = rule ? rule : FUZZY_RULE_PD;
    Fuzzy_Reset(fc);
}

/* Bộ điều khiển không có trạng thái nội (không tích phân) -> chỉ xóa giá trị telemetry */
void Fuzzy_Reset(FuzzyController_t *fc) {
    fc->e_norm = fc->de_norm = 0.0f;
}

/* ============================================================================
 * MỜ HÓA 1 BIẾN x ∈ [-1, 1]: 5 tập phân hoạch chuẩn, đỉnh tại p = {-NB, -NS, 0, +NS, +NB}
 *   NB: vai trái (=1 khi x <= -NB) | NS, ZE, PS: tam giác | PB: vai phải (=1 khi x >= +NB)
 *   p[k] <= x <= p[k+1]  ->  mu[k] = 1 - t, mu[k+1] = t,  t = (x - p[k]) / (p[k+1] - p[k])
 *   => Σmu = 1 tại mọi x, tối đa 2 tập kề nhau khác 0. (x = NaN -> mọi mu = 0 -> u = 0)
 * ============================================================================ */
static void Fuzzify(float x, float ns, float nb, float mu[FUZZY_N_IN]) {
    const float p[FUZZY_N_IN] = { -nb, -ns, 0.0f, ns, nb };
    uint8_t k;

    memset(mu, 0, FUZZY_N_IN * sizeof(float));
    if (x <= p[0])              { mu[0] = 1.0f;              return; }
    if (x >= p[FUZZY_N_IN - 1]) { mu[FUZZY_N_IN - 1] = 1.0f; return; }
    for (k = 0; k < FUZZY_N_IN - 1u; k++) {
        if (x <= p[k + 1]) {
            float t   = (x - p[k]) / (p[k + 1] - p[k]);
            mu[k]     = 1.0f - t;
            mu[k + 1] = t;
            return;
        }
    }
}

/* ============================================================================
 * FUZZY PD:  u = ku * Σ(w_ij * U[rule_ij]) / Σ(w_ij),   w_ij = μe_i(e_n) * μde_j(de_n)
 *  Khâu 1 - Tiền xử lý : e = measured - setpoint; e_n = sat(e*ke), de_n = sat(de*kde), sat = kẹp [-1, 1]
 *  Khâu 2 - Mờ hóa     : 5 tập NB..PB cho mỗi đầu vào
 *  Khâu 3 - Suy diễn   : 25 luật, toán tử AND = tích đại số (prod)
 *  Khâu 4 - Giải mờ    : trung bình có trọng số 7 singleton (wtaver) -> u_n ∈ [-1, 1]
 *  Khâu 5 - Hậu xử lý  : u = u_n * ku
 *  Do phân hoạch chuẩn nên Σw = 1 và mặt điều khiển chính là nội suy song tuyến tính của bảng luật.
 * ============================================================================ */
float FuzzyPD_Compute(FuzzyController_t *fc, float setpoint, float measured, float dMeasured) {
    float mu_e[FUZZY_N_IN], mu_de[FUZZY_N_IN];
    float num = 0.0f, den = 0.0f;
    uint8_t i, j;

    fc->e_norm  = Sat1((measured - setpoint) * fc->ke);
    fc->de_norm = Sat1(dMeasured * fc->kde);

    Fuzzify(fc->e_norm,  fc->e_ns,  fc->e_nb,  mu_e);
    Fuzzify(fc->de_norm, fc->de_ns, fc->de_nb, mu_de);

    for (i = 0; i < FUZZY_N_IN; i++) {
        for (j = 0; j < FUZZY_N_IN; j++) {
            float w = mu_e[i] * mu_de[j];
            num += w * fc->out[fc->rule[i][j]];
            den += w;
        }
    }
    return (den > 1e-6f) ? (num / den) * fc->ku : 0.0f;
}
