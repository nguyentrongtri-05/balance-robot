#include "fuzzy.h"
#include <math.h>

/* Fuzzy Rule Base Matrix (5x5)
 * Rows: Error (NB, NS, ZE, PS, PB)
 * Cols: dError (NB, NS, ZE, PS, PB)
 */
static const FuzzyState_t RuleBase[5][5] = {
    /* NB, NS, ZE, PS, PB */
    { NB, NB, NB, NS, ZE }, /* Error = NB */
    { NB, NB, NS, ZE, PS }, /* Error = NS */
    { NB, NS, ZE, PS, PB }, /* Error = ZE */
    { NS, ZE, PS, PB, PB }, /* Error = PS */
    { ZE, PS, PB, PB, PB }  /* Error = PB */
};

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Fuzzy_Init(FuzzyController_t *fc) {
    if (!fc) return;

    /* Hình dạng tập mờ: đều (0.5) => vùng giữa tuyến tính, dễ tune lần đầu.
     * Sau khi xe đứng được, thử e_small = 0.6, out_small = 0.4 để êm hơn quanh 0. */
    fc->e_small   = 0.5f;
    fc->de_small  = 0.5f;
    fc->out_small = 0.5f;

    /* Initialize gains (can be tuned later)
     * ku  = 100      : toàn dải PWM
     * ke  = 1/10     : |e|  >= 10 deg    là "rất lớn"
     * kde = 1/200    : |de| >= 200 deg/s là "rất lớn"
     * => Kp_eq = 100 * 0.1   = 10  %PWM/deg
     *    Kd_eq = 100 * 0.005 = 0.5 %PWM/(deg/s)   (Kd/Kp = 0.05 s) */
    fc->ku        = 100.0f;
    fc->ke        = 0.1f;
    fc->kde       = 0.005f;
    fc->out_limit = 100.0f;

    fc->dbg_e_n  = 0.0f;
    fc->dbg_de_n = 0.0f;
    fc->dbg_u    = 0.0f;
}

void Fuzzy_SetGains(FuzzyController_t *fc, float ke, float kde, float ku) {
    if (!fc) return;
    fc->ke  = ke;
    fc->kde = kde;
    fc->ku  = ku;
}

void Fuzzy_SetFromPD(FuzzyController_t *fc, float Kp, float Kd) {
    if (!fc || fc->ku == 0.0f || fc->out_small <= 0.0f) return;
    fc->ke  = Kp * fc->e_small  / (fc->ku * fc->out_small);
    fc->kde = Kd * fc->de_small / (fc->ku * fc->out_small);
}

/**
 * Mờ hóa trên miền chuẩn hóa [-1, 1] với 5 tập tam giác, đỉnh tại
 * -1, -c, 0, +c, +1 (NB/PB là vai bão hòa ngoài ±1).
 * Mỗi tập có chân đặt đúng tại đỉnh của tập kề bên => tổng mu luôn = 1
 * (partition of unity) và tại mỗi điểm chỉ có tối đa 2 tập kích hoạt.
 *
 * Bản cũ: NS = tri(-20,-5,+5) và PS = tri(-5,+5,20) tràn sang phía đối diện,
 * nên tại e = 0 có NS = PS = 0.5, ZE = 1 (tổng = 2) -> mặt điều khiển bị méo.
 */
static void Fuzzify(float x, float c, float *mu) {
    float t;
    int k;

    for (k = 0; k < 5; k++) mu[k] = 0.0f;

    x = clampf(x, -1.0f, 1.0f);
    c = clampf(c, 0.05f, 0.95f);

    if (x <= -c) {
        t = (x + 1.0f) / (1.0f - c);    /* 0 tại -1, 1 tại -c */
        mu[NB] = 1.0f - t;
        mu[NS] = t;
    } else if (x <= 0.0f) {
        t = (x + c) / c;                /* 0 tại -c, 1 tại 0 */
        mu[NS] = 1.0f - t;
        mu[ZE] = t;
    } else if (x <= c) {
        t = x / c;                      /* 0 tại 0, 1 tại +c */
        mu[ZE] = 1.0f - t;
        mu[PS] = t;
    } else {
        t = (x - c) / (1.0f - c);       /* 0 tại +c, 1 tại +1 */
        mu[PS] = 1.0f - t;
        mu[PB] = t;
    }
}

float Fuzzy_Compute(FuzzyController_t *fc, float error, float dError) {
    float mu_e[5];
    float mu_de[5];
    float out_vals[5];
    float num = 0.0f;
    float den = 0.0f;
    float e_n, de_n, u_n, u;
    int i, j;

    if (!fc) return 0.0f;

    /* Chặn NaN/Inf từ cảm biến: an toàn nhất là xuất 0 */
    if ((error != error) || (dError != dError)) {
        fc->dbg_u = 0.0f;
        return 0.0f;
    }

    /* ----------------------------------------------------
     * 1. TIỀN XỬ LÝ (Pre-processing)
     * Nhân tín hiệu đầu vào với các hệ số ke, kde rồi bão hòa về [-1, 1]
     * ---------------------------------------------------- */
    e_n  = clampf(error  * fc->ke,  -1.0f, 1.0f);
    de_n = clampf(dError * fc->kde, -1.0f, 1.0f);

    /* ----------------------------------------------------
     * 2. MỜ HÓA (Fuzzification)
     * ---------------------------------------------------- */
    Fuzzify(e_n,  fc->e_small,  mu_e);
    Fuzzify(de_n, fc->de_small, mu_de);

    out_vals[NB] = -1.0f;
    out_vals[NS] = -fc->out_small;
    out_vals[ZE] =  0.0f;
    out_vals[PS] =  fc->out_small;
    out_vals[PB] =  1.0f;

    /* ----------------------------------------------------
     * 3. HỆ QUI TẮC & PHƯƠNG PHÁP SUY LUẬN (Rule Base & Inference)
     * Kết hợp cả 4. GIẢI MỜ (Defuzzification) theo phương pháp Fuzzy Mean
     * (Trung bình có trọng số). Chỉ tối đa 2x2 = 4 luật kích hoạt mỗi lần.
     * ---------------------------------------------------- */
    for (i = 0; i < 5; i++) {
        if (mu_e[i] <= 0.0f) continue;
        for (j = 0; j < 5; j++) {
            float weight;
            if (mu_de[j] <= 0.0f) continue;
#if FUZZY_USE_MIN_AND
            weight = (mu_e[i] < mu_de[j]) ? mu_e[i] : mu_de[j];
#else
            weight = mu_e[i] * mu_de[j];
#endif
            num += weight * out_vals[RuleBase[i][j]];
            den += weight;
        }
    }

    /* 4. Hoàn tất quá trình giải mờ (Fuzzy Mean) */
    u_n = (den > 0.0f) ? (num / den) : 0.0f;

    /* ----------------------------------------------------
     * 5. HẬU XỬ LÝ (Post-processing)
     * Nhân kết quả giải mờ với hệ số ngõ ra ku, giới hạn biên độ
     * ---------------------------------------------------- */
    u = clampf(u_n * fc->ku, -fc->out_limit, fc->out_limit);

    fc->dbg_e_n  = e_n;
    fc->dbg_de_n = de_n;
    fc->dbg_u    = u;

    return u;
}

