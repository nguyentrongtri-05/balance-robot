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

void Fuzzy_Init(FuzzyController_t *fc) {
    if (!fc) return;

    /* Initialize gains (can be tuned later) */
    fc->ke = 1.0f;
    fc->kde = 1.0f;
    fc->ku = 1.0f;

    /* Initialize limits for membership functions (Example values) */
    fc->e_limit_nb_ns = -20.0f;
    fc->e_limit_ns_ze = -5.0f;
    fc->e_limit_ze_ps = 5.0f;
    fc->e_limit_ps_pb = 20.0f;

    fc->de_limit_nb_ns = -50.0f;
    fc->de_limit_ns_ze = -10.0f;
    fc->de_limit_ze_ps = 10.0f;
    fc->de_limit_ps_pb = 50.0f;
    
    /* Initialize output singletons */
    fc->out_nb = -100.0f; /* Max reverse PWM */
    fc->out_ns = -50.0f;
    fc->out_ze = 0.0f;
    fc->out_ps = 50.0f;
    fc->out_pb = 100.0f;  /* Max forward PWM */
}

/* Helper function for triangular membership calculation */
static float membership_tri(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

/* Helper function for trapezoidal membership calculation (left edge) */
static float membership_trap_L(float x, float a, float b) {
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    return (b - x) / (b - a);
}

/* Helper function for trapezoidal membership calculation (right edge) */
static float membership_trap_R(float x, float a, float b) {
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (x - a) / (b - a);
}

static void Fuzzify(float x, float limit_nb_ns, float limit_ns_ze, float limit_ze_ps, float limit_ps_pb, float *mu) {
    /* Limit variables */
    float nb_center = limit_nb_ns - 10.0f; // Arbitrary left center
    float pb_center = limit_ps_pb + 10.0f; // Arbitrary right center
    
    mu[NB] = membership_trap_L(x, limit_nb_ns, limit_ns_ze);
    mu[NS] = membership_tri(x, limit_nb_ns, limit_ns_ze, limit_ze_ps);
    mu[ZE] = membership_tri(x, limit_ns_ze, 0.0f, limit_ze_ps);
    mu[PS] = membership_tri(x, limit_ns_ze, limit_ze_ps, limit_ps_pb);
    mu[PB] = membership_trap_R(x, limit_ze_ps, limit_ps_pb);
}

float Fuzzy_Compute(FuzzyController_t *fc, float error, float dError) {
    float e_scaled = error * fc->ke;
    float de_scaled = dError * fc->kde;

    float mu_e[5];
    float mu_de[5];

    /* 1. Fuzzification */
    Fuzzify(e_scaled, fc->e_limit_nb_ns, fc->e_limit_ns_ze, fc->e_limit_ze_ps, fc->e_limit_ps_pb, mu_e);
    Fuzzify(de_scaled, fc->de_limit_nb_ns, fc->de_limit_ns_ze, fc->de_limit_ze_ps, fc->de_limit_ps_pb, mu_de);

    /* 2. Rule Evaluation and Defuzzification (Weighted Average / Sugeno) */
    float num = 0.0f;
    float den = 0.0f;
    
    float out_vals[5] = {fc->out_nb, fc->out_ns, fc->out_ze, fc->out_ps, fc->out_pb};

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            /* AND operator -> Min */
            float weight = mu_e[i] < mu_de[j] ? mu_e[i] : mu_de[j];
            
            if (weight > 0.0f) {
                FuzzyState_t out_state = RuleBase[i][j];
                num += weight * out_vals[out_state];
                den += weight;
            }
        }
    }

    float output = 0.0f;
    if (den > 0.0f) {
        output = num / den;
    }

    return output * fc->ku;
}

