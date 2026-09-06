#ifndef __FUZZY_H
#define __FUZZY_H

#include <stdint.h>

/* Fuzzy logic states for 5 membership functions:
 * NB: Negative Big
 * NS: Negative Small
 * ZE: Zero
 * PS: Positive Small
 * PB: Positive Big
 */
typedef enum {
    NB = 0,
    NS,
    ZE,
    PS,
    PB
} FuzzyState_t;

/* Struct to hold fuzzy controller parameters and state */
typedef struct {
    /* Gains for inputs and output */
    float ke;   /* Gain for Error (e.g., Angle error) */
    float kde;  /* Gain for dError (e.g., Angle rate error) */
    float ku;   /* Gain for Output (e.g., PWM) */

    /* Membership function limits for Error */
    float e_limit_nb_ns;
    float e_limit_ns_ze;
    float e_limit_ze_ps;
    float e_limit_ps_pb;

    /* Membership function limits for dError */
    float de_limit_nb_ns;
    float de_limit_ns_ze;
    float de_limit_ze_ps;
    float de_limit_ps_pb;
    
    /* Output singletons for defuzzification */
    float out_nb;
    float out_ns;
    float out_ze;
    float out_ps;
    float out_pb;
    
} FuzzyController_t;

/* Function Prototypes */

/**
 * @brief  Initialize the fuzzy controller with default parameters
 * @param  fc: Pointer to FuzzyController_t struct
 */
void Fuzzy_Init(FuzzyController_t *fc);

/**
 * @brief  Compute the fuzzy logic output based on error and change in error
 * @param  fc: Pointer to FuzzyController_t struct
 * @param  error: The error value (e.g., Target Angle - Current Angle from IMU)
 * @param  dError: The change in error (e.g., Gyro rate or derivative of error)
 * @retval The control output (e.g., Motor PWM)
 */
float Fuzzy_Compute(FuzzyController_t *fc, float error, float dError);

#endif /* __FUZZY_H */

