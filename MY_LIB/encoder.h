#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

#define ENCODER_PULSES_PER_REV 330u
#define ENCODER_SAMPLE_TIME    0.010f

typedef struct
{
    int32_t pulse;
    int32_t pre_pulse;
    int32_t delta_pulse;
    float speed;
} MOTOR;

/**
 * @brief Initialize Encoder timer interfaces on PB4/PB5 and PB6/PB7
 */
void Encoder_Config(void);

/**
 * @brief Read both wheel encoder speeds in one structure-based function
 * @param motorLeft: Pointer to left wheel encoder struct
 * @param motorRight: Pointer to right wheel encoder struct
 */
void Encoder_ReadStruct(MOTOR *motorLeft, MOTOR *motorRight);

#endif /* __ENCODER_H */

