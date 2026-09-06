#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

/**
 * @brief Initialize Encoder timer interfaces
 */
void Encoder_Config(void);

/**
 * @brief Read speed/position from Encoder
 * @param speed: Pointer to store motor speed
 */
void Encoder_Read(float *speed);

#endif /* __ENCODER_H */

