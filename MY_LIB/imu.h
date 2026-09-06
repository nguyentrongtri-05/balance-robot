#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx.h"

/**
 * @brief Initialize IMU sensor
 */
void IMU_Config(void);

/**
 * @brief Read data from IMU
 * @param pitchAngle: Pointer to store pitch angle
 * @param pitchRate: Pointer to store pitch rate
 */
void IMU_Read(float *pitchAngle, float *pitchRate);

#endif /* __IMU_H */

