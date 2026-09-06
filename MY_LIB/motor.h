#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

/**
 * @brief Initialize Motor PWM control
 */
void Motor_Config(void);

/**
 * @brief Set Motor PWM
 * @param pwm: Motor control value (could be -100 to 100 or specific duty cycle)
 */
void Motor_SetPWM(float pwm);

#endif /* __MOTOR_H */

