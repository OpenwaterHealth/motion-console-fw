/*
 * fan_driver.h
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_FAN_DRIVER_H_
#define INC_FAN_DRIVER_H_

#include "main.h"
#include "stm32h7xx_hal.h"

// Define constants
#define FAN1_PWM_CHANNEL TIM_CHANNEL_1
#define FAN2_PWM_CHANNEL TIM_CHANNEL_2
#define TACH_PULSES_PER_REV 2  // Number of tachometer pulses per fan revolution

// Function prototypes
void FAN_Init(void);
void FAN_DeInit(void);
void FAN_SetSpeed(uint32_t channel, uint8_t duty_cycle);
uint32_t FAN_GetRPM(uint32_t channel);

#endif /* INC_FAN_DRIVER_H_ */
