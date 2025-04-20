/*
 * led_driver.h
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_LED_DRIVER_H_
#define INC_LED_DRIVER_H_

#include "main.h"

// LED States
typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LED_State;

// Function prototypes
void LED_Init(void);
void LED_SetState(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, LED_State state);
void LED_Toggle(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);

#endif /* INC_LED_DRIVER_H_ */
