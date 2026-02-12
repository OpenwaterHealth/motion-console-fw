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

typedef enum {
    LED_NONE = 0,
    LED_RED = 1,
    LED_GREEN = 2,
    LED_BLUE = 3
} LED_COLORS;

// Function prototypes
void LED_Init(void);
void LED_SetState(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, LED_State state);
void LED_Toggle(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);
void LED_RGB_SET(uint8_t state);
uint8_t LED_RGB_GET(void);

#endif /* INC_LED_DRIVER_H_ */
