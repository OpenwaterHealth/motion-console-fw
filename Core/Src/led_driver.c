/*
 * led_driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */
#include "led_driver.h"

#include <stdio.h>

// Initialize the LEDs
void LED_Init(void)
{
	printf("Initializing Indicators\r\n");
    // Turn all LEDs off initially
    HAL_GPIO_WritePin(IND1_GPIO_Port, IND1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND2_GPIO_Port, IND2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND3_GPIO_Port, IND3_Pin, GPIO_PIN_SET);
}

// Set the state of an LED (ON/OFF)
void LED_SetState(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, LED_State state)
{
    if (state == LED_ON) {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET); // Turn LED on
    } else {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET); // Turn LED off
    }
}

// Toggle the state of an LED
void LED_Toggle(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIO_Port, GPIO_Pin); // Toggle LED
}


