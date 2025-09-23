/*
 * utils.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <stdint.h>

uint16_t util_crc16(const uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
void printBuffer(const uint8_t* buffer, uint32_t size);
void DWT_Init(void);
void delay_us(uint32_t us);
void GPIO_SetHiZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_SetOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#endif /* INC_UTILS_H_ */
