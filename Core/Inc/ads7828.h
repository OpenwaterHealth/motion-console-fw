/*
 * ads7828.h
 *
 *  Created on: Jun 20, 2025
 *      Author: gvigelet
 */

#ifndef __ADS7828_H__
#define __ADS7828_H__

#include "stm32h7xx_hal.h"

#define ADS7828_CMD_SD_SINGLE    (1 << 7)
#define ADS7828_CMD_PD_DEFAULT   (0x0C)  // Internal ref ON, ADC ON

typedef struct {
    I2C_HandleTypeDef *hi2c;  // Pointer to I2C peripheral
    uint8_t address;          // 7-bit address (e.g., 0x48 or 0x4B)
    float vref;               // Reference voltage (e.g., 3.3)
} ADS7828_HandleTypeDef;

uint16_t ADS7828_ReadChannel(ADS7828_HandleTypeDef *ads, uint8_t channel);
float ADS7828_ConvertToVoltage(ADS7828_HandleTypeDef *ads, uint16_t raw_adc);


#endif /* INC_ADS7828_H_ */
