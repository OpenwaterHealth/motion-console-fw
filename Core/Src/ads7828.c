/*
 * ads7828.c
 *
 *  Created on: Jun 20, 2025
 *      Author: gvigelet
 */

#include "ads7828.h"
#include "tca9548a.h"
#include "utils.h"
#include <stdio.h>
#include <stdint.h>

ADS7828_HandleTypeDef adc_mon[2];

uint16_t ADS7828_ReadChannel(ADS7828_HandleTypeDef *ads, uint8_t channel)
{
    if (channel > 7) return 0xFFFF;

    TCA9548A_SelectChannel(ads->index, ads->channel);
    delay_us(1);

    uint8_t cmd = ADS7828_CMD_SD_SINGLE | ((channel & 0x07) << 4) | ADS7828_CMD_PD_DEFAULT;
    uint8_t rx[2];
    uint8_t i2c_addr = ads->address << 1;  // STM32 HAL requires 8-bit format

    if (HAL_I2C_Master_Transmit(ads->hi2c, i2c_addr, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return 0xFFFF;

    if (HAL_I2C_Master_Receive(ads->hi2c, i2c_addr, rx, 2, HAL_MAX_DELAY) != HAL_OK)
        return 0xFFFF;

    return ((rx[0] << 8) | rx[1]) >> 4;  // Return 12-bit ADC value
}

float ADS7828_ConvertToVoltage(ADS7828_HandleTypeDef *ads, uint16_t raw_adc)
{
    if (raw_adc > 4095) return -1.0f;
    return ((float)raw_adc / 4095.0f) * ads->vref;
}
