/*
 * ads7828.c
 *
 *  Created on: Jun 20, 2025
 *      Author: gvigelet
 */

#include "ads7828.h"
#include "tca9548a.h"
#include "utils.h"
#include "main.h"

#include <stdio.h>
#include <stdint.h>

ADS7828_HandleTypeDef adc_mon[2];

/**
  * @brief  Initialize the ADS7828 ADC
  * @param  hadc: Pointer to ADS7828 handle structure
  * @param  hi2c: Pointer to I2C handle
  * @param  address_bits: Address configuration bits (0-3 for A1, A0 pins)
  * @retval HAL status
  */
HAL_StatusTypeDef ADS7828_Init(ADS7828_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c, uint8_t address_bits) {
    if (hadc == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    // Validate address bits (should be 0-3)
    if (address_bits > 3) {
        return HAL_ERROR;
    }

    hadc->hi2c = hi2c;
    hadc->index = 0x01;
    hadc->channel = 0x00;
    hadc->i2c_address = ADS7828_I2C_ADDRESS_BASE | address_bits;
    hadc->power_mode = ADS7828_PD_OFF;
    hadc->differential_mode = false;

    return HAL_OK;
}

/**
  * @brief  Reverse the first 3 bits of a byte (bits 2,1,0)
  * @param  value: Input byte
  * @retval Byte with first 3 bits reversed
  */
static uint8_t reverse_first_3_bits(uint8_t value) {
    // Extract first 3 bits
    uint8_t first_3_bits = value & 0x07; // 0b00000111

    // Reverse the 3 bits using a lookup table or bit manipulation
    // Method 1: Lookup table (fastest)
    static const uint8_t reverse_3bit_lut[8] = {
        0b000,  // 0 -> 0
        0b100,  // 1 -> 4
        0b010,  // 2 -> 2
        0b110,  // 3 -> 6
        0b001,  // 4 -> 1
        0b101,  // 5 -> 5
        0b011,  // 6 -> 3
        0b111   // 7 -> 7
    };

    // Method 2: Bit manipulation (no lookup table)
    // uint8_t reversed = ((first_3_bits & 0x04) >> 2) |  // Bit 2 -> Bit 0
    //                    ((first_3_bits & 0x02)) |       // Bit 1 -> Bit 1
    //                    ((first_3_bits & 0x01) << 2);   // Bit 0 -> Bit 2

    // Clear original first 3 bits and set reversed ones
    return (value & 0xF8) | reverse_3bit_lut[first_3_bits];
}

/**
  * @brief  Read a single channel in single-ended mode
  * @param  hadc: Pointer to ADS7828 handle structure
  * @param  channel: Channel to read (0-7)
  * @param  result: Pointer to store conversion result
  * @retval HAL status
  */
HAL_StatusTypeDef ADS7828_ReadChannel(ADS7828_HandleTypeDef *hadc, uint8_t channel, uint16_t *result) {
    if (hadc == NULL || result == NULL || channel > 7) {
        return HAL_ERROR;
    }

    uint8_t command;
    uint8_t rx_data[2];
    HAL_StatusTypeDef status;

    TCA9548A_SelectChannel(hadc->index, hadc->channel);
    delay_us(1);
    uint8_t be_chan = reverse_first_3_bits(channel);
    // Build command byte: SD=1 (single-ended), PD mode, channel selection
    command = ADS7828_SD_BIT | (be_chan << 4) | ( hadc->power_mode << 2);
    printf("COMMAND 0x%02X\r\n", command);
    // Send command and read conversion result
    status = HAL_I2C_Master_Transmit(hadc->hi2c, hadc->i2c_address << 1, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Read 2 bytes of conversion data
    status = HAL_I2C_Master_Receive(hadc->hi2c, hadc->i2c_address << 1, rx_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Combine bytes to form 12-bit result
    *result = ((rx_data[0] << 8) | rx_data[1]) >> 4;

    return HAL_OK;
}

/**
  * @brief  Read a differential pair
  * @param  hadc: Pointer to ADS7828 handle structure
  * @param  diff_pair: Differential pair to read (use ADS7828_Differential_t enum)
  * @param  result: Pointer to store conversion result
  * @retval HAL status
  */
HAL_StatusTypeDef ADS7828_ReadDifferential(ADS7828_HandleTypeDef *hadc, uint8_t diff_pair, uint16_t *result) {
    if (hadc == NULL || result == NULL) {
        return HAL_ERROR;
    }

    uint8_t command;
    uint8_t rx_data[2];
    HAL_StatusTypeDef status;

    // Build command byte: SD=0 (differential), PD mode, differential pair selection
    command = (hadc->power_mode << 5) | (diff_pair & 0x3F); // SD bit is 0 for differential

    status = HAL_I2C_Master_Transmit(hadc->hi2c, hadc->i2c_address << 1, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_I2C_Master_Receive(hadc->hi2c, hadc->i2c_address << 1, rx_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    *result = ((rx_data[0] << 8) | rx_data[1]) >> 4;

    return HAL_OK;
}

/**
  * @brief  Read all 8 channels sequentially
  * @param  hadc: Pointer to ADS7828 handle structure
  * @param  results: Array to store conversion results (must be at least 8 elements)
  * @retval HAL status
  */
HAL_StatusTypeDef ADS7828_ReadAllChannels(ADS7828_HandleTypeDef *hadc, uint16_t *results) {
    if (hadc == NULL || results == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    for (uint8_t channel = 0; channel < 8; channel++) {
        status = ADS7828_ReadChannel(hadc, channel, &results[channel]);
        if (status != HAL_OK) {
            return status;
        }

        // Small delay between conversions if needed
        HAL_Delay(1);
    }

    return HAL_OK;
}

/**
  * @brief  Set power-down mode
  * @param  hadc: Pointer to ADS7828 handle structure
  * @param  mode: Power-down mode to set
  * @retval HAL status
  */
HAL_StatusTypeDef ADS7828_SetPowerMode(ADS7828_HandleTypeDef *hadc, ADS7828_PowerDown_t mode) {
    if (hadc == NULL) {
        return HAL_ERROR;
    }

    hadc->power_mode = mode;
    return HAL_OK;
}

/**
  * @brief  Convert ADC value to voltage
  * @param  adc_value: 12-bit ADC value
  * @param  vref: Reference voltage
  * @retval Voltage in volts
  */
float ADS7828_ConvertToVoltage(uint16_t adc_value, float vref) {
    return (adc_value * vref) / 4095.0f; // 4095 = 2^12 - 1
}
