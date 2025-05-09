/*
 * tca9548.c
 *
 *  Created on: May 7, 2025
 *      Author: gvigelet
 */

#include "tca9548a.h"
#include <string.h>

int8_t TCA9548A_Init(TCA9548A_HandleTypeDef *hmux, I2C_HandleTypeDef *hi2c, uint8_t address) {
    if (hmux == NULL || hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    // Initialize handle
    hmux->hi2c = hi2c;
    hmux->i2c_address = address;
    hmux->current_channel = 0xFF; // No channel selected initially

    // Disable all channels to start with a known state
    return TCA9548A_DisableAll(hmux);
}

int8_t TCA9548A_SelectChannel(TCA9548A_HandleTypeDef *hmux, uint8_t channel) {
    if (hmux == NULL || hmux->hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    if (channel > 7) {
        return TCA9548A_ERR_CHANNEL;
    }

    uint8_t control_byte = 1 << channel;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hmux->hi2c, hmux->i2c_address << 1, &control_byte, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return TCA9548A_ERR_BUS;
    }

    hmux->current_channel = channel;
    return TCA9548A_OK;
}

int8_t TCA9548A_GetCurrentChannel(TCA9548A_HandleTypeDef *hmux) {
    if (hmux == NULL) {
        return TCA9548A_ERR_INIT;
    }

    if (hmux->current_channel == 0xFF) {
        return TCA9548A_ERR_CHANNEL; // No channel selected
    }

    return hmux->current_channel;
}

int8_t TCA9548A_DisableAll(TCA9548A_HandleTypeDef *hmux) {
    if (hmux == NULL || hmux->hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    uint8_t control_byte = 0x00; // Disable all channels
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hmux->hi2c, hmux->i2c_address << 1, &control_byte, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return TCA9548A_ERR_BUS;
    }

    hmux->current_channel = 0xFF; // Indicate no channel selected
    return TCA9548A_OK;
}


