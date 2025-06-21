/*
 * tca9548a.h
 *
 *  Created on: May 7, 2025
 *      Author: gvigelet
 */

#ifndef INC_TCA9548A_H_
#define INC_TCA9548A_H_

#include "main.h"
#include <stdbool.h>
// Error codes
#define TCA9548A_OK          0
#define TCA9548A_ERR_INIT    -1
#define TCA9548A_ERR_BUS     -2
#define TCA9548A_ERR_CHANNEL -3

// TCA9548A default address (A0=A1=A2=GND)
#define TCA9548A_DEFAULT_ADDRESS 0x70

typedef struct {
    I2C_HandleTypeDef *hi2c;    // I2C handle
    bool initialized;           // track if this mux was initialized
    uint8_t i2c_address;        // I2C address of the mux
    uint8_t current_channel;    // Currently selected channel
} TCA9548A_HandleTypeDef;

/**
 * @brief Initializes the TCA9548A multiplexer
 * @param mux index to TCA9548A
 * @param hi2c Pointer to I2C handle
 * @param address I2C address of the multiplexer
 * @return TCA9548A_OK on success, error code otherwise
 */
int8_t TCA9548A_Init(uint8_t mux_index, I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Selects a channel on the multiplexer
 * @param mux index to TCA9548A
 * @param channel Channel to select (0-7)
 * @return TCA9548A_OK on success, error code otherwise
 */
int8_t TCA9548A_SelectChannel(uint8_t mux_index, uint8_t channel);

/**
 * @brief Gets the currently selected channel
 * @param mux index to TCA9548A
 * @return Currently selected channel (0-7) or error code
 */
int8_t TCA9548A_GetCurrentChannel(uint8_t mux_index);

/**
 * @brief Disables all channels (no channel selected)
 * @para mmux index to TCA9548A
 * @return TCA9548A_OK on success, error code otherwise
 */
int8_t TCA9548A_DisableAll(uint8_t mux_index);

int TCA9548A_scan_channel(uint8_t mux_index, uint8_t mux_channel, uint8_t* addr_list, size_t list_size, bool display);

int8_t TCA9548A_Write_Data(uint8_t mux_index, uint8_t channel, uint8_t i2c_addr, uint8_t mem_addr, uint8_t data_len, uint8_t* pData);
int8_t TCA9548A_Read_Data(uint8_t mux_index, uint8_t channel, uint8_t i2c_addr, uint8_t mem_addr, uint8_t data_len, uint8_t* pData);
int8_t TCA9548A_TransmitReceive_Data(uint8_t mux_index, uint8_t channel, uint8_t i2c_addr, uint8_t* tx_buff, uint8_t tx_length, uint8_t* rx_buff, uint8_t rx_len);

#endif /* INC_TCA9548A_H_ */
