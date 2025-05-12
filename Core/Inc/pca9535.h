/*
 * pca9535.h
 *
 *  Created on: May 12, 2025
 *      Author: gvigelet
 */

#ifndef INC_PCA9535_H_
#define INC_PCA9535_H_

#include "main.h"

// PCA9535APW I2C address (0x20 with A0-A2 grounded)
#define PCA9535APW_I2C_ADDR         (0x20 << 1)  // STM32 uses 8-bit addresses (7-bit shifted left)

// Register addresses
#define PCA9535APW_INPUT_PORT0      0x00
#define PCA9535APW_INPUT_PORT1      0x01
#define PCA9535APW_OUTPUT_PORT0     0x02
#define PCA9535APW_OUTPUT_PORT1     0x03
#define PCA9535APW_POL_INV_PORT0    0x04
#define PCA9535APW_POL_INV_PORT1    0x05
#define PCA9535APW_CONFIG_PORT0     0x06
#define PCA9535APW_CONFIG_PORT1     0x07

// Error codes
#define PCA9535APW_OK               0
#define PCA9535APW_ERROR           -1

// Function prototypes
void PCA9535APW_Init(I2C_HandleTypeDef *hi2c);
int PCA9535APW_SetPortDirection(uint8_t port, uint8_t direction);
int PCA9535APW_WritePort(uint8_t port, uint8_t value);
int PCA9535APW_ReadPort(uint8_t port, uint8_t *value);
int PCA9535APW_WritePin(uint8_t port, uint8_t pin, uint8_t state);
int PCA9535APW_ReadPin(uint8_t port, uint8_t pin, uint8_t *state);
int PCA9535APW_SetPolarity(uint8_t port, uint8_t polarity);


#endif /* INC_PCA9535_H_ */
