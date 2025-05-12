#include "pca9535.h"
#include <string.h>

static I2C_HandleTypeDef *pca_i2c;

/**
  * @brief  Initialize the PCA9535APW driver
  * @param  hi2c Pointer to I2C handle
  * @retval None
  */
void PCA9535APW_Init(I2C_HandleTypeDef *hi2c)
{
    pca_i2c = hi2c;

    // Set all pins as inputs by default (reset state)
    PCA9535APW_SetPortDirection(0, 0xFF);
    PCA9535APW_SetPortDirection(1, 0xFF);
}

/**
  * @brief  Set port direction (input/output)
  * @param  port: 0 for port 0, 1 for port 1
  * @param  direction: 0 = output, 1 = input (bitwise)
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_SetPortDirection(uint8_t port, uint8_t direction)
{
    uint8_t reg = (port == 0) ? PCA9535APW_CONFIG_PORT0 : PCA9535APW_CONFIG_PORT1;
    return HAL_I2C_Mem_Write(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &direction, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write value to output port
  * @param  port: 0 for port 0, 1 for port 1
  * @param  value: Value to write
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_WritePort(uint8_t port, uint8_t value)
{
    uint8_t reg = (port == 0) ? PCA9535APW_OUTPUT_PORT0 : PCA9535APW_OUTPUT_PORT1;
    return HAL_I2C_Mem_Write(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Read value from input port
  * @param  port: 0 for port 0, 1 for port 1
  * @param  value: Pointer to store read value
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_ReadPort(uint8_t port, uint8_t *value)
{
    uint8_t reg = (port == 0) ? PCA9535APW_INPUT_PORT0 : PCA9535APW_INPUT_PORT1;
    return HAL_I2C_Mem_Read(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write single pin state
  * @param  port: 0 for port 0, 1 for port 1
  * @param  pin: Pin number (0-7)
  * @param  state: 0 = LOW, 1 = HIGH
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_WritePin(uint8_t port, uint8_t pin, uint8_t state)
{
    uint8_t reg = (port == 0) ? PCA9535APW_OUTPUT_PORT0 : PCA9535APW_OUTPUT_PORT1;
    uint8_t current_val;

    // Read current output register value
    if (HAL_I2C_Mem_Read(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &current_val, 1, HAL_MAX_DELAY) != HAL_OK)
        return PCA9535APW_ERROR;

    // Modify the specific bit
    if (state)
        current_val |= (1 << pin);
    else
        current_val &= ~(1 << pin);

    // Write back the new value
    return HAL_I2C_Mem_Write(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &current_val, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Read single pin state
  * @param  port: 0 for port 0, 1 for port 1
  * @param  pin: Pin number (0-7)
  * @param  state: Pointer to store pin state (0 or 1)
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_ReadPin(uint8_t port, uint8_t pin, uint8_t *state)
{
    uint8_t value;
    if (PCA9535APW_ReadPort(port, &value) != PCA9535APW_OK)
        return PCA9535APW_ERROR;

    *state = (value >> pin) & 0x01;
    return PCA9535APW_OK;
}

/**
  * @brief  Set input polarity inversion
  * @param  port: 0 for port 0, 1 for port 1
  * @param  polarity: 0 = normal, 1 = inverted (bitwise)
  * @retval PCA9535APW_OK on success, PCA9535APW_ERROR on failure
  */
int PCA9535APW_SetPolarity(uint8_t port, uint8_t polarity)
{
    uint8_t reg = (port == 0) ? PCA9535APW_POL_INV_PORT0 : PCA9535APW_POL_INV_PORT1;
    return HAL_I2C_Mem_Write(pca_i2c, PCA9535APW_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &polarity, 1, HAL_MAX_DELAY);
}
