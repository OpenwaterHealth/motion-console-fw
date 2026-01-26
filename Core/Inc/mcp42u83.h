#ifndef MCP42U83_H
#define MCP42U83_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MCP42U83 Command Byte Format */
#define MCP42U83_CMD_WRITE          0x00  // Write data to specified pot
#define MCP42U83_CMD_SHUTDOWN       0x20  // Enter low-power shutdown mode

/* Address Bits */
#define MCP42U83_ADDR_POT0          0x01  // Potentiometer 0 address
#define MCP42U83_ADDR_POT1          0x02  // Potentiometer 1 address
#define MCP42U83_ADDR_BOTH          0x03  // Both potentiometers

/* Shutdown Modes */
#define MCP42U83_SHUTDOWN_POT0      0x01  // Shutdown potentiometer 0
#define MCP42U83_SHUTDOWN_POT1      0x02  // Shutdown potentiometer 1
#define MCP42U83_SHUTDOWN_BOTH      0x03  // Shutdown both potentiometers

/* Maximum wiper position (8-bit) */
#define MCP42U83_MAX_POSITION       255
#define MCP42U83_MID_POSITION       128
#define MCP42U83_MIN_POSITION       0

/* Number of potentiometers */
#define MCP42U83_NUM_POTS           2

/**
 * @brief MCP42U83 potentiometer channel enumeration
 */
typedef enum {
    MCP42U83_POT_0 = 0,      // Potentiometer 0
    MCP42U83_POT_1 = 1,      // Potentiometer 1
} mcp42u83_pot_channel;

/**
 * @brief MCP42U83 device structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;      // SPI handle
    GPIO_TypeDef *cs_port;         // Chip select GPIO port
    uint16_t cs_pin;               // Chip select GPIO pin
    uint32_t timeout_ms;           // communication timeout in milliseconds
    uint8_t wiper_pos[MCP42U83_NUM_POTS];  // Current wiper positions (cached)
    float resistance_kohm;         // Total resistance in kΩ (83kΩ for MCP42U83)
    I2C_HandleTypeDef *hi2c;      // I2C handle (set for I2C mode)
    uint16_t i2c_addr;            // 7-bit I2C address (I2C mode)
} mcp42u83_dev;

/**
 * @brief Initialize MCP42U83 device structure
 * 
 * @param dev Pointer to device structure
 * @param hspi Pointer to SPI handle
 * @param cs_port Chip select GPIO port
 * @param cs_pin Chip select GPIO pin
 * @param timeout_ms SPI timeout in milliseconds (default: 100)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
/**
 * @brief Initialize MCP42U83 device structure
 *
 * Either the SPI parameters or the I2C parameters must be provided, but not both.
 * - For SPI mode: provide non-NULL `hspi` and `cs_port`/`cs_pin`, set `hi2c` NULL and `i2c_addr` 0.
 * - For I2C mode: provide non-NULL `hi2c` and a valid 7-bit `i2c_addr`, set `hspi` NULL.
 *
 * @param dev Pointer to device structure
 * @param hspi SPI handle (NULL if using I2C)
 * @param cs_port CS GPIO port (SPI only)
 * @param cs_pin CS GPIO pin (SPI only)
 * @param hi2c I2C handle (NULL if using SPI)
 * @param i2c_addr 7-bit I2C address (0 if using SPI)
 * @param timeout_ms timeout for bus transfers in milliseconds
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef mcp42u83_init(mcp42u83_dev *dev, SPI_HandleTypeDef *hspi,
                                            GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                            I2C_HandleTypeDef *hi2c, uint16_t i2c_addr,
                                            uint32_t timeout_ms);

/**
 * @brief Set wiper position for a specific potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @param position Wiper position (0-255)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t position);

/**
 * @brief Set wiper position for both potentiometers simultaneously
 * 
 * @param dev Pointer to device structure
 * @param position Wiper position (0-255)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_both_wipers(mcp42u83_dev *dev, uint8_t position);

/**
 * @brief Set wiper positions for both potentiometers independently
 * 
 * @param dev Pointer to device structure
 * @param pos0 Wiper position for potentiometer 0 (0-255)
 * @param pos1 Wiper position for potentiometer 1 (0-255)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_wipers(mcp42u83_dev *dev, uint8_t pos0, uint8_t pos1);

/**
 * @brief Get cached wiper position for a specific potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @return uint8_t Current wiper position (0-255)
 */
uint8_t mcp42u83_get_wiper(const mcp42u83_dev *dev, mcp42u83_pot_channel channel);

/**
 * @brief Shutdown specific potentiometer channel
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_shutdown(mcp42u83_dev *dev, mcp42u83_pot_channel channel);

/**
 * @brief Shutdown both potentiometer channels
 * 
 * @param dev Pointer to device structure
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_shutdown_both(mcp42u83_dev *dev);

/**
 * @brief Wake up from shutdown by writing to a potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel to wake up (POT_0 or POT_1)
 * @param position Initial wiper position (0-255)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_wakeup(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                   uint8_t position);

/**
 * @brief Calculate resistance for a given wiper position
 * 
 * @param dev Pointer to device structure
 * @param position Wiper position (0-255)
 * @return float Resistance in kΩ (RAW to wiper)
 */
float mcp42u83_position_to_resistance(const mcp42u83_dev *dev, uint8_t position);

/**
 * @brief Calculate wiper position for a desired resistance
 * 
 * @param dev Pointer to device structure
 * @param resistance_kohm Desired resistance in kΩ
 * @return uint8_t Wiper position (0-255)
 */
uint8_t mcp42u83_resistance_to_position(const mcp42u83_dev *dev, float resistance_kohm);

/**
 * @brief Set resistance for a specific potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @param resistance_kohm Desired resistance in kΩ
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_resistance(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                           float resistance_kohm);

/**
 * @brief Increment wiper position
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @param steps Number of steps to increment (default: 1)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_increment(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t steps);

/**
 * @brief Decrement wiper position
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @param steps Number of steps to decrement (default: 1)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_decrement(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t steps);

#ifdef __cplusplus
}
#endif

#endif /* MCP42U83_H */
