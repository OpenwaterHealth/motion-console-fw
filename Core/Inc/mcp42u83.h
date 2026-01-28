#ifndef MCP42U83_H
#define MCP42U83_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MCP42U83 register addresses (datasheet Table 4-3) */
#define MCP42U83_REG_WIPER0         0x01
#define MCP42U83_REG_WIPER1         0x02
#define MCP42U83_REG_TCON0          0x0A
#define MCP42U83_REG_STATUS         0x08

/* MCP42U83 command codes (datasheet Table 7-1 / 6-2) */
#define MCP42U83_CMD_WRITE          0x00  // 00Xb
#define MCP42U83_CMD_READ           0x06  // 11Xb
#define MCP42U83_CMD_INCREMENT      0x02  // 010b
#define MCP42U83_CMD_DECREMENT      0x04  // 100b

/* TCON0 bits (datasheet Section 4.7.1) */
#define MCP42U83_TCON_R0A            (1U << 0)
#define MCP42U83_TCON_R0W            (1U << 1)
#define MCP42U83_TCON_R0B            (1U << 2)
#define MCP42U83_TCON_R1A            (1U << 3)
#define MCP42U83_TCON_R1W            (1U << 4)
#define MCP42U83_TCON_R1B            (1U << 5)

/* Maximum wiper position (10-bit) */
#define MCP42U83_MAX_POSITION       1023
#define MCP42U83_MID_POSITION       512
#define MCP42U83_DEFAULT_POSITION   302
#define MCP42U83_MIN_POSITION       0

/* Number of potentiometers */
#define MCP42U83_NUM_POTS           2

#define MCP42U83_I2C_ADRESS       0x2C

/**
 * @brief MCP42U83 potentiometer channel enumeration
 */
typedef enum {
    MCP42U83_POT_0 = 0,      // Potentiometer 0
    MCP42U83_POT_1 = 1,      // Potentiometer 1
} mcp42u83_pot_channel;

/**
 * @brief MCP42U83 resistance options (kΩ)
 */
typedef enum {
    MCP42U83_R_AB_5K = 5,
    MCP42U83_R_AB_10K = 10,
    MCP42U83_R_AB_20K = 20,
    MCP42U83_R_AB_50K = 50,
    MCP42U83_R_AB_100K = 100,
} mcp42u83_resistance_option;

/**
 * @brief MCP42U83 resistance options (kΩ)
 */
typedef enum {
    /** Driver not initialized / not ready */
    MCP42U83_DRIVER_STATE_NOT_INITIALIZED = 0,
    /** Driver initialized and ready for use */
    MCP42U83_DRIVER_STATE_READY,
    /** Driver encountered an error */
    MCP42U83_DRIVER_STATE_ERROR,
} mcp42u83_driver_state;

/**
 * @brief MCP42U83 device structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;      // SPI handle
    GPIO_TypeDef *cs_port;         // Chip select GPIO port
    uint16_t cs_pin;               // Chip select GPIO pin
    uint32_t timeout_ms;           // communication timeout in milliseconds
    uint16_t wiper_pos[MCP42U83_NUM_POTS];  // Current wiper positions (cached)
    float resistance_kohm;         // Total resistance in kΩ (5/10/20/50/100kΩ)
    I2C_HandleTypeDef *hi2c;      // I2C handle (set for I2C mode)
    uint16_t i2c_addr;            // 7-bit I2C address (I2C mode)
    /* Driver-managed state. Do not set this field manually. */
    mcp42u83_driver_state state;
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
 * @param resistance Total resistance option for the device (kΩ)
 * @param timeout_ms timeout for bus transfers in milliseconds
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef mcp42u83_init(mcp42u83_dev *dev, SPI_HandleTypeDef *hspi,
                                            GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                            I2C_HandleTypeDef *hi2c, uint16_t i2c_addr,
                                            mcp42u83_resistance_option resistance,
                                            uint32_t timeout_ms);

/**
 * @brief Set wiper position for a specific potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel 
 * @param position Wiper position (0-1023)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint16_t position);

/**
 * @brief Set wiper position for both potentiometers simultaneously
 * 
 * @param dev Pointer to device structure
 * @param position Wiper position (0-1023)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_both_wipers(mcp42u83_dev *dev, uint16_t position);

/**
 * @brief Set wiper positions for both potentiometers independently
 * 
 * @param dev Pointer to device structure
 * @param pos0 Wiper position for potentiometer 0 (0-1023)
 * @param pos1 Wiper position for potentiometer 1 (0-1023)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_set_wipers(mcp42u83_dev *dev, uint16_t pos0, uint16_t pos1);

/**
 * @brief Get cached wiper position for a specific potentiometer
 * 
 * @param dev Pointer to device structure
 * @param channel Potentiometer channel (POT_0 or POT_1)
 * @return uint16_t Current wiper position (0-1023)
 */
uint16_t mcp42u83_get_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel);

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
 * @param position Initial wiper position (0-1023)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef mcp42u83_wakeup(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                   uint16_t position);

/**
 * @brief Calculate resistance for a given wiper position
 * 
 * @param dev Pointer to device structure
 * @param position Wiper position (0-1023)
 * @return float Resistance in kΩ (RAW to wiper)
 */
float mcp42u83_position_to_resistance(const mcp42u83_dev *dev, uint16_t position);

/**
 * @brief Calculate wiper position for a desired resistance
 * 
 * @param dev Pointer to device structure
 * @param resistance_kohm Desired resistance in kΩ
 * @return uint16_t Wiper position (0-1023)
 */
uint16_t mcp42u83_resistance_to_position(const mcp42u83_dev *dev, float resistance_kohm);

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
