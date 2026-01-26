#include "mcp42u83.h"
#include <string.h>
#include <math.h>

/* Internal helper functions */
static inline void cs_low(const mcp42u83_dev *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const mcp42u83_dev *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Write command and data to MCP42U83
 * 
 * @param dev Pointer to device structure
 * @param cmd_byte Command byte (command + address)
 * @param data_byte Data byte (wiper position or shutdown bits)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
static HAL_StatusTypeDef mcp42u83_write(mcp42u83_dev *dev, uint8_t cmd_byte, uint8_t data_byte)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    
    // Build the two-byte command
    tx_data[0] = cmd_byte;
    tx_data[1] = data_byte;
    
    // Pull CS low
    cs_low(dev);
    
    // Transmit the command
    status = HAL_SPI_Transmit(dev->hspi, tx_data, 2, dev->timeout_ms);
    
    // Pull CS high
    cs_high(dev);
    
    return status;
}

/**
 * @brief Initialize MCP42U83 device structure
 */
HAL_StatusTypeDef mcp42u83_init(mcp42u83_dev *dev, SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                 uint32_t timeout_ms)
{
    if (dev == NULL || hspi == NULL || cs_port == NULL) {
        return HAL_ERROR;
    }
    
    // Initialize device structure
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->timeout_ms = (timeout_ms > 0) ? timeout_ms : 100;
    dev->resistance_kohm = 83.0f;  // MCP42U83 has 83kΩ resistance
    
    // Initialize wiper positions to mid-scale
    dev->wiper_pos[MCP42U83_POT_0] = MCP42U83_MID_POSITION;
    dev->wiper_pos[MCP42U83_POT_1] = MCP42U83_MID_POSITION;
    
    // Set CS high (inactive)
    cs_high(dev);
    
    // Set both wipers to mid-scale position
    return mcp42u83_set_both_wipers(dev, MCP42U83_MID_POSITION);
}

/**
 * @brief Set wiper position for a specific potentiometer
 */
HAL_StatusTypeDef mcp42u83_set_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t position)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Validate channel
    if (channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    // Build command byte: write command + potentiometer address
    cmd_byte = MCP42U83_CMD_WRITE | (channel == MCP42U83_POT_0 ? MCP42U83_ADDR_POT0 : MCP42U83_ADDR_POT1);
    
    // Write to device
    status = mcp42u83_write(dev, cmd_byte, position);
    
    // Update cached position on success
    if (status == HAL_OK) {
        dev->wiper_pos[channel] = position;
    }
    
    return status;
}

/**
 * @brief Set wiper position for both potentiometers simultaneously
 */
HAL_StatusTypeDef mcp42u83_set_both_wipers(mcp42u83_dev *dev, uint8_t position)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Build command byte: write command + both pots address
    cmd_byte = MCP42U83_CMD_WRITE | MCP42U83_ADDR_BOTH;
    
    // Write to device
    status = mcp42u83_write(dev, cmd_byte, position);
    
    // Update cached positions on success
    if (status == HAL_OK) {
        dev->wiper_pos[MCP42U83_POT_0] = position;
        dev->wiper_pos[MCP42U83_POT_1] = position;
    }
    
    return status;
}

/**
 * @brief Set wiper positions for both potentiometers independently
 */
HAL_StatusTypeDef mcp42u83_set_wipers(mcp42u83_dev *dev, uint8_t pos0, uint8_t pos1)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Set potentiometer 0
    status = mcp42u83_set_wiper(dev, MCP42U83_POT_0, pos0);
    if (status != HAL_OK) {
        return status;
    }
    
    // Set potentiometer 1
    status = mcp42u83_set_wiper(dev, MCP42U83_POT_1, pos1);
    
    return status;
}

/**
 * @brief Get cached wiper position for a specific potentiometer
 */
uint8_t mcp42u83_get_wiper(const mcp42u83_dev *dev, mcp42u83_pot_channel channel)
{
    if (dev == NULL || channel >= MCP42U83_NUM_POTS) {
        return 0;
    }
    
    return dev->wiper_pos[channel];
}

/**
 * @brief Shutdown specific potentiometer channel
 */
HAL_StatusTypeDef mcp42u83_shutdown(mcp42u83_dev *dev, mcp42u83_pot_channel channel)
{
    uint8_t cmd_byte;
    uint8_t shutdown_bits;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Validate channel
    if (channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    // Build command byte
    cmd_byte = MCP42U83_CMD_SHUTDOWN;
    shutdown_bits = (channel == MCP42U83_POT_0) ? MCP42U83_SHUTDOWN_POT0 : MCP42U83_SHUTDOWN_POT1;
    
    // Write shutdown command
    return mcp42u83_write(dev, cmd_byte, shutdown_bits);
}

/**
 * @brief Shutdown both potentiometer channels
 */
HAL_StatusTypeDef mcp42u83_shutdown_both(mcp42u83_dev *dev)
{
    uint8_t cmd_byte;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Build command byte
    cmd_byte = MCP42U83_CMD_SHUTDOWN;
    
    // Write shutdown command for both pots
    return mcp42u83_write(dev, cmd_byte, MCP42U83_SHUTDOWN_BOTH);
}

/**
 * @brief Wake up from shutdown by writing to a potentiometer
 */
HAL_StatusTypeDef mcp42u83_wakeup(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                   uint8_t position)
{
    // Simply write to the potentiometer to wake it up
    return mcp42u83_set_wiper(dev, channel, position);
}

/**
 * @brief Calculate resistance for a given wiper position
 * 
 * The resistance from RAW to wiper is:
 * R_W(n) = (R_AB * n) / 256 + R_W
 * where n is the wiper position (0-255)
 * R_AB is the total end-to-end resistance (83kΩ for MCP42U83)
 * R_W is the wiper resistance (typically 75-200Ω, negligible)
 */
float mcp42u83_position_to_resistance(const mcp42u83_dev *dev, uint8_t position)
{
    if (dev == NULL) {
        return 0.0f;
    }
    
    // Calculate resistance: R = (R_total * position) / 256
    // For 8-bit pot, we have 256 steps (0-255)
    float resistance = (dev->resistance_kohm * (float)position) / 256.0f;
    
    return resistance;
}

/**
 * @brief Calculate wiper position for a desired resistance
 */
uint8_t mcp42u83_resistance_to_position(const mcp42u83_dev *dev, float resistance_kohm)
{
    if (dev == NULL) {
        return 0;
    }
    
    // Clamp resistance to valid range
    if (resistance_kohm < 0.0f) {
        resistance_kohm = 0.0f;
    }
    if (resistance_kohm > dev->resistance_kohm) {
        resistance_kohm = dev->resistance_kohm;
    }
    
    // Calculate position: n = (R * 256) / R_total
    float position_f = (resistance_kohm * 256.0f) / dev->resistance_kohm;
    
    // Round and clamp to valid range
    uint32_t position = (uint32_t)(position_f + 0.5f);
    if (position > MCP42U83_MAX_POSITION) {
        position = MCP42U83_MAX_POSITION;
    }
    
    return (uint8_t)position;
}

/**
 * @brief Set resistance for a specific potentiometer
 */
HAL_StatusTypeDef mcp42u83_set_resistance(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                           float resistance_kohm)
{
    uint8_t position;
    
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    // Convert resistance to position
    position = mcp42u83_resistance_to_position(dev, resistance_kohm);
    
    // Set wiper position
    return mcp42u83_set_wiper(dev, channel, position);
}

/**
 * @brief Increment wiper position
 */
HAL_StatusTypeDef mcp42u83_increment(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t steps)
{
    uint8_t current_pos;
    uint32_t new_pos;
    
    if (dev == NULL || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    if (steps == 0) {
        return HAL_OK;  // Nothing to do
    }
    
    // Get current position
    current_pos = dev->wiper_pos[channel];
    
    // Calculate new position with saturation
    new_pos = (uint32_t)current_pos + (uint32_t)steps;
    if (new_pos > MCP42U83_MAX_POSITION) {
        new_pos = MCP42U83_MAX_POSITION;
    }
    
    // Set new position
    return mcp42u83_set_wiper(dev, channel, (uint8_t)new_pos);
}

/**
 * @brief Decrement wiper position
 */
HAL_StatusTypeDef mcp42u83_decrement(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t steps)
{
    uint8_t current_pos;
    int32_t new_pos;
    
    if (dev == NULL || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    if (steps == 0) {
        return HAL_OK;  // Nothing to do
    }
    
    // Get current position
    current_pos = dev->wiper_pos[channel];
    
    // Calculate new position with saturation
    new_pos = (int32_t)current_pos - (int32_t)steps;
    if (new_pos < 0) {
        new_pos = 0;
    }
    
    // Set new position
    return mcp42u83_set_wiper(dev, channel, (uint8_t)new_pos);
}
