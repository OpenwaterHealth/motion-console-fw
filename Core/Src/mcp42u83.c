#include "mcp42u83.h"
#include <string.h>
#include <math.h>

mcp42u83_dev mcp42u83_device;

static float mcp42u83_resistance_option_to_kohm(mcp42u83_resistance_option option)
{
    switch (option) {
        case MCP42U83_R_AB_5K:
            return 5.0f;
        case MCP42U83_R_AB_10K:
            return 10.0f;
        case MCP42U83_R_AB_20K:
            return 20.0f;
        case MCP42U83_R_AB_50K:
            return 50.0f;
        case MCP42U83_R_AB_100K:
            return 100.0f;
        default:
            return 10.0f;
    }
}

/* Internal helper functions */
static inline void cs_low(const mcp42u83_dev *dev)
{
    if (dev && dev->hspi && dev->cs_port) {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    }
}

static inline void cs_high(const mcp42u83_dev *dev)
{
    if (dev && dev->hspi && dev->cs_port) {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Write command and data to MCP42U83
 * 
 * @param dev Pointer to device structure
 * @param cmd_byte Command byte (command + address)
 * @param data_byte Data byte (wiper position or shutdown bits)
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
static uint8_t mcp42u83_build_cmd(uint8_t reg_addr, uint8_t cmd)
{
    return (uint8_t)(((reg_addr & 0x1F) << 3) | (cmd & 0x07));
}

static HAL_StatusTypeDef mcp42u83_spi_write(mcp42u83_dev *dev, uint8_t cmd_byte, uint8_t data_hi, uint8_t data_lo)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[3];

    tx_data[0] = cmd_byte;
    tx_data[1] = data_hi;
    tx_data[2] = data_lo;

    cs_low(dev);
    status = HAL_SPI_Transmit(dev->hspi, tx_data, 3, dev->timeout_ms);
    cs_high(dev);
    return status;
}

static HAL_StatusTypeDef mcp42u83_spi_read(mcp42u83_dev *dev, uint8_t cmd_byte, uint16_t *out)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[3] = { cmd_byte, 0x00, 0x00 };
    uint8_t rx_data[3] = {0};

    cs_low(dev);
    status = HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, 3, dev->timeout_ms);
    cs_high(dev);
    if (status != HAL_OK) {
        return status;
    }

    *out = (uint16_t)(((uint16_t)(rx_data[1] & 0x03) << 8) | rx_data[2]);
    return HAL_OK;
}

static HAL_StatusTypeDef mcp42u83_i2c_write(mcp42u83_dev *dev, uint8_t cmd_byte, uint8_t data_hi, uint8_t data_lo)
{
    uint8_t tx_data[3];
    uint16_t addr8 = (uint16_t)(dev->i2c_addr << 1);

    tx_data[0] = cmd_byte;
    tx_data[1] = data_hi;
    tx_data[2] = data_lo;

    return HAL_I2C_Master_Transmit(dev->hi2c, addr8, tx_data, 3, dev->timeout_ms);
}

static HAL_StatusTypeDef mcp42u83_i2c_read(mcp42u83_dev *dev, uint8_t cmd_byte, uint16_t *out)
{
    HAL_StatusTypeDef status;
    uint8_t rx_data[2] = {0};
    uint16_t addr8 = (uint16_t)(dev->i2c_addr << 1);

    status = HAL_I2C_Master_Transmit(dev->hi2c, addr8, &cmd_byte, 1, dev->timeout_ms);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_I2C_Master_Receive(dev->hi2c, addr8, rx_data, 2, dev->timeout_ms);
    if (status != HAL_OK) {
        return status;
    }

    *out = (uint16_t)(((uint16_t)(rx_data[0] & 0x03) << 8) | rx_data[1]);
    return HAL_OK;
}

static HAL_StatusTypeDef mcp42u83_write_register(mcp42u83_dev *dev, uint8_t reg_addr, uint16_t value)
{
    uint8_t cmd = mcp42u83_build_cmd(reg_addr, MCP42U83_CMD_WRITE);
    uint8_t data_hi = (uint8_t)((value >> 8) & 0x03);
    uint8_t data_lo = (uint8_t)(value & 0xFF);

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return HAL_ERROR;
    }

    if (value > MCP42U83_MAX_POSITION && (reg_addr == MCP42U83_REG_WIPER0 || reg_addr == MCP42U83_REG_WIPER1)) {
        return HAL_ERROR;
    }

    if (dev->hspi != NULL) {
        return mcp42u83_spi_write(dev, cmd, data_hi, data_lo);
    }

    if (dev->hi2c != NULL) {
        return mcp42u83_i2c_write(dev, cmd, data_hi, data_lo);
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef mcp42u83_read_register(mcp42u83_dev *dev, uint8_t reg_addr, uint16_t *out)
{
    uint8_t cmd = mcp42u83_build_cmd(reg_addr, MCP42U83_CMD_READ);

    if (dev == NULL || out == NULL) {
        return HAL_ERROR;
    }

    if (dev->hspi != NULL) {
        return mcp42u83_spi_read(dev, cmd, out);
    }

    if (dev->hi2c != NULL) {
        return mcp42u83_i2c_read(dev, cmd, out);
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef mcp42u83_step_wiper(mcp42u83_dev *dev, uint8_t reg_addr, uint8_t cmd, uint8_t steps)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd_byte = mcp42u83_build_cmd(reg_addr, cmd);
    uint16_t i;

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return HAL_ERROR;
    }

    for (i = 0; i < steps; i++) {
        if (dev->hspi != NULL) {
            cs_low(dev);
            status = HAL_SPI_Transmit(dev->hspi, &cmd_byte, 1, dev->timeout_ms);
            cs_high(dev);
        } else if (dev->hi2c != NULL) {
            uint16_t addr8 = (uint16_t)(dev->i2c_addr << 1);
            status = HAL_I2C_Master_Transmit(dev->hi2c, addr8, &cmd_byte, 1, dev->timeout_ms);
        } else {
            return HAL_ERROR;
        }

        if (status != HAL_OK) {
            return status;
        }
    }

    return status;
}

/**
 * @brief Initialize MCP42U83 device structure
 */
HAL_StatusTypeDef mcp42u83_init(mcp42u83_dev *dev, SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                 I2C_HandleTypeDef *hi2c, uint16_t i2c_addr,
                                 mcp42u83_resistance_option resistance,
                                 uint32_t timeout_ms)
{
    if (dev == NULL) {
        return HAL_ERROR;
    }

    /* Start in not-initialized state; driver manages this field. */
    dev->state = MCP42U83_DRIVER_STATE_NOT_INITIALIZED;

    // Initialize device structure
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->hi2c = hi2c;
    dev->i2c_addr = i2c_addr;
    dev->timeout_ms = (timeout_ms > 0) ? timeout_ms : 100;
    dev->resistance_kohm = mcp42u83_resistance_option_to_kohm(resistance);
    
    // Initialize wiper positions to mid-scale
    dev->wiper_pos[MCP42U83_POT_0] = MCP42U83_DEFAULT_POSITION;
    dev->wiper_pos[MCP42U83_POT_1] = MCP42U83_DEFAULT_POSITION;
    
    // Validate mutually exclusive bus selection
    if ((dev->hspi != NULL && dev->hi2c != NULL) || (dev->hspi == NULL && dev->hi2c == NULL)) {
        dev->state = MCP42U83_DRIVER_STATE_ERROR;
        return HAL_ERROR; // must provide exactly one bus
    }

    if (dev->hspi != NULL) {
        // Set CS high (inactive)
        cs_high(dev);
    }
    
    dev->state = MCP42U83_DRIVER_STATE_READY;

    // Initialize device wipers to mid-scale on the selected bus
    HAL_StatusTypeDef status = mcp42u83_set_both_wipers(dev, MCP42U83_DEFAULT_POSITION);
    if (status != HAL_OK) {
        dev->state = MCP42U83_DRIVER_STATE_ERROR;
    }

    return status;
}

/**
 * @brief Set wiper position for a specific potentiometer
 */
HAL_StatusTypeDef mcp42u83_set_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint16_t position)
{
    HAL_StatusTypeDef status;
    uint8_t reg_addr;
    
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return HAL_ERROR;
    }
    
    // Validate channel
    if (channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    if (position > MCP42U83_MAX_POSITION) {
        return HAL_ERROR;
    }

    reg_addr = (channel == MCP42U83_POT_0) ? MCP42U83_REG_WIPER0 : MCP42U83_REG_WIPER1;
    status = mcp42u83_write_register(dev, reg_addr, position);
    
    // Update cached position on success
    if (status == HAL_OK) {
        dev->wiper_pos[channel] = position;
    }
    
    return status;
}

/**
 * @brief Set wiper position for both potentiometers simultaneously
 */
HAL_StatusTypeDef mcp42u83_set_both_wipers(mcp42u83_dev *dev, uint16_t position)
{
    HAL_StatusTypeDef status;
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return HAL_ERROR;
    }

    if (position > MCP42U83_MAX_POSITION) {
        return HAL_ERROR;
    }

    status = mcp42u83_write_register(dev, MCP42U83_REG_WIPER0, position);
    if (status != HAL_OK) {
        return status;
    }

    status = mcp42u83_write_register(dev, MCP42U83_REG_WIPER1, position);
    if (status == HAL_OK) {
        dev->wiper_pos[MCP42U83_POT_0] = position;
        dev->wiper_pos[MCP42U83_POT_1] = position;
    }

    return status;
}

/**
 * @brief Set wiper positions for both potentiometers independently
 */
HAL_StatusTypeDef mcp42u83_set_wipers(mcp42u83_dev *dev, uint16_t pos0, uint16_t pos1)
{
    HAL_StatusTypeDef status;
    
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
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
uint16_t mcp42u83_get_wiper(mcp42u83_dev *dev, mcp42u83_pot_channel channel)
{
    uint16_t value = 0;
    uint8_t reg_addr;

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY || channel >= MCP42U83_NUM_POTS) {
        return 0;
    }

    reg_addr = (channel == MCP42U83_POT_0) ? MCP42U83_REG_WIPER0 : MCP42U83_REG_WIPER1;
    if (mcp42u83_read_register(dev, reg_addr, &value) == HAL_OK) {
        dev->wiper_pos[channel] = value;
    }

    return dev->wiper_pos[channel];
}

/**
 * @brief Shutdown specific potentiometer channel
 */
HAL_StatusTypeDef mcp42u83_shutdown(mcp42u83_dev *dev, mcp42u83_pot_channel channel)
{
    uint16_t tcon = 0;
    uint16_t mask = 0;

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }

    if (mcp42u83_read_register(dev, MCP42U83_REG_TCON0, &tcon) != HAL_OK) {
        return HAL_ERROR;
    }

    if (channel == MCP42U83_POT_0) {
        mask = MCP42U83_TCON_R0A | MCP42U83_TCON_R0W | MCP42U83_TCON_R0B;
    } else {
        mask = MCP42U83_TCON_R1A | MCP42U83_TCON_R1W | MCP42U83_TCON_R1B;
    }

    tcon |= mask; // 1 = open switch (disconnect)
    return mcp42u83_write_register(dev, MCP42U83_REG_TCON0, tcon);
}

/**
 * @brief Shutdown both potentiometer channels
 */
HAL_StatusTypeDef mcp42u83_shutdown_both(mcp42u83_dev *dev)
{
    uint16_t tcon = 0;

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return HAL_ERROR;
    }

    if (mcp42u83_read_register(dev, MCP42U83_REG_TCON0, &tcon) != HAL_OK) {
        return HAL_ERROR;
    }

    tcon |= (MCP42U83_TCON_R0A | MCP42U83_TCON_R0W | MCP42U83_TCON_R0B |
             MCP42U83_TCON_R1A | MCP42U83_TCON_R1W | MCP42U83_TCON_R1B);

    return mcp42u83_write_register(dev, MCP42U83_REG_TCON0, tcon);
}

/**
 * @brief Wake up from shutdown by writing to a potentiometer
 */
HAL_StatusTypeDef mcp42u83_wakeup(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                   uint16_t position)
{
    uint16_t tcon = 0;
    uint16_t mask = 0;

    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }

    if (mcp42u83_read_register(dev, MCP42U83_REG_TCON0, &tcon) != HAL_OK) {
        return HAL_ERROR;
    }

    if (channel == MCP42U83_POT_0) {
        mask = MCP42U83_TCON_R0A | MCP42U83_TCON_R0W | MCP42U83_TCON_R0B;
    } else {
        mask = MCP42U83_TCON_R1A | MCP42U83_TCON_R1W | MCP42U83_TCON_R1B;
    }

    tcon &= (uint16_t)(~mask); // 0 = close switch (connect)
    if (mcp42u83_write_register(dev, MCP42U83_REG_TCON0, tcon) != HAL_OK) {
        return HAL_ERROR;
    }

    return mcp42u83_set_wiper(dev, channel, position);
}

/**
 * @brief Calculate resistance for a given wiper position
 * 
 * The resistance from RAW to wiper is:
 * R_W(n) = (R_AB * n) / 256 + R_W
 * where n is the wiper position (0-255)
 * R_AB is the total end-to-end resistance (configured in init)
 * R_W is the wiper resistance (typically 75-200Ω, negligible)
 */
float mcp42u83_position_to_resistance(const mcp42u83_dev *dev, uint16_t position)
{
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return 0.0f;
    }

    if (position > MCP42U83_MAX_POSITION) {
        position = MCP42U83_MAX_POSITION;
    }

    // Calculate resistance: R = (R_total * position) / 1023
    float resistance = (dev->resistance_kohm * (float)position) / 1023.0f;
    
    return resistance;
}

/**
 * @brief Calculate wiper position for a desired resistance
 */
uint16_t mcp42u83_resistance_to_position(const mcp42u83_dev *dev, float resistance_kohm)
{
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
        return 0;
    }
    
    // Clamp resistance to valid range
    if (resistance_kohm < 0.0f) {
        resistance_kohm = 0.0f;
    }
    if (resistance_kohm > dev->resistance_kohm) {
        resistance_kohm = dev->resistance_kohm;
    }
    
    // Calculate position: n = (R * 1023) / R_total
    float position_f = (resistance_kohm * 1023.0f) / dev->resistance_kohm;
    
    // Round and clamp to valid range
    uint32_t position = (uint32_t)(position_f + 0.5f);
    if (position > MCP42U83_MAX_POSITION) {
        position = MCP42U83_MAX_POSITION;
    }

    return (uint16_t)position;
}

/**
 * @brief Set resistance for a specific potentiometer
 */
HAL_StatusTypeDef mcp42u83_set_resistance(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                           float resistance_kohm)
{
    uint16_t position;
    
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY) {
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
    uint16_t current_pos;
    uint32_t new_pos;
    uint8_t reg_addr;
    
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    if (steps == 0) {
        return HAL_OK;  // Nothing to do
    }
    
    reg_addr = (channel == MCP42U83_POT_0) ? MCP42U83_REG_WIPER0 : MCP42U83_REG_WIPER1;

    current_pos = dev->wiper_pos[channel];
    new_pos = (uint32_t)current_pos + (uint32_t)steps;
    if (new_pos > MCP42U83_MAX_POSITION) {
        new_pos = MCP42U83_MAX_POSITION;
    }

    if (mcp42u83_step_wiper(dev, reg_addr, MCP42U83_CMD_INCREMENT, steps) != HAL_OK) {
        return HAL_ERROR;
    }

    dev->wiper_pos[channel] = (uint16_t)new_pos;
    return HAL_OK;
}

/**
 * @brief Decrement wiper position
 */
HAL_StatusTypeDef mcp42u83_decrement(mcp42u83_dev *dev, mcp42u83_pot_channel channel,
                                      uint8_t steps)
{
    uint16_t current_pos;
    int32_t new_pos;
    uint8_t reg_addr;
    
    if (dev == NULL || dev->state != MCP42U83_DRIVER_STATE_READY || channel >= MCP42U83_NUM_POTS) {
        return HAL_ERROR;
    }
    
    if (steps == 0) {
        return HAL_OK;  // Nothing to do
    }
    
    reg_addr = (channel == MCP42U83_POT_0) ? MCP42U83_REG_WIPER0 : MCP42U83_REG_WIPER1;

    current_pos = dev->wiper_pos[channel];
    new_pos = (int32_t)current_pos - (int32_t)steps;
    if (new_pos < 0) {
        new_pos = 0;
    }

    if (mcp42u83_step_wiper(dev, reg_addr, MCP42U83_CMD_DECREMENT, steps) != HAL_OK) {
        return HAL_ERROR;
    }

    dev->wiper_pos[channel] = (uint16_t)new_pos;
    return HAL_OK;
}
