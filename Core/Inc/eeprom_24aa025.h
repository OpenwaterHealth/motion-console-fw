/*
 * eeprom_24aa025.h
 *
 * Driver for the Microchip 24AA025E48 I2C serial EEPROM (U49 on the console
 * board). Per the schematic the EEPROM sits behind the I2C2 TCA9548 mux
 * ("mux1", @0x70) on channel 2 (SC2/SD2 = EEPROM_SCL/EEPROM_SDA), at device
 * address 0x50. Every access selects that mux channel first, exactly like the
 * other muxed devices (temps on ch1, TEC ADC on ch3, etc.).
 *
 *  - 2 Kbit organised as 256 x 8 (256 bytes total)
 *  - 16-byte page write buffer
 *  - 1,000,000 write-cycle endurance, >200 yr retention
 *  - Upper bytes 0xFA..0xFF hold a factory-programmed, write-protected EUI-48
 *    node address. Never write at/above EEPROM_PROTECTED_START.
 *
 *  Created on: Jun 16, 2026
 *      Author: Claude
 */

#ifndef INC_EEPROM_24AA025_H_
#define INC_EEPROM_24AA025_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define EEPROM_PAGE_SIZE        16u    /* page write buffer size */
#define EEPROM_TOTAL_BYTES      256u   /* full array size */

/* The EUI-48 node address occupies the last 6 bytes (0xFA..0xFF) and the
 * upper block is write-protected. Treat everything from here up as read-only;
 * the odometer ring stays strictly below this. */
#define EEPROM_PROTECTED_START  0xF0u
#define EEPROM_EUI48_ADDR       0xFAu
#define EEPROM_EUI48_LEN        6u

/* Location on the board (see schematic sheet 17). */
#define EEPROM_MUX_INDEX        1u     /* TCA9548 "mux1" on I2C2 */
#define EEPROM_MUX_CHANNEL      2u     /* SC2/SD2 -> EEPROM_SCL/SDA */
#define EEPROM_DEFAULT_ADDR7    0x50u  /* 1010b control code, A2/A1/A0 low */

typedef struct {
    I2C_HandleTypeDef *hi2c;        /* bus the mux hangs on (I2C2) */
    uint8_t            mux_index;   /* TCA9548 index */
    uint8_t            mux_channel; /* channel the EEPROM is on */
    uint8_t            addr7;       /* latched 7-bit device address */
    bool               present;
} eeprom_dev_t;

/**
 * @brief Detect the EEPROM on its mux channel and latch its address. On success
 * dev->present is true. Logs the result. Does NOT touch other mux channels.
 * @return HAL_OK if found, HAL_ERROR otherwise (dev->present == false).
 */
HAL_StatusTypeDef EEPROM_Init(eeprom_dev_t *dev);

/**
 * @brief Sequential read of len bytes starting at mem_addr (selects the mux
 * channel first). Reads may span the whole array in one transfer.
 */
HAL_StatusTypeDef EEPROM_Read(eeprom_dev_t *dev, uint8_t mem_addr,
                              uint8_t *buf, uint16_t len);

/**
 * @brief Write len bytes starting at mem_addr. Selects the mux channel, splits
 * the transfer on 16-byte page boundaries and ACK-polls each page's internal
 * write cycle. Refuses any write that would touch the protected EUI-48 region
 * (>= EEPROM_PROTECTED_START).
 */
HAL_StatusTypeDef EEPROM_Write(eeprom_dev_t *dev, uint8_t mem_addr,
                               const uint8_t *buf, uint16_t len);

/**
 * @brief Read the factory-programmed 6-byte EUI-48 node address.
 */
HAL_StatusTypeDef EEPROM_ReadEUI48(eeprom_dev_t *dev, uint8_t eui[EEPROM_EUI48_LEN]);

#endif /* INC_EEPROM_24AA025_H_ */
