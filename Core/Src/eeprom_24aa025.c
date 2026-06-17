/*
 * eeprom_24aa025.c
 *
 * See eeprom_24aa025.h for the device summary and API contract. The EEPROM is
 * behind the I2C2 TCA9548 mux on channel EEPROM_MUX_CHANNEL; every transaction
 * selects that channel first (mirroring ads7828.c / ads7924.c / XO2_cmds.c).
 *
 *  Created on: Jun 16, 2026
 *      Author: Claude
 */

#include "eeprom_24aa025.h"
#include "tca9548a.h"
#include <stdio.h>
#include <string.h>

/* I2C bus the EEPROM's mux hangs on (configured by MX_I2C2_Init in main.c). */
extern I2C_HandleTypeDef hi2c2;

/* Internal write cycle is <=5 ms; ACK-poll a bit longer to be safe. */
#define EEPROM_WRITE_CYCLE_MAX_MS  10u
#define EEPROM_IO_TIMEOUT_MS       50u

/** @brief Route the mux to the EEPROM's channel before a transaction. */
static HAL_StatusTypeDef eeprom_select(eeprom_dev_t *dev)
{
    return (TCA9548A_SelectChannel(dev->mux_index, dev->mux_channel) == TCA9548A_OK)
               ? HAL_OK : HAL_ERROR;
}

/**
 * @brief Poll the device until it ACKs, signalling the internal write cycle
 * has finished (or timeout). Assumes the mux channel is already selected.
 */
static HAL_StatusTypeDef eeprom_ack_poll(eeprom_dev_t *dev)
{
    uint32_t start = HAL_GetTick();
    do {
        if (HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(dev->addr7 << 1), 1, 2) == HAL_OK) {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start) < EEPROM_WRITE_CYCLE_MAX_MS);
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef EEPROM_Init(eeprom_dev_t *dev)
{
    if (dev == NULL) {
        return HAL_ERROR;
    }
    dev->hi2c = &hi2c2;
    dev->mux_index = EEPROM_MUX_INDEX;
    dev->mux_channel = EEPROM_MUX_CHANNEL;
    dev->addr7 = EEPROM_DEFAULT_ADDR7;
    dev->present = false;

    if (eeprom_select(dev) != HAL_OK) {
        printf("EEPROM: failed to select mux%u ch%u\r\n",
               dev->mux_index, dev->mux_channel);
        return HAL_ERROR;
    }

    /* The A2/A1/A0 straps fix the low address bits; probe the 0x50..0x57 block
     * so we tolerate any strapping. Only the EEPROM lives on this channel. */
    for (uint8_t addr = 0x50u; addr <= 0x57u; ++addr) {
        if (HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(addr << 1), 2, 50) == HAL_OK) {
            dev->addr7 = addr;
            dev->present = true;
            printf("EEPROM 24AA025E48 found on mux%u ch%u @ 0x%02X\r\n",
                   dev->mux_index, dev->mux_channel, addr);
            return HAL_OK;
        }
    }

    printf("EEPROM 24AA025E48 NOT FOUND on mux%u ch%u (0x50..0x57)\r\n",
           dev->mux_index, dev->mux_channel);
    return HAL_ERROR;
}

HAL_StatusTypeDef EEPROM_Read(eeprom_dev_t *dev, uint8_t mem_addr,
                              uint8_t *buf, uint16_t len)
{
    if (dev == NULL || !dev->present || buf == NULL) {
        return HAL_ERROR;
    }
    if ((uint32_t)mem_addr + len > EEPROM_TOTAL_BYTES) {
        return HAL_ERROR;
    }
    if (eeprom_select(dev) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->addr7 << 1), mem_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, len, EEPROM_IO_TIMEOUT_MS);
}

HAL_StatusTypeDef EEPROM_Write(eeprom_dev_t *dev, uint8_t mem_addr,
                               const uint8_t *buf, uint16_t len)
{
    if (dev == NULL || !dev->present || buf == NULL) {
        return HAL_ERROR;
    }
    /* Never write into (or across into) the protected EUI-48 region. */
    if ((uint32_t)mem_addr + len > EEPROM_PROTECTED_START) {
        printf("EEPROM_Write rejected: [0x%02X,+%u) hits protected region\r\n",
               mem_addr, (unsigned)len);
        return HAL_ERROR;
    }
    if (eeprom_select(dev) != HAL_OK) {
        return HAL_ERROR;
    }

    uint16_t written = 0;
    while (written < len) {
        uint8_t  cur_addr   = (uint8_t)(mem_addr + written);
        uint16_t page_room  = EEPROM_PAGE_SIZE - (cur_addr % EEPROM_PAGE_SIZE);
        uint16_t chunk      = (uint16_t)(len - written);
        if (chunk > page_room) {
            chunk = page_room;
        }

        HAL_StatusTypeDef status =
            HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr7 << 1), cur_addr,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buf[written],
                              chunk, EEPROM_IO_TIMEOUT_MS);
        if (status != HAL_OK) {
            return status;
        }

        /* Wait out the internal write cycle before the next page / return. */
        status = eeprom_ack_poll(dev);
        if (status != HAL_OK) {
            return status;
        }

        written += chunk;
    }
    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_ReadEUI48(eeprom_dev_t *dev, uint8_t eui[EEPROM_EUI48_LEN])
{
    if (dev == NULL || !dev->present || eui == NULL) {
        return HAL_ERROR;
    }
    if (eeprom_select(dev) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->addr7 << 1), EEPROM_EUI48_ADDR,
                            I2C_MEMADD_SIZE_8BIT, eui, EEPROM_EUI48_LEN,
                            EEPROM_IO_TIMEOUT_MS);
}
