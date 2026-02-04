/*
 * flash_eeprom.c
 *
 *  Created on: Feb 4, 2026
 *      Author: gvigelet
 */

#include "flash_eeprom.h"


/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if (Address < FLASH_BANK2_BASE)
    {
        sector = (Address - FLASH_BANK1_BASE) / FLASH_SECTOR_SIZE;
    }
    else
    {
        sector = (Address - FLASH_BANK2_BASE) / FLASH_SECTOR_SIZE;
    }

    if (sector > FLASH_SECTOR_7)
        sector = FLASH_SECTOR_7;

    return sector;
}

/* Function to write data to Flash */
__attribute__((section(".ramfunc")))
HAL_StatusTypeDef Flash_Write(uint32_t address, const uint32_t *data, uint32_t size_words)
{
    HAL_StatusTypeDef status;
    uint32_t flash_word[8];

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    uint32_t curr_addr = address;
    uint32_t idx = 0;

    while (idx < size_words)
    {
        uint32_t aligned_addr = curr_addr & ~0x1F;
        uint32_t word_offset = (curr_addr - aligned_addr) / 4;

        /* Read existing flashword */
        for (uint32_t i = 0; i < 8; i++)
        {
            flash_word[i] = *(__IO uint32_t *)(aligned_addr + i * 4);
        }

        /* Merge new data */
        while (word_offset < 8 && idx < size_words)
        {
            flash_word[word_offset++] = data[idx++];
            curr_addr += 4;
        }

        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            aligned_addr,
            (uint32_t)flash_word
        );

        if (status != HAL_OK)
            break;
    }

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}

/* Function to read data from Flash */
HAL_StatusTypeDef Flash_Read(uint32_t address, uint32_t* data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint32_t*)(address + (i * sizeof(uint32_t)));
    }
    return HAL_OK;
}


__attribute__((section(".ramfunc")))
HAL_StatusTypeDef Flash_Write_Bytes(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t size_bytes)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t flash_word[8];   // 256-bit buffer (8 x 32-bit)

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    uint32_t curr_addr = address;
    uint32_t idx = 0;

    while (idx < size_bytes)
    {
        uint32_t aligned_addr = curr_addr & ~0x1F; // 32-byte alignment
        uint32_t byte_offset  = curr_addr - aligned_addr;

        /* Read existing flashword */
        for (uint32_t i = 0; i < 8; i++)
        {
            flash_word[i] = *(__IO uint32_t *)(aligned_addr + i * 4);
        }

        /* Modify bytes inside the flashword */
        uint8_t *fw_bytes = (uint8_t *)flash_word;

        while (byte_offset < 32 && idx < size_bytes)
        {
            fw_bytes[byte_offset++] = data[idx++];
            curr_addr++;
        }

        /* If we didn’t fill the entire flashword, pad with 0x00 */
        while (byte_offset < 32)
        {
            fw_bytes[byte_offset++] = 0x00;
        }

        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            aligned_addr,
            (uint32_t)flash_word
        );

        if (status != HAL_OK)
            break;
    }

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}


/* Function to read data from Flash */
HAL_StatusTypeDef Flash_Read_Bytes(uint32_t address,
                                   uint8_t *data,
                                   uint32_t size_bytes)
{
    for (uint32_t i = 0; i < size_bytes; i++)
    {
        data[i] = *(__IO uint8_t *)(address + i);
    }

    return HAL_OK;
}

/* Function to erase Flash memory */
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase;
    uint32_t error;

    uint32_t first_sector = GetSector(start_address);
    uint32_t last_sector  = GetSector(end_address);
    uint32_t bank;

    if (start_address < FLASH_BANK2_BASE)
        bank = FLASH_BANK_1;
    else
        bank = FLASH_BANK_2;

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Banks        = bank;
    erase.Sector       = first_sector;
    erase.NbSectors    = (last_sector - first_sector) + 1;

    status = HAL_FLASHEx_Erase(&erase, &error);

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}
