/*
 * flash_eeprom.c
 *
 *  Created on: Mar 4, 2024
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
  
  if(((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))    
  {
    sector = FLASH_SECTOR_0;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))    
  {
    sector = FLASH_SECTOR_1;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))    
  {
    sector = FLASH_SECTOR_2;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))    
  {
    sector = FLASH_SECTOR_3;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))    
  {
    sector = FLASH_SECTOR_4;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))    
  {
    sector = FLASH_SECTOR_5;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))    
  {
    sector = FLASH_SECTOR_6;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
  {
     sector = FLASH_SECTOR_7;  
  }
  else
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}

/* Function to write data to Flash */
HAL_StatusTypeDef Flash_Write(uint32_t address, uint32_t* data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK){
    	return status;
    }

    /* Disable instruction cache prior to internal cacheable memory update */
    SCB_DisableICache();

    for (uint32_t i = 0; i < size; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + (i * sizeof(uint32_t)), data[i]);
        if (status != HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();

    /* Enable instruction cache prior to internal cacheable memory update */
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

/* Function to erase Flash memory */
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address) {
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t error;
    uint32_t FirstSector = 0, NbOfSectors = 0;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK){
    	return status;        
    }
    
    /* Disable instruction cache prior to internal cacheable memory update */
    SCB_DisableICache();

    /* Get the 1st sector to erase */
    FirstSector = GetSector(start_address);
    /* Get the number of sector to erase from 1st sector*/
    NbOfSectors = GetSector(end_address) - FirstSector + 1;
    
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    erase_init.Banks         = FLASH_BANK_1;
    erase_init.Sector        = FirstSector;
    erase_init.NbSectors     = NbOfSectors;

    status = HAL_FLASHEx_Erase(&erase_init, &error);

    HAL_FLASH_Lock();

    /* Enable instruction cache prior to internal cacheable memory update */
    SCB_EnableICache();

    return status;
}
