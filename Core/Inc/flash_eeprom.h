/*
 * flash_eeprom.h
 *
 *  Created on: Feb 4, 2026
 *      Author: gvigelet
 */

#ifndef INC_FLASH_EEPROM_H_
#define INC_FLASH_EEPROM_H_

#include "stm32h7xx_hal.h"
#include "memory_map.h"


/* Function prototypes */
HAL_StatusTypeDef Flash_Write(uint32_t address, const uint32_t* data, uint32_t size);
HAL_StatusTypeDef Flash_Read(uint32_t address, uint32_t* data, uint32_t size);
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address);
HAL_StatusTypeDef Flash_Write_Bytes(uint32_t address, const uint8_t *data, uint32_t size_bytes);
HAL_StatusTypeDef Flash_Read_Bytes(uint32_t address, uint8_t *data, uint32_t size_bytes);

#endif /* INC_FLASH_EEPROM_H_ */
