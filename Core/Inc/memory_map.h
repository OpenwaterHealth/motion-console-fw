/*
 * memory_map.h
 *
 *  Created on: Mar 4, 2024
 *      Author: gvigelet
 */

#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_

#ifdef __cplusplus
 extern "C" {
#endif
 /* Base address of the Flash pages */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define ADDR_FLASH_END_ADDRESS     ((uint32_t)0x08200000)

/* Reserved persistent-data sectors. Each persists via full-sector erase
 * (the STM32H7 flash sector is 128 KB and that is the minimum erase
 * granularity), so distinct tenants need distinct sectors. The linker script
 * reserves these by capping FLASH at 1792 KB.
 *
 * NOTE: Sector 6 / bank 2 (0x081C0000) is OWNED BY THE BOOTLOADER as the
 * anti-rollback version floor (openmotion-bl: SBSFU/Target/Src/anti_rollback.c)
 * and is protected from the DFU update path. The application MUST NOT write it.
 * The odometer lives in the external EEPROM (see odometer.h), so the
 * FLASH_ODOMETER_START_ADDR define below is legacy/unused — do not repurpose
 * sector 6 for flash storage without coordinating with the bootloader. */
#define FLASH_USER_START_ADDR     ADDR_FLASH_SECTOR_7_BANK2  /* motion_config / calibration */
#define FLASH_ODOMETER_START_ADDR ADDR_FLASH_SECTOR_6_BANK2  /* RESERVED: bootloader anti-rollback floor — do not use */

#ifdef __cplusplus
}
#endif

#endif /* INC_MEMORY_MAP_H_ */
