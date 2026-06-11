/*
 * odometer.h
 *
 *  Created on: Apr 29, 2026
 *      Author: Claude
 */

#ifndef INC_ODOMETER_H_
#define INC_ODOMETER_H_

#include "stm32h7xx_hal.h"
#include "memory_map.h"
#include <stdint.h>
#include <stdbool.h>

/* Flash memory addresses for odometer storage.
 *
 * Lives in its own 128 KB sector (sector 6 bank 2 via FLASH_ODOMETER_START_ADDR),
 * distinct from FLASH_USER_START_ADDR which is owned by motion_config. The
 * STM32H7 minimum erase granularity is one 128 KB sector, so the two stores
 * must not co-tenant — otherwise either Odometer_Persist_Both() or
 * motion_cfg flash writes would wipe the other tenant. */
#define ODOMETER_FLASH_BASE_ADDR    (FLASH_ODOMETER_START_ADDR)
#define SYSTEM_ODO_FLASH_ADDR       (ODOMETER_FLASH_BASE_ADDR)
#define LASER_ODO_FLASH_ADDR        (ODOMETER_FLASH_BASE_ADDR + 32)

/* Magic number identifying a valid odometer flash block. Reading anything other
 * than this magic + matching CRC at boot means the block is invalid (uninitialised
 * flash, stale data from a previous firmware that used this sector for something
 * else, partial-erase corruption) — the init path treats that as "first boot"
 * and writes a fresh zero block. */
#define ODOMETER_MAGIC 0x4F444F31u  /* "ODO1" */

/* On-flash format. 16 bytes per odometer; written as a 32-byte zero-padded block. */
typedef struct __attribute__((packed)) {
    uint32_t magic;       /* ODOMETER_MAGIC */
    uint32_t value;       /* total_minutes for system, total_pulses for laser */
    uint32_t reserved;    /* 0 for now, future fields */
    uint32_t crc32;       /* CRC32 of magic + value + reserved */
} OdoFlashBlock_t;

/* Update interval: how often we erase+rewrite the flash sector during normal
 * runtime to persist the system odometer. STM32H7 flash sector endurance is
 * ~10K cycles, so we keep this conservative — 6 hours = ~4 writes/day, giving
 * roughly 7 years of continuous-run wear life per device. The laser odometer
 * is only persisted on scan-finish (a natural rate-limiter), so it doesn't
 * need its own interval. */
#define SYSTEM_ODO_UPDATE_INTERVAL_MS  (6UL * 60UL * 60UL * 1000UL)  /* 6 hours */

/* Reset targets for Odometer_Reset() / OW_CTRL_RESET_ODO. */
typedef enum {
    ODO_RESET_SYSTEM = 0,
    ODO_RESET_LASER  = 1,
    ODO_RESET_BOTH   = 2,
} OdoResetTarget;

/* In-memory state — kept opaque, accessed via Odometer_Get_*() */
typedef struct {
    uint32_t total_minutes;      /* Total accumulated minutes */
    uint32_t last_update_tick;   /* Last HAL_GetTick() value when saved */
} SystemOdometer_t;

typedef struct {
    uint32_t total_pulses;       /* Total laser sync pulses */
    uint32_t scan_start_pulses;  /* Pulse count at scan start */
} LaserOdometer_t;

/* Function prototypes */
HAL_StatusTypeDef Odometer_Init(void);
HAL_StatusTypeDef Odometer_Update_System(void);
HAL_StatusTypeDef Odometer_Scan_Start(void);
HAL_StatusTypeDef Odometer_Scan_Finish(void);
uint32_t Odometer_Get_System_Minutes(void);
uint32_t Odometer_Get_Laser_Pulses(void);

/* Reset one or both odometers to zero and persist the cleared state to flash.
 * target ∈ ODO_RESET_SYSTEM / _LASER / _BOTH. Returns HAL_OK on success. */
HAL_StatusTypeDef Odometer_Reset(OdoResetTarget target);

#endif /* INC_ODOMETER_H_ */
