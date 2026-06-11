/*
 * odometer.c
 *
 *  Created on: Apr 29, 2026
 *      Author: Claude
 */

#include "odometer.h"
#include "flash_eeprom.h"
#include "memory_map.h"
#include "trigger.h"
#include <stdio.h>
#include <string.h>

/* Static odometer data */
static SystemOdometer_t system_odo;
static LaserOdometer_t laser_odo;
static bool odometer_initialized = false;

/**
 * @brief Simple CRC32 (poly 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF).
 * Bit-by-bit implementation — odometer reads/writes are infrequent so a table
 * isn't worth the .rodata footprint.
 */
static uint32_t odo_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/**
 * @brief Pack a value into an OdoFlashBlock_t with magic + CRC.
 */
static void odo_pack_block(OdoFlashBlock_t *out, uint32_t value)
{
    out->magic = ODOMETER_MAGIC;
    out->value = value;
    out->reserved = 0;
    out->crc32 = odo_crc32(out, offsetof(OdoFlashBlock_t, crc32));
}

/**
 * @brief Validate an OdoFlashBlock_t read from flash. Returns true if magic
 * matches and CRC is correct.
 */
static bool odo_block_valid(const OdoFlashBlock_t *blk)
{
    if (blk->magic != ODOMETER_MAGIC) {
        return false;
    }
    uint32_t expected = odo_crc32(blk, offsetof(OdoFlashBlock_t, crc32));
    return blk->crc32 == expected;
}

/**
 * @brief Erase the odometer flash sector (shared by both odometers).
 */
static HAL_StatusTypeDef Odometer_Erase_Sector(void)
{
    HAL_StatusTypeDef status = Flash_Erase(ODOMETER_FLASH_BASE_ADDR, ODOMETER_FLASH_BASE_ADDR);
    if (status == HAL_OK) {
        printf("Odometer flash sector erased\r\n");
    } else {
        printf("Failed to erase odometer flash sector\r\n");
    }
    return status;
}

/**
 * @brief Write both odometers to flash. Caller must have erased the sector
 * first (flash bytes must be 0xFF before write).
 */
static HAL_StatusTypeDef Odometer_Write_All_To_Flash(void)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[32];

    /* System odometer block. */
    memset(buffer, 0, sizeof(buffer));
    OdoFlashBlock_t sys_blk;
    odo_pack_block(&sys_blk, system_odo.total_minutes);
    memcpy(buffer, &sys_blk, sizeof(sys_blk));
    status = Flash_Write_Bytes(SYSTEM_ODO_FLASH_ADDR, buffer, sizeof(buffer));
    if (status != HAL_OK) {
        printf("Failed to write system odometer to flash\r\n");
        return status;
    }

    /* Laser odometer block. */
    memset(buffer, 0, sizeof(buffer));
    OdoFlashBlock_t laser_blk;
    odo_pack_block(&laser_blk, laser_odo.total_pulses);
    memcpy(buffer, &laser_blk, sizeof(laser_blk));
    status = Flash_Write_Bytes(LASER_ODO_FLASH_ADDR, buffer, sizeof(buffer));
    if (status != HAL_OK) {
        printf("Failed to write laser odometer to flash\r\n");
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Persist current in-memory odometer values: erase the sector and
 * write both blocks back. Either both succeed or both are left undefined —
 * callers that hit an error should treat the flash state as suspect.
 */
static HAL_StatusTypeDef Odometer_Persist_Both(void)
{
    HAL_StatusTypeDef status = Odometer_Erase_Sector();
    if (status != HAL_OK) {
        return status;
    }
    return Odometer_Write_All_To_Flash();
}

/**
 * @brief Initialize odometer module by reading from flash.
 * If the read block doesn't carry the expected magic + matching CRC, treats
 * it as first boot: zeros the in-memory value AND immediately writes a valid
 * zero block to flash so the next boot reads clean. This recovers automatically
 * from uninitialised flash, partial erases, and stale data left behind by a
 * previous firmware that used the same sector.
 */
HAL_StatusTypeDef Odometer_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[32];
    bool need_rewrite = false;

    /* Zero the in-memory state up front so a partial init still leaves
     * sensible values. */
    memset(&system_odo, 0, sizeof(system_odo));
    memset(&laser_odo, 0, sizeof(laser_odo));

    /* --- System odometer --- */
    status = Flash_Read_Bytes(SYSTEM_ODO_FLASH_ADDR, buffer, sizeof(OdoFlashBlock_t));
    if (status != HAL_OK) {
        printf("Failed to read system odometer from flash\r\n");
        return status;
    }
    OdoFlashBlock_t sys_blk;
    memcpy(&sys_blk, buffer, sizeof(sys_blk));
    if (odo_block_valid(&sys_blk)) {
        system_odo.total_minutes = sys_blk.value;
    } else {
        printf("System odometer flash invalid (magic=0x%08lX) — resetting\r\n",
               (unsigned long)sys_blk.magic);
        system_odo.total_minutes = 0;
        need_rewrite = true;
    }
    system_odo.last_update_tick = HAL_GetTick();

    /* --- Laser odometer --- */
    status = Flash_Read_Bytes(LASER_ODO_FLASH_ADDR, buffer, sizeof(OdoFlashBlock_t));
    if (status != HAL_OK) {
        printf("Failed to read laser odometer from flash\r\n");
        return status;
    }
    OdoFlashBlock_t laser_blk;
    memcpy(&laser_blk, buffer, sizeof(laser_blk));
    if (odo_block_valid(&laser_blk)) {
        laser_odo.total_pulses = laser_blk.value;
    } else {
        printf("Laser odometer flash invalid (magic=0x%08lX) — resetting\r\n",
               (unsigned long)laser_blk.magic);
        laser_odo.total_pulses = 0;
        need_rewrite = true;
    }
    laser_odo.scan_start_pulses = 0;

    odometer_initialized = true;

    /* If we recovered from invalid flash, write clean blocks now so the next
     * boot doesn't have to do this again. Best-effort — a failure here is
     * survivable (we just repeat the recovery on next boot). */
    if (need_rewrite) {
        if (Odometer_Persist_Both() != HAL_OK) {
            printf("Odometer: failed to persist recovered zero state\r\n");
        }
    }

    printf("Odometers initialized: System=%lu min, Laser=%lu pulses\r\n",
           system_odo.total_minutes, laser_odo.total_pulses);
    return HAL_OK;
}

/**
 * @brief Update system odometer (call periodically).
 * Accumulates minutes since last persist; when the configured update interval
 * has elapsed, erases the sector and writes both blocks back.
 */
HAL_StatusTypeDef Odometer_Update_System(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms;

    /* Handle tick overflow (every ~49.7 days) */
    if (current_tick >= system_odo.last_update_tick) {
        elapsed_ms = current_tick - system_odo.last_update_tick;
    } else {
        /* Overflow occurred */
        elapsed_ms = (0xFFFFFFFF - system_odo.last_update_tick) + current_tick + 1;
    }

    /* Persist on the configured interval (default 6 h — see odometer.h). */
    if (elapsed_ms >= SYSTEM_ODO_UPDATE_INTERVAL_MS) {
        uint32_t minutes_to_add = elapsed_ms / (60UL * 1000UL);
        system_odo.total_minutes += minutes_to_add;
        system_odo.last_update_tick = current_tick;

        HAL_StatusTypeDef status = Odometer_Persist_Both();
        if (status != HAL_OK) {
            return status;
        }
        printf("System odometer persisted: %lu minutes\r\n", system_odo.total_minutes);
    }

    return HAL_OK;
}

/**
 * @brief Called when a scan starts. Records the current laser pulse count.
 */
HAL_StatusTypeDef Odometer_Scan_Start(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }
    laser_odo.scan_start_pulses = get_lsync_pulse_count();
    printf("Scan started, laser pulses at start: %lu\r\n", laser_odo.scan_start_pulses);
    return HAL_OK;
}

/**
 * @brief Called when a scan finishes. Calculates pulses during the scan and
 * persists the updated total.
 */
HAL_StatusTypeDef Odometer_Scan_Finish(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    uint32_t current_pulses = get_lsync_pulse_count();
    uint32_t scan_pulses;
    if (current_pulses >= laser_odo.scan_start_pulses) {
        scan_pulses = current_pulses - laser_odo.scan_start_pulses;
    } else {
        /* Counter overflow (unlikely but possible) */
        scan_pulses = (0xFFFFFFFF - laser_odo.scan_start_pulses) + current_pulses + 1;
    }

    laser_odo.total_pulses += scan_pulses;
    printf("Scan finished, pulses this scan: %lu, total: %lu\r\n",
           (unsigned long)scan_pulses, (unsigned long)laser_odo.total_pulses);

    HAL_StatusTypeDef status = Odometer_Persist_Both();
    if (status != HAL_OK) {
        return status;
    }

    laser_odo.scan_start_pulses = 0;
    return HAL_OK;
}

/**
 * @brief Reset one or both odometers to zero and persist.
 * Note: the underlying sector erase wipes BOTH blocks, so even a single-target
 * reset has to re-write the other from in-memory state. That's fine — the
 * other in-memory value is the most up-to-date we have.
 */
HAL_StatusTypeDef Odometer_Reset(OdoResetTarget target)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    switch (target) {
        case ODO_RESET_SYSTEM:
            system_odo.total_minutes = 0;
            system_odo.last_update_tick = HAL_GetTick();
            printf("Odometer reset: system\r\n");
            break;
        case ODO_RESET_LASER:
            laser_odo.total_pulses = 0;
            laser_odo.scan_start_pulses = 0;
            printf("Odometer reset: laser\r\n");
            break;
        case ODO_RESET_BOTH:
            system_odo.total_minutes = 0;
            system_odo.last_update_tick = HAL_GetTick();
            laser_odo.total_pulses = 0;
            laser_odo.scan_start_pulses = 0;
            printf("Odometer reset: both\r\n");
            break;
        default:
            return HAL_ERROR;
    }

    return Odometer_Persist_Both();
}

/**
 * @brief Get current system uptime in minutes.
 * Returns the persisted total plus any accumulated time since the last save,
 * so the value advances smoothly between persists.
 */
uint32_t Odometer_Get_System_Minutes(void)
{
    if (!odometer_initialized) {
        return 0;
    }

    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms;
    if (current_tick >= system_odo.last_update_tick) {
        elapsed_ms = current_tick - system_odo.last_update_tick;
    } else {
        elapsed_ms = (0xFFFFFFFF - system_odo.last_update_tick) + current_tick + 1;
    }

    uint32_t additional_minutes = elapsed_ms / (60UL * 1000UL);
    return system_odo.total_minutes + additional_minutes;
}

/**
 * @brief Get total laser pulses.
 */
uint32_t Odometer_Get_Laser_Pulses(void)
{
    if (!odometer_initialized) {
        return 0;
    }
    return laser_odo.total_pulses;
}
