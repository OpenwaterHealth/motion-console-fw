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
 * @brief Initialize odometer module by reading from flash
 * @retval HAL status
 */
HAL_StatusTypeDef Odometer_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[32];

    /* Read system odometer from flash */
    status = Flash_Read_Bytes(SYSTEM_ODO_FLASH_ADDR, buffer, sizeof(SystemOdometer_t));
    if (status != HAL_OK) {
        printf("Failed to read system odometer from flash\r\n");
        return status;
    }

    memcpy(&system_odo, buffer, sizeof(SystemOdometer_t));

    /* Check if this is first boot (flash is erased = 0xFFFFFFFF) */
    if (system_odo.total_minutes == 0xFFFFFFFF) {
        system_odo.total_minutes = 0;
    }

    /* Initialize the last update tick to current time */
    system_odo.last_update_tick = HAL_GetTick();

    /* Read laser odometer from flash */
    status = Flash_Read_Bytes(LASER_ODO_FLASH_ADDR, buffer, sizeof(LaserOdometer_t));
    if (status != HAL_OK) {
        printf("Failed to read laser odometer from flash\r\n");
        return status;
    }

    memcpy(&laser_odo, buffer, sizeof(LaserOdometer_t));

    /* Check if this is first boot */
    if (laser_odo.total_pulses == 0xFFFFFFFF) {
        laser_odo.total_pulses = 0;
    }

    laser_odo.scan_start_pulses = 0;

    odometer_initialized = true;

    printf("Odometers initialized: System=%lu min, Laser=%lu pulses\r\n",
           system_odo.total_minutes, laser_odo.total_pulses);

    return HAL_OK;
}

/**
 * @brief Update system odometer (call periodically)
 * Updates the in-memory counter and writes to flash every 15 minutes
 * @retval HAL status
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

    /* Check if we need to update (every 15 minutes) */
    if (elapsed_ms >= SYSTEM_ODO_UPDATE_INTERVAL_MS) {
        /* Calculate minutes to add */
        uint32_t minutes_to_add = elapsed_ms / (60 * 1000);
        system_odo.total_minutes += minutes_to_add;

        /* Update last update tick */
        system_odo.last_update_tick = current_tick;

        /* Write to flash */
        uint8_t buffer[32];
        memset(buffer, 0, sizeof(buffer));
        memcpy(buffer, &system_odo, sizeof(SystemOdometer_t));

        /* Erase flash sector before writing */
        HAL_StatusTypeDef status = Flash_Erase(SYSTEM_ODO_FLASH_ADDR, SYSTEM_ODO_FLASH_ADDR);
        if (status != HAL_OK) {
            printf("Failed to erase system odometer flash\r\n");
            return status;
        }

        status = Flash_Write_Bytes(SYSTEM_ODO_FLASH_ADDR, buffer, sizeof(buffer));
        if (status != HAL_OK) {
            printf("Failed to write system odometer to flash\r\n");
            return status;
        }

        printf("System odometer updated: %lu minutes\r\n", system_odo.total_minutes);
    }

    return HAL_OK;
}

/**
 * @brief Called when a scan starts
 * Records the current laser pulse count
 * @retval HAL status
 */
HAL_StatusTypeDef Odometer_Scan_Start(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    /* Record the current pulse count at scan start */
    laser_odo.scan_start_pulses = get_lsync_pulse_count();

    printf("Scan started, laser pulses at start: %lu\r\n", laser_odo.scan_start_pulses);

    return HAL_OK;
}

/**
 * @brief Called when a scan finishes
 * Calculates the pulses during the scan and updates flash
 * @retval HAL status
 */
HAL_StatusTypeDef Odometer_Scan_Finish(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    /* Get current pulse count */
    uint32_t current_pulses = get_lsync_pulse_count();

    /* Calculate pulses during this scan */
    uint32_t scan_pulses = 0;
    if (current_pulses >= laser_odo.scan_start_pulses) {
        scan_pulses = current_pulses - laser_odo.scan_start_pulses;
    } else {
        /* Handle counter overflow (unlikely but possible) */
        scan_pulses = (0xFFFFFFFF - laser_odo.scan_start_pulses) + current_pulses + 1;
    }

    /* Add to total */
    laser_odo.total_pulses += scan_pulses;

    printf("Scan finished, pulses this scan: %lu, total: %lu\r\n",
           scan_pulses, laser_odo.total_pulses);

    /* Write to flash */
    uint8_t buffer[32];
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, &laser_odo, sizeof(LaserOdometer_t));

    /* Erase flash sector before writing */
    HAL_StatusTypeDef status = Flash_Erase(LASER_ODO_FLASH_ADDR, LASER_ODO_FLASH_ADDR);
    if (status != HAL_OK) {
        printf("Failed to erase laser odometer flash\r\n");
        return status;
    }

    status = Flash_Write_Bytes(LASER_ODO_FLASH_ADDR, buffer, sizeof(buffer));
    if (status != HAL_OK) {
        printf("Failed to write laser odometer to flash\r\n");
        return status;
    }

    /* Reset scan start pulses */
    laser_odo.scan_start_pulses = 0;

    return HAL_OK;
}

/**
 * @brief Get current system uptime in minutes
 * Includes both the stored value and any accumulated time since last save
 * @retval Total system runtime in minutes
 */
uint32_t Odometer_Get_System_Minutes(void)
{
    if (!odometer_initialized) {
        return 0;
    }

    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms;

    /* Handle tick overflow */
    if (current_tick >= system_odo.last_update_tick) {
        elapsed_ms = current_tick - system_odo.last_update_tick;
    } else {
        elapsed_ms = (0xFFFFFFFF - system_odo.last_update_tick) + current_tick + 1;
    }

    /* Calculate current total including unsaved time */
    uint32_t additional_minutes = elapsed_ms / (60 * 1000);

    return system_odo.total_minutes + additional_minutes;
}

/**
 * @brief Get total laser pulses
 * @retval Total laser sync pulses accumulated
 */
uint32_t Odometer_Get_Laser_Pulses(void)
{
    if (!odometer_initialized) {
        return 0;
    }

    return laser_odo.total_pulses;
}
