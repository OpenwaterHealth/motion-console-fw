/*
 * odometer.h
 *
 *  Created on: Apr 29, 2026
 *      Author: Claude
 */

#ifndef INC_ODOMETER_H_
#define INC_ODOMETER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Flash memory addresses for odometer storage */
#define ODOMETER_FLASH_BASE_ADDR    (FLASH_USER_START_ADDR)
#define SYSTEM_ODO_FLASH_ADDR       (ODOMETER_FLASH_BASE_ADDR)
#define LASER_ODO_FLASH_ADDR        (ODOMETER_FLASH_BASE_ADDR + 32)

/* Update intervals */
#define SYSTEM_ODO_UPDATE_INTERVAL_MS  (15 * 60 * 1000)  /* 15 minutes */

/* Odometer data structure */
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

#endif /* INC_ODOMETER_H_ */
