/*
 * odometer.h
 *
 *  Created on: Apr 29, 2026
 *      Author: Claude
 *
 * System + laser odometers, persisted to the external 24AA025E48 I2C EEPROM
 * (U49) instead of internal flash. The EEPROM endures ~1,000,000 write cycles
 * per byte (vs ~10,000 for an STM32H7 flash sector) and is written a 16-byte
 * page at a time with no bulk erase, so we can persist far more often. A simple
 * wear-levelling ring spreads writes across ODO_RING_SLOTS slots, multiplying
 * device life by that factor again.
 */

#ifndef INC_ODOMETER_H_
#define INC_ODOMETER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Wear-levelling ring in the user region of the EEPROM. Each record is exactly
 * one 16-byte EEPROM page, page-aligned, so every persist is a single page
 * write. The ring lives in 0x00..0xCF (13 slots); 0xD0..0xEF holds the console
 * serial record (see console_serial.h); 0xF0..0xFF is left untouched (the
 * 24AA025E48's protected EUI-48 node address sits at 0xFA..0xFF). */
#define ODO_RECORD_SIZE   16u
#define ODO_RING_BASE     0x00u
#define ODO_RING_SLOTS    13u   /* 13 * 16 = 208 bytes -> ends at 0xD0; serial record lives at 0xD0..0xEF */

/* Magic + sequence + CRC identify the newest valid record at boot and reject
 * uninitialised / foreign bytes. */
#define ODO_RECORD_MAGIC  0x4F44u  /* "OD" */

/* On-EEPROM record layout. Exactly ODO_RECORD_SIZE bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;          /* ODO_RECORD_MAGIC */
    uint16_t seq;            /* wrapping write sequence; newest wins */
    uint32_t total_minutes;  /* system odometer */
    uint32_t total_pulses;   /* laser odometer */
    uint16_t reserved;       /* 0 for now */
    uint16_t crc16;          /* CRC16-CCITT over the preceding 14 bytes */
} OdoRecord_t;

_Static_assert(sizeof(OdoRecord_t) == ODO_RECORD_SIZE,
               "OdoRecord_t must be exactly one EEPROM page");

/* How often the system odometer is persisted during normal runtime. The
 * external EEPROM's endurance lets us go much tighter than the old 6-hour
 * internal-flash cadence: at 2 min that's ~720 writes/day, spread over
 * ODO_RING_SLOTS slots => ~48 writes/byte/day => >50 yr at 1e6 cycles. */
#define SYSTEM_ODO_UPDATE_INTERVAL_MS  (2UL * 60UL * 1000UL)  /* 2 minutes */

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

/* Read the EEPROM's factory-programmed 6-byte EUI-48 (a unique per-board
 * serial). Returns HAL_OK and fills eui[6] on success; HAL_ERROR if the EEPROM
 * wasn't detected at init. */
HAL_StatusTypeDef Odometer_Get_EUI48(uint8_t eui[6]);

/* Reset one or both odometers to zero and persist the cleared state to the
 * EEPROM. target ∈ ODO_RESET_SYSTEM / _LASER / _BOTH. Returns HAL_OK on success. */
HAL_StatusTypeDef Odometer_Reset(OdoResetTarget target);

#endif /* INC_ODOMETER_H_ */
