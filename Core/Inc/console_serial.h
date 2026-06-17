/*
 * console_serial.h
 *
 * Console hardware serial number, persisted in the external 24AA025E48 EEPROM
 * (U49) at offset SERIAL_EEPROM_ADDR. The 32-byte record sits just below the
 * odometer ring and below the write-protected EUI-48 region; see
 * docs/superpowers/specs/2026-06-17-console-serial-eeprom-design.md.
 */

#ifndef INC_CONSOLE_SERIAL_H_
#define INC_CONSOLE_SERIAL_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SERIAL_RECORD_MAGIC    0x4E53u   /* 'SN' */
#define SERIAL_RECORD_VERSION  1u
#define SERIAL_MAX_LEN         24u
#define SERIAL_EEPROM_ADDR     0xD0u     /* 32 bytes -> ends at 0xF0 (protected start) */

/* On-EEPROM layout. Exactly 32 bytes = two 16-byte pages. */
typedef struct __attribute__((packed)) {
    uint16_t magic;                 /* SERIAL_RECORD_MAGIC */
    uint8_t  version;               /* SERIAL_RECORD_VERSION */
    uint8_t  len;                   /* valid chars in serial[], 1..SERIAL_MAX_LEN */
    char     serial[SERIAL_MAX_LEN];/* ASCII, uppercase alnum, NOT NUL-terminated */
    uint16_t reserved;              /* 0 */
    uint16_t crc16;                 /* CRC16-CCITT over the preceding 30 bytes */
} serial_record_t;

_Static_assert(sizeof(serial_record_t) == 32,
               "serial_record_t must be exactly 32 bytes (two EEPROM pages)");

/** @brief Detect the EEPROM and latch the device. Call once at boot. */
HAL_StatusTypeDef Serial_Init(void);

/** @brief True if a valid (magic+version+len+CRC) serial record is stored. */
bool Serial_IsProgrammed(void);

/**
 * @brief Read the stored serial into @p out (capacity >= SERIAL_MAX_LEN) and
 * set *len to its length. Returns HAL_ERROR (and *len = 0) if unprogrammed.
 */
HAL_StatusTypeDef Serial_Read(char *out, uint8_t *len);

/**
 * @brief Validate (1..SERIAL_MAX_LEN uppercase-alnum chars) and write the
 * serial. If a valid serial already exists and @p force is false, returns
 * HAL_BUSY without writing. Read-back-verifies on success.
 */
HAL_StatusTypeDef Serial_Write(const char *s, uint8_t len, bool force);

#endif /* INC_CONSOLE_SERIAL_H_ */
