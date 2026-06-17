/*
 * console_serial.c — see console_serial.h.
 *
 * Owns its own eeprom_dev_t (EEPROM_Init is a non-destructive detect, safe to
 * call alongside odometer.c's handle). CRC16-CCITT matches odometer.c /
 * motion_config.c (poly 0x1021, init 0xFFFF, no final XOR).
 */

#include "console_serial.h"
#include "eeprom_24aa025.h"
#include <string.h>
#include <stddef.h>
#include <stdio.h>

static eeprom_dev_t s_eeprom;
static bool         s_inited = false;

static uint16_t serial_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool char_is_serial_ok(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
}

static bool record_valid(const serial_record_t *rec)
{
    if (rec->magic != SERIAL_RECORD_MAGIC)        return false;
    if (rec->version != SERIAL_RECORD_VERSION)    return false;
    if (rec->len < 1 || rec->len > SERIAL_MAX_LEN) return false;
    uint16_t want = serial_crc16(rec, offsetof(serial_record_t, crc16));
    return rec->crc16 == want;
}

HAL_StatusTypeDef Serial_Init(void)
{
    if (EEPROM_Init(&s_eeprom) != HAL_OK) {
        printf("Serial: EEPROM not present — serial number unavailable\r\n");
        s_inited = false;
        return HAL_ERROR;
    }
    s_inited = true;
    return HAL_OK;
}

bool Serial_IsProgrammed(void)
{
    serial_record_t rec;
    if (!s_inited) {
        return false;
    }
    if (EEPROM_Read(&s_eeprom, SERIAL_EEPROM_ADDR, (uint8_t *)&rec, sizeof(rec)) != HAL_OK) {
        return false;
    }
    return record_valid(&rec);
}

HAL_StatusTypeDef Serial_Read(char *out, uint8_t *len)
{
    serial_record_t rec;
    if (!s_inited || out == NULL || len == NULL) {
        return HAL_ERROR;
    }
    *len = 0;
    if (EEPROM_Read(&s_eeprom, SERIAL_EEPROM_ADDR, (uint8_t *)&rec, sizeof(rec)) != HAL_OK) {
        return HAL_ERROR;
    }
    if (!record_valid(&rec)) {
        return HAL_ERROR;
    }
    memcpy(out, rec.serial, rec.len);
    *len = rec.len;
    return HAL_OK;
}

HAL_StatusTypeDef Serial_Write(const char *s, uint8_t len, bool force)
{
    serial_record_t rec;
    serial_record_t check;

    if (!s_inited || s == NULL) {
        return HAL_ERROR;
    }
    if (len < 1 || len > SERIAL_MAX_LEN) {
        return HAL_ERROR;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (!char_is_serial_ok(s[i])) {
            return HAL_ERROR;
        }
    }
    if (!force && Serial_IsProgrammed()) {
        return HAL_BUSY;   /* refuse to clobber an existing serial */
    }

    memset(&rec, 0, sizeof(rec));
    rec.magic   = SERIAL_RECORD_MAGIC;
    rec.version = SERIAL_RECORD_VERSION;
    rec.len     = len;
    memcpy(rec.serial, s, len);
    rec.reserved = 0;
    rec.crc16   = serial_crc16(&rec, offsetof(serial_record_t, crc16));

    if (EEPROM_Write(&s_eeprom, SERIAL_EEPROM_ADDR,
                     (const uint8_t *)&rec, sizeof(rec)) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Read-back verify. */
    if (EEPROM_Read(&s_eeprom, SERIAL_EEPROM_ADDR,
                    (uint8_t *)&check, sizeof(check)) != HAL_OK) {
        return HAL_ERROR;
    }
    if (memcmp(&rec, &check, sizeof(rec)) != 0) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
