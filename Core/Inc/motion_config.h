#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "stm32h7xx_hal.h"
#include "flash_eeprom.h"
#include "memory_map.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Magic/version identifiers stored in flash
#define MOTION_MAGIC   (0x4D4F5449UL)  // 'MOTI'
#define MOTION_VER     (0x00010000UL)  // bump if layout changes

// Flash layout info: one 2KB page at 0x0801F000
#define MOTION_CFG_PAGE_ADDR      (ADDR_FLASH_SECTOR_7_BANK2)
#define MOTION_CFG_PAGE_END       (ADDR_FLASH_END_ADDRESS)
#define MOTION_CFG_PAGE_SIZE      (2036U)

// magic(4) + version(4) + seq(4) + hv_settng(2) + hv_enabled(1) + auto_on(1) = 16 bytes
#define MOTION_CFG_HEADER_SIZE    (16U)


// The rest of the page is JSON storage (must include '\0'):
// 2048 - 16 - 2 = 2030
#define MOTION_CFG_JSON_MAX       (MOTION_CFG_PAGE_SIZE - MOTION_CFG_HEADER_SIZE)
// -> 2030 bytes

// Persistent config blob that exactly fills one flash page.
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;        // MOTION_MAGIC
    uint32_t version;      // MOTION_VER
    uint32_t seq;          // monotonic counter

    uint16_t crc;          // CRC16-CCITT over bytes [0 .. offsetof(crc)-1]
    uint8_t  reserved;
    uint8_t  reserved2;

    char     json[MOTION_CFG_JSON_MAX]; // NUL-terminated text blob
} motion_cfg_t;

// Sanity checks for layout
_Static_assert(sizeof(motion_cfg_t) == MOTION_CFG_PAGE_SIZE,
               "motion_cfg_t must fill one 128KB flash page");
_Static_assert((sizeof(motion_cfg_t) % 4U) == 0U,
               "motion_cfg_t size must be 32-bit word aligned for flash writes");

// ======================== PUBLIC API ========================

// Returns pointer to the live in-RAM copy of the config.
// On first call, it will load from flash, validate magic/version/CRC,
// and if invalid it writes factory defaults to flash and returns that.
const motion_cfg_t *motion_cfg_get(void);

// Copies the current config into *out so you can edit it offline.
// Example:
//    motion_cfg_t work;
//    motion_cfg_snapshot(&work);
//    work.hv_settng = 123;
//    motion_cfg_save(&work);
HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out);

// Saves a modified struct to flash.
// - You pass in a struct you edited (typically from motion_cfg_snapshot).
// - We copy fields we care about into internal storage,
//   bump seq, recalc CRC, erase/program flash.
HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg);

// Commits the *current* live config (as returned by motion_cfg_get())
// back to flash. This is only needed if you directly mutate *motion_cfg_get().
HAL_StatusTypeDef motion_cfg_commit(void);

// Restores factory defaults and writes them to flash.
HAL_StatusTypeDef motion_cfg_factory_reset(void);

// Convenience helpers for working with the JSON blob.
// - motion_cfg_get_json_ptr() returns a pointer into the live config (NUL-terminated).
// - motion_cfg_set_json() persists the provided JSON text into flash.
const char *motion_cfg_get_json_ptr(void);
HAL_StatusTypeDef motion_cfg_set_json(const char *json, size_t len);

// ======================== WIRE FORMAT (UART/USB) ========================
// When sending config over the command interface, we serialize as:
//   [motion_cfg_wire_hdr_t][json bytes (json_len)]
// For WRITE, you may send either:
//   - full wire buffer (header+json), OR
//   - raw JSON bytes (no header)
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t seq;
    uint16_t crc;
    uint16_t json_len; // number of JSON bytes included (may include trailing '\0')
} motion_cfg_wire_hdr_t;

// Produces a pointer to an internal buffer containing the serialized config.
// max_payload_len is the caller's maximum allowed payload size.
HAL_StatusTypeDef motion_cfg_wire_read(const uint8_t **out_buf,
                                       uint16_t *out_len,
                                       uint16_t max_payload_len);

// Applies a serialized buffer to config and writes it to flash.
// Accepts either full wire buffer (header+json) or raw JSON bytes.
HAL_StatusTypeDef motion_cfg_wire_write(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // MOTION_CONFIG_H
