#include "motion_config.h"

#include <string.h>
#include <stdbool.h>

static motion_cfg_t g_cfg;
static bool       g_cfg_loaded = false;

// ------------------- CRC16-CCITT -------------------
// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no final XOR.
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint16_t motion_cfg_calc_crc(const motion_cfg_t *cfg)
{
    // compute CRC across everything BEFORE the crc field
    return crc16_ccitt((const uint8_t *)cfg,
                       offsetof(motion_cfg_t, crc));
}

// ------------------- Helpers -------------------

// Ensure json is NUL-terminated and zero-pad tail so CRC is stable
static void motion_cfg_normalize_json(motion_cfg_t *cfg)
{
    cfg->json[MOTION_CFG_JSON_MAX - 1U] = '\0';

    size_t used = strnlen(cfg->json, MOTION_CFG_JSON_MAX);
    if (used < (MOTION_CFG_JSON_MAX - 1U)) {
        memset(&cfg->json[used + 1U],
               0U,
               (MOTION_CFG_JSON_MAX - 1U) - used);
    }
}

// Build a clean default config in RAM
static void motion_cfg_make_defaults(motion_cfg_t *dst)
{
    memset(dst, 0, sizeof(motion_cfg_t));

    dst->magic      = MOTION_MAGIC;
    dst->version    = MOTION_VER;
    dst->seq        = 0;

    dst->json[0]    = '\0';
    if (MOTION_CFG_JSON_MAX > 1U) {
        memset(&dst->json[1],
               0,
               MOTION_CFG_JSON_MAX - 1U);
    }

    motion_cfg_normalize_json(dst);
    dst->crc        = motion_cfg_calc_crc(dst);
}

// Validate magic, version, CRC, and that JSON is terminated
static bool motion_cfg_is_valid(const motion_cfg_t *cfg)
{
    if (cfg->magic   != MOTION_MAGIC) { return false; }
    if (cfg->version != MOTION_VER)   { return false; }

    // json must contain a '\0' somewhere in range
    if (memchr(cfg->json, '\0', MOTION_CFG_JSON_MAX) == NULL) {
        return false;
    }

    uint16_t calc = motion_cfg_calc_crc(cfg);
    if (calc != cfg->crc)           { return false; }

    return true;
}

// Do a complete write of g_cfg into flash page.
// - bumps seq
// - normalizes json
// - recomputes crc
// - erases page and writes words
static HAL_StatusTypeDef motion_cfg_writeback(void)
{
    HAL_StatusTypeDef st;

    // bump monotonic sequence
    g_cfg.seq++;

    motion_cfg_normalize_json(&g_cfg);
    g_cfg.crc = motion_cfg_calc_crc(&g_cfg);

    // Erase page [ADDR_FLASH_PAGE_62 .. ADDR_FLASH_PAGE_63)
    st = Flash_Erase(MOTION_CFG_PAGE_ADDR, MOTION_CFG_PAGE_END);
    if (st != HAL_OK) {
        return st;
    }

    // Program entire struct word-by-word
    st = Flash_Write(MOTION_CFG_PAGE_ADDR,
                     (uint32_t *)&g_cfg,
                     (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));

    return st;
}

// Raw load from flash into g_cfg
static void motion_cfg_load_raw(void)
{
    Flash_Read(MOTION_CFG_PAGE_ADDR,
               (uint32_t *)&g_cfg,
               (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));
}

// Ensure g_cfg is initialized and valid
static void motion_cfg_ensure_loaded(void)
{
    if (g_cfg_loaded) {
        return;
    }

    motion_cfg_load_raw();

    if (!motion_cfg_is_valid(&g_cfg)) {
        // First boot or corrupt -> defaults and persist
        motion_cfg_make_defaults(&g_cfg);
        (void)motion_cfg_writeback();
    }

    g_cfg_loaded = true;
}

// ------------------- Public API -------------------

const motion_cfg_t *motion_cfg_get(void)
{
    motion_cfg_ensure_loaded();
    return &g_cfg;
}

HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();
    memcpy(out, &g_cfg, sizeof(motion_cfg_t));
    return HAL_OK;
}

HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg)
{
    motion_cfg_ensure_loaded();

    // Copy caller-updated fields into our working config.
    // We *trust* their chosen values for hv_settng/hv_enabled/auto_on/json.
    // We IGNORE their seq/crc/magic/version and regenerate those.

    g_cfg.magic      = MOTION_MAGIC;
    g_cfg.version    = MOTION_VER;

    // Copy JSON safely. Caller might not have padded or '\0' at the end.
    memcpy(g_cfg.json, new_cfg->json, MOTION_CFG_JSON_MAX);
    g_cfg.json[MOTION_CFG_JSON_MAX - 1U] = '\0';

    // Now write full page with updated data
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_commit(void)
{
    motion_cfg_ensure_loaded();
    // Write current g_cfg (useful if caller directly edited *motion_cfg_get()).
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_factory_reset(void)
{
    motion_cfg_ensure_loaded();

    motion_cfg_make_defaults(&g_cfg);
    return motion_cfg_writeback();
}
