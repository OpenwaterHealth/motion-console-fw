/* Core/Src/demod.c
 *
 * Demod-frame interleaving.
 *
 * The Seed FPGA drives a DDS modulation device. Per the Unified Board FPGA
 * Memory Map (700-00010) and the bring-up sequence from H. Tang (Sep 2025):
 *   1. Write the modulation phase word (regs 0x00-0x01) and the modulation
 *      frequency word (regs 0x0A-0x0D).
 *   2. Strobe DYNAMIC CONTROL D[0] (reg 0x22) - the FPGA loads frequency and
 *      phase into the modulation device.
 *   3. STATIC CONTROL D[0] (reg 0x20) turns modulation on/off.
 * For interleaved demod frames the MCU performs step 3 cycle-by-cycle: after
 * laser pulse N-1 ends, modulation is switched on so pulse N is a demod
 * pulse, then switched off again once pulse N has fired.
 *
 * The frequency/phase words are raw register values; converting Hz/radians
 * to register words is the host's job (the formulas live in the DDS data
 * sheet and depend on the DDS reference clock).
 *
 * ISR/main-loop split mirrors pdc_poll.c: the LSYNC rising-edge ISR only
 * records the cycle number and a timestamp; all I2C traffic happens in
 * Demod_Tick() from the main loop.
 */
#include "main.h"
#include "demod.h"
#include "trigger.h"
#include "tca9548a.h"
#include "jsmn.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define SEED_MUX_INDEX        1
#define SEED_CHANNEL          5
#define SEED_I2C_ADDR         0x41
#define SEED_REG_MOD_PHASE    0x00 /* 2 bytes, PHASEREG[11:0] */
#define SEED_REG_MOD_FREQ     0x0A /* 4 bytes, FREQ[31:0] */
#define SEED_REG_STATIC_CTRL  0x20 /* 2 bytes, D[0] = modulate ON */
#define SEED_REG_DYNAMIC_CTRL 0x22 /* 2 bytes, D[0] = modulation configure */

Demod_Config_t demod_config = { 0, 0, 0 };

/* ISR -> main-loop handoff: latest laser cycle and its rising-edge tick.
 * Last-value-wins; Demod_Tick recomputes the desired modulation state from
 * the newest cycle, so a missed event self-corrects on the next frame. */
static volatile uint32_t s_cycle = 0;
static volatile uint32_t s_cycle_tick = 0;
static volatile bool     s_cycle_pending = false;

static bool s_mod_is_on = false;          /* last state successfully written */
static volatile bool s_force_off = false; /* one-shot OFF request, serviced in Demod_Tick */
static uint32_t s_fail_count = 0;

/* Cap forced-OFF retries so an unreachable Seed FPGA (e.g. laser board
 * unpowered after a stop) doesn't hammer the I2C bus every main-loop tick.
 * An unreachable device isn't modulating anyway, and the next trigger start
 * re-arms a fresh forced OFF. */
#define DEMOD_FORCE_OFF_MAX_ATTEMPTS 8u
static uint32_t s_force_off_attempts = 0;

static int8_t seed_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    return TCA9548A_Write_Data(SEED_MUX_INDEX, SEED_CHANNEL, SEED_I2C_ADDR,
                               reg, len, (uint8_t *)data);
}

static int8_t write_modulation_state(bool on)
{
    const uint8_t buf[2] = { on ? 0x01u : 0x00u, 0x00u };
    return seed_write(SEED_REG_STATIC_CTRL, buf, 2);
}

/* Write the DDS registers the host provided, then strobe the configure bit
 * so the FPGA loads them into the modulation device. */
static HAL_StatusTypeDef apply_dds_config(bool write_freq, bool write_phase)
{
    bool wrote = false;

    if (write_phase) {
        const uint8_t buf[2] = {
            (uint8_t)(demod_config.modPhaseWord & 0xFFu),
            (uint8_t)((demod_config.modPhaseWord >> 8) & 0x0Fu),
        };
        if (seed_write(SEED_REG_MOD_PHASE, buf, 2) != TCA9548A_OK) {
            printf("demod: phase write failed\r\n");
            return HAL_ERROR;
        }
        wrote = true;
    }

    if (write_freq) {
        const uint32_t f = demod_config.modFrequencyWord;
        /* Seed FPGA rev 1.1.0 I2C write-path bug (registers.v:183): a write
         * to reg 0x0C lands in modulate_frequency_temp[23:0], zeroing bits
         * [23:8]. Write one byte at a time with 0x0C FIRST so the later
         * bytes repair the damage; on affected images bits [23:16] are
         * stuck at zero, so use frequency words with [23:16] == 0. Fixed
         * images are order-insensitive. */
        static const uint8_t freq_byte_order[4] = { 2u, 0u, 1u, 3u };
        for (unsigned k = 0; k < 4u; k++) {
            const uint8_t off = freq_byte_order[k];
            const uint8_t b = (uint8_t)((f >> (8u * off)) & 0xFFu);
            if (seed_write((uint8_t)(SEED_REG_MOD_FREQ + off), &b, 1) != TCA9548A_OK) {
                printf("demod: frequency write failed\r\n");
                return HAL_ERROR;
            }
        }
        wrote = true;
    }

    if (wrote) {
        const uint8_t buf[2] = { 0x01u, 0x00u };
        if (seed_write(SEED_REG_DYNAMIC_CTRL, buf, 2) != TCA9548A_OK) {
            printf("demod: configure strobe failed\r\n");
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

static int jsoneq(const char *json, const jsmntok_t *tok, const char *s)
{
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

static int jsonToDemodConfigData(const char *jsonString, Demod_Config_t *newConfig,
                                 bool *freq_present, bool *phase_present)
{
    int i, r;
    jsmn_parser parser;
    jsmntok_t t[16];

    jsmn_init(&parser, NULL);
    r = jsmn_parse(&parser, jsonString, strlen(jsonString), t, sizeof(t) / sizeof(t[0]), NULL);
    if (r < 0) {
        printf("jsonToDemodConfigData Failed to parse JSON: %d\n", r);
        return 1;
    }

    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("jsonToDemodConfigData Object expected\n");
        return 1;
    }

    for (i = 1; i < r; i++) {
        if (jsoneq(jsonString, &t[i], "DemodPulseInterval") == 0) {
            newConfig->demodPulseInterval = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "ModulationFrequencyWord") == 0) {
            newConfig->modFrequencyWord = strtoul(jsonString + t[i + 1].start, NULL, 10);
            *freq_present = true;
            i++;
        } else if (jsoneq(jsonString, &t[i], "ModulationPhaseWord") == 0) {
            newConfig->modPhaseWord = (uint16_t)(strtoul(jsonString + t[i + 1].start, NULL, 10) & 0x0FFFu);
            *phase_present = true;
            i++;
        }
    }
    return 0;
}

HAL_StatusTypeDef Demod_SetConfigFromJSON(const char *jsonString, size_t str_len)
{
    char tempArr[255] = {0};

    if (str_len >= sizeof(tempArr)) {
        return HAL_ERROR;
    }
    memcpy(tempArr, jsonString, str_len);

    /* Seed from current config so any field absent from JSON keeps its value */
    Demod_Config_t new_config = demod_config;
    bool freq_present = false;
    bool phase_present = false;

    if (jsonToDemodConfigData(tempArr, &new_config, &freq_present, &phase_present) != 0) {
        return HAL_ERROR;
    }

    demod_config = new_config;
    printf("Demod_Config_t:\r\n");
    printf("  demodPulseInterval: %lu\r\n", demod_config.demodPulseInterval);
    printf("  modFrequencyWord:   %lu\r\n", demod_config.modFrequencyWord);
    printf("  modPhaseWord:       %u\r\n", demod_config.modPhaseWord);

    if (demod_config.demodPulseInterval == 0 && s_mod_is_on) {
        s_force_off = true;
    }

    return apply_dds_config(freq_present, phase_present);
}

HAL_StatusTypeDef Demod_GetConfigToJSON(char *jsonString, size_t max_length)
{
    memset(jsonString, 0, max_length);
    snprintf(jsonString, max_length,
             "{"
             "\"DemodPulseInterval\": %lu,"
             "\"ModulationFrequencyWord\": %lu,"
             "\"ModulationPhaseWord\": %u"
             "}",
             demod_config.demodPulseInterval,
             demod_config.modFrequencyWord,
             demod_config.modPhaseWord);
    return HAL_OK;
}

/* A demod frame must be a laser-on frame: dark frames win on collision, and
 * the NUM_DARK_FRAMES_AT_START startup darks are never demod frames. */
bool Demod_CycleIsDemod(uint32_t cycle)
{
    return demod_config.demodPulseInterval > 0
        && cycle >= NUM_DARK_FRAMES_AT_START
        && (cycle % demod_config.demodPulseInterval) == 0
        && !Trigger_CycleIsDark(cycle);
}

void Demod_NotifyCycleISR(uint32_t cycle)
{
    s_cycle = cycle;
    s_cycle_tick = HAL_GetTick();
    s_cycle_pending = true;
}

void Demod_Tick(void)
{
    if (s_force_off) {
        if (write_modulation_state(false) == TCA9548A_OK) {
            s_mod_is_on = false;
            s_force_off = false;
            s_force_off_attempts = 0;
        } else {
            s_fail_count++;
            s_force_off_attempts++;
            if (s_force_off_attempts >= DEMOD_FORCE_OFF_MAX_ATTEMPTS) {
                s_force_off = false;
                s_force_off_attempts = 0;
                printf("demod: forced OFF abandoned after %u attempts\r\n",
                       (unsigned int)DEMOD_FORCE_OFF_MAX_ATTEMPTS);
            }
        }
        return;
    }

    if (!s_cycle_pending) {
        return;
    }

    /* Wait until the current cycle's laser pulse (delay + optional skip delay
     * + width) has completed before touching modulation state. */
    uint32_t settle_ms = (trigger_config.laserPulseDelayUsec
                        + trigger_config.LaserPulseSkipDelayUsec
                        + trigger_config.laserPulseWidthUsec) / 1000u + 2u;
    uint32_t elapsed = HAL_GetTick() - s_cycle_tick;
    if (elapsed < settle_ms) {
        return;
    }

    uint32_t cycle = s_cycle;
    s_cycle_pending = false;

    /* If the main loop stalled past this cycle's period the next pulse may
     * already be underway - don't flip modulation mid-pulse; the next cycle's
     * event recomputes the correct state. */
    if (trigger_config.frequencyHz >= 1.0f) {
        uint32_t period_ms = (uint32_t)(1000.0f / trigger_config.frequencyHz);
        if (elapsed >= period_ms) {
            return;
        }
    }

    bool desired = Demod_CycleIsDemod(cycle + 1);
    if (desired == s_mod_is_on) {
        return;
    }

    if (write_modulation_state(desired) == TCA9548A_OK) {
        s_mod_is_on = desired;
    } else {
        s_fail_count++;
        if ((s_fail_count & 0x3Fu) == 1) {
            printf("demod: modulation %s write failed (count=%lu)\r\n",
                   desired ? "ON" : "OFF", (unsigned long)s_fail_count);
        }
    }
}

void Demod_OnTriggerStart(void)
{
    s_cycle_pending = false;
    /* Hardware state is unknown at scan start - force a clean OFF before the
     * startup dark frames complete. Skipped entirely when the feature is
     * disabled so unconfigured systems see no Seed FPGA traffic. */
    s_mod_is_on = false;
    if (demod_config.demodPulseInterval > 0) {
        s_force_off = true;
        s_force_off_attempts = 0;
    }
}

void Demod_OnTriggerStop(void)
{
    s_cycle_pending = false;
    if (demod_config.demodPulseInterval > 0 || s_mod_is_on) {
        s_force_off = true;
    }
}
