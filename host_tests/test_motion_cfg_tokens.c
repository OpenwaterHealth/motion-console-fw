#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jsmn.h"
#include "motion_cfg_limits.h"

/* Pull jsmn in directly for the host build (no STM32 HAL dependencies). */
#include "../Core/Src/jsmn.c"

/*
 * Regression test for the TEC-trip "won't trip" bug.
 *
 * motion_cfg_apply_settings() (Core/Src/main.c) tokenizes the persisted config
 * JSON and, for the "TEC_TRIP" key, converts the setpoint into TEC_TRIP_VALUE --
 * the threshold the telemetry loop compares against. That routine used a fixed
 * jsmntok_t[32]. A real config is ~1 KB with dozens of keys, so jsmn ran out of
 * tokens and returned JSMN_ERROR_NOMEM; the routine then bailed before applying
 * anything, leaving TEC_TRIP_VALUE == 0 and the thermal trip silently disarmed.
 *
 * This test pins three things:
 *   1. a representative full config needs MORE than the old 32 tokens (the bug),
 *   2. it fits within MOTION_CFG_MAX_JSON_TOKENS (the fix), and
 *   3. it parses cleanly and the TEC_TRIP value is extractable.
 */

/* Representative of what the test app writes to the console (compare the
 * 1013-byte blob seen in field logs): safety params + writable FPGA keys. */
static const char REAL_CONFIG[] =
    "{"
    "\"TEC_TRIP\": 40, \"OPT_THRESH\": 7143, \"EE_THRESH\": 5000, "
    "\"EE_GAIN\": 1.86, \"OPT_GAIN\": 1.86, "
    "\"TA_PULSE_WIDTH\": 500, \"TA_PERIOD\": 1000, \"TA_CURRENT_DRV\": 4050, "
    "\"TA_CURRENT_LIMIT\": 4095, \"TA_PWM_MON_CL\": 100, \"TA_CW_MON_CL\": 100, "
    "\"SEED_DDS_GAIN\": 100, \"SEED_CW_GAIN\": 140, \"SEED_DDS_CL\": 200, "
    "\"SEED_CW_CL\": 200, \"SEED_ADC_DDS_CL\": 100, \"SEED_ADC_CW_CL\": 100, "
    "\"EE_PULSE_WIDTH_LL\": 0, \"EE_PULSE_WIDTH_UL\": 525, \"EE_RATE_LL\": 23125, "
    "\"EE_RATE_UL\": 30000, \"EE_DRIVE_CL\": 4016, \"EE_PWM_CURRENT\": 100, "
    "\"EE_CW_CURRENT\": 100, \"EE_PWM_MONITOR_CL\": 50, \"EE_CW_MONITOR_CL\": 50, "
    "\"OPT_PULSE_WIDTH_LL\": 0, \"OPT_PULSE_WIDTH_UL\": 525, \"OPT_RATE_LL\": 23125, "
    "\"OPT_RATE_UL\": 30000, \"OPT_DRIVE_CL\": 1028, \"OPT_PWM_CURRENT\": 100, "
    "\"OPT_CW_CURRENT\": 100, \"OPT_PWM_MONITOR_CL\": 50, \"OPT_CW_MONITOR_CL\": 50"
    "}";

/* jsmn counting pass: tokens == NULL returns the number of tokens required. */
static int tokens_required(const char *json) {
    jsmn_parser p;
    jsmn_init(&p, NULL);
    return jsmn_parse(&p, json, strlen(json), NULL, 0, NULL);
}

int main(void) {
    int needed = tokens_required(REAL_CONFIG);
    printf("config needs %d jsmn tokens\n", needed);
    assert(needed > 0);

    /* (1) The historical fixed buffer of 32 was too small -- this is exactly the
     * condition that silently disarmed the TEC trip. Prove it still would. */
    assert(needed > 32);
    jsmntok_t small[32];
    jsmn_parser p32;
    jsmn_init(&p32, NULL);
    int r32 = jsmn_parse(&p32, REAL_CONFIG, strlen(REAL_CONFIG), small, 32, NULL);
    assert(r32 == JSMN_ERROR_NOMEM);

    /* (2) The production bound must hold a full config. */
    assert(needed <= (int)MOTION_CFG_MAX_JSON_TOKENS);

    /* (3) Parsing into a production-sized buffer succeeds with a root object. */
    jsmntok_t toks[MOTION_CFG_MAX_JSON_TOKENS];
    jsmn_parser p;
    jsmn_init(&p, NULL);
    int r = jsmn_parse(&p, REAL_CONFIG, strlen(REAL_CONFIG), toks,
                       MOTION_CFG_MAX_JSON_TOKENS, NULL);
    assert(r == needed);
    assert(toks[0].type == JSMN_OBJECT);

    /* ...and TEC_TRIP is extractable, mirroring apply_tec_trip_setting(). */
    int found_trip = 0;
    double trip = 0.0;
    for (int i = 1; i < r; i++) {
        if (toks[i].type != JSMN_STRING) {
            continue;
        }
        int klen = toks[i].end - toks[i].start;
        if (klen == (int)strlen("TEC_TRIP") &&
            strncmp(REAL_CONFIG + toks[i].start, "TEC_TRIP", klen) == 0) {
            int v = i + 1;
            assert(v < r && toks[v].type == JSMN_PRIMITIVE);
            char buf[32];
            int vlen = toks[v].end - toks[v].start;
            assert(vlen < (int)sizeof(buf));
            memcpy(buf, REAL_CONFIG + toks[v].start, vlen);
            buf[vlen] = '\0';
            trip = strtod(buf, NULL);
            found_trip = 1;
            break;
        }
    }
    assert(found_trip);
    assert(trip == 40.0);

    printf("motion_cfg token sizing host tests OK\n");
    return 0;
}
