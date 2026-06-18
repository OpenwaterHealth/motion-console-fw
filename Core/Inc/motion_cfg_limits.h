#ifndef MOTION_CFG_LIMITS_H
#define MOTION_CFG_LIMITS_H

/*
 * Upper bound on the number of jsmn tokens motion_cfg_apply_settings() needs to
 * tokenize the persisted motion-config JSON.
 *
 * A JSON object with N keys requires 1 + 2*N tokens. The console config carries
 * the safety params (TEC_TRIP, OPT/EE thresholds and gains) plus the writable
 * FPGA register keys from fpga_register_map.c -- on the order of ~75 keys today
 * (~151 tokens). 256 leaves comfortable headroom for schema growth.
 *
 * This bound is sized deliberately. The previous fixed value of 32 was silently
 * too small for a real (~1 KB) config: jsmn returned JSMN_ERROR_NOMEM, so the
 * firmware skipped applying settings entirely -- which left TEC_TRIP_VALUE at 0
 * and disarmed the TEC thermal trip without any error. If even this bound is
 * exceeded the parse now fails loudly (printf + host system-error message)
 * instead of silently. host_tests/test_motion_cfg_tokens.c guards the bound.
 */
#define MOTION_CFG_MAX_JSON_TOKENS  256U

#endif /* MOTION_CFG_LIMITS_H */
