/*
 * console_i2c_health.h
 *
 * Boot-time I2C device health scan for the console controller.  Passively
 * pings every device the console expects across its two TCA9548 muxes (on
 * I2C1 and I2C2) plus the fan controller on I2C4, and caches a snapshot the
 * host can query over OW_CTRL_I2C_STATUS.
 *
 * Unlike the sensor module's scan, this is strictly passive: it only selects
 * mux channels and address-pings.  It never powers, resets, or sends
 * activation/config sequences to the laser-driver or safety FPGAs.
 */

#ifndef INC_CONSOLE_I2C_HEALTH_H_
#define INC_CONSOLE_I2C_HEALTH_H_

#include <stdint.h>

#define CONSOLE_I2C_HEALTH_VERSION  1u

/* temp_present bit positions (MAX31875 sensors on mux1 channel 1). */
#define I2C_HEALTH_TEMP1_BIT  (1u << 0)  /* 0x49 */
#define I2C_HEALTH_TEMP2_BIT  (1u << 1)  /* 0x4A */
#define I2C_HEALTH_TEMP3_BIT  (1u << 2)  /* 0x4B */
#define I2C_HEALTH_TEMP_ALL   (I2C_HEALTH_TEMP1_BIT | I2C_HEALTH_TEMP2_BIT | I2C_HEALTH_TEMP3_BIT)

/* fpga_present bit positions chosen to match the mux1 channel numbers. */
#define I2C_HEALTH_FPGA_TA_BIT    (1u << 4)  /* mux1 ch4, 0x41 */
#define I2C_HEALTH_FPGA_SEED_BIT  (1u << 5)  /* mux1 ch5, 0x41 */
#define I2C_HEALTH_FPGA_EE_BIT    (1u << 6)  /* mux1 ch6, 0x41 */
#define I2C_HEALTH_FPGA_OPT_BIT   (1u << 7)  /* mux1 ch7, 0x41 */
#define I2C_HEALTH_FPGA_ALL       (I2C_HEALTH_FPGA_TA_BIT | I2C_HEALTH_FPGA_SEED_BIT | \
                                   I2C_HEALTH_FPGA_EE_BIT | I2C_HEALTH_FPGA_OPT_BIT)

/* Fixed 16-byte wire layout returned verbatim by OW_CTRL_I2C_STATUS.  Do not
 * reorder fields without bumping CONSOLE_I2C_HEALTH_VERSION and the SDK parser. */
typedef struct __attribute__((packed)) {
	uint8_t version;        /* = CONSOLE_I2C_HEALTH_VERSION */
	uint8_t mux0;           /* TCA9548 #0 @0x70 on I2C1 */
	uint8_t mux1;           /* TCA9548 #1 @0x70 on I2C2 */
	uint8_t seed_cfg_fpga;  /* XO2 config block @0x40, mux0 ch0 */
	uint8_t gpio_exp;       /* PCA9535 @0x20, mux1 ch0 */
	uint8_t pdu_adc0;       /* ADS7828 @0x48, mux1 ch0 */
	uint8_t pdu_adc1;       /* ADS7828 @0x4B, mux1 ch0 */
	uint8_t tec_adc;        /* ADS7924 @0x49, mux1 ch3 */
	uint8_t fan;            /* MAX6663 @0x2C, I2C4 (no mux) */
	uint8_t temp_present;   /* I2C_HEALTH_TEMP*_BIT, MAX31875 on mux1 ch1 */
	uint8_t fpga_present;   /* I2C_HEALTH_FPGA_*_BIT, app FPGAs @0x41 on mux1 ch4..7 */
	uint8_t all_present;    /* 1 iff every expected device responded */
	uint8_t reserved[4];
} console_i2c_health_t;

/* Run the passive scan and populate the cached snapshot.  Blocking (~1 s);
 * intended to run once at boot, but safe to re-run on demand (serialized with
 * telemetry in the main loop). */
void console_i2c_health_scan(void);

/* Pointer to the cached snapshot (zeroed until the first scan). */
const console_i2c_health_t *console_i2c_health_get(void);

#endif /* INC_CONSOLE_I2C_HEALTH_H_ */
