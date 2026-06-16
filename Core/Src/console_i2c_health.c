/*
 * console_i2c_health.c
 *
 * Boot-time I2C device health scan for the console controller.
 * See console_i2c_health.h for the device map and wire format.
 */

#include "main.h"
#include "console_i2c_health.h"
#include "tca9548a.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;  /* mux0 + Seed config FPGA */
extern I2C_HandleTypeDef hi2c2;  /* mux1 + PDU/temp/TEC/app-FPGA bus */
extern I2C_HandleTypeDef hi2c4;  /* fan controller (no mux) */

#define MUX_ADDR            0x70u  /* both TCA9548 muxes */
#define SEED_CFG_FPGA_ADDR  0x40u  /* XO2 config block (mux0 ch0) */
#define GPIO_EXP_ADDR       0x20u  /* PCA9535 (mux1 ch0) */
#define PDU_ADC0_ADDR       0x48u  /* ADS7828 (mux1 ch0) */
#define PDU_ADC1_ADDR       0x4Bu  /* ADS7828 (mux1 ch0) */
#define TEC_ADC_ADDR        0x49u  /* ADS7924 (mux1 ch3) */
#define FAN_ADDR            0x2Cu  /* MAX6663 (I2C4) */
#define TEMP1_ADDR          0x49u  /* MAX31875 (mux1 ch1) */
#define TEMP2_ADDR          0x4Au
#define TEMP3_ADDR          0x4Bu
#define APP_FPGA_ADDR       0x41u  /* TA/Seed/EE/OPT register block (mux1 ch4..7) */

/* Short bounded ping so a missing/wedged device can't stall the boot scan. */
#define HEALTH_PING_TRIALS    2u
#define HEALTH_PING_TIMEOUT   20u

static console_i2c_health_t s_health;  /* zeroed at startup */

static bool ping(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
	return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr7 << 1),
	                             HEALTH_PING_TRIALS, HEALTH_PING_TIMEOUT) == HAL_OK;
}

/* Select (mux_index, channel); return true on success. */
static bool mux_select(uint8_t mux_index, uint8_t channel)
{
	return TCA9548A_SelectChannel(mux_index, channel) == TCA9548A_OK;
}

void console_i2c_health_scan(void)
{
	console_i2c_health_t h;
	memset(&h, 0, sizeof(h));
	h.version = CONSOLE_I2C_HEALTH_VERSION;

	printf("Console I2C health scan...\r\n");

	/* --- Muxes themselves (no channel selected) --- */
	TCA9548A_DisableAll(0);
	TCA9548A_DisableAll(1);
	h.mux0 = ping(&hi2c1, MUX_ADDR) ? 1u : 0u;
	h.mux1 = ping(&hi2c2, MUX_ADDR) ? 1u : 0u;

	/* --- mux0 ch0: Seed config FPGA (XO2) --- */
	if (mux_select(0, 0)) {
		h.seed_cfg_fpga = ping(&hi2c1, SEED_CFG_FPGA_ADDR) ? 1u : 0u;
	}

	/* --- mux1 ch0: GPIO expander + two PDU monitor ADCs --- */
	if (mux_select(1, 0)) {
		h.gpio_exp = ping(&hi2c2, GPIO_EXP_ADDR) ? 1u : 0u;
		h.pdu_adc0 = ping(&hi2c2, PDU_ADC0_ADDR) ? 1u : 0u;
		h.pdu_adc1 = ping(&hi2c2, PDU_ADC1_ADDR) ? 1u : 0u;
	}

	/* --- mux1 ch1: three MAX31875 temperature sensors --- */
	if (mux_select(1, 1)) {
		if (ping(&hi2c2, TEMP1_ADDR)) h.temp_present |= I2C_HEALTH_TEMP1_BIT;
		if (ping(&hi2c2, TEMP2_ADDR)) h.temp_present |= I2C_HEALTH_TEMP2_BIT;
		if (ping(&hi2c2, TEMP3_ADDR)) h.temp_present |= I2C_HEALTH_TEMP3_BIT;
	}

	/* --- mux1 ch3: TEC ADC (ADS7924) --- */
	if (mux_select(1, 3)) {
		h.tec_adc = ping(&hi2c2, TEC_ADC_ADDR) ? 1u : 0u;
	}

	/* --- mux1 ch4..7: TA / Seed / EE / OPT application FPGAs at 0x41 --- */
	for (uint8_t ch = 4; ch <= 7; ch++) {
		if (mux_select(1, ch) && ping(&hi2c2, APP_FPGA_ADDR)) {
			h.fpga_present |= (uint8_t)(1u << ch);
		}
	}

	/* --- I2C4: fan controller (no mux) --- */
	h.fan = ping(&hi2c4, FAN_ADDR) ? 1u : 0u;

	/* Leave both muxes in the no-channel state. */
	TCA9548A_DisableAll(0);
	TCA9548A_DisableAll(1);

	h.all_present = (h.mux0 && h.mux1 && h.seed_cfg_fpga && h.gpio_exp &&
	                 h.pdu_adc0 && h.pdu_adc1 && h.tec_adc && h.fan &&
	                 h.temp_present == I2C_HEALTH_TEMP_ALL &&
	                 h.fpga_present == I2C_HEALTH_FPGA_ALL) ? 1u : 0u;

	s_health = h;

	printf("Console I2C health: mux0=%u mux1=%u seedcfg=%u gpio=%u "
	       "pdu=%u/%u tec=%u fan=%u temp=0x%02X fpga=0x%02X all=%u\r\n",
	       h.mux0, h.mux1, h.seed_cfg_fpga, h.gpio_exp, h.pdu_adc0, h.pdu_adc1,
	       h.tec_adc, h.fan, h.temp_present, h.fpga_present, h.all_present);
}

const console_i2c_health_t *console_i2c_health_get(void)
{
	return &s_health;
}
