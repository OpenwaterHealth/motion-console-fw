#ifndef __ADS7924_H__
#define __ADS7924_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========= Basics ========= */

/* ADS7924 has one address pin A0:
 * A0=0 -> 0b1001000 (0x48)
 * A0=1 -> 0b1001001 (0x49)
 * Pass 7-bit address (0x48 or 0x49); driver will shift for HAL. */
#define ADS7924_ADDR_A0_GND   (0x48U)
#define ADS7924_ADDR_A0_DVDD  (0x49U)

/* Channels */
typedef enum {
    ADS7924_CH0 = 0,
    ADS7924_CH1 = 1,
    ADS7924_CH2 = 2,
    ADS7924_CH3 = 3
} ADS7924_Channel;

/* Error codes */
typedef enum {
    ADS7924_OK          = 0,
    ADS7924_ERR_I2C     = -1,
    ADS7924_ERR_PARAM   = -2,
    ADS7924_ERR_TIMEOUT = -3
} ADS7924_Status;

/* Handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t index;
	uint8_t channel;
	uint8_t i2c_addr_7bit;      /* 0x48 or 0x49 */
    float   vref;               /* AVDD used as reference (Volts). Set to your analog rail. */
    uint32_t i2c_timeout_ms;    /* HAL I2C timeout */
    /* Cached config (optional extensions later) */
    uint8_t acqconfig;          /* ACQCONFIG register value (0x14) */
    uint8_t slpconfig;          /* SLPCONFIG register value (0x13) */
    uint8_t pwrconfig;          /* PWRCONFIG register value (0x15) */
} ADS7924_HandleTypeDef;

/* ========= Public API ========= */

/* Initialize:
 * - takes ownership of hi2c pointer
 * - sets Idle mode, default acquisition time (min), no sleep, no PWR CON
 * - soft-reset optional (set do_reset=true for a clean start)
 */
ADS7924_Status ADS7924_Init(ADS7924_HandleTypeDef *hadc,
                            I2C_HandleTypeDef *hi2c,
							uint8_t idx,
							uint8_t chan,
                            uint8_t i2c_addr_7bit,
                            float vref_volts,
                            bool do_reset);

/* Configure acquisition time:
 * ACQ time = (ACQTIME[4:0]*2us) + 6us
 * Pass acq_time_steps in [0..31] for 0..(62us extra).
 */
ADS7924_Status ADS7924_SetAcquisitionSteps(ADS7924_HandleTypeDef *hadc, uint8_t acq_time_steps);

/* Single-shot conversion on channel:
 * Triggers Manual-Single, polls BUSY bit, then reads DATAx.
 */
ADS7924_Status ADS7924_ReadRaw(ADS7924_HandleTypeDef *hadc,
                               ADS7924_Channel ch,
                               uint16_t *code12,
                               uint32_t timeout_ms);

/* Convert raw code to volts using hadc->vref (AVDD). Code is 0..4095. */
static inline float ADS7924_CodeToVolts(const ADS7924_HandleTypeDef *hadc, uint16_t code12)
{
    return (hadc->vref * (float)code12) / 4096.0f;
}

/* Convenience: read and convert to volts */
ADS7924_Status ADS7924_ReadVoltage(ADS7924_HandleTypeDef *hadc,
                                   ADS7924_Channel ch,
                                   float *volts,
                                   uint32_t timeout_ms);

/* Read all four channels without retriggering (if you’re running a scan mode)
 * or by issuing four manual singles back-to-back. For simplicity, this uses
 * manual singles sequentially.
 */
ADS7924_Status ADS7924_ReadAllRaw(ADS7924_HandleTypeDef *hadc, uint16_t out_codes[4], uint32_t timeout_ms);

/* Optional: put device in Idle (lowest power). */
ADS7924_Status ADS7924_GotoIdle(ADS7924_HandleTypeDef *hadc);

/* Optional: Wake (Awake mode) – keeps clocks on but not converting automatically. */
ADS7924_Status ADS7924_Wake(ADS7924_HandleTypeDef *hadc);

/* ========= Register addresses (from datasheet Table 3) ========= */
#define ADS7924_REG_MODECNTRL   0x00U
#define ADS7924_REG_INTCNTRL    0x01U
#define ADS7924_REG_DATA0_U     0x02U
#define ADS7924_REG_DATA0_L     0x03U
#define ADS7924_REG_DATA1_U     0x04U
#define ADS7924_REG_DATA1_L     0x05U
#define ADS7924_REG_DATA2_U     0x06U
#define ADS7924_REG_DATA2_L     0x07U
#define ADS7924_REG_DATA3_U     0x08U
#define ADS7924_REG_DATA3_L     0x09U
#define ADS7924_REG_ULR0        0x0AU
#define ADS7924_REG_LLR0        0x0BU
#define ADS7924_REG_ULR1        0x0CU
#define ADS7924_REG_LLR1        0x0DU
#define ADS7924_REG_ULR2        0x0EU
#define ADS7924_REG_LLR2        0x0FU
#define ADS7924_REG_ULR3        0x10U
#define ADS7924_REG_LLR3        0x11U
#define ADS7924_REG_INTCONFIG   0x12U
#define ADS7924_REG_SLPCONFIG   0x13U
#define ADS7924_REG_ACQCONFIG   0x14U
#define ADS7924_REG_PWRCONFIG   0x15U
#define ADS7924_REG_RESET       0x16U

/* MODECNTRL bits:
 * [7:2] MODE[5:0], [1:0] = SEL/ID (channel select)
 * Modes (subset):
 * 000000 = Idle
 * 100000 = Awake
 * 110000 = Manual-Single (convert selected channel once)
 * (others: scan/auto modes not used here)
 */
#define ADS7924_MODE_IDLE           0x00U
#define ADS7924_MODE_AWAKE          0x80U
#define ADS7924_MODE_MANUAL_SINGLE  0xC0U

/* INTCONFIG (0x12): we only use BUSY/INT bit (bit 2) to poll busy/status. */
#define ADS7924_INTCONFIG_BUSY_MASK (1U << 2)

/* HAL expects 8-bit address (left-shifted). */
static inline uint16_t ADS7924_HAL_Addr8(uint8_t addr7) { return (uint16_t)(addr7 << 1); }

/* Helper to combine DATAx_U/L into 12-bit code:
 * U: [11:4] in bits 7:0
 * L: [3:0] in bits 7:4 (lower nibble reserved)
 */
static inline uint16_t ADS7924_Combine12(uint8_t upper, uint8_t lower)
{
    return (uint16_t)(((uint16_t)upper << 4) | (lower >> 4));
}

#ifdef __cplusplus
}
#endif

#endif /* __ADS7924_H__ */
