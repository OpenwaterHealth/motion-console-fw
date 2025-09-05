/*
 * ad5761r.c
 *
 *  Created on: Sep 5, 2025
 *      Author: gvigelet
 */
#include "ad5761r.h"
#include <string.h>

/* ---- Range parameters (m, c) identical to the Linux driver ---------------- */
typedef struct { int32_t m; int32_t c; } range_param_t;
static const range_param_t s_range[8] = {
/* AD5761_VOLTAGE_RANGE_M10V_10V */ { 80, 40 },
/* AD5761_VOLTAGE_RANGE_0V_10V   */ { 40,  0 },
/* AD5761_VOLTAGE_RANGE_M5V_5V   */ { 40, 20 },
/* AD5761_VOLTAGE_RANGE_0V_5V    */ { 20,  0 },
/* AD5761_VOLTAGE_RANGE_M2V5_7V5 */ { 40, 10 },
/* AD5761_VOLTAGE_RANGE_M3V_3V   */ { 24, 12 },
/* AD5761_VOLTAGE_RANGE_0V_16V   */ { 64,  0 },
/* AD5761_VOLTAGE_RANGE_0V_20V   */ { 80,  0 },
};

/* ---- Small helpers -------------------------------------------------------- */
static inline void drv_gpio_write(AD5761R_Gpio g, GPIO_PinState st) {
    if (g.Port) HAL_GPIO_WritePin(g.Port, g.Pin, st);
}
static inline void cs_assert(const AD5761R_Handle *dev)  { drv_gpio_write(dev->CSn,  GPIO_PIN_RESET); }
static inline void cs_deassert(const AD5761R_Handle *dev){ drv_gpio_write(dev->CSn,  GPIO_PIN_SET);   }

static inline uint8_t msb(uint32_t v, int byte_index) {
    return (uint8_t)((v >> (8 * (2 - byte_index))) & 0xFF); /* 24-bit big-endian stream */
}

/* Transmit a single 24-bit frame (3 bytes, MSB first). If CS GPIO provided,
   we control it; else rely on HAL NSS settings. */
static HAL_StatusTypeDef tx24(AD5761R_Handle *dev, uint32_t frame)
{
    uint8_t buf[3] = { msb(frame, 0), msb(frame, 1), msb(frame, 2) };

    if (dev->CSn.Port) cs_assert(dev);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, buf, sizeof(buf), HAL_MAX_DELAY);
    if (dev->CSn.Port) cs_deassert(dev);
    return st;
}

/* Two-frame transaction with /CS separation, receiving 3 bytes on the 2nd frame.
   For readback: send cmd, then send NOOP while reading returned payload. */
static HAL_StatusTypeDef txrx24_twostep(AD5761R_Handle *dev, uint32_t frame_cmd, uint32_t frame_noop, uint16_t *out16)
{
    uint8_t tx1[3] = { msb(frame_cmd, 0), msb(frame_cmd, 1), msb(frame_cmd, 2) };
    uint8_t tx2[3] = { msb(frame_noop, 0), msb(frame_noop, 1), msb(frame_noop, 2) };
    uint8_t rx2[3] = {0};

    if (dev->CSn.Port) {
        cs_assert(dev);
        HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, tx1, sizeof(tx1), HAL_MAX_DELAY);
        cs_deassert(dev);
        if (st != HAL_OK) return st;

        cs_assert(dev);
        st = HAL_SPI_TransmitReceive(dev->hspi, tx2, rx2, sizeof(tx2), HAL_MAX_DELAY);
        cs_deassert(dev);
        if (st != HAL_OK) return st;
    } else {
        /* If using hardware NSS, we cannot enforce an inter-frame CS toggle.
           Most HAL configs will still toggle between calls; verify on logic analyzer. */
        HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, tx1, sizeof(tx1), HAL_MAX_DELAY);
        if (st != HAL_OK) return st;
        st = HAL_SPI_TransmitReceive(dev->hspi, tx2, rx2, sizeof(tx2), HAL_MAX_DELAY);
        if (st != HAL_OK) return st;
    }

    /* Return the lower 16 bits captured (device shifts data into D[15:0]) */
    *out16 = ((uint16_t)rx2[1] << 8) | rx2[2];
    return HAL_OK;
}

/* ---- Public API ----------------------------------------------------------- */

HAL_StatusTypeDef AD5761R_Init(AD5761R_Handle *dev)
{
    if (!dev || !dev->hspi) return HAL_ERROR;
    if (dev->use_internal_ref) {
        /* Internal reference is 2.5V (per family ‘R’ parts); keep external for non-R unless verified */
        dev->vref_mV = 2500;
    } else {
        if (dev->vref_mV < 2000 || dev->vref_mV > 3000) return HAL_ERROR;
    }
    if (dev->n_bits == 0) dev->n_bits = 16; /* default for AD5761(R) */

    /* Make sure optional pins default high (inactive) */
    drv_gpio_write(dev->CSn,    GPIO_PIN_SET);

    /* SW full reset */
    HAL_StatusTypeDef st = tx24(dev, AD5761_ADDR(AD5761_ADDR_SW_FULL_RESET) | 0x0000);
    if (st != HAL_OK) return st;

    /* Program CTRL: range + ETS (+ internal ref if requested) */
    uint16_t ctrl = (uint16_t)((dev->range & 0x7) | AD5761_CTRL_ETS);
    if (dev->use_internal_ref) ctrl |= AD5761_CTRL_USE_INTVREF;

    st = tx24(dev, AD5761_ADDR(AD5761_ADDR_CTRL_WRITE_REG) | ctrl);
    if (st != HAL_OK) return st;

    /* Optional: Software data reset to midscale */
    // st = tx24(dev, AD5761_ADDR(AD5761_ADDR_SW_DATA_RESET) | 0x0000);

    return st;
}

HAL_StatusTypeDef AD5761R_SoftwareFullReset(AD5761R_Handle *dev)
{
    if (!dev) return HAL_ERROR;
    return tx24(dev, AD5761_ADDR(AD5761_ADDR_SW_FULL_RESET) | 0x0000);
}

HAL_StatusTypeDef AD5761R_SoftwareDataReset(AD5761R_Handle *dev)
{
    if (!dev) return HAL_ERROR;
    return tx24(dev, AD5761_ADDR(AD5761_ADDR_SW_DATA_RESET) | 0x0000);
}

HAL_StatusTypeDef AD5761R_SetRange(AD5761R_Handle *dev, AD5761R_Range range)
{
    if (!dev) return HAL_ERROR;

    /* Re-issue CTRL with new range */
    uint16_t ctrl = (uint16_t)((range & 0x7) | AD5761_CTRL_ETS);
    if (dev->use_internal_ref) ctrl |= AD5761_CTRL_USE_INTVREF;

    /* The Linux driver does a SW full reset before setting CTRL; we follow that for safety */
    HAL_StatusTypeDef st = AD5761R_SoftwareFullReset(dev);
    if (st != HAL_OK) return st;

    st = tx24(dev, AD5761_ADDR(AD5761_ADDR_CTRL_WRITE_REG) | ctrl);
    if (st == HAL_OK) dev->range = range;
    return st;
}

HAL_StatusTypeDef AD5761R_WriteRaw(AD5761R_Handle *dev, uint16_t code)
{
    if (!dev) return HAL_ERROR;

    /* If device resolution < 16, left-shift to 16-bit storage */
    uint8_t shift = (uint8_t)(16 - (dev->n_bits > 16 ? 16 : dev->n_bits));
    uint16_t payload = (uint16_t)(code << shift);

    HAL_StatusTypeDef st = tx24(dev, AD5761_ADDR(AD5761_ADDR_DAC_WRITE) | payload);
    if (st == HAL_OK) {
        dev->last_code = payload;
    }
    return st;
}

HAL_StatusTypeDef AD5761R_ReadRaw(AD5761R_Handle *dev, uint16_t *out_code)
{
    if (!dev || !out_code) return HAL_ERROR;

    uint16_t readback = 0;
    HAL_StatusTypeDef st = txrx24_twostep(
        dev,
        AD5761_ADDR(AD5761_ADDR_DAC_READ),           /* 1st frame: request readback */
        AD5761_ADDR(AD5761_ADDR_NOOP),               /* 2nd frame: NOOP clocks data out */
        &readback
    );
    if (st != HAL_OK) return st;

    /* Right-justify for caller according to n_bits */
    uint8_t shift = (uint8_t)(16 - (dev->n_bits > 16 ? 16 : dev->n_bits));
    *out_code = (uint16_t)(readback >> shift);
    return HAL_OK;
}

/* scale = (vref_mV * m / 10) / 2^n_bits  (per Linux driver’s IIO scale reporting) */
void AD5761R_GetScale_mV_per_code(const AD5761R_Handle *dev, int32_t *num_mV, uint8_t *den_pow2)
{
    int32_t m = s_range[dev->range].m;
    int32_t num = (int32_t)dev->vref_mV * m; /* mV * m */
    /* Linux divides by 10 after multiplying by m */
    if (num_mV) *num_mV = num / 10;
    if (den_pow2) *den_pow2 = dev->n_bits;
}

/* offset (LSBs) = -2^n_bits * c / m */
int32_t AD5761R_GetOffset_LSB(const AD5761R_Handle *dev)
{
    int32_t m = s_range[dev->range].m;
    int32_t c = s_range[dev->range].c;
    int32_t two_pow_n = (dev->n_bits >= 31) ? 0x7FFFFFFF : (1 << dev->n_bits); /* safe */
    return -( (two_pow_n * c) / m );
}

/* Vout = ((Code/2^n_bits) * (m * Vref/10)) - (c * Vref/10)
   => Code = clamp( round( (Vout + c*Vref/10) * 2^n / (m*Vref/10) ), 0 .. 2^n-1 ) */
HAL_StatusTypeDef AD5761R_SetVoltage_mV(AD5761R_Handle *dev, int32_t vout_mV)
{
    if (!dev) return HAL_ERROR;

    const range_param_t rp = s_range[dev->range];
    const int32_t Vref10 = (int32_t)dev->vref_mV / 10; /* Vref/10 in mV */
    if (Vref10 <= 0) return HAL_ERROR;

    const int32_t n = dev->n_bits > 16 ? 16 : dev->n_bits;
    const int32_t fullscale = (1 << n);

    /* Compute code in Q0 integer math */
    /* num = (Vout + c*Vref/10) * 2^n */
    int64_t num = (int64_t)vout_mV + (int64_t)rp.c * Vref10;
    num *= (int64_t)fullscale;

    /* den = m*Vref/10 */
    int64_t den = (int64_t)rp.m * Vref10;
    if (den == 0) return HAL_ERROR;

    int64_t code = (num + (den > 0 ? den/2 : 0)) / den; /* round */

    if (code < 0) code = 0;
    if (code > (fullscale - 1)) code = fullscale - 1;

    return AD5761R_WriteRaw(dev, (uint16_t)code);
}

int32_t AD5761R_GetVoltage_mV(AD5761R_Handle *dev)
{
    uint16_t code;
    if (AD5761R_ReadRaw(dev, &code) != HAL_OK) {
        return -1; // error indicator
    }

    const int32_t n = dev->n_bits > 16 ? 16 : dev->n_bits;
    const int32_t fullscale = (1 << n);
    const int32_t Vref10 = dev->vref_mV / 10;

    const int32_t m = s_range[dev->range].m;
    const int32_t c = s_range[dev->range].c;

    // Vout = (code/2^n * m * Vref/10) - (c * Vref/10)
    int64_t num = (int64_t)code * m * Vref10;
    int32_t vout_mV = (int32_t)(num / fullscale) - (c * Vref10);

    return vout_mV;
}

HAL_StatusTypeDef AD5761R_ReadControl(const AD5761R_Handle *dev, uint16_t *out_ctrl)
{
    if (!dev || !out_ctrl) return HAL_ERROR;

    uint16_t rb = 0;
    /* Issue CTRL_READ (first frame), then NOOP (second) to clock data out */
    HAL_StatusTypeDef st = txrx24_twostep(
        (AD5761R_Handle*)dev,
        AD5761_ADDR(AD5761_ADDR_CTRL_READ_REG),
        AD5761_ADDR(AD5761_ADDR_NOOP),
        &rb
    );
    if (st != HAL_OK) return st;

    /* Device returns 16 bits; the CTRL content is in the lower byte per datasheet.
       Keep 16 bits to match how we wrote it (we only use lower bits anyway). */
    *out_ctrl = rb & 0x00FF;
    return HAL_OK;
}

HAL_StatusTypeDef AD5761R_CheckComms(const AD5761R_Handle *dev)
{
    if (!dev) return HAL_ERROR;

    uint16_t ctrl = 0;
    HAL_StatusTypeDef st = AD5761R_ReadControl(dev, &ctrl);
    if (st != HAL_OK) return st;

    /* Expected CTRL we programmed in Init(): range + ETS + (optional) internal VREF */
    uint16_t expected = (uint16_t)((dev->range & 0x7) | AD5761_CTRL_ETS);
    if (dev->use_internal_ref) expected |= AD5761_CTRL_USE_INTVREF;

    /* Only compare the bits we care about (range + ETS + USE_INTVREF) */
    uint16_t mask = (uint16_t)(0x07 | AD5761_CTRL_ETS | AD5761_CTRL_USE_INTVREF);

    return ((ctrl & mask) == (expected & mask)) ? HAL_OK : HAL_ERROR;
}
