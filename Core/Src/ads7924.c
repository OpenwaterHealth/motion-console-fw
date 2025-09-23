#include "ads7924.h"
#include "tca9548a.h"

/* ========= Internal helpers ========= */

static ADS7924_Status ads7924_write_reg(ADS7924_HandleTypeDef *hadc, uint8_t reg, uint8_t val)
{
    if (!hadc || !hadc->hi2c) return ADS7924_ERR_PARAM;
    uint8_t buf[2] = { reg & 0x1F, val }; /* Pointer: INC=0 (MSB=0), P[4:0]=reg */

    TCA9548A_SelectChannel(hadc->index, hadc->channel);

    if (HAL_I2C_Master_Transmit(hadc->hi2c,
                                ADS7924_HAL_Addr8(hadc->i2c_addr_7bit),
                                buf, sizeof(buf),
                                hadc->i2c_timeout_ms) != HAL_OK) {
        return ADS7924_ERR_I2C;
    }
    return ADS7924_OK;
}

static ADS7924_Status ads7924_read_reg(ADS7924_HandleTypeDef *hadc, uint8_t reg, uint8_t *val)
{
    if (!hadc || !hadc->hi2c || !val) return ADS7924_ERR_PARAM;

    TCA9548A_SelectChannel(hadc->index, hadc->channel);

    uint8_t ptr = (reg & 0x1F); /* INC=0 */
    if (HAL_I2C_Master_Transmit(hadc->hi2c,
                                ADS7924_HAL_Addr8(hadc->i2c_addr_7bit),
                                &ptr, 1, hadc->i2c_timeout_ms) != HAL_OK) {
        return ADS7924_ERR_I2C;
    }
    if (HAL_I2C_Master_Receive(hadc->hi2c,
                               ADS7924_HAL_Addr8(hadc->i2c_addr_7bit),
                               val, 1, hadc->i2c_timeout_ms) != HAL_OK) {
        return ADS7924_ERR_I2C;
    }
    return ADS7924_OK;
}

/* Read two consecutive registers with INC=1 (auto-increment) */
static ADS7924_Status ads7924_read_reg2_inc(ADS7924_HandleTypeDef *hadc, uint8_t reg_start, uint8_t *out2)
{
    if (!hadc || !hadc->hi2c || !out2) return ADS7924_ERR_PARAM;

    TCA9548A_SelectChannel(hadc->index, hadc->channel);

    uint8_t ptr = (uint8_t)(0x80U | (reg_start & 0x1F)); /* INC=1 in MSB */
    if (HAL_I2C_Master_Transmit(hadc->hi2c,
                                ADS7924_HAL_Addr8(hadc->i2c_addr_7bit),
                                &ptr, 1, hadc->i2c_timeout_ms) != HAL_OK) {
        return ADS7924_ERR_I2C;
    }
    if (HAL_I2C_Master_Receive(hadc->hi2c,
                               ADS7924_HAL_Addr8(hadc->i2c_addr_7bit),
                               out2, 2, hadc->i2c_timeout_ms) != HAL_OK) {
        return ADS7924_ERR_I2C;
    }
    return ADS7924_OK;
}

/* Poll BUSY bit in INTCONFIG (bit2). Returns ADS7924_OK when not busy. */
static ADS7924_Status ads7924_poll_ready(ADS7924_HandleTypeDef *hadc, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t intcfg = 0;
        ADS7924_Status s = ads7924_read_reg(hadc, ADS7924_REG_INTCONFIG, &intcfg);
        if (s != ADS7924_OK) return s;
        if ((intcfg & ADS7924_INTCONFIG_BUSY_MASK) == 0) {
            return ADS7924_OK; /* Ready */
        }
        /* Small non-blocking wait */
        /* On H7, HAL_Delay(1) is fine; conversions are ~10us min, but allow slack. */
    }
    return ADS7924_ERR_TIMEOUT;
}

/* ========= Public API ========= */

ADS7924_Status ADS7924_Init(ADS7924_HandleTypeDef *hadc,
                            I2C_HandleTypeDef *hi2c,
							uint8_t idx,
							uint8_t chan,
                            uint8_t i2c_addr_7bit,
                            float vref_volts,
                            bool do_reset)
{
    if (!hadc || !hi2c || (i2c_addr_7bit != ADS7924_ADDR_A0_GND && i2c_addr_7bit != ADS7924_ADDR_A0_DVDD))
        return ADS7924_ERR_PARAM;

    hadc->hi2c          = hi2c;
    hadc->index         = idx;
    hadc->channel       = chan;
    hadc->i2c_addr_7bit = i2c_addr_7bit;
    hadc->vref          = (vref_volts > 0.0f) ? vref_volts : 3.3f; /* AVDD used as reference */
    hadc->i2c_timeout_ms = 50U;

    /* Optional soft reset: write any value to RESET (0x16). Common practice is 0x00. */
    if (do_reset) {
        ADS7924_Status s = ads7924_write_reg(hadc, ADS7924_REG_RESET, 0x00);
        if (s != ADS7924_OK) return s;
        HAL_Delay(1); /* allow reset to take effect */
    }

    /* Go to Idle */
    {
        ADS7924_Status s = ads7924_write_reg(hadc, ADS7924_REG_MODECNTRL, ADS7924_MODE_IDLE);
        if (s != ADS7924_OK) return s;
    }

    /* Default timing:
     * ACQCONFIG: ACQTIME[4:0]=0 -> tACQ = 6us (minimum)
     * SLPCONFIG: default 0
     * PWRCONFIG: default 0 (PWRCON disabled, CAL off)
     */
    hadc->acqconfig = 0x00;
    hadc->slpconfig = 0x00;
    hadc->pwrconfig = 0x00;

    ADS7924_Status s1 = ads7924_write_reg(hadc, ADS7924_REG_ACQCONFIG, hadc->acqconfig);
    if (s1 != ADS7924_OK) return s1;
    ADS7924_Status s2 = ads7924_write_reg(hadc, ADS7924_REG_SLPCONFIG, hadc->slpconfig);
    if (s2 != ADS7924_OK) return s2;
    ADS7924_Status s3 = ads7924_write_reg(hadc, ADS7924_REG_PWRCONFIG, hadc->pwrconfig);
    if (s3 != ADS7924_OK) return s3;

    return ADS7924_OK;
}

ADS7924_Status ADS7924_SetAcquisitionSteps(ADS7924_HandleTypeDef *hadc, uint8_t acq_time_steps)
{
    if (!hadc) return ADS7924_ERR_PARAM;
    acq_time_steps &= 0x1F; /* ACQTIME[4:0] */
    hadc->acqconfig = (uint8_t)((hadc->acqconfig & 0xE0U) | acq_time_steps);
    return ads7924_write_reg(hadc, ADS7924_REG_ACQCONFIG, hadc->acqconfig);
}

ADS7924_Status ADS7924_ReadRaw(ADS7924_HandleTypeDef *hadc,
                               ADS7924_Channel ch,
                               uint16_t *code12,
                               uint32_t timeout_ms)
{
    if (!hadc || !code12 || ch > ADS7924_CH3) return ADS7924_ERR_PARAM;

    /* Trigger Manual-Single for selected channel:
     * MODECNTRL = MODE=110000b | SEL/ID[1:0] = ch
     */
    uint8_t mode = (uint8_t)(ADS7924_MODE_MANUAL_SINGLE | (ch & 0x03U));
    ADS7924_Status s = ads7924_write_reg(hadc, ADS7924_REG_MODECNTRL, mode);
    if (s != ADS7924_OK) return s;

    /* Poll BUSY until ready */
    s = ads7924_poll_ready(hadc, timeout_ms);
    if (s != ADS7924_OK) return s;

    /* Read DATAx_U and DATAx_L (two bytes, auto-increment) */
    uint8_t reg_u = (uint8_t)(ADS7924_REG_DATA0_U + (ch * 2U));
    uint8_t raw[2] = {0};
    s = ads7924_read_reg2_inc(hadc, reg_u, raw);
    if (s != ADS7924_OK) return s;

    *code12 = ADS7924_Combine12(raw[0], raw[1]);
    return ADS7924_OK;
}

ADS7924_Status ADS7924_ReadVoltage(ADS7924_HandleTypeDef *hadc,
                                   ADS7924_Channel ch,
                                   float *volts,
                                   uint32_t timeout_ms)
{
    if (!volts) return ADS7924_ERR_PARAM;
    uint16_t code = 0;
    ADS7924_Status s = ADS7924_ReadRaw(hadc, ch, &code, timeout_ms);
    if (s != ADS7924_OK) return s;
    *volts = ADS7924_CodeToVolts(hadc, code);
    return ADS7924_OK;
}

ADS7924_Status ADS7924_ReadAllRaw(ADS7924_HandleTypeDef *hadc, uint16_t out_codes[4], uint32_t timeout_ms)
{
    if (!hadc || !out_codes) return ADS7924_ERR_PARAM;
    for (uint8_t ch = 0; ch < 4; ++ch) {
        ADS7924_Status s = ADS7924_ReadRaw(hadc, (ADS7924_Channel)ch, &out_codes[ch], timeout_ms);
        if (s != ADS7924_OK) return s;
    }
    return ADS7924_OK;
}

ADS7924_Status ADS7924_GotoIdle(ADS7924_HandleTypeDef *hadc)
{
    if (!hadc) return ADS7924_ERR_PARAM;
    return ads7924_write_reg(hadc, ADS7924_REG_MODECNTRL, ADS7924_MODE_IDLE);
}

ADS7924_Status ADS7924_Wake(ADS7924_HandleTypeDef *hadc)
{
    if (!hadc) return ADS7924_ERR_PARAM;
    return ads7924_write_reg(hadc, ADS7924_REG_MODECNTRL, ADS7924_MODE_AWAKE);
}
