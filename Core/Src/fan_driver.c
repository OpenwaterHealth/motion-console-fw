/*
 * fan_driver.c
 *
 *  Created on: May 21, 2025
 *      Author: gvigelet
 */

#include "fan_driver.h"

FAN_Driver fan;

static bool fan_write(FAN_Driver *dev, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg, 1, &value, 1, HAL_MAX_DELAY) == HAL_OK;
}

static bool fan_read(FAN_Driver *dev, uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg, 1, value, 1, HAL_MAX_DELAY) == HAL_OK;
}

void FAN_Init(FAN_Driver *dev, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    dev->hi2c = hi2c;
    dev->i2c_addr = addr << 1; // STM HAL uses 8-bit addr
}

bool FAN_EnableMonitoring(FAN_Driver *dev) {
    uint8_t config;
    if (!fan_read(dev, MAX6663_REG_CONFIG1, &config)) return false;

    config = 0x01; // Bit 0 = Monitoring enable
    return fan_write(dev, MAX6663_REG_CONFIG1, config);
}

bool FAN_SetPWMDuty(FAN_Driver *dev, uint8_t duty_code) {
    if (duty_code > 0x0F) return false;

    return fan_write(dev, MAX6663_REG_PWM_CONFIG, duty_code);
}

/* Lookup table: PWM register code (0x0–0xF) → duty-cycle percent (Table 9). */
static const uint8_t pwm_code_to_percent[16] = {
    0, 7, 14, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 100
};

bool FAN_SetManualPWM(FAN_Driver *dev, uint8_t duty_percent) {
    if (duty_percent > 100U) { duty_percent = 100U; }

    /* Find the highest settable code (0x0–0xE) whose threshold ≤ duty_percent. */
    uint8_t duty_code = 0U;
    for (int8_t i = 14; i >= 0; i--) {
        if (duty_percent >= pwm_code_to_percent[i]) {
            duty_code = (uint8_t)i;
            break;
        }
    }

    /* Set mode: PWM control mode, monitoring on
     * Bit 0 = monitoring enabled, bits 7:5 = 000 (PWM mode) */
    uint8_t config1 = 0x01U;
    if (!fan_write(dev, MAX6663_REG_CONFIG1, config1)) { return false; }

    return fan_write(dev, MAX6663_REG_PWM_CONFIG, duty_code);
}

uint8_t FAN_GetPWMDuty(FAN_Driver *dev) {
    uint8_t reg;
    if (!fan_read(dev, MAX6663_REG_PWM_CONFIG, &reg)) { return (uint8_t)-1; }
    return pwm_code_to_percent[reg & 0x0Fu];
}

int8_t FAN_ReadLocalTemp(FAN_Driver *dev) {
    uint8_t val;
    if (!fan_read(dev, MAX6663_REG_TEMP_LOCAL, &val)) return -128;
    return (int8_t)val;
}

int8_t FAN_ReadRemoteTemp(FAN_Driver *dev) {
    uint8_t val;
    if (!fan_read(dev, MAX6663_REG_TEMP_REMOTE, &val)) return -128;
    return (int8_t)val;
}

uint8_t FAN_ReadFanSpeed(FAN_Driver *dev) {
    uint8_t speed;
    if (!fan_read(dev, MAX6663_REG_FAN_SPEED, &speed)) return 0xFF;
    return speed;
}

uint8_t FAN_ReadStatus1(FAN_Driver *dev) {
    uint8_t val;
    fan_read(dev, MAX6663_REG_STATUS1, &val);
    return val;
}

uint8_t FAN_ReadStatus2(FAN_Driver *dev) {
    uint8_t val;
    fan_read(dev, MAX6663_REG_STATUS2, &val);
    return val;
}

bool FAN_ClearFaults(FAN_Driver *dev) {
    uint8_t dummy;
    return fan_read(dev, MAX6663_REG_STATUS1, &dummy) && fan_read(dev, MAX6663_REG_STATUS2, &dummy);
}

uint8_t FAN_ReadDeviceID(FAN_Driver *dev) {
    uint8_t val = 0;
    fan_read(dev, MAX6663_REG_DEVICE_ID, &val);
    return val;
}

uint8_t FAN_ReadManufacturerID(FAN_Driver *dev) {
    uint8_t val = 0;
    fan_read(dev, MAX6663_REG_MAN_ID, &val);
    return val;
}
