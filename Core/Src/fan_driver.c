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

bool FAN_SetManualPWM(FAN_Driver *dev, uint8_t duty_percent) {
    uint8_t duty_code;

    if (duty_percent > 100) duty_percent = 100;

    // Map percentage to duty code (Table 9 in datasheet)
    if      (duty_percent >= 93) duty_code = 0xE;
    else if (duty_percent >= 87) duty_code = 0xD;
    else if (duty_percent >= 80) duty_code = 0xC;
    else if (duty_percent >= 73) duty_code = 0xB;
    else if (duty_percent >= 67) duty_code = 0xA;
    else if (duty_percent >= 60) duty_code = 0x9;
    else if (duty_percent >= 53) duty_code = 0x8;
    else if (duty_percent >= 47) duty_code = 0x7;
    else if (duty_percent >= 40) duty_code = 0x6;
    else if (duty_percent >= 33) duty_code = 0x5;
    else if (duty_percent >= 27) duty_code = 0x4;
    else if (duty_percent >= 20) duty_code = 0x3;
    else if (duty_percent >= 14) duty_code = 0x2;
    else if (duty_percent >= 7)  duty_code = 0x1;
    else                         duty_code = 0x0;

    // Set mode: PWM control mode, monitoring on
    uint8_t config1 = 0x01; // Bit 0 = monitoring enabled, bits 7:5 = 000 (PWM mode)
    if (!fan_write(dev, MAX6663_REG_CONFIG1, config1)) return false;

    // Write duty cycle
    return fan_write(dev, MAX6663_REG_PWM_CONFIG, duty_code);
}

uint8_t FAN_GetPWMDuty(FAN_Driver *dev) {
    uint8_t reg;
    if (!fan_read(dev, MAX6663_REG_PWM_CONFIG, &reg)) return -1;

    switch (reg & 0x0F) {
        case 0x0: return 0;
        case 0x1: return 7;
        case 0x2: return 14;
        case 0x3: return 20;
        case 0x4: return 27;
        case 0x5: return 33;
        case 0x6: return 40;
        case 0x7: return 47;
        case 0x8: return 53;
        case 0x9: return 60;
        case 0xA: return 67;
        case 0xB: return 73;
        case 0xC: return 80;
        case 0xD: return 87;
        case 0xE: return 93;
        case 0xF: return 100;
        default: return 0;
    }
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
