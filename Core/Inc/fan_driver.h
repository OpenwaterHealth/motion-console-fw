/*
 * fan_driver.h
 *
 *  Created on: May 21, 2025
 *      Author: gvigelet
 */

#ifndef INC_FAN_DRIVER_H_
#define INC_FAN_DRIVER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


// I2C Address (default ADDR = GND)
#define MAX6663_I2C_ADDR (0x2C << 1)  // 7-bit address shifted left for STM32 HAL
#define MAX6663_DEVICE_ID			  0x38
#define MAX6663_MAN_ID			  	  0x4D

// MAX6663 Register Addresses
#define MAX6663_REG_CONFIG1       0x00
#define MAX6663_REG_CONFIG2       0x01
#define MAX6663_REG_STATUS1       0x02
#define MAX6663_REG_STATUS2       0x03
#define MAX6663_REG_FAN_SPEED     0x08
#define MAX6663_REG_TEMP_LOCAL    0x0A
#define MAX6663_REG_TEMP_REMOTE   0x0B
#define MAX6663_REG_FAN_LIMIT     0x10
#define MAX6663_REG_PWM_CONFIG    0x22
#define MAX6663_REG_DEVICE_ID	  0x3D
#define MAX6663_REG_MAN_ID    	  0x3E

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
} FAN_Driver;


void FAN_Init(FAN_Driver *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

bool FAN_EnableMonitoring(FAN_Driver *dev);
bool FAN_SetPWMDuty(FAN_Driver *dev, uint8_t duty_code);  // 0x0 to 0xF
bool FAN_SetManualPWM(FAN_Driver *dev, uint8_t duty_percent);
uint8_t FAN_GetPWMDuty(FAN_Driver *dev);

int8_t FAN_ReadLocalTemp(FAN_Driver *dev);
int8_t FAN_ReadRemoteTemp(FAN_Driver *dev);
uint8_t FAN_ReadFanSpeed(FAN_Driver *dev);

uint8_t FAN_ReadStatus1(FAN_Driver *dev);
uint8_t FAN_ReadStatus2(FAN_Driver *dev);
bool FAN_ClearFaults(FAN_Driver *dev);

uint8_t FAN_ReadDeviceID(FAN_Driver *dev);
uint8_t FAN_ReadManufacturerID(FAN_Driver *dev);

#endif /* INC_FAN_DRIVER_H_ */
