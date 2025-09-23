#ifndef ADS7828_H
#define ADS7828_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define ADS7828_I2C_ADDRESS_BASE 0x48 // Base address (A0, A1 grounded)

// Command byte definitions
#define ADS7828_SD_BIT          (1 << 7)  // Single-ended/Differential bit
#define ADS7828_PD_MASK         (0x03 << 5) // Power-down mode mask
#define ADS7828_CHANNEL_MASK    (0x0F)    // Channel selection mask

// Single-ended input channels
typedef enum {
    ADS7828_CH0 = 0x00,  // Channel 0 single-ended
    ADS7828_CH1 = 0x04,  // Channel 1 single-ended
    ADS7828_CH2 = 0x08,  // Channel 2 single-ended
    ADS7828_CH3 = 0x0C,  // Channel 3 single-ended
    ADS7828_CH4 = 0x10,  // Channel 4 single-ended
    ADS7828_CH5 = 0x14,  // Channel 5 single-ended
    ADS7828_CH6 = 0x18,  // Channel 6 single-ended
    ADS7828_CH7 = 0x1C   // Channel 7 single-ended
} ADS7828_Channel_t;

// Differential input pairs
typedef enum {
    ADS7828_DIFF_CH0_CH1 = 0x00, // CH0+ CH1-
    ADS7828_DIFF_CH2_CH3 = 0x04, // CH2+ CH3-
    ADS7828_DIFF_CH4_CH5 = 0x08, // CH4+ CH5-
    ADS7828_DIFF_CH6_CH7 = 0x0C, // CH6+ CH7-
    ADS7828_DIFF_CH1_CH0 = 0x20, // CH1+ CH0-
    ADS7828_DIFF_CH3_CH2 = 0x24, // CH3+ CH2-
    ADS7828_DIFF_CH5_CH4 = 0x28, // CH5+ CH4-
    ADS7828_DIFF_CH7_CH6 = 0x2C  // CH7+ CH6-
} ADS7828_Differential_t;

// Power-down modes
typedef enum {
    ADS7828_PD_OFF = 0x00,           // Power down between conversions
    ADS7828_PD_ON = 0x01,            // Internal reference OFF, A/D ON
    ADS7828_PD_OFF_REF_ON = 0x02,    // Internal reference ON, A/D OFF
    ADS7828_PD_ON_REF_ON = 0x03      // Internal reference ON, A/D ON
} ADS7828_PowerDown_t;

// ADC configuration structure
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t index;
    uint8_t channel;
    uint8_t i2c_address;
    ADS7828_PowerDown_t power_mode;
    bool differential_mode;
} ADS7828_HandleTypeDef;

// Function prototypes
HAL_StatusTypeDef ADS7828_Init(ADS7828_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c, uint8_t address_bits);
HAL_StatusTypeDef ADS7828_ReadChannel(ADS7828_HandleTypeDef *hadc, uint8_t channel, uint16_t *result);
HAL_StatusTypeDef ADS7828_ReadDifferential(ADS7828_HandleTypeDef *hadc, uint8_t diff_pair, uint16_t *result);
HAL_StatusTypeDef ADS7828_ReadAllChannels(ADS7828_HandleTypeDef *hadc, uint16_t *results);
HAL_StatusTypeDef ADS7828_SetPowerMode(ADS7828_HandleTypeDef *hadc, ADS7828_PowerDown_t mode);
float ADS7828_ConvertToVoltage(uint16_t adc_value, float vref);

#endif // ADS7828_H

