/*
 * trigger.h
 *
 *  Created on: Nov 25, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_TRIGGER_H_
#define INC_TRIGGER_H_

#include "jsmn.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t frequencyHz;        // Trigger frequency in Hz
    uint32_t triggerPulseWidthUsec;     // Pulse width in microseconds
    uint32_t laserPulseDelayUsec;     // Pulse width in microseconds
    uint32_t laserPulseWidthUsec;     // Pulse width in microseconds
} Trigger_Config_t;

HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config);
HAL_StatusTypeDef Trigger_Start() ;
HAL_StatusTypeDef Trigger_Stop();
HAL_StatusTypeDef Trigger_SetConfigFromJSON(char *jsonString, size_t str_len);
HAL_StatusTypeDef Trigger_GetConfigToJSON(char *jsonString);

extern Trigger_Config_t trigger_config;

#endif /* INC_TRIGGER_H_ */

