/* Core/Inc/demod.h */
#ifndef INC_DEMOD_H_
#define INC_DEMOD_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint32_t demodPulseInterval;  /* every Nth laser cycle is a demod frame; 0 = disabled */
    uint32_t modFrequencyWord;    /* raw DDS frequency tuning word (Seed FPGA 0x0A-0x0D) */
    uint16_t modPhaseWord;        /* raw 12-bit DDS phase word (Seed FPGA 0x00-0x01) */
} Demod_Config_t;

HAL_StatusTypeDef Demod_SetConfigFromJSON(const char *jsonString, size_t str_len);
HAL_StatusTypeDef Demod_GetConfigToJSON(char *jsonString, size_t max_length);

bool Demod_CycleIsDemod(uint32_t cycle);
void Demod_NotifyCycleISR(uint32_t cycle);
void Demod_Tick(void);
void Demod_OnTriggerStart(void);
void Demod_OnTriggerStop(void);

extern Demod_Config_t demod_config;

#endif /* INC_DEMOD_H_ */
