/*
 * trigger.c
 *
 *  Created on: Nov 25, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "trigger.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


// setup default
Trigger_Config_t trigger_config = { 40, 1000, 250, 1000 };

volatile uint32_t fsync_counter = 0;
volatile uint32_t lsync_counter = 0;

uint32_t prev_lsync_arr = 0;
uint32_t prev_lsync_ccr1 = 0;

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

static int jsonToTriggerConfigData(const char *jsonString, Trigger_Config_t* newConfig)
{
    int i, r;
    jsmn_parser parser;
    jsmntok_t t[32]; // Increased size to handle more tokens

    jsmn_init(&parser, NULL);
    r = jsmn_parse(&parser, jsonString, strlen(jsonString), t, sizeof(t) / sizeof(t[0]), NULL);
    if (r < 0) {
        printf("jsonToTriggerConfigData Failed to parse JSON: %d\n", r);
        return 1;
    }

    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("jsonToTriggerConfigData Object expected\n");
        return 1;
    }

    for (i = 1; i < r; i++) {
        if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0) {
            newConfig->frequencyHz = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0) {
            newConfig->triggerPulseWidthUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseDelayUsec") == 0) {
            newConfig->laserPulseDelayUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseWidthUsec") == 0) {
            newConfig->laserPulseWidthUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseSkipInterval") == 0) {
            newConfig->LaserPulseSkipInterval = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "TriggerStatus") == 0) {
            newConfig->TriggerStatus = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "EnableSyncOut") == 0) {
            newConfig->EnableSyncOut = (strncmp(jsonString + t[i + 1].start, "true", 4) == 0);
            i++;
        } else if (jsoneq(jsonString, &t[i], "EnableTaTrigger") == 0) {
            newConfig->EnableTaTrigger = (strncmp(jsonString + t[i + 1].start, "true", 4) == 0);
            i++;
        }
    }

    return 0; // Success
}

static void trigger_GetConfigJSON(char *jsonString, size_t max_length)
{
    memset(jsonString, 0, max_length);
    snprintf(jsonString, max_length,
             "{"
             "\"TriggerFrequencyHz\": %lu,"
             "\"TriggerPulseWidthUsec\": %lu,"
             "\"LaserPulseDelayUsec\": %lu,"
             "\"LaserPulseWidthUsec\": %lu,"
             "\"LaserPulseSkipInterval\": %lu,"
             "\"EnableSyncOut\": %s,"
             "\"EnableTaTrigger\": %s,"
             "\"TriggerStatus\": %lu"
             "}",
             trigger_config.frequencyHz,
             trigger_config.triggerPulseWidthUsec,
             trigger_config.laserPulseDelayUsec,
             trigger_config.laserPulseWidthUsec,
             trigger_config.LaserPulseSkipInterval,
             trigger_config.EnableSyncOut ? "true" : "false",
             trigger_config.EnableTaTrigger ? "true" : "false",
			 trigger_config.TriggerStatus);
}

static void updateTimerDataFromPeripheral()
{
	 // Assuming you have the timer configuration and status
	 uint32_t preScaler = FSYNC_TIMER.Instance->PSC;
	 uint32_t timerClockFrequency = HAL_RCC_GetPCLK1Freq() / (preScaler + 1);
	 uint32_t TIM_ARR = FSYNC_TIMER.Instance->ARR;
	 uint32_t TIM_CCRx = HAL_TIM_ReadCapturedValue(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
	 trigger_config.frequencyHz = timerClockFrequency / (TIM_ARR + 1);
	 trigger_config.triggerPulseWidthUsec = ((TIM_CCRx * 100000) / timerClockFrequency) * 10; // Set the pulse width as needed

	 uint32_t LASER_ARR = LASER_TIMER.Instance->ARR;
	 uint32_t LASER_CCRx = HAL_TIM_ReadCapturedValue(&LASER_TIMER, FSYNC_TIMER_CHAN);
	 trigger_config.laserPulseDelayUsec = LASER_CCRx;
	 trigger_config.laserPulseWidthUsec = LASER_ARR - LASER_CCRx + 1; // Set the pulse width as needed

	 // Check the timer status to determine if it's running
	 trigger_config.TriggerStatus = TIM_CHANNEL_STATE_GET(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
}

HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config) {
    if (config == NULL) {
        return HAL_ERROR; // Null pointer guard
    }

    // Add range checks for the configuration parameters
    if (config->frequencyHz == 0 || config->triggerPulseWidthUsec == 0 || config->frequencyHz > 100) {
        return HAL_ERROR; // Invalid configuration values
    }

	if(trigger_config.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		// stop timer pwm
		HAL_TIM_OC_Stop_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN);

		trigger_config.TriggerStatus = HAL_TIM_CHANNEL_STATE_READY;

		HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, GPIO_PIN_SET); // disable fsync output
		HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, GPIO_PIN_SET); // disable TA Trigger to fpga
	}

    // Use fixed 1 MHz timer tick (1 µs per tick)
    uint32_t fsync_prescaler = 119; // (e.g., 120MHz / (119+1) = 1 MHz)
    FSYNC_TIMER.Instance->PSC = fsync_prescaler;

    // Calculate ARR and CCR1
    uint32_t arr_ticks = (1000000UL / config->frequencyHz); // period in µs
    if (config->triggerPulseWidthUsec >= arr_ticks) {
        return HAL_ERROR; // Pulse width too long
    }

    FSYNC_TIMER.Instance->ARR = arr_ticks - 1;
    FSYNC_TIMER.Instance->CCR1 = config->triggerPulseWidthUsec;

    // Configure LASER timer (same tick frequency)
    LASER_TIMER.Instance->PSC = fsync_prescaler;

    uint32_t laser_delay_ticks = config->laserPulseDelayUsec;
    uint32_t laser_width_ticks = config->laserPulseWidthUsec;
    LASER_TIMER.Instance->ARR = laser_delay_ticks + laser_width_ticks - 1;
    LASER_TIMER.Instance->CCR1 = laser_delay_ticks;

    // Force register update
    FSYNC_TIMER.Instance->EGR |= TIM_EGR_UG;
    LASER_TIMER.Instance->EGR |= TIM_EGR_UG;

    // Update the global trigger configuration
    trigger_config = *config;
    return HAL_OK;
}

HAL_StatusTypeDef Trigger_Start() {

	HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, trigger_config.EnableSyncOut? GPIO_PIN_RESET:GPIO_PIN_SET); // fsync out
	HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, trigger_config.EnableTaTrigger? GPIO_PIN_RESET:GPIO_PIN_SET); // TA Trigger enable

	lsync_counter = 0;
	fsync_counter = 0;

	__HAL_TIM_ENABLE_IT(&LASER_TIMER, TIM_IT_CC1);
	if(HAL_TIM_OC_Start_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
	}

    return HAL_OK;
}

HAL_StatusTypeDef Trigger_Stop() {
	if (HAL_TIM_OC_Stop_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
	}

	HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, GPIO_PIN_SET); // disable fsync output
	HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, GPIO_PIN_SET); // disable TA Trigger to fpga

    return HAL_OK;
}


HAL_StatusTypeDef Trigger_SetConfigFromJSON(char *jsonString, size_t str_len)
{
	uint8_t tempArr[255] = {0};
	bool ret = HAL_OK;

	Trigger_Config_t new_config;
    // Copy the JSON string to tempArr
    memcpy((char *)tempArr, (char *)jsonString, str_len);

	if (jsonToTriggerConfigData((const char *)tempArr, &new_config) == 0)
	{
		Trigger_SetConfig(&new_config);
		ret = HAL_OK;
	}
	else{
		ret = HAL_ERROR;
	}

	return ret;

}

HAL_StatusTypeDef Trigger_GetConfigToJSON(char *jsonString, size_t max_length)
{
	updateTimerDataFromPeripheral();
	trigger_GetConfigJSON(jsonString, 0xFF);
    return HAL_OK;
}

uint32_t get_lsync_pulse_count(void)
{
	return lsync_counter;
}

uint32_t get_fsync_pulse_count(void)
{
	return fsync_counter;
}

void FSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    fsync_counter++;

    if (trigger_config.LaserPulseSkipInterval > 0) {

		if ((fsync_counter % trigger_config.LaserPulseSkipInterval) == 0) {
			/*
            // Disable LASER_TIMER triggering this cycle
			__HAL_TIM_DISABLE(&LASER_TIMER);
			__HAL_TIM_DISABLE_IT(&LASER_TIMER, TIM_IT_UPDATE);
			__HAL_TIM_MOE_DISABLE(&LASER_TIMER);
			__HAL_TIM_DISABLE_DMA(&LASER_TIMER, TIM_DMA_UPDATE);

			// Remove trigger by disabling slave mode
			LASER_TIMER.Instance->SMCR &= ~TIM_SMCR_SMS;
            */

            // Save previous values for laser ARR/CRR1
            prev_lsync_arr = LASER_TIMER.Instance->ARR;
            prev_lsync_ccr1 = LASER_TIMER.Instance->CCR1;

            // Set laser timer to a time beyond the next fsync pulse
            uint32_t pulse_delay_time_us = 10000;
            LASER_TIMER.Instance->ARR = prev_lsync_arr + pulse_delay_time_us;
            LASER_TIMER.Instance->CCR1 = prev_lsync_ccr1 + pulse_delay_time_us;

            // force laser timer update
            LASER_TIMER.Instance->EGR |= TIM_EGR_UG;


		} else {
			/*
            // Re-enable one pulse slave trigger
			__HAL_TIM_DISABLE(&LASER_TIMER);
			__HAL_TIM_MOE_ENABLE(&LASER_TIMER);
			LASER_TIMER.Instance->SMCR &= ~TIM_SMCR_SMS; // Clear first
			LASER_TIMER.Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
		    */

            // Restore previous values for laser ARR/CCR1
            LASER_TIMER.Instance->ARR = prev_lsync_arr;
            LASER_TIMER.Instance->CCR1 = prev_lsync_ccr1;
            // force laser timer update
            LASER_TIMER.Instance->EGR |= TIM_EGR_UG;
        }
    }
#if 0
    if (fsync_counter % 200 == 0) {
    	printf("FSYNC tick: %lu\r\n", fsync_counter);
    }
#endif

}

void LSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    lsync_counter++;

#if 0
    if (lsync_counter % 200 == 0) {
    	printf("LSYNC tick: %lu\r\n", lsync_counter);
    }
#endif

}
