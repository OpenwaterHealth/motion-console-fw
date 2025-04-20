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
Trigger_Config_t trigger_config = { 40, 1000, 100, 100 };


static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

static int jsonToTriggerConfigData(const char *jsonString, Trigger_Config_t* newConfig)
{
	int i;
	int r;
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];

	// init parser
	jsmn_init(&parser, NULL);
	r = jsmn_parse(&parser, jsonString, strlen(jsonString), t,
				 sizeof(t) / sizeof(t[0]), NULL);
	if (r < 0) {
		printf("jsonToTriggerConfigData Failed to parse JSON: %d\n", r);
		return 1;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		printf("jsonToTriggerConfigData Object expected\n");
		return 1;
	}


	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
	    if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0) {
			/* We may use strndup() to fetch string value */
	    	newConfig->frequencyHz = strtol(jsonString + t[i + 1].start, NULL, 10);
			//printf("- frequencyHz: %.*s\r\n", t[i + 1].end - t[i + 1].start,
			//		jsonString + t[i + 1].start);
			i++;
	    } else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0) {
	    	/* We may additionally check if the value is either "true" or "false" */
	    	newConfig->triggerPulseWidthUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
	    	i++;
		} else if (jsoneq(jsonString, &t[i], "LaserPulseDelayUsec") == 0) {
			/* We may want to do strtol() here to get numeric value */
			newConfig->laserPulseDelayUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		} else if (jsoneq(jsonString, &t[i], "LaserPulseWidthUsec") == 0) {
			/* We may want to do strtol() here to get numeric value */
			newConfig->laserPulseWidthUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}

	}

    return 0; // Successful parsing
}


static void trigger_GetConfigJSON(char *jsonString, size_t max_length)
{
    memset(jsonString, 0, max_length);
    snprintf(jsonString, max_length,
             "{"
             "\"TriggerFrequencyHz\": %lu,"
             "\"TriggerPulseWidthUsec\": %lu,"
             "\"LaserPulseDelayUsec\": %lu,"
             "\"LaserPulseWidthUsec\": %lu"
             "}",
             trigger_config.frequencyHz,
             trigger_config.triggerPulseWidthUsec,
             trigger_config.laserPulseDelayUsec,
             trigger_config.laserPulseWidthUsec);
}

HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config) {
    if (config == NULL) {
        return HAL_ERROR; // Null pointer guard
    }

    // Add range checks for the configuration parameters
    if (config->frequencyHz == 0 || config->triggerPulseWidthUsec == 0) {
        return HAL_ERROR; // Invalid configuration values
    }

    // TIM2 Configuration: Trigger Frequency
    uint32_t tim2_period = 1000000 / config->frequencyHz; // Convert frequency to period in microseconds
    __HAL_TIM_SET_AUTORELOAD(&FSYNC_TIMER, tim2_period - 1);    // Set TIM2 period
    __HAL_TIM_SET_COMPARE(&FSYNC_TIMER, FSYNC_TIMER_CHAN, config->triggerPulseWidthUsec); // Set PWM pulse width for TIM2 Channel 1

    // TIM4 Configuration: Laser Pulse Delay
    uint32_t adjusted_pulse_delay = config->laserPulseDelayUsec/2;
    __HAL_TIM_SET_AUTORELOAD(&LASER_TIMER, adjusted_pulse_delay); // Set TIM4 period to ensure timing range
    __HAL_TIM_SET_COMPARE(&LASER_TIMER, LASER_TIMER_CHAN, adjusted_pulse_delay/2); // Set PWM for 50% duty cycle on Channel 1

    // TIM5 Configuration: Laser Pulse Width (with inverted signal logic)

    __HAL_TIM_SET_AUTORELOAD(&SYNC_TIMER, config->laserPulseWidthUsec + adjusted_pulse_delay); // Extend by 50 us
    __HAL_TIM_SET_COMPARE(&SYNC_TIMER, SYNC_TIMER_CHAN, adjusted_pulse_delay); // Set inverted PWM pulse width on Channel 2

    // Update the global trigger configuration
    trigger_config = *config;
    return HAL_OK;
}


HAL_StatusTypeDef Trigger_Start() {
    if (HAL_TIM_PWM_Start(&LASER_TIMER, LASER_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Start(&SYNC_TIMER, SYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Start(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }

    return HAL_OK;
}

HAL_StatusTypeDef Trigger_Stop() {
    if (HAL_TIM_PWM_Stop(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Stop(&SYNC_TIMER, SYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Stop(&LASER_TIMER, LASER_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }

    return HAL_OK;
}


HAL_StatusTypeDef Trigger_SetConfigFromJSON(char *jsonString, size_t str_len)
{
	uint8_t tempArr[255] = {0};
	bool ret = HAL_OK;

	Trigger_Config_t new_config = { 40, 1000, 100, 100 };
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

HAL_StatusTypeDef Trigger_GetConfigToJSON(char *jsonString)
{
	trigger_GetConfigJSON(jsonString, 0xFF);
    return HAL_OK;
}
