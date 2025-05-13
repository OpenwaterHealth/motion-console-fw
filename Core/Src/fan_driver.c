/*
 * fan_driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */
#include "fan_driver.h"
#include "main.h"

#include <stdio.h>

static uint8_t curr_fan_dutycycle[2] = {0};

static void OW_FAN_Init(void)
{

	  /* USER CODE BEGIN TIM15_Init 0 */

	  /* USER CODE END TIM15_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM15_Init 1 */

	  /* USER CODE END TIM15_Init 1 */
	  htim15.Instance = TIM15;
	  htim15.Init.Prescaler = 60-1;
	  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim15.Init.Period = 100-1;
	  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim15.Init.RepetitionCounter = 0;
	  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 50;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 0;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.BreakFilter = 0;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM15_Init 2 */

	  /* USER CODE END TIM15_Init 2 */
	  HAL_TIM_MspPostInit(&htim15);


}

static void OW_FAN_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Stop the PWM channels for TIM1
    HAL_TIM_PWM_Stop(&FAN_PWM_TIMER, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&FAN_PWM_TIMER, TIM_CHANNEL_2);

    // Deinitialize TIM1
    HAL_TIM_PWM_DeInit(&FAN_PWM_TIMER);

    // Disable the GPIO pins used for FAN1 and FAN2 PWM
    HAL_GPIO_DeInit(FAN1_PWM_GPIO_Port, FAN1_PWM_Pin);
    HAL_GPIO_DeInit(FAN2_PWM_GPIO_Port, FAN2_PWM_Pin);

    HAL_GPIO_WritePin(FAN1_PWM_GPIO_Port, FAN1_PWM_Pin|FAN2_PWM_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : FAN1_PWM_Pin FAN2_PWM_Pin */
    GPIO_InitStruct.Pin = FAN1_PWM_Pin|FAN2_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FAN1_PWM_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(FAN1_PWM_GPIO_Port, FAN1_PWM_Pin|FAN2_PWM_Pin, GPIO_PIN_SET);

}

void FAN_Init(void)
{
	printf("Initializing Fans\r\n");
	OW_FAN_Init();

	FAN_SetSpeed(FAN1_PWM_CHANNEL, 0);
	FAN_SetSpeed(FAN2_PWM_CHANNEL, 0);

	HAL_TIM_PWM_Start(&FAN_PWM_TIMER, FAN1_PWM_CHANNEL);
	HAL_TIM_PWM_Start(&FAN_PWM_TIMER, FAN2_PWM_CHANNEL);

}

void FAN_DeInit(void)
{
	printf("De-Initializing Fans\r\n");
	OW_FAN_DeInit();
}

void FAN_SetSpeed(uint32_t channel, uint8_t duty_cycle)
{
    uint32_t pulse = 0;
    uint8_t index = 0;
    // Check that the channel is valid
    if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2) {
        printf("Invalid fan channel provided\r\n");
        return; // Invalid channel, do nothing
    }

    // Ensure duty cycle is within 0-100%
    if (duty_cycle > 100) {
        duty_cycle = 100;
    }

    if(channel == TIM_CHANNEL_2) index = 1;
    else index = 0;

    curr_fan_dutycycle[index] = duty_cycle;

    // Calculate pulse width based on duty cycle
    pulse = (__HAL_TIM_GET_AUTORELOAD(&FAN_PWM_TIMER) + 1) * duty_cycle / 100;

    // Set PWM duty cycle for the specified fan
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIMER, channel, pulse);
}

uint8_t FAN_GetSpeed(uint32_t channel)
{
    uint8_t index = 0;

    // Check that the channel is valid
    if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2) {
        printf("Invalid fan channel provided\r\n");
        return; // Invalid channel, do nothing
    }

    if(channel == TIM_CHANNEL_2) index = 1;
    else index = 0;

    return curr_fan_dutycycle[index];
}

uint32_t FAN_GetRPM(uint32_t channel)
{
    uint32_t pulse_count = 0;
    uint32_t rpm = 0;
    // Check that the channel is valid
    if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2) {
        printf("Invalid fan channel provided\r\n");
        return 0; // Invalid channel, do nothing
    }

    // Read the tachometer pin for the specified fan
    if (channel == TIM_CHANNEL_1) {
        pulse_count = HAL_GPIO_ReadPin(FAN1_TACH_GPIO_Port, FAN1_TACH_Pin);
    } else {
        pulse_count = HAL_GPIO_ReadPin(FAN2_TACH_GPIO_Port, FAN2_TACH_Pin);
    }

    // Calculate RPM from pulse count
    // This requires capturing pulse counts over a fixed time period
    // Placeholder logic assumes pulse_count is pre-measured
    rpm = (pulse_count * 60) / TACH_PULSES_PER_REV;

    return rpm;
}
