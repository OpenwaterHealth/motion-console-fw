/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HUB_RESET_Pin GPIO_PIN_13
#define HUB_RESET_GPIO_Port GPIOC
#define nTRIG_Pin GPIO_PIN_2
#define nTRIG_GPIO_Port GPIOE
#define SYS_EN_Pin GPIO_PIN_9
#define SYS_EN_GPIO_Port GPIOB
#define LED_ON_Pin GPIO_PIN_8
#define LED_ON_GPIO_Port GPIOB
#define IND3_Pin GPIO_PIN_5
#define IND3_GPIO_Port GPIOD
#define SP_GPIO2_Pin GPIO_PIN_2
#define SP_GPIO2_GPIO_Port GPIOD
#define FAN_TOP_GD2_Pin GPIO_PIN_1
#define FAN_TOP_GD2_GPIO_Port GPIOE
#define MCU_GPIO1_Pin GPIO_PIN_0
#define MCU_GPIO1_GPIO_Port GPIOE
#define IND2_Pin GPIO_PIN_4
#define IND2_GPIO_Port GPIOD
#define DBG_RX_Pin GPIO_PIN_0
#define DBG_RX_GPIO_Port GPIOD
#define SCL_CFG_Pin GPIO_PIN_8
#define SCL_CFG_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_1
#define DBG_TX_GPIO_Port GPIOD
#define SDA_REM_Pin GPIO_PIN_9
#define SDA_REM_GPIO_Port GPIOC
#define FAN_TOP_GD1_Pin GPIO_PIN_7
#define FAN_TOP_GD1_GPIO_Port GPIOC
#define SP_GPIO3_Pin GPIO_PIN_8
#define SP_GPIO3_GPIO_Port GPIOC
#define TA_TRIGGER_Pin GPIO_PIN_6
#define TA_TRIGGER_GPIO_Port GPIOC
#define FSYNC_Pin GPIO_PIN_0
#define FSYNC_GPIO_Port GPIOA
#define SYNC_IN_Pin GPIO_PIN_4
#define SYNC_IN_GPIO_Port GPIOC
#define SYNC_OUT_Pin GPIO_PIN_5
#define SYNC_OUT_GPIO_Port GPIOC
#define POWER_DETECT_Pin GPIO_PIN_15
#define POWER_DETECT_GPIO_Port GPIOE
#define FULL_ON_n_Pin GPIO_PIN_10
#define FULL_ON_n_GPIO_Port GPIOD
#define IO_EXP_RSTN_Pin GPIO_PIN_2
#define IO_EXP_RSTN_GPIO_Port GPIOA
#define SP_GPIO1_Pin GPIO_PIN_9
#define SP_GPIO1_GPIO_Port GPIOD
#define IND1_Pin GPIO_PIN_3
#define IND1_GPIO_Port GPIOA
#define enSyncOUT_Pin GPIO_PIN_7
#define enSyncOUT_GPIO_Port GPIOA
#define FAN1_TACH_Pin GPIO_PIN_9
#define FAN1_TACH_GPIO_Port GPIOE
#define enSyncIN_Pin GPIO_PIN_8
#define enSyncIN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;

extern CRC_HandleTypeDef   hcrc;
extern UART_HandleTypeDef huart4;

#define DEBUG_UART huart4

#define FAN_PWM_TIMER htim15  //
#define LASER_TIMER htim3
#define LASER_TIMER_CHAN TIM_CHANNEL_1
#define FSYNC_TIMER htim2
#define FSYNC_TIMER_CHAN TIM_CHANNEL_1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
