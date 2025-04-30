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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "pid.h"
#include "spi.h"
#include "usart.h"

#include "general_def.h"
#include "bsp_can.h"
#include "bsp_opamp.h"
#include "bsp_adc.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "BLDCMotor.h"
#include "my_queue.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define VBUS_SP_Pin GPIO_PIN_4
#define VBUS_SP_GPIO_Port GPIOA
#define MOTOR_TEMP_SP_Pin GPIO_PIN_1
#define MOTOR_TEMP_SP_GPIO_Port GPIOB
#define MOS_TEMP_SP_Pin GPIO_PIN_11
#define MOS_TEMP_SP_GPIO_Port GPIOB
#define SPI3_CS_Pin GPIO_PIN_12
#define SPI3_CS_GPIO_Port GPIOB
#define LIN3_Pin GPIO_PIN_13
#define LIN3_GPIO_Port GPIOB
#define LIN2_Pin GPIO_PIN_14
#define LIN2_GPIO_Port GPIOB
#define LIN1_Pin GPIO_PIN_15
#define LIN1_GPIO_Port GPIOB
#define HIN3_Pin GPIO_PIN_8
#define HIN3_GPIO_Port GPIOA
#define HIN2_Pin GPIO_PIN_9
#define HIN2_GPIO_Port GPIOA
#define HIN1_Pin GPIO_PIN_10
#define HIN1_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_3
#define KEY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_R_ON  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1)
#define LED_R_OFF HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0)
#define LED_G_ON  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1)
#define LED_G_OFF HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
