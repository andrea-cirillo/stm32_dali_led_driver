/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define GPIO_Output___MCU_TXD_Pin GPIO_PIN_0
#define GPIO_Output___MCU_TXD_GPIO_Port GPIOF
#define GPIO_Input___MCU_RXD_Pin GPIO_PIN_1
#define GPIO_Input___MCU_RXD_GPIO_Port GPIOF
#define GPIO_Input___NFC_EventDetect_Pin GPIO_PIN_0
#define GPIO_Input___NFC_EventDetect_GPIO_Port GPIOA
#define GPIO_Input___NFC_EventDetect_EXTI_IRQn EXTI0_1_IRQn
#define ADC_IN1___uVLED_Pin GPIO_PIN_1
#define ADC_IN1___uVLED_GPIO_Port GPIOA
#define GPIO_Output___OFF_Pin GPIO_PIN_3
#define GPIO_Output___OFF_GPIO_Port GPIOA
#define TIM14_CH1___PWM_OUT_Pin GPIO_PIN_4
#define TIM14_CH1___PWM_OUT_GPIO_Port GPIOA
#define GPIO_Output___STCP_Pin GPIO_PIN_6
#define GPIO_Output___STCP_GPIO_Port GPIOA
#define GPIO_Output___SHCP_Pin GPIO_PIN_7
#define GPIO_Output___SHCP_GPIO_Port GPIOA
#define GPIO_Output___CurrentLimit_DS_Pin GPIO_PIN_1
#define GPIO_Output___CurrentLimit_DS_GPIO_Port GPIOB
#define I2C1_SCL___EEPROM_Pin GPIO_PIN_9
#define I2C1_SCL___EEPROM_GPIO_Port GPIOA
#define I2C1_SDA___EEPROM_Pin GPIO_PIN_10
#define I2C1_SDA___EEPROM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
