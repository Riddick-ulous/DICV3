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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Heart_Pin GPIO_PIN_0
#define Heart_GPIO_Port GPIOC
#define Error_Pin GPIO_PIN_1
#define Error_GPIO_Port GPIOC
#define SPI2_MOSI_RGBLED_Pin GPIO_PIN_3
#define SPI2_MOSI_RGBLED_GPIO_Port GPIOC
#define nCS_Display_Pin GPIO_PIN_4
#define nCS_Display_GPIO_Port GPIOA
#define SPI1_SCK_Display_Pin GPIO_PIN_5
#define SPI1_SCK_Display_GPIO_Port GPIOA
#define D_nC_Display_Pin GPIO_PIN_6
#define D_nC_Display_GPIO_Port GPIOA
#define SPI1_MOSI_Display_Pin GPIO_PIN_7
#define SPI1_MOSI_Display_GPIO_Port GPIOA
#define nRES_Display_Pin GPIO_PIN_4
#define nRES_Display_GPIO_Port GPIOC
#define ADC2_NTC_CTRL_Pin GPIO_PIN_0
#define ADC2_NTC_CTRL_GPIO_Port GPIOB
#define ADC1_NTC_CTRL_Pin GPIO_PIN_1
#define ADC1_NTC_CTRL_GPIO_Port GPIOB
#define ADC0_NTC_CTRL_Pin GPIO_PIN_2
#define ADC0_NTC_CTRL_GPIO_Port GPIOB
#define SPI2_SCK_RGBLED_Pin GPIO_PIN_10
#define SPI2_SCK_RGBLED_GPIO_Port GPIOB
#define ADC1_CTRL_Pin GPIO_PIN_11
#define ADC1_CTRL_GPIO_Port GPIOB
#define ADC2_CTRL_Pin GPIO_PIN_12
#define ADC2_CTRL_GPIO_Port GPIOB
#define ADC3_CTRL_Pin GPIO_PIN_13
#define ADC3_CTRL_GPIO_Port GPIOB
#define GPIO3_In_Pin GPIO_PIN_14
#define GPIO3_In_GPIO_Port GPIOB
#define GPIO3_5VOut_Pin GPIO_PIN_15
#define GPIO3_5VOut_GPIO_Port GPIOB
#define GPIO3_12VOut_Pin GPIO_PIN_6
#define GPIO3_12VOut_GPIO_Port GPIOC
#define GPIO2_In_Pin GPIO_PIN_7
#define GPIO2_In_GPIO_Port GPIOC
#define GPIO2_5VOut_Pin GPIO_PIN_8
#define GPIO2_5VOut_GPIO_Port GPIOC
#define GPIO2_12VOut_Pin GPIO_PIN_9
#define GPIO2_12VOut_GPIO_Port GPIOC
#define GPIO1_In_Pin GPIO_PIN_8
#define GPIO1_In_GPIO_Port GPIOA
#define GPIO1_5VOut_Pin GPIO_PIN_9
#define GPIO1_5VOut_GPIO_Port GPIOA
#define GPIO1_12VOut_Pin GPIO_PIN_10
#define GPIO1_12VOut_GPIO_Port GPIOA
#define GPIO6_In_Pin GPIO_PIN_10
#define GPIO6_In_GPIO_Port GPIOC
#define GPIO6_5VOut_Pin GPIO_PIN_11
#define GPIO6_5VOut_GPIO_Port GPIOC
#define GPIO6_12VOut_Pin GPIO_PIN_12
#define GPIO6_12VOut_GPIO_Port GPIOC
#define GPIO5_In_Pin GPIO_PIN_2
#define GPIO5_In_GPIO_Port GPIOD
#define GPIO5_5VOut_Pin GPIO_PIN_3
#define GPIO5_5VOut_GPIO_Port GPIOB
#define GPIO5_12VOut_Pin GPIO_PIN_4
#define GPIO5_12VOut_GPIO_Port GPIOB
#define GPIO4_In_Pin GPIO_PIN_5
#define GPIO4_In_GPIO_Port GPIOB
#define GPIO4_5VOut_Pin GPIO_PIN_6
#define GPIO4_5VOut_GPIO_Port GPIOB
#define GPIO4_12VOut_Pin GPIO_PIN_7
#define GPIO4_12VOut_GPIO_Port GPIOB
#define ADC3_NTC_CTRL_Pin GPIO_PIN_8
#define ADC3_NTC_CTRL_GPIO_Port GPIOB
#define ADC0_CTRL_Pin GPIO_PIN_9
#define ADC0_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
