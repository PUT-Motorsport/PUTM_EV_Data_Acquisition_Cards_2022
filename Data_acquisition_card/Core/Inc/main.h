/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define ADC1_BRAKE_PRESSURE_1_Pin GPIO_PIN_0
#define ADC1_BRAKE_PRESSURE_1_GPIO_Port GPIOC
#define ADC1_BRAKE_PRESSURE_2_Pin GPIO_PIN_1
#define ADC1_BRAKE_PRESSURE_2_GPIO_Port GPIOC
#define ADC1_SUSP_R_Pin GPIO_PIN_2
#define ADC1_SUSP_R_GPIO_Port GPIOC
#define ADC2_ANALOG_1_Pin GPIO_PIN_3
#define ADC2_ANALOG_1_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define ADC2_ANALOG_2_Pin GPIO_PIN_4
#define ADC2_ANALOG_2_GPIO_Port GPIOC
#define ADC2_SUSP_L_Pin GPIO_PIN_5
#define ADC2_SUSP_L_GPIO_Port GPIOC
#define Kill_driver_sense_Pin GPIO_PIN_0
#define Kill_driver_sense_GPIO_Port GPIOB
#define Kill_right_sense_Pin GPIO_PIN_1
#define Kill_right_sense_GPIO_Port GPIOB
#define Kill_left_sense_Pin GPIO_PIN_2
#define Kill_left_sense_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_12
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define BSPD_Sense_Pin GPIO_PIN_3
#define BSPD_Sense_GPIO_Port GPIOB
#define Overtravel_sense_Pin GPIO_PIN_4
#define Overtravel_sense_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
