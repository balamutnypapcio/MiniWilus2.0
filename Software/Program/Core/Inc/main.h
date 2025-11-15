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
#define EXTI1_Pin GPIO_PIN_13
#define EXTI1_GPIO_Port GPIOC
#define EXTI1_EXTI_IRQn EXTI15_10_IRQn
#define EXTI2_Pin GPIO_PIN_14
#define EXTI2_GPIO_Port GPIOC
#define EXTI2_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT2_Pin GPIO_PIN_15
#define XSHUT2_GPIO_Port GPIOC
#define XSHUT1_Pin GPIO_PIN_0
#define XSHUT1_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define XSHUT3_Pin GPIO_PIN_5
#define XSHUT3_GPIO_Port GPIOA
#define XSHUT4_Pin GPIO_PIN_6
#define XSHUT4_GPIO_Port GPIOA
#define M1_OUT1_Pin GPIO_PIN_7
#define M1_OUT1_GPIO_Port GPIOA
#define M1_OUT2_Pin GPIO_PIN_0
#define M1_OUT2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define EXTI4_Pin GPIO_PIN_10
#define EXTI4_GPIO_Port GPIOA
#define EXTI4_EXTI_IRQn EXTI15_10_IRQn
#define EXTI3_Pin GPIO_PIN_11
#define EXTI3_GPIO_Port GPIOA
#define EXTI3_EXTI_IRQn EXTI15_10_IRQn
#define PWM2_Pin GPIO_PIN_4
#define PWM2_GPIO_Port GPIOB
#define M2_OUT1_Pin GPIO_PIN_6
#define M2_OUT1_GPIO_Port GPIOB
#define M2_OUT2_Pin GPIO_PIN_7
#define M2_OUT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
