/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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
#define DC_OFF_Pin GPIO_PIN_13
#define DC_OFF_GPIO_Port GPIOC
#define GATE_Pin GPIO_PIN_15
#define GATE_GPIO_Port GPIOC
#define OVERCURRENT_Pin GPIO_PIN_5
#define OVERCURRENT_GPIO_Port GPIOA
#define OVERCURRENT_EXTI_IRQn EXTI9_5_IRQn
#define ENC28J60_CS_Pin GPIO_PIN_12
#define ENC28J60_CS_GPIO_Port GPIOB
#define FAN_DC_Pin GPIO_PIN_9
#define FAN_DC_GPIO_Port GPIOA
#define ENC28J60_RESET_Pin GPIO_PIN_10
#define ENC28J60_RESET_GPIO_Port GPIOA
#define LED_DC_Pin GPIO_PIN_11
#define LED_DC_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_3
#define FAN_GPIO_Port GPIOB
#define MAX6675_CS_Pin GPIO_PIN_7
#define MAX6675_CS_GPIO_Port GPIOB
#define MAX6675_CLK_Pin GPIO_PIN_8
#define MAX6675_CLK_GPIO_Port GPIOB
#define MAX6675_SO_Pin GPIO_PIN_9
#define MAX6675_SO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define ENC28J60_hspi hspi2
//#define ENC28J60_LED_GPIO_PORT GPIOB
//#define ENC28J60_LED_PIN GPIO_PIN_13
/*
#define MAX6675_hspi hspi2
#define MAX6675_CLK_Pin GPIO_PIN_13
#define MAX6675_CLK_GPIO_Port GPIOB
#define MAX6675_SO_Pin GPIO_PIN_14
#define MAX6675_SO_GPIO_Port GPIOB
#define MAX6675_SI_Pin GPIO_PIN_15
#define MAX6675_SI_GPIO_Port GPIOB
*/
#define LM92_hi2c hi2c2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
