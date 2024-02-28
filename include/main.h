/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_CTS_Pin GPIO_PIN_0
#define ETH_CTS_GPIO_Port GPIOA
#define ETH_RTS_Pin GPIO_PIN_1
#define ETH_RTS_GPIO_Port GPIOA
#define ETH_TX_Pin GPIO_PIN_2
#define ETH_TX_GPIO_Port GPIOA
#define ETH_RX_Pin GPIO_PIN_3
#define ETH_RX_GPIO_Port GPIOA
#define ETH_nRST_Pin GPIO_PIN_4
#define ETH_nRST_GPIO_Port GPIOA
#define ETH_CP2_Pin GPIO_PIN_5
#define ETH_CP2_GPIO_Port GPIOA
#define RADIO_DIO5_Pin GPIO_PIN_6
#define RADIO_DIO5_GPIO_Port GPIOA
#define RADIO_DIO4_Pin GPIO_PIN_7
#define RADIO_DIO4_GPIO_Port GPIOA
#define RADIO_DIO3_Pin GPIO_PIN_0
#define RADIO_DIO3_GPIO_Port GPIOB
#define RADIO_DIO2_Pin GPIO_PIN_1
#define RADIO_DIO2_GPIO_Port GPIOB
#define RADIO_DIO1_Pin GPIO_PIN_2
#define RADIO_DIO1_GPIO_Port GPIOB
#define RADIO_DIO0_Pin GPIO_PIN_10
#define RADIO_DIO0_GPIO_Port GPIOB
#define RADIO_nRST_Pin GPIO_PIN_11
#define RADIO_nRST_GPIO_Port GPIOB
#define RADIO_nCS_Pin GPIO_PIN_12
#define RADIO_nCS_GPIO_Port GPIOB
#define RADIO_SCK_Pin GPIO_PIN_13
#define RADIO_SCK_GPIO_Port GPIOB
#define RADIO_MISO_Pin GPIO_PIN_14
#define RADIO_MISO_GPIO_Port GPIOB
#define RADIO_MOSI_Pin GPIO_PIN_15
#define RADIO_MOSI_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
