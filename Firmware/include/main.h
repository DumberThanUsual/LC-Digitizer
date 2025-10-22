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
#include "stm32f3xx_hal.h"

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
#define OFFSET_CENTER_Pin GPIO_PIN_13
#define OFFSET_CENTER_GPIO_Port GPIOC
#define OFFSET_RIGHT_Pin GPIO_PIN_14
#define OFFSET_RIGHT_GPIO_Port GPIOC
#define OFFSET_UP_Pin GPIO_PIN_15
#define OFFSET_UP_GPIO_Port GPIOC
#define INPUT_COM_Pin GPIO_PIN_1
#define INPUT_COM_GPIO_Port GPIOF
#define OFFSET_COM_Pin GPIO_PIN_0
#define OFFSET_COM_GPIO_Port GPIOC
#define SCALE_B_Pin GPIO_PIN_1
#define SCALE_B_GPIO_Port GPIOC
#define SCALE_A_Pin GPIO_PIN_2
#define SCALE_A_GPIO_Port GPIOC
#define SCALE_PUSH_Pin GPIO_PIN_3
#define SCALE_PUSH_GPIO_Port GPIOC
#define RX1_ADC_Pin GPIO_PIN_4
#define RX1_ADC_GPIO_Port GPIOF
#define RX1_COMP_Pin GPIO_PIN_4
#define RX1_COMP_GPIO_Port GPIOA
#define RX3_COMP_Pin GPIO_PIN_5
#define RX3_COMP_GPIO_Port GPIOA
#define RX3_ADC_Pin GPIO_PIN_6
#define RX3_ADC_GPIO_Port GPIOA
#define TP908_Pin GPIO_PIN_7
#define TP908_GPIO_Port GPIOA
#define TP906_Pin GPIO_PIN_4
#define TP906_GPIO_Port GPIOC
#define TP907_Pin GPIO_PIN_5
#define TP907_GPIO_Port GPIOC
#define RX2_COMP_Pin GPIO_PIN_0
#define RX2_COMP_GPIO_Port GPIOB
#define RX2_ADC_Pin GPIO_PIN_1
#define RX2_ADC_GPIO_Port GPIOB
#define VREF_Pin GPIO_PIN_2
#define VREF_GPIO_Port GPIOB
#define TP909_Pin GPIO_PIN_10
#define TP909_GPIO_Port GPIOB
#define RX4_COMP_Pin GPIO_PIN_11
#define RX4_COMP_GPIO_Port GPIOB
#define RX4_ADC_Pin GPIO_PIN_12
#define RX4_ADC_GPIO_Port GPIOB
#define VREFB13_Pin GPIO_PIN_13
#define VREFB13_GPIO_Port GPIOB
#define VREFB14_Pin GPIO_PIN_14
#define VREFB14_GPIO_Port GPIOB
#define VREFB15_Pin GPIO_PIN_15
#define VREFB15_GPIO_Port GPIOB
#define MUX_A_Pin GPIO_PIN_6
#define MUX_A_GPIO_Port GPIOC
#define MUX_B_Pin GPIO_PIN_7
#define MUX_B_GPIO_Port GPIOC
#define MUX_C_Pin GPIO_PIN_8
#define MUX_C_GPIO_Port GPIOC
#define MUX_INHIBIT_Pin GPIO_PIN_9
#define MUX_INHIBIT_GPIO_Port GPIOC
#define TX_SLEEP_Pin GPIO_PIN_8
#define TX_SLEEP_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_10
#define OLED_SDA_GPIO_Port GPIOA
#define PUSH_1_Pin GPIO_PIN_15
#define PUSH_1_GPIO_Port GPIOA
#define UI_SCK_Pin GPIO_PIN_10
#define UI_SCK_GPIO_Port GPIOC
#define UI_RCLK_Pin GPIO_PIN_11
#define UI_RCLK_GPIO_Port GPIOC
#define UI_MOSI_Pin GPIO_PIN_12
#define UI_MOSI_GPIO_Port GPIOC
#define TX_CLK_Pin GPIO_PIN_3
#define TX_CLK_GPIO_Port GPIOB
#define PUSH_2_Pin GPIO_PIN_4
#define PUSH_2_GPIO_Port GPIOB
#define TX_MOSI_Pin GPIO_PIN_5
#define TX_MOSI_GPIO_Port GPIOB
#define TX_RCLK_Pin GPIO_PIN_6
#define TX_RCLK_GPIO_Port GPIOB
#define PUSH_3_Pin GPIO_PIN_7
#define PUSH_3_GPIO_Port GPIOB
#define OFFSET_LEFT_Pin GPIO_PIN_8
#define OFFSET_LEFT_GPIO_Port GPIOB
#define OFFSET_DOWN_Pin GPIO_PIN_9
#define OFFSET_DOWN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
