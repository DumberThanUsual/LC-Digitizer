/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comp.c
  * @brief   This file provides code for the configuration
  *          of the COMP instances.
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
/* Includes ------------------------------------------------------------------*/
#include "comp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp5;
COMP_HandleTypeDef hcomp6;

/* COMP3 init function */
void MX_COMP3_Init(void)
{
  hcomp3.Instance = COMP3;
  hcomp3.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp3.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp3.Init.Output = COMP_OUTPUT_TIM15IC1;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp3.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp3.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
}
/* COMP4 init function */
void MX_COMP4_Init(void)
{
  hcomp4.Instance = COMP4;
  hcomp4.Init.InvertingInput = COMP_INVERTINGINPUT_IO2;
  hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp4.Init.Output = COMP_OUTPUT_TIM4IC2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp4.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp4.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
}
/* COMP5 init function */
void MX_COMP5_Init(void)
{
  hcomp5.Instance = COMP5;
  hcomp5.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp5.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp5.Init.Output = COMP_OUTPUT_TIM17IC1;
  hcomp5.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
  hcomp5.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp5.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp5.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp5.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp5.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp5) != HAL_OK)
  {
    Error_Handler();
  }
}
/* COMP6 init function */
void MX_COMP6_Init(void)
{
  hcomp6.Instance = COMP6;
  hcomp6.Init.InvertingInput = COMP_INVERTINGINPUT_IO2;
  hcomp6.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp6.Init.Output = COMP_OUTPUT_TIM16IC1;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp6.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp6.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP3)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP3 GPIO Configuration
    PA4     ------> COMP3_INM
    PB14     ------> COMP3_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(compHandle->Instance==COMP4)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    PB2     ------> COMP4_INM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(compHandle->Instance==COMP5)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP5 GPIO Configuration
    PA5     ------> COMP5_INM
    PB13     ------> COMP5_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(compHandle->Instance==COMP6)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    PB15     ------> COMP6_INM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{

  if(compHandle->Instance==COMP3)
  {
    /**COMP3 GPIO Configuration
    PA4     ------> COMP3_INM
    PB14     ------> COMP3_INP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
  }
  else if(compHandle->Instance==COMP4)
  {
    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    PB2     ------> COMP4_INM
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_2);
  }
  else if(compHandle->Instance==COMP5)
  {
    /**COMP5 GPIO Configuration
    PA5     ------> COMP5_INM
    PB13     ------> COMP5_INP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
  }
  else if(compHandle->Instance==COMP6)
  {
    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    PB15     ------> COMP6_INM
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11|GPIO_PIN_15);
  }
}