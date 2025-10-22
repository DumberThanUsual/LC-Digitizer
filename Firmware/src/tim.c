/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim4_ch2;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;
DMA_HandleTypeDef hdma_tim16_ch1_up;
DMA_HandleTypeDef hdma_tim17_ch1_up;

/* TIM4 init function */
void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 41;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 40;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM15 init function */
void MX_TIM15_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/* TIM16 init function */
void MX_TIM16_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC = {0};

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/* TIM17 init function */
void MX_TIM17_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC = {0};

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
  if(tim_icHandle->Instance==TIM4)
  {
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4 DMA Init */
    /* TIM4_CH2 Init */
    hdma_tim4_ch2.Instance = DMA1_Channel4;
    hdma_tim4_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim4_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim4_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim4_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim4_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim4_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim4_ch2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim4_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_CC2],hdma_tim4_ch2);
  }
  else if(tim_icHandle->Instance==TIM15)
  {
    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();

    /* TIM15 DMA Init */
    /* TIM15_CH1_UP_TRIG_COM Init */
    hdma_tim15_ch1_up_trig_com.Instance = DMA1_Channel5;
    hdma_tim15_ch1_up_trig_com.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim15_ch1_up_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim15_ch1_up_trig_com.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim15_ch1_up_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim15_ch1_up_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim15_ch1_up_trig_com.Init.Mode = DMA_NORMAL;
    hdma_tim15_ch1_up_trig_com.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim15_ch1_up_trig_com) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_CC1],hdma_tim15_ch1_up_trig_com);
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_UPDATE],hdma_tim15_ch1_up_trig_com);
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_TRIGGER],hdma_tim15_ch1_up_trig_com);
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim15_ch1_up_trig_com);
  }
}

/**
  * @brief TIM_OC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_oc: TIM_OC handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspInit 0 */

    /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    /* USER CODE BEGIN TIM8_MspInit 1 */

    /* USER CODE END TIM8_MspInit 1 */

  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM16)
  {
    /* TIM16 clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* TIM16 DMA Init */
    /* TIM16_CH1_UP Init */
    hdma_tim16_ch1_up.Instance = DMA1_Channel3;
    hdma_tim16_ch1_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim16_ch1_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim16_ch1_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim16_ch1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim16_ch1_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim16_ch1_up.Init.Mode = DMA_NORMAL;
    hdma_tim16_ch1_up.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim16_ch1_up) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim16_ch1_up);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_UPDATE],hdma_tim16_ch1_up);
  }
  else if(tim_baseHandle->Instance==TIM17)
  {
    /* TIM17 clock enable */
    __HAL_RCC_TIM17_CLK_ENABLE();

    /* TIM17 DMA Init */
    /* TIM17_CH1_UP Init */
    hdma_tim17_ch1_up.Instance = DMA1_Channel7;
    hdma_tim17_ch1_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim17_ch1_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim17_ch1_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim17_ch1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim17_ch1_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim17_ch1_up.Init.Mode = DMA_NORMAL;
    hdma_tim17_ch1_up.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim17_ch1_up) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_TIM17_DMA1_CH7);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim17_ch1_up);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_UPDATE],hdma_tim17_ch1_up);
  }
  else if(tim_baseHandle->Instance==TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);

    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* TIM4 DMA DeInit */
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_CC2]);
  }
  else if(tim_icHandle->Instance==TIM15)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();

    /* TIM15 DMA DeInit */
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_TRIGGER]);
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_COMMUTATION]);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM16)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* TIM16 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_UPDATE]);
  }
  else if(tim_baseHandle->Instance==TIM17)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM17_CLK_DISABLE();

    /* TIM17 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_UPDATE]);
  }
}
