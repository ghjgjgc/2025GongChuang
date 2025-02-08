/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#define Test 0
struct Motor_Statue Motor_StatueTotal;
uint8_t Gyroscopes_ARRAY[11];
float Gyroscopes_ERRARRAY[5];
uint8_t rk3588_Arry[9]={0x00,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  HAL_UART_Receive_DMA(&huart4,rk3588_Arry,9);
  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  HAL_UART_Receive_DMA(&huart5,Gyroscopes_ARRAY,11);
  // __HAL_UART_ENABLE_IT(&huart5,ENABLE);
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA1     ------> UART4_RX
    PC10     ------> UART4_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA1     ------> UART4_RX
    PC10     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}
/* USER CODE BEGIN 1 */
/**
 * @brief UART4 Send Byte
 * 
 * @param Byte Byte
 */
void Send_Byte(uint8_t Byte)
{
  HAL_UART_Transmit(&huart4,(const uint8_t*)&Byte,1,0xFF);
}
/**
 * @brief Serial_Pow
 * 
 * @param X X
 * @param Y Y
 * @return uint32_t 
 */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}
/**
 * @brief Send Nuber
 * 
 * @param Number Uint32
 * @param Length Length
 */
void Send_Number(uint32_t Number,uint8_t Length)
{
  uint8_t i=0;
  for(i=0;i<Length;i++)
  {
    Send_Byte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
  }
}
uint8_t MOTOR_UARTPACK_SpeedControl[]={0x01/*Address*/,0xF6/*Mission_Identifier*/,
    0x01/*Direction*/,0x01,0x01/*Speed*/,0x00/*Accleration*/,0x01/*Multiple_Identiflier*/,0x6B/*CheckSum*/};
uint8_t SpeedControl_ReceivePACK[4]={0x00,0x00,0x00,0x00};
// const uint8_t SpeedControl_ReceiveSuccPack[]={0x01,0xF6,0xE2,0x6B};
// const uint8_t SpeedControl_ReceiveErrPack[]={0x01,0x00,0xEE,0x6B};
/**
 * @brief Control Motor by Speed UART
 * 
 * @param Motor_ID  MOTORx , x from 1 to 4
 * @param Direction Motor_Forward or Motor_Backward
 * @param Speed uint32_t, 0 to 65535
 * @param Accleration uint8_t,0 to 255 , 0 mean Start immediately
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef Motor_SpeedControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, uint32_t Speed,uint8_t Accleration)
{
    MOTOR_UARTPACK_SpeedControl[0]=Motor_ID;
    MOTOR_UARTPACK_SpeedControl[2]=Direction;
    MOTOR_UARTPACK_SpeedControl[3]=(Speed>>8)&0xFF;
    MOTOR_UARTPACK_SpeedControl[4]=Speed&0xFF;
    MOTOR_UARTPACK_SpeedControl[5]=Accleration;
    HAL_UART_Transmit(&huart1,(const uint8_t*)MOTOR_UARTPACK_SpeedControl,8,0xff);
    #if Test==0
    HAL_UART_Receive(&huart1,(uint8_t*)SpeedControl_ReceivePACK,4,0xff);
    #else
    HAL_UART_Transmit(&huart4,(const uint8_t*)MOTOR_UARTPACK_SpeedControl,8,0xff);
    #endif
    if (SpeedControl_ReceivePACK[3]==0xE2)
    {
        SpeedControl_ReceivePACK[3]=0x00;
        return HAL_OK;
    }
    else 
    {SpeedControl_ReceivePACK[3]=0x00;return HAL_ERROR;}
}
uint8_t MOTOR_UARTPACK_PositionControl[13]={0x01/*Address*/,0xFD/*Mission_Identifier*/,
    0x01/*Direction*/,0x01,0x01/*Speed*/,0x01/*Accleration*/,0x00,0x00,0x00,0x00/*PluseNum*/,0x00/*Relative or absolute position*/,0x01/*Multiple_Identiflier*/,0x6B/*CheckSum*/};
uint8_t PositionControl_ReceivePACK[4]={0x00,0x00,0x00,0x00};
// const uint8_t PositionControl_ReceiveSuccPack[]={0x01,0xFD,0xE2,0x6B};
// const uint8_t PositonControl_ReceiveErrPack[]={0x01,0x00,0xEE,0x6B};
/**
 * @brief Control Motor by Positon UART
 * 
 * @param Motor_ID  MOTORx , x from 1 to 4
 * @param Direction Motor_Forward or Motor_Backward
 * @param Speed uint32_t, 0 to 65535
 * @param Accleration uint8_t,0 to 255 , 0 mean Start immediately
 * @param PluseNum uint64_t , 0 to 4294967295
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef Motor_PositionControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, uint32_t Speed,uint8_t Accleration,uint64_t PluseNum)
{

    MOTOR_UARTPACK_PositionControl[0]=Motor_ID;
    MOTOR_UARTPACK_PositionControl[2]=Direction;
    MOTOR_UARTPACK_PositionControl[3]=(Speed>>8)&0xFF;
    MOTOR_UARTPACK_PositionControl[4]=Speed&0xFF;
    MOTOR_UARTPACK_PositionControl[5]=Accleration;
    MOTOR_UARTPACK_PositionControl[6]=PluseNum>>24&0xFF; 
    MOTOR_UARTPACK_PositionControl[7]=PluseNum>>16&0xFF;
    MOTOR_UARTPACK_PositionControl[8]=PluseNum>>8&0xFF;
    MOTOR_UARTPACK_PositionControl[9]=PluseNum&0xFF;
    HAL_UART_Transmit(&huart1,(const uint8_t*)MOTOR_UARTPACK_PositionControl,13,0xff);
    #if Test==0
    HAL_UART_Receive(&huart1,(uint8_t*)PositionControl_ReceivePACK,4,0xff);
    #else
    HAL_UART_Transmit(&huart4,(const uint8_t*)MOTOR_UARTPACK_PositionControl,13,0xff);
    #endif
    if (PositionControl_ReceivePACK[3]==0xE2)
    {
        PositionControl_ReceivePACK[3]=0x00;
        return HAL_OK;
    }
    else 
    {PositionControl_ReceivePACK[3]=0x00;return HAL_ERROR;}
}
const uint8_t MOTOR_SendMultipleStart_ARRAY[4]={0x00,0xFF,0x66,0x6B};
/**
 * @brief MOTOR_SendMultipleStart
 * 
 */
void MOTOR_SendMultipleStart(void)
{
  HAL_UART_Transmit(&huart1,(const uint8_t*)MOTOR_SendMultipleStart_ARRAY,4,0xff);
}
uint8_t MOTOR_UARTPACK_StopControl[]={0x00/*Address*/,0xFE,0x98/*Mission_Identifier*/,
    0x00/*Multiple_Identiflier*/,0x6B/*CheckSum*/};
/**
 * @brief Stop The Car
 * 
 */
void MOTOR_Stop(void)
{
	HAL_UART_Transmit(&huart1,(const uint8_t*)MOTOR_UARTPACK_StopControl,5,0xff);
}
uint8_t Mission_Pack[4]={0xBB,0x70,0x63,0xCC};
/**
 * @brief Send  task code
 * 
 * @param MissionCode MissionCode
 * @param Color Color
 */
void Send_MissionPack(uint8_t MissionCode,uint8_t Color)
{
  Mission_Pack[1]=MissionCode;
  Mission_Pack[2]=Color;
  HAL_UART_Transmit(&huart4,(const uint8_t*)Mission_Pack,4,0xFFFF);
}
float Target_angle=0,angle=0,prev_angle=0,Err_Angle=0,Err_Angle_Past=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{ 
  if(huart->Instance==UART5)
  {
    if(Gyroscopes_ARRAY[1]==0x53)
    {
      float Raw_angle=0,Delta_angle=0;
      Raw_angle=(float)((Gyroscopes_ARRAY[7]<<8)|Gyroscopes_ARRAY[6])/32768.0000f*180.0000f;
      Delta_angle=Raw_angle-prev_angle;
      if(Delta_angle>=180.0f)
      {
        Delta_angle-=360.0f;
      }
      else if(Delta_angle<=-180.0f)
      {
        Delta_angle+=360.0f;
      }
      prev_angle=angle;
      angle+=Delta_angle;
      Err_Angle_Past=Err_Angle;
      Err_Angle=angle-Target_angle;
    }
  }
}
/* USER CODE END 1 */
