/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
typedef enum{Motor_Backward,Motor_Forward}Motor_DIRECTION;
typedef enum{Motor1=1,Motor2=2,Motor3=3,Motor4=4}Motor_ID;
#define UARTTest 0
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern float Target_angle,angle,Err_Angle,Err_Angle_Past,Raw_angle;
void MOTOR_SendMultipleStart(void);
HAL_StatusTypeDef Motor_SpeedControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, float Speed,uint8_t Accleration);
HAL_StatusTypeDef Motor_PositionControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, float Speed,uint8_t Accleration,uint64_t PluseNum);
void Send_MissionPack(uint8_t MissionCode,uint8_t Color);
void MOTOR_Stop(void);
void Send_Number(int32_t Number,uint8_t Length);
void Send_Byte(uint8_t Byte);
HAL_StatusTypeDef Gyroscopes_ARRAY_Handle(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

