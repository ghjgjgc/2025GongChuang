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
typedef enum{Motor_Forward=0x00,Motor_Backward=0x01}Motor_DIRECTION;
typedef enum{Motor1=1,Motor2=2,Motor3=3,Motor4=4}Motor_ID;
struct Motor_Statue{
  uint32_t Motor1_Speed;
  uint32_t Motor2_Speed;
  uint32_t Motor3_Speed;
  uint32_t Motor4_Speed;

  Motor_DIRECTION Motor1_Dir;
  Motor_DIRECTION Motor2_Dir;
  Motor_DIRECTION Motor3_Dir;
  Motor_DIRECTION Motor4_Dir;

  uint64_t Motor1_PluseNum;
  uint64_t Motor2_PluseNum;
  uint64_t Motor3_PluseNum;
  uint64_t Motor4_PluseNum;

  uint8_t Motor1_Accleration;
  uint8_t Motor2_Accleration;
  uint8_t Motor3_Accleration;
  uint8_t Motor4_Accleration;
};
extern struct Motor_Statue Motor_StatueTotal;

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern float Target_angle,angle,Err_Angle,Err_Angle_Past;
void MOTOR_SendMultipleStart(void);
HAL_StatusTypeDef Motor_SpeedControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, uint32_t Speed,uint8_t Accleration);
HAL_StatusTypeDef Motor_PositionControl_UART(Motor_ID Motor_ID, Motor_DIRECTION Direction, uint32_t Speed,uint8_t Accleration,uint64_t PluseNum);
void Send_MissionPack(uint8_t MissionCode,uint8_t Color);
void MOTOR_Stop(void);
void Send_Number(uint32_t Number,uint8_t Length);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

