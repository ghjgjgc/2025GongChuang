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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "Function.h"
#include <stdio.h>  
#include <stdlib.h> 
#include <string.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern UART_HandleTypeDef huart4;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

typedef enum{Claw_Release=1,Claw_Clawing=2,Claw_ReleaseFull=3}Claw_Status;
extern uint8_t Gyroscopes_ARRAY[11],rk3588_Arry[9];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM5_DIR_Pin GPIO_PIN_2
#define TIM5_DIR_GPIO_Port GPIOA
#define TIM2_DIR_Pin GPIO_PIN_4
#define TIM2_DIR_GPIO_Port GPIOA
#define TIM3_DIR_Pin GPIO_PIN_7
#define TIM3_DIR_GPIO_Port GPIOA
#define TIM4_DIR_Pin GPIO_PIN_13
#define TIM4_DIR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
typedef struct {
    float XPosition;
    float YPosition;
    float ZPosition;
    Claw_Status Claw_Control;
}ROBOTICArm_Pose;

#define PI 3.141592653589793f
#define SpecialONOFF 0
extern uint8_t DEBUG_USART[];
typedef enum{Yes=1,No=0}Variable_Flag;
extern Color Color_Order[3];//Color_Order
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
