#ifndef __MOTOR_CARBASE_H__
#define __MOTOR_CARBASE_H__
#include "main.h"
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "ROBOTIC_Arm.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN PTD */
typedef enum{UnStacking_Correction=2,Stacking_Correction=3,Angle_compensation=4,Raw_Material_Compensation=5,Raw_Material_Mission=1}Mission_Code;
typedef enum{Car_Posture_Up=0,Car_Posture_Down=180,Car_Posture_Left=90,Car_Posture_Right=-90,Car_DoingNothing=-1}Car_Posture;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void CarMove_TO_Global(float X_AIMPOSITION,float Y_AIMPOSITION,Car_Posture Aim_Posture,
    HAL_StatusTypeDef Part_TimeJob(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time ));
HAL_StatusTypeDef VOID_FUNCTION(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time );
HAL_StatusTypeDef Error_compensation(Mission_Code CODE);
extern struct Car_Pose None_CARPOSITION;
/* USER CODE END PFP */
#endif 
