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
typedef enum{Car_Posture_Up=0,Car_Posture_Down=180,Car_Posture_Left=90,Car_Posture_Right=-90}Car_Posture;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
struct Car_Pose{
    float X_POSITION;
    float Y_POSITION;
    Car_Posture Posture;
};
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void CarMove_TO_Global(float X_AIMPOSITION,float Y_AIMPOSITION,Car_Posture Aim_Posture,HAL_StatusTypeDef Part_TimeJob(float Final_XPosition,float Final_YPosition,float Final_ZPosition));
HAL_StatusTypeDef VOID_FUNCTION(float AIM_XPosition,float AIM_YPosition,float AIM_ZPosition);
HAL_StatusTypeDef Error_compensation(Mission_Code CODE);
void CarTurn(Car_Posture Current_Direction,Car_Posture Aim_Direction);
extern struct Car_Pose None_CARPOSITION,QR_CARPOSITION,RawMaterial_CARPOSITION,RoughProcess_CARPOSITION,Temporary_CARPOSITION;
/* USER CODE END PFP */
#endif 
