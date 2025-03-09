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
typedef enum{Car_Posture_Up,Car_Posture_Left,Car_Posture_Down,Car_Posture_Right}Car_Posture;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
struct Car_Pose{
    float X_POSITION;
    float Y_POSITION;
    Car_Posture Posture;
};
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void CarMove_TO_Global(float X_AIMPOSITION,float Y_AIMPOSITION,Car_Posture Aim_Posture,HAL_StatusTypeDef Part_TimeJob(void));
HAL_StatusTypeDef VOID_FUNCTION(void);
HAL_StatusTypeDef Error_compensation(Mission_Code CODE);
void TURN(Car_Posture Aim_CARPOSITON_RELATIVE,Car_Posture Current_CARPOSITION_RELATIVE);
extern struct Car_Pose Start_CARPOSITION,None_CARPOSITION,QR_CARPOSITION,RawMaterial_CARPOSITION,RoughProcess_CARPOSITION,Temporary_CARPOSITION,Start_CARPOSITION_Second,RawMaterial_CARPOSITION_Second,RoughProcess_CARPOSITION_Second,Temporary_CARPOSITION_Second;
/* USER CODE END PFP */
#endif 
