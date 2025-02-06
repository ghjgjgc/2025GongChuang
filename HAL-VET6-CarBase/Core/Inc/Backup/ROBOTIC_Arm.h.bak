#ifndef __ROBOTIC_ARM_H__
#define __ROBOTIC_ARM_H__
#include "main.h"
/* USER CODE BEGIN Includes */
#include "tim.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PV */
extern const ROBOTICArm_Pose Raw_Material_Scanning,Raw_Material_ClawFront,Raw_Material_ClawLeft,Raw_Material_ClawRight,
	Blue_Warehouse,Green_Warehouse,Red_Warehouse,
	Blue_PlacementLocation_Unstack,Green_PlacementLocation_Unstack,Red_PlacementLocation_Unstack,
	Blue_PlacementLocation_stack,Green_PlacementLocation_stack,Red_PlacementLocation_stack,
	Relay_point;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void Final_PoseSet(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control);
HAL_StatusTypeDef ROBOTICArm_Coordinate_Calculation(float AIM_XPosition,float AIM_YPosition,float AIM_ZPosition,Claw_Status Claw_Control);
HAL_StatusTypeDef ROBOTICArm_linearInterpolationAlgorithm(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time);
HAL_StatusTypeDef ROBOTICArm_DirectlyMove(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time );
void ROBOTICArm_initialize(void);
/* USER CODE END PFP */
#endif 
