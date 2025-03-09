#include "ROBOTIC_Arm.h"
/*Setting*/
#define Insurance_set 1
#define Raw_Material_Area_Height 135
#define WareHouseHeight 145
#define UnstackHeight 70
#define StackHeight 130
/*Storage of underlying data*/
const ROBOTICArm_Pose Raw_Material_Scanning={-165,0,220};
const ROBOTICArm_Pose Raw_Material_ClawFront={-115,0,Raw_Material_Area_Height};
const ROBOTICArm_Pose Raw_Material_ClawLeft={-320,-30,Raw_Material_Area_Height};
const ROBOTICArm_Pose Raw_Material_ClawRight={-312.5,80,Raw_Material_Area_Height};

const ROBOTICArm_Pose Blue_Warehouse={165,80,WareHouseHeight};
const ROBOTICArm_Pose Green_Warehouse={168,5,WareHouseHeight};
const ROBOTICArm_Pose Red_Warehouse={165,-80,WareHouseHeight};

const ROBOTICArm_Pose Place_Scaning={-6,210,190};

const ROBOTICArm_Pose Blue_PlacementLocation_Unstack={-140,223,UnstackHeight};
const ROBOTICArm_Pose Green_PlacementLocation_Unstack={-7,223.5,UnstackHeight};
const ROBOTICArm_Pose Red_PlacementLocation_Unstack={132,234,UnstackHeight};

const ROBOTICArm_Pose Blue_PlacementLocation_stack={-140,220,StackHeight};
const ROBOTICArm_Pose Green_PlacementLocation_stack={-7,220,StackHeight};
const ROBOTICArm_Pose Red_PlacementLocation_stack={132,230,StackHeight};

const ROBOTICArm_Pose Relay_point={-11.5,200,240};

ROBOTICArm_Pose Initial_state={0,200,200};
ROBOTICArm_Pose Past_state={0,200,200};
ROBOTICArm_Pose Current_state={0,200,200};
ROBOTICArm_Pose Final_state={0,200,200};

const float Arm_Compensation=95.0;
const float Base_Height=144.0;
const float Forearm_Length=120.0;
const float Bigarm_Length=120.0;

const float Bigarm_UpwardAngle=103.0;
const float Forearm_UptwardAngle=65.0;
const float Base_GreenAngle=47.5;
#define linearInterpolationAlgorithm_StepNum 10.0f
/**
 * @brief Robotic arm moves to the designated position.
 * 
 * @param AIM_XPosition AIM_XPosition
 * @param AIM_YPosition AIM_YPosition
 * @param AIM_ZPosition AIM_ZPosition
 * @param Claw_Control Mechanical jaw status
 * @return HAL_StatusTypeDef Succ or Fail
 */
HAL_StatusTypeDef ROBOTICArm_Coordinate_Calculation(float AIM_XPosition,float AIM_YPosition,float AIM_ZPosition)
{
    /*Coordinate compensation*/
    AIM_XPosition+=0;
    AIM_YPosition+=0;
    AIM_ZPosition+=30;//due to bracket
    /*Calculate secondary data*/
    float Arm_Projection=sqrtf(AIM_XPosition*AIM_XPosition+AIM_YPosition*AIM_YPosition)-Arm_Compensation;
	float Short_Edge=fabs(Base_Height-AIM_ZPosition);	
	float Oblique_Edge=sqrtf(Short_Edge*Short_Edge+Arm_Projection*Arm_Projection);
    /*Angle Calculate*/
    float Big_ArmAngle2;
    if(Base_Height==AIM_ZPosition)
		{Big_ArmAngle2=0;}
	else if(Base_Height>AIM_ZPosition)
		{Big_ArmAngle2=atan(Arm_Projection/Short_Edge)*180.0f/PI;}
	else if(Base_Height<AIM_ZPosition)
		{Big_ArmAngle2=atan(Short_Edge/Arm_Projection)*180.0f/PI+90.0f;}
    float Big_ArmAngle1=acos((Bigarm_Length*Bigarm_Length+Oblique_Edge*Oblique_Edge-Forearm_Length*Forearm_Length)/(2.0f*Bigarm_Length*Oblique_Edge))*180.0f/PI;
    float Big_ArmAngle=Big_ArmAngle1+Big_ArmAngle2;

    float ForeArm_angle=acos((Bigarm_Length*Bigarm_Length+Forearm_Length*Forearm_Length-Oblique_Edge*Oblique_Edge)/(2.0f*Forearm_Length*Bigarm_Length))*180.0f/PI;
    
    float Base_Angle=atan(AIM_YPosition/AIM_XPosition)*180.0f/PI;
    if(AIM_XPosition>=0)
    {
        Base_Angle+=Base_GreenAngle;
    }
    else Base_Angle+=Base_GreenAngle+180;

    /*Insurance settings*/
    HAL_StatusTypeDef Coordinate_Calculation_Flag=HAL_OK;
    #if Insurance_set==1
        Coordinate_Calculation_Flag=(270>Base_Angle&&Base_Angle>0)&&(170>(Bigarm_UpwardAngle+180-Big_ArmAngle)&&(Bigarm_UpwardAngle+180-Big_ArmAngle)>75)&&(160>(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180))&&(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180))>50)?HAL_OK:HAL_ERROR;
        uint8_t Fail_Message[]={(270>Base_Angle&&Base_Angle>0),Base_Angle,
        (170>(Bigarm_UpwardAngle+180-Big_ArmAngle)&&(Bigarm_UpwardAngle+180-Big_ArmAngle)>75),(Bigarm_UpwardAngle+180-Big_ArmAngle),
        (160>(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180))&&(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180))>50),(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180))};
        // HAL_UART_Transmit(&huart4,(const uint8_t*)Fail_Message,6,0xff);
    #endif
    if(Coordinate_Calculation_Flag==HAL_OK)
    {
        SERVO_AngleSet(SERVO_Base,Base_Angle);
        SERVO_AngleSet(SERVO_BigArm,(Bigarm_UpwardAngle+180-Big_ArmAngle));
        SERVO_AngleSet(SERVO_ForeArm,(Forearm_UptwardAngle+(Big_ArmAngle+ForeArm_angle-180)));
        return HAL_OK;
    }
    else return HAL_ERROR;
}
/**
 * @brief Resetting and initializing the robotic arm.
 * 
 */
void ROBOTICArm_initialize(void)
{
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    ROBOTICArm_Coordinate_Calculation(Initial_state.XPosition,Initial_state.YPosition,Initial_state.ZPosition);
    SERVO_ClawSet(Claw_Release);
    Current_state=Initial_state;
    Past_state=Initial_state;
}
/**
 * @brief Linear interpolation of the robotic arm.
 * 
 * @param Final_XPosition Final_XPosition
 * @param Final_YPosition Final_YPosition
 * @param Final_ZPosition Final_ZPosition
 * @param Claw_Control Claw_Control
 * @param Delay_Time Delay_Time
 * @return HAL_StatusTypeDef Busy or OK
 */
HAL_StatusTypeDef ROBOTICArm_linearInterpolationAlgorithm_Moving(float Final_XPosition,float Final_YPosition,float Final_ZPosition)
{
    /*normal variable*/
    const float epsilon = 1e-3;
    static uint8_t linearInterpolationAlgorithm_state=0;
    static uint16_t FeedCNT=0;
	static float LinearX_variation,LinearY_variation,LinearZ_variation;
	static float FeedAmount,Step_X,Step_Y,Step_Z;
    const uint16_t StartEndCNT=30;

    switch (linearInterpolationAlgorithm_state)
    {
        case 0:
            //Refresh state
            Current_state=Past_state;
            Final_state.XPosition=Final_XPosition;
            Final_state.YPosition=Final_YPosition;
            Final_state.ZPosition=Final_ZPosition;

            LinearX_variation=Final_state.XPosition-Past_state.XPosition;
            LinearY_variation=Final_state.YPosition-Past_state.YPosition;
            LinearZ_variation=Final_state.ZPosition-Past_state.ZPosition;

            FeedAmount=linearInterpolationAlgorithm_StepNum*fmaxf(fmaxf(fabsf(LinearX_variation),fabsf(LinearY_variation)),fabsf(LinearZ_variation));
            
            Step_X=LinearX_variation/FeedAmount;
            Step_Y=LinearY_variation/FeedAmount;
            Step_Z=LinearZ_variation/FeedAmount;
            linearInterpolationAlgorithm_state=1;

            FeedCNT=0;
            LED_ONOFF(LED4,1);
            return HAL_BUSY;
        case 1:
            //Feed
            Current_state.XPosition += Step_X;
            Current_state.YPosition += Step_Y;
            Current_state.ZPosition += Step_Z;
            FeedCNT++;

            ROBOTICArm_Coordinate_Calculation(Current_state.XPosition,Current_state.YPosition,Current_state.ZPosition);

            if(FeedCNT>FeedAmount||(fabsf(Current_state.XPosition - Final_state.XPosition) < epsilon &&fabsf(Current_state.YPosition - Final_state.YPosition) < epsilon &&fabsf(Current_state.ZPosition - Final_state.ZPosition) < epsilon))
            {
                ROBOTICArm_Coordinate_Calculation(Final_state.XPosition,Final_state.YPosition,Final_state.ZPosition);//due to float
                linearInterpolationAlgorithm_state=0; 
                FeedCNT=0;
                Past_state=Final_state;
                LED_ONOFF(LED4,0);
                return HAL_OK;
            }
            if(FeedCNT <= StartEndCNT || FeedCNT >= FeedAmount - StartEndCNT)
                Self_Delay(100);
            else Self_Delay(5);

            return HAL_BUSY;
        default:
            return HAL_OK;
    }
}
HAL_StatusTypeDef ROBOTICArm_linearInterpolationAlgorithm(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t Claw_Delay_Time)
{
    while(ROBOTICArm_linearInterpolationAlgorithm_Moving(Final_XPosition,Final_YPosition,Final_ZPosition)!=HAL_OK);
    SERVO_ClawSet(Claw_Control);
    HAL_Delay(Claw_Delay_Time);
    return HAL_OK;
}
/**
 * @brief Direct movement of the robotic arm
 * 
 * @param Final_XPosition Final_XPosition
 * @param Final_YPosition 
 * @param Final_ZPositionFinal_ZPosition
 * @param Claw_Control Claw_Control
 * @param Delay_Time Delay_Time
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef ROBOTICArm_DirectlyMove(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time )
{
    ROBOTICArm_Coordinate_Calculation(Final_XPosition,Final_YPosition,Final_ZPosition);//due to float
	HAL_Delay(GoTO_Delay_Time);
    SERVO_ClawSet(Claw_Control);
    HAL_Delay(Claw_Delay_Time);

    Current_state.XPosition=Final_XPosition;
    Current_state.YPosition=Final_YPosition;
    Current_state.ZPosition=Final_ZPosition;

    Past_state.XPosition=Final_XPosition;
    Past_state.YPosition=Final_YPosition;
    Past_state.ZPosition=Final_ZPosition;

    Past_state.Claw_Control=Claw_Control;
    Current_state.Claw_Control=Claw_Control;
    return HAL_OK;
}

