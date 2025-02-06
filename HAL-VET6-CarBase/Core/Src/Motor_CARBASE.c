#include "Motor_CARBASE.h"
/* USER CODE BEGIN PD */
    #define TURNED_ANGLEPLUSE 4400
    #define offsetsX 2.36
	#define offsetsY 2.50
/* USER CODE END PD */

/* USER CODE BEGIN PV */
    //Define Key Value
    const float Motor2_P=1,Motor3_P=1,Motor2_D=1,Motor3_D=1;
    
    const float TimerCost=(1679+1)*(9999+1)/168000000; //0.1s
    const float Maxium_Speed_Set = 5000; // mm/Maxium_Speed_Set
    const float Motor_Acceleration = 1.0f * PI * 75.0f / 60.0f ; //mm/TimerCost //3.92
    //Define Stored global variables
    int32_t StartEnd_CostCNTX=0,StartEnd_CostCNTY=0,SpeedHold_CNTX=0,SpeedHold_CNTY=0,TIM_CNT=0,Total_CNTX=0,Total_CNTY=0,Key_CNT=0;
    Variable_Flag NotUp_To_MaxSpeed_FlagX=No,NotUp_To_MaxSpeed_FlagY=No,If_Motor_Busy=No;
    Motor_DIRECTION Motor1_Direction = Motor_Forward,Motor2_Direction = Motor_Forward,Motor3_Direction = Motor_Forward,Motor4_Direction = Motor_Forward;
    float Maxium_Speed_ActualX = 0 , Maxium_Speed_ActualY=0;
    float Motor1_Speed=0, Motor2_Speed=0, Motor3_Speed=0, Motor4_Speed=0,
        Motor1_SpeedX=0, Motor2_SpeedX=0, Motor3_SpeedX=0, Motor4_SpeedX=0,
        Motor1_SpeedY=0, Motor2_SpeedY=0, Motor3_SpeedY=0, Motor4_SpeedY=0;
    int8_t OverAll_PostureX,OverAll_PostureY;
    //Definition of data type
    struct Car_Pose{
    float X_POSITION;
    float Y_POSITION;
    Car_Posture Posture;
    }Current_CARPOSITION_GLOBAL={0.0,0.0,Car_Posture_Up};
    extern TIM_HandleTypeDef htim10;
    extern const ROBOTICArm_Pose Relay_point;
/* USER CODE END PV */
/**
 * @brief Move the vehicle to the corresponding world coordinate point.
 * 
 * @param X_AIMPOSITION world coordinate X_AIMPOSITION.
 * @param Y_AIMPOSITION world coordinate Y_AIMPOSITION.
 * @param Aim_Posture CarPosture
 * @param function part_timeJob
 */
void CarMove_TO(float X_AIMPOSITION,float Y_AIMPOSITION,Car_Posture Aim_Posture,HAL_StatusTypeDef Part_TimeJob(float Final_XPosition,float Final_YPosition,float Final_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time ))
{
    struct Car_Pose Current_CARPOSITION_RELATIVE,Aim_CARPOSITON;
    /**
     * @param Car_Posture_Up: x2 = x1 ; y2 = y1
     * @param Car_Posture_Left: x3 = -y1 ; y3 = x1
     * @param Car_Posture_Down: x4 = -x1 ; y4 = -y1
     * @param Car_Posture_Right: x5 = y1 ; y5 = -x1
     */
    switch(Current_CARPOSITION_GLOBAL.Posture)//coordinate transformation 
    {
        case Car_Posture_Up:
            Current_CARPOSITION_RELATIVE.X_POSITION=Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON.X_POSITION=X_AIMPOSITION;
            Aim_CARPOSITON.Y_POSITION=Y_AIMPOSITION;
            Aim_CARPOSITON.Posture=Aim_Posture;
            break;
        case Car_Posture_Left:
            Current_CARPOSITION_RELATIVE.X_POSITION=-Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON.X_POSITION=-Y_AIMPOSITION;
            Aim_CARPOSITON.Y_POSITION=X_AIMPOSITION;
            Aim_CARPOSITON.Posture=Aim_Posture;
            break;
        case Car_Posture_Down:
            Current_CARPOSITION_RELATIVE.X_POSITION=-Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=-Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON.X_POSITION=-X_AIMPOSITION;
            Aim_CARPOSITON.Y_POSITION=-Y_AIMPOSITION;
            Aim_CARPOSITON.Posture=Aim_Posture;
            break;
        case Car_Posture_Right:
            Current_CARPOSITION_RELATIVE.X_POSITION=Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=-Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON.X_POSITION=Y_AIMPOSITION;
            Aim_CARPOSITON.Y_POSITION=-X_AIMPOSITION;
            Aim_CARPOSITON.Posture=Aim_Posture;
            break;
        default:break;
    }
    float X_Distance = offsetsX*fabs(Aim_CARPOSITON.X_POSITION - Current_CARPOSITION_RELATIVE.X_POSITION);
    float Y_Distance = offsetsY*fabs(Aim_CARPOSITON.Y_POSITION - Current_CARPOSITION_RELATIVE.Y_POSITION);

    OverAll_PostureX=(Aim_CARPOSITON.X_POSITION - Current_CARPOSITION_RELATIVE.X_POSITION) >= 0 ? 1:-1;
    OverAll_PostureY=(Aim_CARPOSITON.Y_POSITION - Current_CARPOSITION_RELATIVE.Y_POSITION) >= 0 ? 1:-1;

    //---------------------------Begin Setting Maxium Speed----------------------------//
    
    if(Maxium_Speed_Set*Maxium_Speed_Set/(Motor_Acceleration)>X_Distance) //Indicates that the distance required for acceleration is greater than the target distance at this point
        {Maxium_Speed_ActualX = sqrt(X_Distance * (Motor_Acceleration));NotUp_To_MaxSpeed_FlagX=Yes;SpeedHold_CNTX=0;}
    else {Maxium_Speed_ActualX = Maxium_Speed_Set;NotUp_To_MaxSpeed_FlagX=No;SpeedHold_CNTX=X_Distance-(Maxium_Speed_Set*Maxium_Speed_Set)/Motor_Acceleration;}
    StartEnd_CostCNTX=Maxium_Speed_ActualX / Motor_Acceleration;

    if(Maxium_Speed_Set*Maxium_Speed_Set/(Motor_Acceleration)>Y_Distance) //Indicates that the distance required for acceleration is greater than the target distance at this point
        {Maxium_Speed_ActualY = sqrt(Y_Distance * (Motor_Acceleration));NotUp_To_MaxSpeed_FlagY=Yes;SpeedHold_CNTY=0;}
    else {Maxium_Speed_ActualY = Maxium_Speed_Set;NotUp_To_MaxSpeed_FlagY=No;SpeedHold_CNTY=Y_Distance-(Maxium_Speed_Set*Maxium_Speed_Set)/Motor_Acceleration;}
    StartEnd_CostCNTY=Maxium_Speed_ActualY / Motor_Acceleration;

    Total_CNTX=StartEnd_CostCNTX*2+SpeedHold_CNTX;
    Total_CNTY=StartEnd_CostCNTY*2+SpeedHold_CNTY;
		
    if(Total_CNTX>=Total_CNTY)
    Key_CNT=Total_CNTX;
    else Key_CNT=Total_CNTY;

    //Now we have the CNTs needed for acceleration and deceleration, and the CNTs needed to maintain speed if any
    //---------------------------End Setting Maxium Speed----------------------------//
    If_Motor_Busy=Yes;
    HAL_TIM_Base_Start_IT(&htim10);

    //---------------------BEGIN PART-TIME JOB------------------------------------//
    while(If_Motor_Busy==Yes)
    {
        Part_TimeJob(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_Release,200,200);
    }
    //-----------------------END PART-TIME JOB------------------------------------//
    MOTOR_Stop();
    HAL_Delay(20);
    if(Current_CARPOSITION_RELATIVE.Posture!=Aim_CARPOSITON.Posture)
    {
        float TURNED_ANGLE=Aim_CARPOSITON.Posture-Current_CARPOSITION_RELATIVE.Posture;
        Motor_PositionControl_UART(Motor1,TURNED_ANGLE>=0?Motor_Backward:Motor_Forward,1000,0x10,(uint32_t)fabs(TURNED_ANGLE)/90.0*TURNED_ANGLEPLUSE);
        Motor_PositionControl_UART(Motor2,TURNED_ANGLE>=0?Motor_Forward:Motor_Backward,1000,0x10,(uint32_t)fabs(TURNED_ANGLE)/90.0*TURNED_ANGLEPLUSE);
        Motor_PositionControl_UART(Motor3,TURNED_ANGLE>=0?Motor_Backward:Motor_Forward,1000,0x10,(uint32_t)fabs(TURNED_ANGLE)/90.0*TURNED_ANGLEPLUSE);
        Motor_PositionControl_UART(Motor4,TURNED_ANGLE>=0?Motor_Forward:Motor_Backward,1000,0x10,(uint32_t)fabs(TURNED_ANGLE)/90.0*TURNED_ANGLEPLUSE);
        MOTOR_SendMultipleStart();
        HAL_Delay(2400*fabs(TURNED_ANGLE)/90.0);
        MOTOR_Stop();
        Target_angle=Aim_CARPOSITON.Posture;
    }
    //------------------------BEGIN REFRESH GLOBAL POSE---------------------------//
    Current_CARPOSITION_GLOBAL.X_POSITION=X_AIMPOSITION;
    Current_CARPOSITION_GLOBAL.Y_POSITION=Y_AIMPOSITION;
    Current_CARPOSITION_GLOBAL.Posture=Aim_Posture;
    //------------------------END REFRESH GLOBAL POSE----------------------------//
    MOTOR_Stop();
}

HAL_StatusTypeDef Error_compensation(Mission_Code CODE)
{
    switch (CODE)
    {
        case UnStacking_Correction:
            Send_MissionPack(UnStacking_Correction,Green);
            ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_ReleaseFull,700,500);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CarMove_TO(Current_CARPOSITION_GLOBAL.X_POSITION+rk3588_Arry[4]*(rk3588_Arry[2]==0?1:-1),Current_CARPOSITION_GLOBAL.Y_POSITION+rk3588_Arry[5]*(rk3588_Arry[3]==0?1:-1),Current_CARPOSITION_GLOBAL.Posture,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition-100,Claw_ReleaseFull,700,500);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CarMove_TO(Current_CARPOSITION_GLOBAL.X_POSITION+rk3588_Arry[4]*(rk3588_Arry[2]==0?1:-1),Current_CARPOSITION_GLOBAL.Y_POSITION+rk3588_Arry[5]*(rk3588_Arry[3]==0?1:-1),Current_CARPOSITION_GLOBAL.Posture,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            return HAL_OK;
        case Angle_compensation:
            Send_MissionPack(Angle_compensation,Green);
        case Raw_Material_Compensation:
            Send_MissionPack(Raw_Material_Compensation,Green);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CarMove_TO(Current_CARPOSITION_GLOBAL.X_POSITION+rk3588_Arry[4]*(rk3588_Arry[2]==0?1:-1),Current_CARPOSITION_GLOBAL.Y_POSITION+rk3588_Arry[5]*(rk3588_Arry[3]==0?1:-1),Current_CARPOSITION_GLOBAL.Posture,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            return HAL_OK;
        default:
            return HAL_OK;
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM10)
    {
        //Calculation of the speed of each motor in the X direction.
        switch (NotUp_To_MaxSpeed_FlagX)
        {
            case Yes:
                if(TIM_CNT<StartEnd_CostCNTX)
                {
                    Motor1_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor2_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor3_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor4_SpeedX=(TIM_CNT*Motor_Acceleration);
                }
                else if(TIM_CNT>(Total_CNTX-StartEnd_CostCNTX)&&TIM_CNT<Total_CNTX)
                {
                    Motor1_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor2_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor3_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor4_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                }
                break;
            case No:
                if(TIM_CNT<StartEnd_CostCNTX)
                {
                    Motor1_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor2_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor3_SpeedX=(TIM_CNT*Motor_Acceleration);
                    Motor4_SpeedX=(TIM_CNT*Motor_Acceleration);
                }
                else if(TIM_CNT>(Total_CNTX-StartEnd_CostCNTX)&&TIM_CNT<Total_CNTX)
                {
                    Motor1_SpeedX=Maxium_Speed_ActualX-((Total_CNTX-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor2_SpeedX=Maxium_Speed_ActualX-((Total_CNTX-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor3_SpeedX=Maxium_Speed_ActualX-((Total_CNTX-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                    Motor4_SpeedX=Maxium_Speed_ActualX-((Total_CNTX-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
                }
                else if(TIM_CNT>Total_CNTX)
                {
                    Motor1_SpeedX=0;
                    Motor2_SpeedX=0;
                    Motor3_SpeedX=0;
                    Motor4_SpeedX=0;
                }
                else
                {
                    Motor1_SpeedX=(Maxium_Speed_ActualX);
                    Motor2_SpeedX=(Maxium_Speed_ActualX);
                    Motor3_SpeedX=(Maxium_Speed_ActualX);
                    Motor4_SpeedX=(Maxium_Speed_ActualX);
                }
                break;
            default:
                break;
        }
        //Calculation of the speed of each motor in the Y direction.
        switch (NotUp_To_MaxSpeed_FlagY)
        {
            case Yes:
                if(TIM_CNT<StartEnd_CostCNTY)
                {
                    Motor1_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor2_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor3_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor4_SpeedY=(TIM_CNT*Motor_Acceleration);
                }
                else if(TIM_CNT>(Total_CNTY-StartEnd_CostCNTY)&&TIM_CNT<Total_CNTY)
                {
                    Motor1_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor2_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor3_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor4_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                }
                break;
            case No:
                if(TIM_CNT<StartEnd_CostCNTY)
                {
                    Motor1_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor2_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor3_SpeedY=(TIM_CNT*Motor_Acceleration);
                    Motor4_SpeedY=(TIM_CNT*Motor_Acceleration);
                }
                else if(TIM_CNT>(Total_CNTY-StartEnd_CostCNTY)&&TIM_CNT<Total_CNTY)
                {
                    Motor1_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor2_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor3_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                    Motor4_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
                }
                else if(TIM_CNT>Total_CNTY)
								{
									Motor1_SpeedY=0;
									Motor2_SpeedY=0;
									Motor3_SpeedY=0;
									Motor4_SpeedY=0;
								}
								else 
                {
                    Motor1_SpeedY=(Maxium_Speed_ActualY);
                    Motor2_SpeedY=(Maxium_Speed_ActualY);
                    Motor3_SpeedY=(Maxium_Speed_ActualY);
                    Motor4_SpeedY=(Maxium_Speed_ActualY);
                }
                break;
            default:
                break;
        }
        //Accumulate all motor speeds
        Motor1_Speed=-OverAll_PostureX*Motor1_SpeedX+OverAll_PostureY*Motor1_SpeedY;
        Motor2_Speed=OverAll_PostureX*Motor2_SpeedX+OverAll_PostureY*Motor2_SpeedY;
        Motor3_Speed=OverAll_PostureX*Motor3_SpeedX+OverAll_PostureY*Motor3_SpeedY;
        Motor4_Speed=-OverAll_PostureX*Motor4_SpeedX+OverAll_PostureY*Motor4_SpeedY;
        //PID control quantity input
        Motor2_Speed-=Err_Angle*Motor2_P+Err_Angle_Past*Motor2_D;
        Motor3_Speed+=Err_Angle*Motor3_P+Err_Angle_Past*Motor3_D;
        //Calculate the direction of rotation of the motor
        Motor1_Direction=Motor1_Speed>=0?Motor_Forward:Motor_Backward;
        Motor2_Direction=Motor2_Speed>=0?Motor_Forward:Motor_Backward;
        Motor3_Direction=Motor3_Speed>=0?Motor_Forward:Motor_Backward;
        Motor4_Direction=Motor4_Speed>=0?Motor_Forward:Motor_Backward;
        //Motor assignment
        Motor_SpeedControl_UART(Motor1,Motor1_Direction,(uint32_t)fabs(Motor1_Speed)*10,0);
        Motor_SpeedControl_UART(Motor2,Motor2_Direction,(uint32_t)fabs(Motor2_Speed)*10,0);
        Motor_SpeedControl_UART(Motor3,Motor3_Direction,(uint32_t)fabs(Motor3_Speed)*10,0);
        Motor_SpeedControl_UART(Motor4,Motor4_Direction,(uint32_t)fabs(Motor4_Speed)*10,0);
        MOTOR_SendMultipleStart();
		TIM_CNT++;
        if(TIM_CNT>=Key_CNT)
        {
            // HAL_Delay(200);
            MOTOR_Stop();
            MOTOR_SendMultipleStart();
            TIM_CNT=0;
            If_Motor_Busy=No;
            HAL_TIM_Base_Stop_IT(&htim10);
        }
        
    }
}
/**
 * @brief Just to VOID_FUNCTION
 *  
 */
HAL_StatusTypeDef VOID_FUNCTION(float AIM_XPosition,float AIM_YPosition,float AIM_ZPosition,Claw_Status Claw_Control,uint32_t GoTO_Delay_Time ,uint32_t Claw_Delay_Time){return HAL_OK;}

