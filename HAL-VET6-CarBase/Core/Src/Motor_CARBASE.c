#include "Motor_CARBASE.h"

#if SpecialONOFF==0
/* USER CODE BEGIN PD */
    
/* USER CODE END PD */

/* USER CODE BEGIN PV */
    //Define Key Value
    #define TURNED_ANGLEPLUSE 4350
    #define offsetsX 3.20f
	#define offsetsY 2.90f

    const float Motor_P=3,Motor_D=0.8;
    const float TimerCost=(1679+1)*(9999+1)/168000000; //0.1s
    const float Maxium_Speed_Set = 4000; // mm/Maxium_Speed_Set
    float Motor_Acceleration = 4.0f * PI * 75.0f / 60.0f ; //mm/TimerCost 

    struct Car_Pose QR_CARPOSITION={160,500,Car_Posture_Up};
    struct Car_Pose RawMaterial_CARPOSITION={160,1350,Car_Posture_Right};
    struct Car_Pose RoughProcess_CARPOSITION={1780,960,Car_Posture_Down};
    struct Car_Pose Temporary_CARPOSITION={900,1650,Car_Posture_Left};

    struct Car_Pose Start_CARPOSITION_Second={0,0,Car_Posture_Up};
    struct Car_Pose RawMaterial_CARPOSITION_Second={155,1250,Car_Posture_Right};
    struct Car_Pose RoughProcess_CARPOSITION_Second={1780,960,Car_Posture_Down};
    struct Car_Pose Temporary_CARPOSITION_Second={900,1650,Car_Posture_Left};

    //Definition of data type
    struct Car_Pose Current_CARPOSITION_GLOBAL={0,0,Car_Posture_Up};
    struct Car_Pose None_CARPOSITION={0,0,Car_Posture_Up};

    extern const ROBOTICArm_Pose Relay_point,Place_Scaning;

    //Define Stored global variables
    int32_t StartEnd_CostCNTX=0,StartEnd_CostCNTY=0,SpeedHold_CNTX=0,SpeedHold_CNTY=0,TIM_CNT=0,Total_CNTX=0,Total_CNTY=0,Key_CNT=0;
    Variable_Flag If_Motor_Busy=No;
    Motor_DIRECTION Motor1_Direction = Motor_Forward,Motor2_Direction = Motor_Forward,Motor3_Direction = Motor_Forward,Motor4_Direction = Motor_Forward;
    float Maxium_Speed_ActualX = 0 , Maxium_Speed_ActualY=0;
    float Motor1_Speed=0, Motor2_Speed=0, Motor3_Speed=0, Motor4_Speed=0,
		Motor1_PIDSpeed=0,Motor2_PIDSpeed=0,Motor3_PIDSpeed=0,Motor4_PIDSpeed=0,
        Motor1_SpeedX=0, Motor2_SpeedX=0, Motor3_SpeedX=0, Motor4_SpeedX=0,
        Motor1_SpeedY=0, Motor2_SpeedY=0, Motor3_SpeedY=0, Motor4_SpeedY=0;
    int8_t OverAll_PostureX,OverAll_PostureY,Motor1_PID_Direction,Motor2_PID_Direction,Motor3_PID_Direction,Motor4_PID_Direction;
/* USER CODE END PV */
void TURN(Car_Posture Aim_CARPOSITON_RELATIVE,Car_Posture Current_CARPOSITION_RELATIVE)
{
    float TURNED_ANGLE;uint8_t ArrivePack[4]={0x00,0x00,0x00,0x00};
    int8_t DeltaNum=Aim_CARPOSITON_RELATIVE-Current_CARPOSITION_RELATIVE;
    switch (abs(DeltaNum))
    {
    case 3:
    TURNED_ANGLE=DeltaNum>0?-90:90;
        break;
    case 2:
    TURNED_ANGLE=DeltaNum<0?-180:180;
        break;
    case 1:
    TURNED_ANGLE=DeltaNum<0?-90:90;
        break;
    default:
        break;
    }
    Motor_PositionControl_UART(Motor1,TURNED_ANGLE>=0?Motor_Backward:Motor_Forward,1000,0x10,fabs(TURNED_ANGLE)/90*TURNED_ANGLEPLUSE);
    Motor_PositionControl_UART(Motor2,TURNED_ANGLE>=0?Motor_Forward:Motor_Backward,1000,0x10,fabs(TURNED_ANGLE)/90*TURNED_ANGLEPLUSE);
    Motor_PositionControl_UART(Motor3,TURNED_ANGLE>=0?Motor_Backward:Motor_Forward,1000,0x10,fabs(TURNED_ANGLE)/90*TURNED_ANGLEPLUSE);
    Motor_PositionControl_UART(Motor4,TURNED_ANGLE>=0?Motor_Forward:Motor_Backward,1000,0x10,fabs(TURNED_ANGLE)/90*TURNED_ANGLEPLUSE);
    MOTOR_SendMultipleStart();
    while(HAL_UART_Receive(&huart1,ArrivePack,4,0xFFFF)!=HAL_OK){Gyroscopes_ARRAY_Handle();}
    HAL_Delay(10);
    MOTOR_Stop();
    ArrivePack[2]=0;
    Target_angle+=TURNED_ANGLE;
}
Variable_Flag Raw_Material_Compensation_Flag=No,Error_Flag=No,X_Flag=No;
void CarMove_TO_Relative(struct Car_Pose Aim_CARPOSITON_RELATIVE,struct Car_Pose Current_CARPOSITION_RELATIVE , 
    HAL_StatusTypeDef Part_TimeJob(float Final_XPosition,float Final_YPosition,float Final_ZPosition))
{
    //Calculate the actual operating distance.
    float X_Distance = offsetsX*fabs(Aim_CARPOSITON_RELATIVE.X_POSITION - Current_CARPOSITION_RELATIVE.X_POSITION);
    float Y_Distance = offsetsY*fabs(Aim_CARPOSITON_RELATIVE.Y_POSITION - Current_CARPOSITION_RELATIVE.Y_POSITION);

    if(X_Distance<=0.1f)X_Distance=0;
    else if(X_Distance<=1.0f)X_Distance=1.0f;
    if(Y_Distance<=0.1f)Y_Distance=0;
    else if(Y_Distance<=1.0f)Y_Distance=1.0f;

    //Determine the direction of the vehicle's operation.
    OverAll_PostureX=(Aim_CARPOSITON_RELATIVE.X_POSITION - Current_CARPOSITION_RELATIVE.X_POSITION) >= 0 ? 1:-1;
    OverAll_PostureY=(Aim_CARPOSITON_RELATIVE.Y_POSITION - Current_CARPOSITION_RELATIVE.Y_POSITION) >= 0 ? 1:-1;

    if(X_Distance==0)
    {
        X_Flag=No;
        Motor1_PID_Direction=1;
        Motor2_PID_Direction=-1;
        Motor3_PID_Direction=1;
        Motor4_PID_Direction=-1;
    }
    else if(Y_Distance==0)
    {
        X_Flag=Yes;
        Motor1_PID_Direction=-1;
        Motor2_PID_Direction=-1;
        Motor3_PID_Direction=1;
        Motor4_PID_Direction=1;
    }

    //Calculate the maximum speed of the entire vehicle and the count value of special points.
    if(Maxium_Speed_Set*Maxium_Speed_Set/(Motor_Acceleration)>X_Distance) //Indicates that the distance required for acceleration is greater than the target distance at this point
        {Maxium_Speed_ActualX = sqrt(X_Distance * (Motor_Acceleration));SpeedHold_CNTX=0;}
    else {Maxium_Speed_ActualX = Maxium_Speed_Set;SpeedHold_CNTX=X_Distance-(Maxium_Speed_Set*Maxium_Speed_Set)/Motor_Acceleration;}
    StartEnd_CostCNTX=Maxium_Speed_ActualX / Motor_Acceleration;

    if(Maxium_Speed_Set*Maxium_Speed_Set/(Motor_Acceleration)>Y_Distance) //Indicates that the distance required for acceleration is greater than the target distance at this point
        {Maxium_Speed_ActualY = sqrt(Y_Distance * (Motor_Acceleration));SpeedHold_CNTY=0;}
    else {Maxium_Speed_ActualY = Maxium_Speed_Set;SpeedHold_CNTY=Y_Distance-(Maxium_Speed_Set*Maxium_Speed_Set)/Motor_Acceleration;}
    StartEnd_CostCNTY=Maxium_Speed_ActualY / Motor_Acceleration;

    Total_CNTX=StartEnd_CostCNTX*2+SpeedHold_CNTX;
    Total_CNTY=StartEnd_CostCNTY*2+SpeedHold_CNTY;
    
    //Determine the stopping count value.
    Key_CNT=Total_CNTX>=Total_CNTY?Total_CNTX:Total_CNTY;
    /*Now we have the CNTs needed for acceleration and deceleration, and the CNTs needed to maintain speed if any*/

    //The setting of the flag and the timer start to control the motor.
    If_Motor_Busy=Yes;
    HAL_TIM_Base_Start_IT(&htim10);

    //I call it "Part_timeJob" , But this feature has quite a lot of limitations
    while(If_Motor_Busy==Yes)
    {
        Gyroscopes_ARRAY_Handle();
        // Send_Number(angle,3);
        /*
        // //Calculation of the speed of each motor in the X direction.
        // if(TIM_CNT<StartEnd_CostCNTX)
        // {
        //     Motor1_SpeedX=(TIM_CNT*Motor_Acceleration);
        //     Motor2_SpeedX=(TIM_CNT*Motor_Acceleration);
        //     Motor3_SpeedX=(TIM_CNT*Motor_Acceleration);
        //     Motor4_SpeedX=(TIM_CNT*Motor_Acceleration);
        // }
        // else if(TIM_CNT>(Total_CNTX-StartEnd_CostCNTX)&&TIM_CNT<Total_CNTX)
        // {
        //     Motor1_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
        //     Motor2_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
        //     Motor3_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
        //     Motor4_SpeedX=Maxium_Speed_ActualX-((TIM_CNT-(Total_CNTX-StartEnd_CostCNTX))*Motor_Acceleration);
        // }
        // else if(TIM_CNT>Total_CNTX)
        // {
        //     Motor1_SpeedX=0;
        //     Motor2_SpeedX=0;
        //     Motor3_SpeedX=0;
        //     Motor4_SpeedX=0;
        // }
        // else
        // {
        //     Motor1_SpeedX=(Maxium_Speed_ActualX);
        //     Motor2_SpeedX=(Maxium_Speed_ActualX);
        //     Motor3_SpeedX=(Maxium_Speed_ActualX);
        //     Motor4_SpeedX=(Maxium_Speed_ActualX);
        // }
        // //Calculation of the speed of each motor in the Y direction.
        // if(TIM_CNT<StartEnd_CostCNTY)
        // {
        //     Motor1_SpeedY=(TIM_CNT*Motor_Acceleration);
        //     Motor2_SpeedY=(TIM_CNT*Motor_Acceleration);
        //     Motor3_SpeedY=(TIM_CNT*Motor_Acceleration);
        //     Motor4_SpeedY=(TIM_CNT*Motor_Acceleration);
        // }
        // else if(TIM_CNT>(Total_CNTY-StartEnd_CostCNTY)&&TIM_CNT<Total_CNTY)
        // {
        //     Motor1_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
        //     Motor2_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
        //     Motor3_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
        //     Motor4_SpeedY=Maxium_Speed_ActualY-((TIM_CNT-(Total_CNTY-StartEnd_CostCNTY))*Motor_Acceleration);
        // }
        // else if(TIM_CNT>Total_CNTY)
        // {
        //     Motor1_SpeedY=0;
        //     Motor2_SpeedY=0;
        //     Motor3_SpeedY=0;
        //     Motor4_SpeedY=0;
        // }
        // else 
        // {
        //     Motor1_SpeedY=(Maxium_Speed_ActualY);
        //     Motor2_SpeedY=(Maxium_Speed_ActualY);
        //     Motor3_SpeedY=(Maxium_Speed_ActualY);
        //     Motor4_SpeedY=(Maxium_Speed_ActualY);
        // }
        // //Accumulate all motor speeds
        // Motor1_Speed=-OverAll_PostureX*Motor1_SpeedX+OverAll_PostureY*Motor1_SpeedY;
        // Motor2_Speed=OverAll_PostureX*Motor2_SpeedX+OverAll_PostureY*Motor2_SpeedY;
        // Motor3_Speed=OverAll_PostureX*Motor3_SpeedX+OverAll_PostureY*Motor3_SpeedY;
        // Motor4_Speed=-OverAll_PostureX*Motor4_SpeedX+OverAll_PostureY*Motor4_SpeedY;
        // //PID control quantity input
        // if(Raw_Material_Compensation_Flag==Yes||(Error_Flag!=Yes&&X_Flag==Yes)){Motor1_PID_Direction=0,Motor2_PID_Direction=0,Motor3_PID_Direction=0,Motor4_PID_Direction=0;}
        // Motor1_PIDSpeed=Motor1_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor1_PID_Direction;
        // Motor2_PIDSpeed=Motor2_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor2_PID_Direction;
        // Motor3_PIDSpeed=Motor3_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor3_PID_Direction;
        // Motor4_PIDSpeed=Motor4_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor4_PID_Direction;
        // //Calculate the direction of rotation of the motor
        // Motor1_Direction=Motor1_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        // Motor2_Direction=Motor2_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        // Motor3_Direction=Motor3_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        // Motor4_Direction=Motor4_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        // //Motor assignment
        // Motor_SpeedControl_UART(Motor1,Motor1_Direction,(fabs(Motor1_PIDSpeed)*10),0);
        // Motor_SpeedControl_UART(Motor2,Motor2_Direction,(fabs(Motor2_PIDSpeed)*10),0);
        // Motor_SpeedControl_UART(Motor3,Motor3_Direction,(fabs(Motor3_PIDSpeed)*10),0);
        // Motor_SpeedControl_UART(Motor4,Motor4_Direction,(fabs(Motor4_PIDSpeed)*10),0);
        // MOTOR_SendMultipleStart();
        
        // if(TIM_CNT>=Key_CNT)
        // {
        //     MOTOR_Stop();
        //     TIM_CNT=0;
        //     If_Motor_Busy=No;
        //     HAL_TIM_Base_Stop_IT(&htim10);
        // }*/
    }
    LED_ONOFF(LED1,1);
    //The vehicle XYmovement has ended.
    HAL_Delay(100);
    MOTOR_Stop();
    HAL_Delay(500);

    //The change in the angle of the entire vehicle.
    if(Current_CARPOSITION_RELATIVE.Posture!=Aim_CARPOSITON_RELATIVE.Posture)
    {
        TURN(Aim_CARPOSITON_RELATIVE.Posture,Current_CARPOSITION_RELATIVE.Posture);
    }
    LED_ONOFF(LED1,0);
    MOTOR_Stop();
}
/**
 * @brief Move the vehicle to the corresponding world coordinate point.
 * 
 * @param X_AIMPOSITION world coordinate X_AIMPOSITION.
 * @param Y_AIMPOSITION world coordinate Y_AIMPOSITION.
 * @param Aim_Posture CarPosture
 * @param function part_timeJob
 */
void CarMove_TO_Global(float X_AIMPOSITION,float Y_AIMPOSITION,Car_Posture Aim_Posture,HAL_StatusTypeDef Part_TimeJob(float Final_XPosition,float Final_YPosition,float Final_ZPosition))
{
    struct Car_Pose Current_CARPOSITION_RELATIVE , Aim_CARPOSITON_RELATIVE , Aim_CARPOSITON_GLOBAL;
    Aim_CARPOSITON_GLOBAL.X_POSITION=X_AIMPOSITION;
    Aim_CARPOSITON_GLOBAL.Y_POSITION=Y_AIMPOSITION;
    Aim_CARPOSITON_GLOBAL.Posture=Aim_Posture;
    /**
     * @param Car_Posture_Up: x2 = x1 ; y2 = y1
     * @param Car_Posture_Left: x3 = -y1 ; y3 = x1
     * @param Car_Posture_Down: x4 = -x1 ; y4 = -y1
     * @param Car_Posture_Right: x5 = y1 ; y5 = -x1
     */
    //Coordinate transformation 
    switch(Current_CARPOSITION_GLOBAL.Posture)
    {
        case Car_Posture_Up:
            Current_CARPOSITION_RELATIVE.X_POSITION=Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON_RELATIVE.X_POSITION=Aim_CARPOSITON_GLOBAL.X_POSITION;
            Aim_CARPOSITON_RELATIVE.Y_POSITION=Aim_CARPOSITON_GLOBAL.Y_POSITION;
            Aim_CARPOSITON_RELATIVE.Posture=Aim_Posture;
            break;
        case Car_Posture_Left:
            Current_CARPOSITION_RELATIVE.X_POSITION=-Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON_RELATIVE.X_POSITION=-Aim_CARPOSITON_GLOBAL.Y_POSITION;
            Aim_CARPOSITON_RELATIVE.Y_POSITION=Aim_CARPOSITON_GLOBAL.X_POSITION;
            Aim_CARPOSITON_RELATIVE.Posture=Aim_Posture;
            break;
        case Car_Posture_Down:
            Current_CARPOSITION_RELATIVE.X_POSITION=-Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=-Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON_RELATIVE.X_POSITION=-Aim_CARPOSITON_GLOBAL.X_POSITION;
            Aim_CARPOSITON_RELATIVE.Y_POSITION=-Aim_CARPOSITON_GLOBAL.Y_POSITION;
            Aim_CARPOSITON_RELATIVE.Posture=Aim_Posture;
            break;
        case Car_Posture_Right:
            Current_CARPOSITION_RELATIVE.X_POSITION=Current_CARPOSITION_GLOBAL.Y_POSITION;
            Current_CARPOSITION_RELATIVE.Y_POSITION=-Current_CARPOSITION_GLOBAL.X_POSITION;
            Current_CARPOSITION_RELATIVE.Posture=Current_CARPOSITION_GLOBAL.Posture;
            Aim_CARPOSITON_RELATIVE.X_POSITION=Aim_CARPOSITON_GLOBAL.Y_POSITION;
            Aim_CARPOSITON_RELATIVE.Y_POSITION=-Aim_CARPOSITON_GLOBAL.X_POSITION;
            Aim_CARPOSITON_RELATIVE.Posture=Aim_Posture;
            break;
        default:break;
    }
    
    //REFRESH GLOBAL POSE
    Current_CARPOSITION_GLOBAL=Aim_CARPOSITON_GLOBAL;

    //CarMove_TO_Relative Pose
    CarMove_TO_Relative(Aim_CARPOSITON_RELATIVE,Current_CARPOSITION_RELATIVE,Part_TimeJob);
}

HAL_StatusTypeDef Error_compensation(Mission_Code CODE)
{
    struct Car_Pose CompensationAim_Pose={0,0,Current_CARPOSITION_GLOBAL.Posture};
    switch (CODE)
    {
        case UnStacking_Correction:/*____________________________________________________________________________________*/
            Motor_Acceleration=0.5f * PI * 75.0f / 60.0f;
            Send_MissionPack(UnStacking_Correction,Blue);
            Error_Flag=Yes;
            ROBOTICArm_DirectlyMove(Place_Scaning.XPosition,Place_Scaning.YPosition,Place_Scaning.ZPosition,Claw_ReleaseFull,700,500);
            rk3588_Arry[6]=0x00;
            LED_ONOFF(LED1,1);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CompensationAim_Pose.X_POSITION=rk3588_Arry[2]*(rk3588_Arry[4]==0?1:-1)*0.9;
                    CompensationAim_Pose.Y_POSITION=None_CARPOSITION.Y_POSITION;
                    CompensationAim_Pose.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    None_CARPOSITION.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    CompensationAim_Pose.X_POSITION=None_CARPOSITION.X_POSITION;
                    CompensationAim_Pose.Y_POSITION=rk3588_Arry[3]*(rk3588_Arry[5]==1?1:-1);
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            LED_ONOFF(LED1,0);
            rk3588_Arry[6]=0x00;
            ROBOTICArm_linearInterpolationAlgorithm(Place_Scaning.XPosition,Place_Scaning.YPosition,Place_Scaning.ZPosition-85,Claw_ReleaseFull,1);
            while (rk3588_Arry[6]!=0x01)
            {
                LED_ONOFF(LED1,1);
                HAL_Delay(100);
                Send_MissionPack(UnStacking_Correction,Blue);
            }
            LED_ONOFF(LED1,0);
            rk3588_Arry[6]=0x00;
            LED_ONOFF(LED1,1);
            Motor_Acceleration=0.1f * PI * 75.0f / 60.0f;
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CompensationAim_Pose.X_POSITION=rk3588_Arry[2]*(rk3588_Arry[4]==0?1:-1)*0.6;
                    CompensationAim_Pose.Y_POSITION=None_CARPOSITION.Y_POSITION;
                    CompensationAim_Pose.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    None_CARPOSITION.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    CompensationAim_Pose.X_POSITION=None_CARPOSITION.X_POSITION;
                    CompensationAim_Pose.Y_POSITION=rk3588_Arry[3]*(rk3588_Arry[5]==1?1:-1)*0.7;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            LED_ONOFF(LED1,0);
            Send_MissionPack(9,Blue);
            Motor_Acceleration = 4.0f * PI * 75.0f / 60.0f ; //mm/TimerCost 
            Error_Flag=No;
            return HAL_OK;
        case Stacking_Correction:/*____________________________________________________________________________________*/
            Send_MissionPack(Stacking_Correction,Green);
            Error_Flag=Yes;
            Motor_Acceleration=0.5f * PI * 75.0f / 60.0f;
            ROBOTICArm_DirectlyMove(Place_Scaning.XPosition,Place_Scaning.YPosition,Place_Scaning.ZPosition,Claw_ReleaseFull,700,500);
            rk3588_Arry[6]=0x00;
            LED_ONOFF(LED1,1);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
                    CompensationAim_Pose.X_POSITION=rk3588_Arry[2]*(rk3588_Arry[4]==0?1:-1)*0.6;
                    CompensationAim_Pose.Y_POSITION=None_CARPOSITION.Y_POSITION;
                    CompensationAim_Pose.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    None_CARPOSITION.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    CompensationAim_Pose.X_POSITION=None_CARPOSITION.X_POSITION;
                    CompensationAim_Pose.Y_POSITION=rk3588_Arry[3]*(rk3588_Arry[5]==1?1:-1)*0.7;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                }
            }
            LED_ONOFF(LED1,0);
            Motor_Acceleration = 4.0f * PI * 75.0f / 60.0f ; //mm/TimerCost 
            Error_Flag=No;
            return HAL_OK;
        case Angle_compensation:/*____________________________________________________________________________________*/
            Send_MissionPack(Angle_compensation,Green);
            ROBOTICArm_DirectlyMove(Place_Scaning.XPosition,Place_Scaning.YPosition,Place_Scaning.ZPosition,Claw_ReleaseFull,700,500);
            rk3588_Arry[6]=0x00;
            LED_ONOFF(LED1,1);
            while (rk3588_Arry[5]!=0x03)
            { 
                LED_ONOFF(LED2,1);
                if(rk3588_Arry[5]==0x01)
                {
                    Motor_PositionControl_UART(Motor1,rk3588_Arry[4] == 0 ? Motor_Forward : Motor_Backward,20,0,15);
                    Motor_PositionControl_UART(Motor2,rk3588_Arry[4] == 0 ? Motor_Backward : Motor_Forward,20,0,15);
                    Motor_PositionControl_UART(Motor3,rk3588_Arry[4] == 0 ? Motor_Forward : Motor_Backward,20,0,15);
                    Motor_PositionControl_UART(Motor4,rk3588_Arry[4] == 0 ? Motor_Backward : Motor_Forward,20,0,15);
                    MOTOR_SendMultipleStart();
                    LED_ONOFF(LED2,0);
                    rk3588_Arry[5]=0x00;
                }
            }
            MOTOR_Stop();
            LED_ONOFF(LED2,0);
            LED_ONOFF(LED1,0);
            Gyroscopes_ARRAY_Handle();
            Target_angle = angle;
            return HAL_OK;
        case Raw_Material_Compensation:/*____________________________________________________________________________________*/
            Send_MissionPack(Raw_Material_Compensation,1);
            Raw_Material_Compensation_Flag=Yes;
            ROBOTICArm_DirectlyMove(Raw_Material_Scanning.XPosition,Raw_Material_Scanning.YPosition,Raw_Material_Scanning.ZPosition,Claw_ReleaseFull,1,1);
            Motor_Acceleration=0.5f * PI * 75.0f / 60.0f;
            rk3588_Arry[6]=0x00;
            LED_ONOFF(LED1,1);
            while (rk3588_Arry[6]!=0x03)
            {
                if(rk3588_Arry[6]==0x01)
                {
					LED_ONOFF(LED1,1);
                    Send_MissionPack(Raw_Material_Compensation,0);
                    CompensationAim_Pose.X_POSITION=rk3588_Arry[3]*(rk3588_Arry[5]==1?1:-1)*0.4;
                    CompensationAim_Pose.Y_POSITION=None_CARPOSITION.Y_POSITION;
                    CompensationAim_Pose.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    None_CARPOSITION.Posture=Current_CARPOSITION_GLOBAL.Posture;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    CompensationAim_Pose.X_POSITION=None_CARPOSITION.X_POSITION;
                    CompensationAim_Pose.Y_POSITION=rk3588_Arry[2]*(rk3588_Arry[4]==0?-1: 1)*0.5;
                    CarMove_TO_Relative(None_CARPOSITION,CompensationAim_Pose,VOID_FUNCTION);
                    rk3588_Arry[6]=0x00;
                    Send_MissionPack(Raw_Material_Compensation,1);
                }
            }
            Send_MissionPack(9,Blue);
            LED_ONOFF(LED1,0);
            Raw_Material_Compensation_Flag=No;
            Motor_Acceleration = 4.0f * PI * 75.0f / 60.0f ; //mm/TimerCost 
            return HAL_OK;
        default:/*____________________________________________________________________________________*/
            Send_MissionPack(9,Blue);
            Motor_Acceleration = 4.0f * PI * 75.0f / 60.0f ; //mm/TimerCost 
            return HAL_OK;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM10)
    {
        //Calculation of the speed of each motor in the X direction.
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
        //Calculation of the speed of each motor in the Y direction.
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
        //Accumulate all motor speeds
        Motor1_Speed=-OverAll_PostureX*Motor1_SpeedX+OverAll_PostureY*Motor1_SpeedY;
        Motor2_Speed=OverAll_PostureX*Motor2_SpeedX+OverAll_PostureY*Motor2_SpeedY;
        Motor3_Speed=OverAll_PostureX*Motor3_SpeedX+OverAll_PostureY*Motor3_SpeedY;
        Motor4_Speed=-OverAll_PostureX*Motor4_SpeedX+OverAll_PostureY*Motor4_SpeedY;
        //PID control quantity input
        if(Raw_Material_Compensation_Flag==Yes||(Error_Flag!=Yes&&X_Flag==Yes)){Motor1_PID_Direction=0,Motor2_PID_Direction=0,Motor3_PID_Direction=0,Motor4_PID_Direction=0;}
		Motor1_PIDSpeed=Motor1_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor1_PID_Direction;
        Motor2_PIDSpeed=Motor2_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor2_PID_Direction;
        Motor3_PIDSpeed=Motor3_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor3_PID_Direction;
		Motor4_PIDSpeed=Motor4_Speed+(Err_Angle*Motor_P+(Err_Angle-Err_Angle_Past)*Motor_D)*Motor4_PID_Direction;
        //Calculate the direction of rotation of the motor
        Motor1_Direction=Motor1_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        Motor2_Direction=Motor2_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        Motor3_Direction=Motor3_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        Motor4_Direction=Motor4_PIDSpeed>=0?Motor_Forward:Motor_Backward;
        //Motor assignment
        Motor_SpeedControl_UART(Motor1,Motor1_Direction,(fabs(Motor1_PIDSpeed)*10),0);
        Motor_SpeedControl_UART(Motor2,Motor2_Direction,(fabs(Motor2_PIDSpeed)*10),0);
        Motor_SpeedControl_UART(Motor3,Motor3_Direction,(fabs(Motor3_PIDSpeed)*10),0);
        Motor_SpeedControl_UART(Motor4,Motor4_Direction,(fabs(Motor4_PIDSpeed)*10),0);
        MOTOR_SendMultipleStart();
		
        if(TIM_CNT>=Key_CNT)
        {
            MOTOR_Stop();
            TIM_CNT=0;
            If_Motor_Busy=No;
            HAL_TIM_Base_Stop_IT(&htim10);
        }
        TIM_CNT++;
    }
}
/**
 * @brief Just to VOID_FUNCTION
 *  
 */
HAL_StatusTypeDef VOID_FUNCTION(float AIM_XPosition,float AIM_YPosition,float AIM_ZPosition){return HAL_OK;}
#endif 
