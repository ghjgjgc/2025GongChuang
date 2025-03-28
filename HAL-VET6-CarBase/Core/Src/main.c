/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor_CARBASE.h"
#include "ROBOTIC_Arm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t DEBUG_USART[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Color Mission_Order[6]={Green,Blue,Red,Blue,Green,Red};
Color Color_Order[3]={Blue,Green,Red};                 //颜色顺序
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Raw_Material(void);
void Rough_processing(void);
void Temporary_storage(void);
void Position_Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define Rotation_direction  -1 //逆时针-1顺时针1全夹0
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LED_ONOFF(LED_ALL,1);
  HAL_Delay(1000);
  OLED_Init();
  ROBOTICArm_initialize();
  OLED_Clear();
  enum Mission_Current {None,Test,QR_First,Raw_Material_First,Rough_processing_First,Temporary_storage_First,QR_Second,Raw_Material_Second,End,Rough_processing_Second,Temporary_storage_Second};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LED_ONOFF(LED_ALL,0);
  ////////////////////////Test
  enum Mission_Current Mission_Select=QR_First;
  ///////////////////////QR_First
  while (1)
  {
		switch (Mission_Select)
		{
			case Test:
        LED_ONOFF(LED3,1);//while(1);
        /*_____________CARBASE_____________*/
        // TURN(Car_Posture_Right,Car_Posture_Up);
        // CarMove_TO_Global(RoughProcess_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
        // CarMove_TO_Global(Temporary_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
        // CarMove_TO_Global(0,10,Car_Posture_Left,VOID_FUNCTION);
        // HAL_Delay(500);
        // Error_compensation(Raw_Material_Compensation); 
        // Error_compensation(Angle_compensation);
        // Error_compensation(UnStacking_Correction);
        // while (1)
        // {
        //   Gyroscopes_ARRAY_Handle();
        //   HAL_Delay(100);
        // }
        // Position_Test();
        /*_____________ROBOTICARM_____________*/
        // ROBOTICArm_DirectlyMove(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition,Claw_ReleaseFull,1,500);
        // ROBOTICArm_DirectlyMove(Raw_Material_Scanning.XPosition-110,Raw_Material_Scanning.YPosition,Raw_Material_Scanning.ZPosition+10,Claw_ReleaseFull,200,1000);
        // ROBOTICArm_DirectlyMove(Raw_Material_ClawFront.XPosition,Raw_Material_ClawFront.YPosition,Raw_Material_ClawFront.ZPosition,Claw_ReleaseFull,200,1000);
        // ROBOTICArm_DirectlyMove(Raw_Material_ClawLeft.XPosition+20,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition+50,Claw_ReleaseFull,200,1);
        // SERVO_ClawSet(Claw_ReleaseFull);         
        /*_____________MISSION_____________*/
        // while(rk3588_Arry[1]==0x67)HAL_Delay(10);
        // LED_ONOFF(LED1,0);
        // for (uint8_t i = 0; i < 6; i++)Mission_Order[i]=(Color)(rk3588_Arry[i+2]-1);
        Raw_Material();
        // Rough_processing();
        // Temporary_storage();
        // HAL_Delay(100);
        // Temporary_storage();
        /*_____________OTHER_____________*/ 
        OLED_SHOWBIGNUM();
        LED_ONOFF(LED3,0);
        Mission_Select=None;
        break;  
      /*-------------------------------------------------------------------------------------------------------------------------------------------*/
			case QR_First:
        //Start and scanning QR code
        CarMove_TO_Global(QR_CARPOSITION.X_POSITION,Start_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
        CarMove_TO_Global(QR_CARPOSITION.X_POSITION,QR_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
        LED_ONOFF(LED1,1);
        while(rk3588_Arry[1]==0x67)HAL_Delay(10);
        LED_ONOFF(LED1,0);
        for (uint8_t i = 0; i < 6; i++)Mission_Order[i]=(Color)(rk3588_Arry[i+2]-1);
        OLED_SHOWBIGNUM();
			case Raw_Material_First:
        //Raw_Material
        CarMove_TO_Global(RawMaterial_CARPOSITION.X_POSITION,QR_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
        CarMove_TO_Global(RawMaterial_CARPOSITION.X_POSITION,RawMaterial_CARPOSITION.Y_POSITION,RawMaterial_CARPOSITION.Posture,VOID_FUNCTION);
				Raw_Material();
      case Rough_processing_First:
        //Rough processing area
        CarMove_TO_Global(RawMaterial_CARPOSITION.X_POSITION,RoughProcess_CARPOSITION.Y_POSITION,RawMaterial_CARPOSITION.Posture,VOID_FUNCTION);
        CarMove_TO_Global(RoughProcess_CARPOSITION.X_POSITION,RoughProcess_CARPOSITION.Y_POSITION,RoughProcess_CARPOSITION.Posture,VOID_FUNCTION);
        Rough_processing();
      case Temporary_storage_First:
        //Temporary_storage
        CarMove_TO_Global(RoughProcess_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
        CarMove_TO_Global(Temporary_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
        Temporary_storage();
        ROBOTICArm_initialize();
      /*-------------------------------------------------------------------------------------------------------------------------------------------*/
      case Raw_Material_Second: 
        //Raw_Material_Second
        CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Car_Posture_Up,ROBOTICArm_initialize);
        CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,RawMaterial_CARPOSITION_Second.Y_POSITION,RawMaterial_CARPOSITION_Second.Posture,VOID_FUNCTION);
        Raw_Material();
      case Rough_processing_Second:
        //Rough_processing_Second
        CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,RoughProcess_CARPOSITION_Second.Y_POSITION,RawMaterial_CARPOSITION_Second.Posture,VOID_FUNCTION);
        CarMove_TO_Global(RoughProcess_CARPOSITION_Second.X_POSITION,RoughProcess_CARPOSITION_Second.Y_POSITION,RoughProcess_CARPOSITION_Second.Posture,VOID_FUNCTION);
        Rough_processing();
      case Temporary_storage_Second:
        //Temporary_storage_Second
        CarMove_TO_Global(RoughProcess_CARPOSITION_Second.X_POSITION,Temporary_CARPOSITION_Second.Y_POSITION,Temporary_CARPOSITION_Second.Posture,VOID_FUNCTION);
        CarMove_TO_Global(Temporary_CARPOSITION_Second.X_POSITION,Temporary_CARPOSITION_Second.Y_POSITION,Temporary_CARPOSITION_Second.Posture,VOID_FUNCTION);
        Temporary_storage();
        ROBOTICArm_initialize();
      case End:
        //End
        CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,Temporary_CARPOSITION_Second.Y_POSITION,Start_CARPOSITION_Second.Posture,VOID_FUNCTION);
        CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,Start_CARPOSITION_Second.Y_POSITION,Start_CARPOSITION_Second.Posture,VOID_FUNCTION);
        CarMove_TO_Global(Start_CARPOSITION_Second.X_POSITION,Start_CARPOSITION_Second.Y_POSITION,Start_CARPOSITION_Second.Posture,VOID_FUNCTION);
        Mission_Select=None;
			default:
        LED_ONOFF(LED_ALL,1);
				break;
		}
  }
  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*Go to the QR code and raw material area.*/
void Raw_Material(void)
{
  static uint8_t Raw_Material_CNT=0;
  uint8_t Forward_Color=Blue;
  uint8_t Forward_ColorNum=0,Mission_ColorNum=0;
  //Error compensation before clamping
  Error_compensation(Raw_Material_Compensation);
  LED_ONOFF(LED3,1);
  HAL_Delay(100);
  Send_MissionPack(Raw_Material_Mission,Red);
	rk3588_Arry[6]=0; 
  while(rk3588_Arry[6]!=0x03)HAL_Delay(10);
  rk3588_Arry[6]=0; 
  for (uint8_t i = 0; i < 3; i++)Color_Order[i]=(Color)(rk3588_Arry[i+2]);
  HAL_Delay(100);
  LED_ONOFF(LED3,0);
  //Three times clamping
  rk3588_Arry[6]=0x00;
  Variable_Flag Claw_=No;
  for (int8_t n = 0; n < 3;)
  {
    ROBOTICArm_DirectlyMove(Raw_Material_Scanning.XPosition,Raw_Material_Scanning.YPosition,Raw_Material_Scanning.ZPosition,Claw_ReleaseFull,1,1);
    //Send task and wait for feedback
    Forward_Color=7;
    rk3588_Arry[6]=0x00;
    Send_MissionPack(Raw_Material_Mission,Mission_Order[n+Raw_Material_CNT*3]);
    while (rk3588_Arry[6]!=0x03){LED_ONOFF(LED2,1);}LED_ONOFF(LED2,0);
    HAL_Delay(10);
    Forward_Color=rk3588_Arry[7];
    rk3588_Arry[6]=0x00;
    //Analysis sequence
    if(Mission_Order[n+Raw_Material_CNT*3]==Forward_Color)//就在面前
    {
      // ROBOTICArm_DirectlyMove(Raw_Material_ClawFront.XPosition-10,Raw_Material_ClawFront.YPosition-10,Raw_Material_ClawFront.ZPosition+100,Claw_ReleaseFull,200,1);
      #if Rotation_direction==-1
      ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawFront.XPosition-40,Raw_Material_ClawFront.YPosition+15,Raw_Material_ClawFront.ZPosition,Claw_Clawing,400);
      #elif Rotation_direction==1
      ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawFront.XPosition-40,Raw_Material_ClawFront.YPosition-15,Raw_Material_ClawFront.ZPosition,Claw_Clawing,200);
      #elif Rotation_direction==0
      ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawFront.XPosition,Raw_Material_ClawFront.YPosition,Raw_Material_ClawFront.ZPosition,Claw_Clawing,200);
      #endif
      ROBOTICArm_DirectlyMove(Raw_Material_ClawFront.XPosition,Raw_Material_ClawFront.YPosition,Raw_Material_ClawFront.ZPosition+100,Claw_Clawing,100,1);
      Claw_=Yes;
    }
    else 
    {
      //Obtain the order of the front color and target color positions
      for (uint8_t i = 0; i < 3; i++)
      {
        if(Color_Order[i]==Forward_Color)
        {
          Forward_ColorNum=i;break;
        }
      }
      for (uint8_t i = 0; i < 3; i++)
      {
        if(Color_Order[i]==Mission_Order[n+Raw_Material_CNT*3])
        {
          Mission_ColorNum=i;break;
        }
      }
      //面前的和任务所需的索引差值
      int8_t Delta_Num=((int8_t)Forward_ColorNum-(int8_t)Mission_ColorNum);
      if(abs(Delta_Num)==2)//不是相邻的
      {
        if(Delta_Num>0)
        {//面前的在靠后索引，所需的在靠前索引
          #if Rotation_direction==1||Rotation_direction==0
          ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawRight.XPosition+50,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition+10,Claw_ReleaseFull,1);
					ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawRight.XPosition,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition,Claw_Clawing,250);
          ROBOTICArm_DirectlyMove(Raw_Material_ClawRight.XPosition,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition+150,Claw_Clawing,50,1);
          Claw_=Yes;
          #endif
        }
        else
        {//面前的在靠前索引，所需的在靠后索引
          #if Rotation_direction==-1||Rotation_direction==0
          ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawLeft.XPosition+50,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition+10,Claw_ReleaseFull,1);
					ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawLeft.XPosition,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition,Claw_Clawing,250);
          ROBOTICArm_DirectlyMove(Raw_Material_ClawLeft.XPosition,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition+150,Claw_Clawing,50,1);
          Claw_=Yes;
          #endif
        }
      }
      else //相邻的
      {
        if(Delta_Num>0)
        {//面前的靠后，目标的靠前
          #if Rotation_direction==-1||Rotation_direction==0
					ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawLeft.XPosition+50,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition+10,Claw_ReleaseFull,1);
					ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawLeft.XPosition,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition,Claw_Clawing,250);
          ROBOTICArm_DirectlyMove(Raw_Material_ClawLeft.XPosition,Raw_Material_ClawLeft.YPosition,Raw_Material_ClawLeft.ZPosition+150,Claw_Clawing,50,1);
          Claw_=Yes;
          #endif
        }
        else
        {//面前的靠前，目标的靠后
          #if Rotation_direction==1||Rotation_direction==0
          ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawRight.XPosition+50,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition+10,Claw_ReleaseFull,1);
					ROBOTICArm_linearInterpolationAlgorithm(Raw_Material_ClawRight.XPosition,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition,Claw_Clawing,250);
          ROBOTICArm_DirectlyMove(Raw_Material_ClawRight.XPosition,Raw_Material_ClawRight.YPosition,Raw_Material_ClawRight.ZPosition+150,Claw_Clawing,50,1);
          Claw_=Yes;
          #endif
        }
      }
    }
    // ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_Clawing,100,1);
    //Store in warehouse
    if(Claw_==Yes)
    {
      Claw_=No;
      ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_Clawing,100,1);
      switch (Mission_Order[n+Raw_Material_CNT*3])
      {
      case Red:
        ROBOTICArm_DirectlyMove(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+90,Claw_Clawing,800,1);
        ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition,Claw_Release,150);
        ROBOTICArm_DirectlyMove(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+50,Claw_Release,100,1);
        break;
      case Green:
        ROBOTICArm_DirectlyMove(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+80,Claw_Clawing,600,1);
				ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition,Claw_Release,150);
        ROBOTICArm_DirectlyMove(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+50,Claw_Release,100,1);
        break;
      case Blue:
        ROBOTICArm_DirectlyMove(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+90,Claw_Clawing,500,1);
				ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition,Claw_Release,150);
        ROBOTICArm_DirectlyMove(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+90,Claw_Release,100,1);
        break;
      default:
        break;
      }
      n++;
    }
  }
  Raw_Material_CNT++;
  Send_MissionPack(9,1);
}
void Rough_processing(void)
{
  static uint8_t Rough_processing_CNT=0;
  if(Rough_processing_CNT==0)
  {
    Error_compensation(Angle_compensation);
  }
  Error_compensation(UnStacking_Correction);
  uint8_t n = 0;
  //claw from warehouse
  ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
  for (; n < 3;)
  {
    switch (Mission_Order[n+Rough_processing_CNT*3])
    {
    case Red:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+70,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+70,Claw_Clawing,1);
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+60,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition,Claw_Release,200);
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition+60,Claw_Release,1);
      break;
    case Green:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+70,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+70,Claw_Clawing,1);
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+70,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_PlacementLocation_Unstack.ZPosition+60,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_PlacementLocation_Unstack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_PlacementLocation_Unstack.ZPosition+60,Claw_Release,1);
      break;
    case Blue:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+70,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+70,Claw_Clawing,1);
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+70,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_PlacementLocation_Unstack.ZPosition+50,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_PlacementLocation_Unstack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_PlacementLocation_Unstack.ZPosition+50,Claw_Release,1);
      break;
    default:
      break;
    }
    ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+70,Claw_Release,1);
    //Goto Midpoint
    n++;
  }
  n=0;
  //claw form unstacking point
  for (; n < 3;)
  {
    switch (Mission_Order[n+Rough_processing_CNT*3])
    {
    case Red:
      //Claw from PacePoint
      ROBOTICArm_DirectlyMove(Red_PlacementLocation_Unstack.XPosition-5,Red_PlacementLocation_Unstack.YPosition-5,Red_Warehouse.ZPosition+75,Claw_ReleaseFull,300,1);
      ROBOTICArm_DirectlyMove(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition,Claw_Clawing,200,300);
      ROBOTICArm_DirectlyMove(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,100,1);
      //Goto WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition,Claw_Release,300);
      ROBOTICArm_DirectlyMove(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+75,Claw_Release,100,1);
      break;
    case Green:
      //Claw from PacePoint
      ROBOTICArm_DirectlyMove(Green_PlacementLocation_Unstack.XPosition+10,Green_PlacementLocation_Unstack.YPosition-10,Green_Warehouse.ZPosition+75,Claw_ReleaseFull,200,300);
      ROBOTICArm_DirectlyMove(Green_PlacementLocation_Unstack.XPosition+10,Green_PlacementLocation_Unstack.YPosition-10,Green_PlacementLocation_Unstack.ZPosition+65,Claw_ReleaseFull,1,200);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_PlacementLocation_Unstack.ZPosition,Claw_Clawing,300);
      //Goto Midpoint
      ROBOTICArm_DirectlyMove(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_Warehouse.ZPosition+75,Claw_Clawing,200,1);
      //Goto WareHouse
      ROBOTICArm_DirectlyMove(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+75,Claw_Clawing,400,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition,Claw_Release,300);
      ROBOTICArm_DirectlyMove(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+75,Claw_Release,100,1);
      break;
    case Blue:
      //Claw from PacePoint
      ROBOTICArm_DirectlyMove(Blue_PlacementLocation_Unstack.XPosition+13,Blue_PlacementLocation_Unstack.YPosition-18,Blue_Warehouse.ZPosition+75,Claw_Release,200,300);
      ROBOTICArm_DirectlyMove(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_PlacementLocation_Unstack.ZPosition,Claw_Clawing,200,200);
      //Goto Midpoint
      ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_Clawing,200,1);
      //Goto WareHouse
      ROBOTICArm_DirectlyMove(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+75,Claw_Clawing,300,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition,Claw_Release,300);
      ROBOTICArm_DirectlyMove(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+75,Claw_Release,100,1);
      break;
    default:
      break;
    }
    //Goto Midpoint
    ROBOTICArm_DirectlyMove(Relay_point.XPosition,Relay_point.YPosition,Relay_point.ZPosition,Claw_Release,200,1);
    n++;
  }
  Rough_processing_CNT++;
  Send_MissionPack(9,1);
}
void Temporary_storage(void)
{
  static uint8_t Temporary_storage_CNT = 0;
  if(Temporary_storage_CNT==0)
  {
    Error_compensation(Angle_compensation);
    Error_compensation(UnStacking_Correction);
  }
  else
  {
    Error_compensation(Angle_compensation);
    Error_compensation(Stacking_Correction);
  }
  uint8_t n = 0;
  //claw from warehouse
  for (; n < 3;)
  {
    ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
    switch (Mission_Order[n+Temporary_storage_CNT*3])
    {
    case Red:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+75,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Red_Warehouse.XPosition,Red_Warehouse.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
      if(Temporary_storage_CNT==0)
      {
        //Goto Midpoint
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
        //Goto PacePoint
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition,Claw_Release,200);
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_PlacementLocation_Unstack.ZPosition+100,Claw_Release,1);
      }
      else
      {
        //Goto Midpoint
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_stack.XPosition,Red_PlacementLocation_stack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
        //Goto PacePoint
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_stack.XPosition,Red_PlacementLocation_stack.YPosition,Red_PlacementLocation_stack.ZPosition,Claw_Release,200);
        ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_stack.XPosition,Red_PlacementLocation_stack.YPosition,Red_PlacementLocation_stack.ZPosition+50,Claw_Release,1);
      }
      break;
    case Green:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+75,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Green_Warehouse.XPosition,Green_Warehouse.YPosition,Green_Warehouse.ZPosition+75,Claw_Clawing,1);
      if(Temporary_storage_CNT==0)
      {
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Green_Warehouse.ZPosition+75,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_Warehouse.ZPosition+50,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_PlacementLocation_Unstack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_Unstack.XPosition,Green_PlacementLocation_Unstack.YPosition,Green_Warehouse.ZPosition+50,Claw_Release,1);
      }
      else
      {
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Red_Warehouse.ZPosition+75,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_stack.XPosition,Green_PlacementLocation_stack.YPosition,Green_Warehouse.ZPosition+50,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_stack.XPosition,Green_PlacementLocation_stack.YPosition,Green_PlacementLocation_stack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Green_PlacementLocation_stack.XPosition,Green_PlacementLocation_stack.YPosition,Green_Warehouse.ZPosition+50,Claw_Release,1);
      }
      break;
    case Blue:
      //Claw from WareHouse
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+75,Claw_Release,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition,Claw_Clawing,300);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_Warehouse.XPosition,Blue_Warehouse.YPosition,Blue_Warehouse.ZPosition+75,Claw_Clawing,1);
      if(Temporary_storage_CNT==0)
      {
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Blue_Warehouse.ZPosition+75,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_Warehouse.ZPosition+50,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_PlacementLocation_Unstack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_Unstack.XPosition,Blue_PlacementLocation_Unstack.YPosition,Blue_Warehouse.ZPosition+50,Claw_Release,1);
      }
      else
      {
      //Goto Midpoint
      ROBOTICArm_linearInterpolationAlgorithm(Red_PlacementLocation_Unstack.XPosition,Red_PlacementLocation_Unstack.YPosition,Blue_Warehouse.ZPosition+75,Claw_Clawing,1);
      //Goto PacePoint
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_stack.XPosition,Blue_PlacementLocation_stack.YPosition,Blue_Warehouse.ZPosition+50,Claw_Clawing,1);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_stack.XPosition,Blue_PlacementLocation_stack.YPosition,Blue_PlacementLocation_stack.ZPosition,Claw_Release,300);
      ROBOTICArm_linearInterpolationAlgorithm(Blue_PlacementLocation_stack.XPosition,Blue_PlacementLocation_stack.YPosition,Blue_Warehouse.ZPosition+50,Claw_Release,1);
      }
      break;
    default:
      break;
    }
    n++;
  }
  Temporary_storage_CNT++;
  Send_MissionPack(9,1);
}
void Position_Test(void)
{
  CarMove_TO_Global(QR_CARPOSITION.X_POSITION,Start_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
  CarMove_TO_Global(QR_CARPOSITION.X_POSITION,QR_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
  
  CarMove_TO_Global(RawMaterial_CARPOSITION.X_POSITION,RawMaterial_CARPOSITION.Y_POSITION,RawMaterial_CARPOSITION.Posture,VOID_FUNCTION);
  ROBOTICArm_DirectlyMove(Raw_Material_Scanning.XPosition,Raw_Material_Scanning.YPosition,Raw_Material_Scanning.ZPosition,Claw_ReleaseFull,700,500);
  Error_compensation(Raw_Material_Compensation);
  ROBOTICArm_initialize();
  
  CarMove_TO_Global(RawMaterial_CARPOSITION.X_POSITION,RoughProcess_CARPOSITION.Y_POSITION,RawMaterial_CARPOSITION.Posture,VOID_FUNCTION);
  CarMove_TO_Global(RoughProcess_CARPOSITION.X_POSITION,RoughProcess_CARPOSITION.Y_POSITION,RoughProcess_CARPOSITION.Posture,VOID_FUNCTION);
  Error_compensation(Angle_compensation);
  Error_compensation(UnStacking_Correction);
  
  CarMove_TO_Global(RoughProcess_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
  CarMove_TO_Global(Temporary_CARPOSITION.X_POSITION,Temporary_CARPOSITION.Y_POSITION,Temporary_CARPOSITION.Posture,VOID_FUNCTION);
  Error_compensation(Angle_compensation);
  Error_compensation(UnStacking_Correction);

  CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,Temporary_CARPOSITION.Y_POSITION,RawMaterial_CARPOSITION_Second.Posture,VOID_FUNCTION);
  CarMove_TO_Global(RawMaterial_CARPOSITION_Second.X_POSITION,RawMaterial_CARPOSITION_Second.Y_POSITION,RawMaterial_CARPOSITION_Second.Posture,VOID_FUNCTION);
  ROBOTICArm_DirectlyMove(Raw_Material_Scanning.XPosition,Raw_Material_Scanning.YPosition,Raw_Material_Scanning.ZPosition,Claw_ReleaseFull,700,500);
  Error_compensation(Raw_Material_Compensation);
  ROBOTICArm_initialize();

  CarMove_TO_Global(QR_CARPOSITION.X_POSITION,QR_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
  CarMove_TO_Global(Start_CARPOSITION.X_POSITION,Start_CARPOSITION.Y_POSITION,QR_CARPOSITION.Posture,VOID_FUNCTION);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  uint8_t ERROR_Positon[]={*file,line};
  HAL_UART_Transmit(&huart4,(const uint8_t*)ERROR_Positon,2,0xff);
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
