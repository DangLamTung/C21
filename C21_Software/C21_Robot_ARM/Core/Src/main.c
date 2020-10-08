/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t a[6];
float temp[6];
uint8_t buf[12];
uint16_t value,value1, value2;
int count = 0;

uint8_t pin_selected = 0;
uint8_t calib1,calib2;

const float l0 = 78;
const float l1 = 104;
const float l2 = 98;
const float l3 = 96;
const float l4 = 11;

//
uint16_t convert_angle(float angle){
    uint16_t temp = 0;
    temp = (uint16_t) ((angle/45.0)*500 + 1500);
    return temp;
}

typedef struct{
	float theta0;
	float theta1;
	float theta2;
	float theta3;
	float theta4;
	float theta5;
}theta_angle;
theta_angle inverse_kinematic(float x, float y, float z){
	theta_angle temp;
	float d;
	d = sqrt(x*x + y*y + (z-l0)*(z-l0));
	temp.theta0 = atan(x/y)*180/3.14159;
	temp.theta1 = 90 - (acos((l1*l1 + d*d - l2*l2)/(2*l1*d))  + atan((z - l0)/sqrt(x*x+y*y)))*180/3.14159;
	temp.theta2 = -(((acos((l1*l1 - d*d + l2*l2)/(2*l1*l2)))*180/3.14159));
	temp.theta3 = -temp.theta2 - (acos((l1*l1 + d*d - l2*l2)/(2*l1*d))  + atan((z - l0)/d))*180/3.14159;
    return temp;
}
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);



  HAL_ADC_Start_DMA(&hadc1,a,4);
   float temp1,temp2;
   for(int i = 0 ; i < 100; i++){
       temp1 += floor(a[0]/4096.0*255);
       temp2 += floor(a[1]/4096.0*255);
       HAL_Delay(2);
   }
   temp1/=100;
   temp2/=100;

   calib1 = (uint8_t) temp1;
   calib2 = (uint8_t) temp2;

   uint16_t theta[6];
   for(int i = 0; i<6; i++){
	   theta[i] = 1500;
   }
   theta_angle angle;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//     for(float i = 78; i< 160;i+=0.1){
//	  angle = inverse_kinematic(70,70,i);
//
//	  		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, convert_angle( angle.theta1));
//	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle( angle.theta2));
//	  		 HAL_Delay(5);
//     }
//
//     for(float i = 160; i> 78;i-=0.1){
//     	  angle = inverse_kinematic(70,70,i);
//
//     	  		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, convert_angle( angle.theta1));
//     	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle( angle.theta2));
//     	  		 HAL_Delay(5);
//          }
//	  		  angle = inverse_kinematic(70,70,120);
//
//	  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, convert_angle( angle.theta1));
//	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle( angle.theta2));
//	  		  HAL_Delay(1000);
	  //	  }
	  //
	  //	  for(float angle = 45; angle > -45; angle -=0.1){
	  //		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle(angle));

//	  for(float angle = -45; angle < 45; angle +=0.1){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle( angle));
//		  HAL_Delay(10);
//	  }
//
//	  for(float angle = 45; angle > -45; angle -=0.1){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, convert_angle(angle));
//		  HAL_Delay(10);
//	  }
	  if(a[0] < 1000){
		  if(theta[3] > 1000){
	      theta[3]--;
//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);

		  }


	  }

	  if(a[0] > 3000){
	 		  if(theta[3] < 2000){
	 	      theta[3]++;
	 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
	 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
//	 	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, theta[5]);
	 		  }


	 	  }

	  if(a[1] < 1000){
			  if(theta[4] > 1000){
		      theta[4]--;
	//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
	//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);

			  }


		  }

		  if(a[1] > 3000){
		 		  if(theta[4] < 2000){
		 	      theta[4]++;
		 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
		 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
	//	 	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, theta[5]);
		 		  }


		 	  }

		  if(a[3] < 1000){
					  if(theta[2] > 1000){
				      theta[2]--;
			//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
			//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);

					  }


				  }

				  if(a[3] > 3000){
				 		  if(theta[2] < 2000){
				 	      theta[2]++;
				 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
				 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
			//	 	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, theta[5]);
				 		  }


		 	  }

				  if(a[2] < 1000){
							  if(theta[1] > 800){
						      theta[1]--;
					//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
					//	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);

							  }


						  }

						  if(a[2] > 3000){
						 		  if(theta[1] < 2200){
						 	      theta[1]++;
						 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
						 //	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
					//	 	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, theta[5]);
						 		  }


				 	  }
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, theta[2]);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, theta[1]);
//	  for(uint16_t i = 1000; i < 1100; i++){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(5);
//	  }
//
//	  for(uint16_t i = 1100; i < 1400; i++){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(2);
//	  }
//
//	  for(uint16_t i = 1400; i < 1500; i++){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(5);
//	  }
//
//	  for(uint16_t i = 1500; i > 1400; i--){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(5);
//	  }
//
//	  for(uint16_t i = 1400; i > 1100; i--){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(2);
//	  }
//
//	  for(uint16_t i = 1100; i > 1000; i--){
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
//		  HAL_Delay(5);
//	  }

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, theta[3]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, theta[4]);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, theta[5]);
      HAL_Delay(5);
//	  for(int i = 2200; i>800;i-- ){
////	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
////	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
//	     HAL_Delay(2);
//	  }

  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
