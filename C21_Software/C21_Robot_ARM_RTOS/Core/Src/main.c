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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId control_servoHandle;
osThreadId update_lcdHandle;
/* USER CODE BEGIN PV */
FATFS fs ;
FATFS  * pfs ;
FIL fil ;
FRESULT fres ;
DWORD fre_clust ;
//uint32_t total , free ;
char buffer [ 100 ] ;

uint16_t joystick_value[6];
uint8_t calib1,calib2;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void update_state(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void turn_on(){
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

}
void set_home(){

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);
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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
   ST7735_Init();
   ST7735_FillScreen(ST7735_WHITE);
   int i = 0;

   fres = f_mount ( &fs ,  "" ,   1);
      while ( fres !=  FR_OK ){
    	  fres = f_mount ( &fs ,  "" ,   1);
    	  HAL_Delay(100);
      }

      HAL_ADC_Start_DMA(&hadc1,joystick_value,4);
         float temp1,temp2;
         for(int i = 0 ; i < 100; i++){
             temp1 += floor(joystick_value[1]/4096.0*255);
             temp2 += floor(joystick_value[2]/4096.0*255);
             HAL_Delay(2);
         }
         temp1/=100;
         temp2/=100;

         calib1 = (uint8_t) temp1;
         calib2 = (uint8_t) temp2;

//   ST7735_DrawImage(0, 0, 128, 128, &back_ground);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of control_servo */
  osThreadDef(control_servo, StartDefaultTask, osPriorityRealtime, 0, 128);
  control_servoHandle = osThreadCreate(osThread(control_servo), NULL);

  /* definition and creation of update_lcd */
  osThreadDef(update_lcd, update_state, osPriorityNormal, 0, 128);
  update_lcdHandle = osThreadCreate(osThread(update_lcd), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);
    osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_update_state */
/**
* @brief Function implementing the update_lcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_update_state */
void update_state(void const * argument)
{
  /* USER CODE BEGIN update_state */
  /* Infinite loop */
  for(;;)
  {
//	  uint8_t buffer[] = "Hello, World!\r\n";
//	  CDC_Transmit_FS(buffer, sizeof(buffer));

	  ST7735_FillRectangle(0, 30, 10, 40, ST7735_BLUE);
    osDelay(100);
  }
  /* USER CODE END update_state */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
