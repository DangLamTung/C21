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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "st7735.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "back_ground.h"
#include "bird.h"
#include "gameover.h"
#include "font_score.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t pin_selected;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//FATFS fs ;
//FATFS  * pfs ;
//FIL fil ;
//FRESULT fres ;
//DWORD fre_clust ;
int i = 0;
uint8_t sound = 0;
uint8_t sound_on =0;
int  printRandoms(int lower, int upper)
{
        int num;
        num = (rand() % (upper - lower + 1)) + lower;
        return num;
}
//
int col_array[3];
int col_pos[3];

void print_score(uint8_t score){
	switch(score){
	case 0:
		ST7735_DrawImage(10, 58, 8, 8, &zero);
	break;
	case 1:
		ST7735_DrawImage(10, 58, 8, 8, &one);
	break;
	case 2:
		ST7735_DrawImage(10, 58, 8, 8, &two);
	break;
	case 3:
		ST7735_DrawImage(10, 58, 8, 8, &three);
	break;
	case 4:
		ST7735_DrawImage(10, 58, 8, 8, &four);
	break;
	case 5:
		ST7735_DrawImage(10, 58, 8, 8, &five);
	break;
	case 6:
		ST7735_DrawImage(10, 58, 8, 8, &six);
	break;
	case 7:
		ST7735_DrawImage(10,58, 8, 8, &seven);
	break;
	case 8:
		ST7735_DrawImage(10, 58, 8, 8, &eight);
	break;
	case 9:
		ST7735_DrawImage(10, 58, 8, 8, &nine);
	break;
	}
}

void draw_bird(uint16_t x, uint16_t y, const uint16_t* bird){
	uint16_t buffer[15][15];
	if(x<128){
	for(int i = 0; i < 15; i++){
		for(int j = 0; j < 15; j++){
			buffer[i][j] = bird[i*15 + j] ;
			if( 6>i || i> 10){
                if(buffer[i*15 + j] == 0xFFFF){
                      buffer[i][j] = ground[i + y][j+x];
                }


			}

			if( 6<j || j< 10){
			                            if(buffer[i][j] == 0xFFFF){
			                                  buffer[i][j] = ground[i][j+x];
			                            }
			            			}

			}
	}
	}
	ST7735_DrawImage(i, 0, 15, 15, &buffer);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();

  ST7735_DrawImage(0, 0, 128, 128, &ground);


  int j = 128;

//  fres = f_mount ( &fs ,  "" ,   1);
//   while ( fres !=  FR_OK ){
// 	  fres = f_mount ( &fs ,  "" ,   1);
// 	  HAL_Delay(100);
//   }
//  FIL file;
//      FRESULT res = f_open(&file, "rose.bmp", FA_READ);
//      if(res != FR_OK) {
////          UART_Printf("f_open() failed, res = %d\r\n", res);
//          return -1;
//      }
//
//  unsigned int bytesRead;
//  uint8_t header[34];
//  res = f_read(&file, header, sizeof(header), &bytesRead);
//  if(res != FR_OK) {
////      UART_Printf("f_read() failed, res = %d\r\n", res);
////      f_close(&file);
//      return -2;
//  }
//
////  if((header[0] != 137) || (header[1] != 80)) {
//////      UART_Printf("Wrong BMP signature: 0x%02X 0x%02X\r\n", header[0], header[1]);
//////      f_close(&file);
////      return -3;
////  }
//
//  uint32_t imageOffset = header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
//  uint32_t imageWidth = header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24);
//  uint32_t imageHeight = header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24);
//  uint16_t imagePlanes = header[26] | (header[27] << 8);
//  uint16_t imageBitsPerPixel = header[28] | (header[29] << 8);
//  uint32_t imageCompression = header[30] | (header[31] << 8) | (header[32] << 16) | (header[33] << 24);
//
//
//  if((imageWidth != ST7735_WIDTH) || (imageHeight != ST7735_HEIGHT)) {
////      UART_Printf("Wrong BMP size, %dx%d expected\r\n", ST7735_WIDTH, ST7735_HEIGHT);
////      f_close(&file);
////      return -4;
//  }

//  if((imagePlanes != 1) || (imageBitsPerPixel != 24) || (imageCompression != 0)) {
////      UART_Printf("Unsupported image format\r\n");
////      f_close(&file);
////      return -5;
//  }
//
//  res = f_lseek(&file, imageOffset);
//  if(res != FR_OK) {
////      UART_Printf("f_lseek() failed, res = %d\r\n", res);
////      f_close(&file);
////      return -6;
//  }

//  // row size is aligned to 4 bytes
//  uint8_t imageRow[(ST7735_WIDTH * 3 + 3) & ~3];
//  for(uint32_t y = 0; y < imageHeight; y++) {
//      uint32_t rowIdx = 0;
//      res = f_read(&file, imageRow, sizeof(imageRow), &bytesRead);
//      if(res != FR_OK) {
////          UART_Printf("f_read() failed, res = %d\r\n", res);
//          f_close(&file);
//          return -7;
//      }
//
//      for(uint32_t x = 0; x < imageWidth; x++) {
//          uint8_t b = imageRow[rowIdx++];
//          uint8_t g = imageRow[rowIdx++];
//          uint8_t r = imageRow[rowIdx++];
//          uint16_t color565 = (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3));
//          ST7735_DrawPixel(x, imageHeight - y - 1, color565);
//      }
//  }

//  res = f_close(&file);
  int col_height = 30;


  col_array[0] = printRandoms(30, 100);
  col_array[1] = printRandoms(30, 100);
  col_pos[0] = 128;
  col_pos[1] = 128;
  col_pos[2] = 128;
  uint8_t game_over = 0;
  volatile uint8_t at_the_end = 0;
  volatile uint8_t score = 0;

  HAL_TIM_Base_Start_IT(&htim3);
   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100);
   __HAL_TIM_SET_AUTORELOAD(&htim2,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(!game_over){
	   uint16_t * ptr;
	   uint16_t * ptr1;
	   uint16_t * ptr2;
//	   ST7735_WriteString(0, 0, "Score:", Font_11x18, ST7735_GREEN, ST7735_BLACK);
		  ptr = (uint16_t *) calloc(1280, sizeof(uint16_t));
		  for(int a = 0; a < 10; a++){
			  for(int b = 0; b < 128; b++){
				  ptr[a*128 + b] = ground[col_pos[0] +a][b];
			  }
		  }


		  ptr1 = (uint16_t *) calloc(1280, sizeof(uint16_t));
				  for(int a = 0; a < 10; a++){
					  for(int b = 0; b < 128; b++){
						  ptr1[a*128 + b] = ground[col_pos[1] +a][b];
					  }
				  }

				  ptr2 = (uint16_t *) calloc(1280, sizeof(uint16_t));
				 		  for(int a = 0; a < 10; a++){
				 			  for(int b = 0; b < 128; b++){
				 				  ptr2[a*128 + b] = ground[col_pos[2] +a][b];
				 			  }
				 		  }
//	  ptr1 = (uint16_t *) calloc(780, sizeof(uint16_t));

	  for(int a = 0; a < 15; a++){
		  for(int b = 0; b < 15; b++){
			  buffer[a][b] = ground[a][b + i];
		  }
	  }
//


      if(i < 108){
    	  i+=2;
      }
      else{
    	  ST7735_DrawImage(50, 0, 33, 128, &over);
    	  game_over = 1;
      }




      if(col_pos[0] > 0){
    	  if((col_pos[0] == 128)){
    		  if(at_the_end == 1){
    		  score++;
    		  at_the_end = 0;
    		  sound_on = 1;
    		  }
    	  }
    	  col_pos[0]--;

      }
      else{
    	  j = 128;
    	  col_height = printRandoms(0, 100);
    	  col_pos[0] = 128;
    	  col_array[0] = printRandoms(0, 100);
      }
      if(col_pos[1] < 0 ){
    	  col_pos[1] = 128;
    	  col_array[1] = printRandoms(0, 100);
      }
      else{
    	  if(col_pos[1] != 128){
     	     col_pos[1] --;
    	  }
    	  else{
             	if(at_the_end == 1){
               		score++;
               		at_the_end = 0;
               		sound_on = 1;
               	}

             if(col_pos[0] < 88){
            	 col_pos[1] --;
             }
    	  }
      }
      if(col_pos[2] < 0){
    	  col_pos[2] = 128;
    	  col_array[2] = printRandoms(0, 100);
      }
      else{
          	  if(col_pos[2] != 128){
          		  col_pos[2] --;
          	  }

          	   else{

             	if(at_the_end == 1){
             		score++;
             		at_the_end = 0;
             		sound_on = 1;
             	}

          	      if(col_pos[1] < 88){
          	         col_pos[2] --;
          	           }
          	    }
            }
      if(j > 0){
    	  j--;
      }
      else{
    	  j = 128;
    	  col_height = printRandoms(0, 100);
      }
//      for(uint8_t check; check < 3; check ++){
          if((col_pos[0] < 15) && (col_pos[0] > 0)){
        	  if( (col_array[0] > i) || ((col_array[0]+10) < i )){
        		  ST7735_DrawImage(0, 0, 128, 128, &ground);
        	      ST7735_DrawImage(50, 0, 33, 128, &over);
        	      game_over = 1;
        	      break;
        	  }
        	   at_the_end = 1;
          }

          if((col_pos[1] < 15) && (col_pos[1] > 0)){
        	  if( (col_array[1] > i) || ((col_array[1]+10) < i )){
             		 ST7735_DrawImage(0, 0, 128, 128, &ground);
             	     ST7735_DrawImage(50, 0, 33, 128, &over);
             	    game_over = 1;
             	   break;
             	  }
             	 at_the_end = 1;
               }

          if((col_pos[2] < 15) && (col_pos[2] > 0)){
                  	  if( (col_array[2] > i) || ((col_array[2]+10) < i )){
             		 ST7735_DrawImage(0, 0, 128, 128, &ground);
             	     ST7735_DrawImage(50, 0, 33, 128, &over);
             	    game_over = 1;
             	    break;
             	  }
             	 at_the_end = 1;
               }
//      }
          draw_bird(i, 0, &bird);
//	  ST7735_DrawImage(i, 0, 15, 15, &bird);

	  ST7735_FillRectangle(0, col_pos[0], col_array[0] , 10, ST7735_GREEN);
	  ST7735_FillRectangle(col_array[0] + 20,col_pos[0], 88 - col_array[0] , 10, ST7735_GREEN);


	  if(col_pos[1] < 128){
		  ST7735_FillRectangle(0, col_pos[1], col_array[1] , 10, ST7735_GREEN);
		  ST7735_FillRectangle(col_array[1] + 20,col_pos[1], 88 - col_array[1] , 10, ST7735_GREEN);
	  }

	  if(col_pos[2] < 128){
	 		  ST7735_FillRectangle(0, col_pos[2], col_array[2] , 10, ST7735_GREEN);
	 		  ST7735_FillRectangle(col_array[2] + 20,col_pos[2], 88 - col_array[2] , 10, ST7735_GREEN);
	 	  }

//	  ST7735_FillRectangle(0, j, col_height , 10, ST7735_GREEN);
//	  ST7735_FillRectangle(col_height + 20, j, 88 - col_height , 10, ST7735_GREEN);

	  HAL_Delay(100);
	  ST7735_DrawImage(i, 0, 15, 15, &buffer);
	  ST7735_DrawImage(0, col_pos[0], 128, 10, ptr);
	  ST7735_DrawImage(0, col_pos[1], 128, 10, ptr1);
	  ST7735_DrawImage(0, col_pos[2], 128, 10, ptr2);

	  print_score(score);
//	  ST7735_DrawImage(50, j, 78, 10, ptr1);
	  free(ptr);
	  free(ptr1);
	  free(ptr2);
	  }

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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 12500;
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
  htim2.Init.Period = 1632;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 12500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Period = 2000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 SD_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  if(pin_selected == 3 ){
  	if(HAL_GPIO_ReadPin(GPIOC, 8192) == GPIO_PIN_SET){
//  		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
    	  for(int a = 0; a < 15; a++){
    			  for(int b = 0; b < 15; b++){
    				  buffer[a][b] = ground[a][b + i];
    			  }
    		  }
    	ST7735_DrawImage(i, 0, 15, 15, &buffer);

  		i -= 5;


  		HAL_TIM_Base_Stop_IT(&htim4);
  	}

  	pin_selected = 0;
    }
    if(pin_selected == 4 ){
  	if(HAL_GPIO_ReadPin(GPIOC, 16384) == GPIO_PIN_SET){

  		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);

  	  for(int a = 0; a < 15; a++){
  	    			  for(int b = 0; b < 15; b++){
  	    				  buffer[a][b] = ground[a][b + i];
  	    			  }
  	    		  }
  	    	ST7735_DrawImage(i, 0, 15, 15, &buffer);


  		i -= 10;
  		HAL_TIM_Base_Stop_IT(&htim4);
  	}

  	pin_selected = 0;
    }
    if(pin_selected == 4 ){
  	if(HAL_GPIO_ReadPin(GPIOC,  32768) == GPIO_PIN_SET){

//  		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);

  		HAL_TIM_Base_Stop_IT(&htim4);
  	}

  	pin_selected = 0;
    }

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
