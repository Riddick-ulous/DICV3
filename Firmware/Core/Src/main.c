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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN.h"
#include "ERROR.h"
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "RGBLED.h"
#include "LOOP_TIMER.h"
#include "ADC.h"
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern volatile unsigned long systime;
CAN_Message_t CANmsg;
/*
unsigned long time_old_10ms = 0;
uint8_t sys_10ms_cycle_cplt = 0;
unsigned long time_old_100ms = 0;
uint8_t sys_100ms_cycle_cplt = 0;
unsigned long time_old_200ms = 0;
uint8_t sys_200ms_cycle_cplt = 0;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_TIM_Base_Start(&htim2);
  RGBLED_Init();
  ADC_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  ssd1306_Init();
  ssd1306_Fill(White);
  ssd1306_DrawPixel(2, 2, Black);
  ssd1306_DrawPixel(1, 1, Black);
  ssd1306_DrawPixel(0, 0, Black);
  ssd1306_UpdateScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Hello World", Font_6x8, White);
  ssd1306_UpdateScreen();


  HAL_GPIO_WritePin(GPIO1_12VOut_GPIO_Port,GPIO1_12VOut_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO1_5VOut_GPIO_Port,GPIO1_5VOut_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(ADC1_CTRL_GPIO_Port,ADC1_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADC1_NTC_CTRL_GPIO_Port,ADC1_NTC_CTRL_Pin, GPIO_PIN_RESET);

  // Loops anlegen + 0-init
  loop_timer_t loop_1ms   = {0};
  loop_timer_t loop_10ms  = {0};
  loop_timer_t loop_100ms = {0};
  loop_timer_t loop_200ms = {0};

  void init_loops(void)
  {
      loop_init(&loop_1ms,   1,   NULL);
      loop_init(&loop_10ms,  10,  &loop_1ms);
      loop_init(&loop_100ms, 100, &loop_10ms);
      loop_init(&loop_200ms, 200, &loop_100ms);
  }

  init_loops();

  while (1)
  {

	    uint32_t systime = HAL_GetTick();

	  	  // --- 1 ms Loop ---
	  	  if (loop_due(&loop_1ms, systime)) {
	  		  loop_start(&loop_1ms, systime);
	  		  // 1ms-Tasks hier
	  		  loop_end(&loop_1ms, ERROR_LOOP_OVERRUN_1MS);
	  	  }

	  	// --- 10 ms Loop ---
	  	    if (loop_due(&loop_10ms, systime)) {
	  	        loop_start(&loop_10ms, systime);


				// Senden der Loop usages
				// Nachricht vorbereiten
				CANmsg.id = 0x100;
				CANmsg.dlc = 8;
				CANmsg.data[0] = loop_1ms.max_prev_x10  & 0xFF; CANmsg.data[1] = loop_1ms.max_prev_x10  >> 8;
				CANmsg.data[2] = loop_10ms.max_prev_x10 & 0xFF; CANmsg.data[3] = loop_10ms.max_prev_x10 >> 8;
				CANmsg.data[4] = loop_100ms.max_prev_x10& 0xFF; CANmsg.data[5] = loop_100ms.max_prev_x10>> 8;
				CANmsg.data[6] = loop_200ms.max_prev_x10& 0xFF; CANmsg.data[7] = loop_200ms.max_prev_x10>> 8;

				if (CAN_QueueMessage(&CANmsg) != HAL_OK)
				{
				   Error_Register(ERROR_CAN_QUEUE_FULL);
				}

	  	        // ADC-Werte holen (gemittelt)
				uint16_t ch0 = ADC_GetAverage(0);
				uint16_t ch1 = ADC_GetAverage(1);
				uint16_t ch2 = ADC_GetAverage(2);
				uint16_t ch3 = ADC_GetAverage(3);

				// CAN-Nachricht vorbereiten
				CANmsg.id  = 0x101;   // frei wählen
				CANmsg.dlc = 8;
				CANmsg.data[0] = ch0 & 0xFF;
				CANmsg.data[1] = ch0 >> 8;
				CANmsg.data[2] = ch1 & 0xFF;
				CANmsg.data[3] = ch1 >> 8;
				CANmsg.data[4] = ch2 & 0xFF;
				CANmsg.data[5] = ch2 >> 8;
				CANmsg.data[6] = ch3 & 0xFF;
				CANmsg.data[7] = ch3 >> 8;

				if (CAN_QueueMessage(&CANmsg) != HAL_OK)
				{
				   Error_Register(ERROR_CAN_QUEUE_FULL);
				}

	  	      loop_end(&loop_10ms, ERROR_LOOP_OVERRUN_10MS);
	  	    }

	  	    // --- 100 ms Loop ---
	  	    if (loop_due(&loop_100ms, systime)) {
	  	        loop_start(&loop_100ms, systime);

			  ssd1306_SetCursor(0, 0);
			  ssd1306_WriteString("Hello World", Font_6x8, White);

			  RGBLED_TestPattern();
			  RGBLED_Update(&hspi2);

			  CAN_Message_t CANmsg;
			  CANmsg.id = 0x123;
			  CANmsg.dlc = 2;
			  CANmsg.data[0] = 0xAB;
			  CANmsg.data[1] = 0xCD;

			  if (CAN_QueueMessage(&CANmsg) != HAL_OK)
			  {
				   Error_Register(ERROR_CAN_QUEUE_FULL);
				   // Programm läuft regulär weiter
			  }

			  CAN_Message_t* received = CAN_GetMessage(0x321);
			  uint16_t value = 0;

			  if (received && received->dlc >= 2) {
			      value = (uint16_t)received->data[0] | ((uint16_t)received->data[1] << 8);
			  } else {
			      Error_Register(ERROR_CAN_RX_EMPTY);
			  }

			  char buffer[16];
			  snprintf(buffer, sizeof(buffer), "Wert: %u", value);

			  ssd1306_SetCursor(0, 10);
			  ssd1306_WriteString(buffer, Font_6x8, White);

			  Display_ErrorStatus();

			  ssd1306_UpdateScreen();

			  loop_end(&loop_100ms, ERROR_LOOP_OVERRUN_100MS);
		}

			// --- 200 ms Loop ---
			if (loop_due(&loop_200ms, systime)) {
				loop_start(&loop_200ms, systime);

	  		  HAL_GPIO_TogglePin(Heart_GPIO_Port,Heart_Pin);
	  		  //HAL_GPIO_TogglePin(Error_GPIO_Port,Error_Pin);

	  		loop_end(&loop_200ms, ERROR_LOOP_OVERRUN_200MS);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  // While(1) Klammer below
  }
  // Main Klammer below
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_Init();
  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Heart_Pin|Error_Pin|nRES_Display_Pin|GPIO3_12VOut_Pin
                          |GPIO2_5VOut_Pin|GPIO2_12VOut_Pin|GPIO6_5VOut_Pin|GPIO6_12VOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nCS_Display_Pin|D_nC_Display_Pin|GPIO1_5VOut_Pin|GPIO1_12VOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADC2_NTC_CTRL_Pin|ADC1_NTC_CTRL_Pin|ADC0_NTC_CTRL_Pin|ADC1_CTRL_Pin
                          |ADC2_CTRL_Pin|ADC3_CTRL_Pin|GPIO3_5VOut_Pin|GPIO5_5VOut_Pin
                          |GPIO5_12VOut_Pin|GPIO4_5VOut_Pin|GPIO4_12VOut_Pin|ADC3_NTC_CTRL_Pin
                          |ADC0_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Heart_Pin Error_Pin GPIO3_12VOut_Pin GPIO2_5VOut_Pin
                           GPIO2_12VOut_Pin GPIO6_5VOut_Pin GPIO6_12VOut_Pin */
  GPIO_InitStruct.Pin = Heart_Pin|Error_Pin|GPIO3_12VOut_Pin|GPIO2_5VOut_Pin
                          |GPIO2_12VOut_Pin|GPIO6_5VOut_Pin|GPIO6_12VOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : nCS_Display_Pin D_nC_Display_Pin */
  GPIO_InitStruct.Pin = nCS_Display_Pin|D_nC_Display_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nRES_Display_Pin */
  GPIO_InitStruct.Pin = nRES_Display_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRES_Display_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC2_NTC_CTRL_Pin ADC1_NTC_CTRL_Pin ADC0_NTC_CTRL_Pin ADC1_CTRL_Pin
                           ADC2_CTRL_Pin ADC3_CTRL_Pin GPIO3_5VOut_Pin GPIO5_5VOut_Pin
                           GPIO5_12VOut_Pin GPIO4_5VOut_Pin GPIO4_12VOut_Pin ADC3_NTC_CTRL_Pin
                           ADC0_CTRL_Pin */
  GPIO_InitStruct.Pin = ADC2_NTC_CTRL_Pin|ADC1_NTC_CTRL_Pin|ADC0_NTC_CTRL_Pin|ADC1_CTRL_Pin
                          |ADC2_CTRL_Pin|ADC3_CTRL_Pin|GPIO3_5VOut_Pin|GPIO5_5VOut_Pin
                          |GPIO5_12VOut_Pin|GPIO4_5VOut_Pin|GPIO4_12VOut_Pin|ADC3_NTC_CTRL_Pin
                          |ADC0_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO3_In_Pin GPIO4_In_Pin */
  GPIO_InitStruct.Pin = GPIO3_In_Pin|GPIO4_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO2_In_Pin GPIO6_In_Pin */
  GPIO_InitStruct.Pin = GPIO2_In_Pin|GPIO6_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_In_Pin */
  GPIO_InitStruct.Pin = GPIO1_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO1_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO1_5VOut_Pin GPIO1_12VOut_Pin */
  GPIO_InitStruct.Pin = GPIO1_5VOut_Pin|GPIO1_12VOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO5_In_Pin */
  GPIO_InitStruct.Pin = GPIO5_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO5_In_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
