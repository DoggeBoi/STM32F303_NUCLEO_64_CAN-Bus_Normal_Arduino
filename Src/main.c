/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t CAN_Tx_Mailboxes;

uint8_t TxData[8];
uint8_t RxData[8];

uint8_t str0[] = {0x01};
uint8_t str1[] = {0x02};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, &RxData[0]);
	//HAL_UART_Transmit(&huart2, (uint8_t *)str0, strlen((char*)str0), HAL_MAX_DELAY);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &RxHeader, &RxData[1]);
	//HAL_UART_Transmit(&huart2, (uint8_t *)str1, strlen((char*)str1), HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  TxHeader.DLC 					= 	1;					// Data length in bytes
  TxHeader.ExtId 				= 	0;					// Set extended id, not used, 0
  TxHeader.IDE 					= 	CAN_ID_STD;			// Use 11/29-bit identifier
  TxHeader.RTR 					= 	CAN_RTR_DATA;		// Send/request data
  TxHeader.StdId 				=	0x0211;				// Identifier, Priority 1 (Second highest, FIFO0), Device id 00001, operation 0001
  TxHeader.TransmitGlobalTime 	=	DISABLE;			// Time-stamp disable.

  TxData[0] 					= 	0xAA;

  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &CAN_Tx_Mailboxes);

  TxHeader.StdId 				=	0x0611;				// Same but with priority 4, lowest, goes to FIFO2
  TxData[0] 					= 	0x01;

  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &CAN_Tx_Mailboxes);


  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  /*	CAN FIFO Filter configuration declaration	*/
  CAN_FilterTypeDef CAN_Filterconfig_FIFO_0_High_Priority;
  CAN_FilterTypeDef CAN_Filterconfig_FIFO_1_Low_Priority;

  /*	https://www.st.com/resource/en/reference_manual/rm0316-stm32f303xbcde-stm32f303x68-stm32f328x8-stm32f358xc-stm32f398xe-advanced-armbased-mcus-stmicroelectronics.pdf page 1030	*/

  /*	CAN FIFO_0 Filter configuration		*/												// Shift 5 since identifier 11 bits, high register 16 bits
  CAN_Filterconfig_FIFO_0_High_Priority.FilterIdHigh 			= 	0x0010 << 5;			// Bit 0 must be 0 (High priority), bit 1 don't care (Both high priority),bit 2-6 must be 00001 (Device id 1), bit 7-10 don't care (Operation)
  CAN_Filterconfig_FIFO_0_High_Priority.FilterIdLow 			= 	0x0000;
  CAN_Filterconfig_FIFO_0_High_Priority.FilterMaskIdHigh 		= 	0x05F0 << 5;
  CAN_Filterconfig_FIFO_0_High_Priority.FilterMaskIdLow			= 	0x0000;
  CAN_Filterconfig_FIFO_0_High_Priority.FilterFIFOAssignment 	= 	CAN_FILTER_FIFO0;
  CAN_Filterconfig_FIFO_0_High_Priority.FilterBank 				= 	0;						// Selects used filter bank as 0 for FIFO0
  CAN_Filterconfig_FIFO_0_High_Priority.FilterMode 				= 	CAN_FILTERMODE_IDMASK;
  CAN_Filterconfig_FIFO_0_High_Priority.FilterScale 			= 	CAN_FILTERSCALE_32BIT;	// Use 32 bit, only need single identifier mask
  CAN_Filterconfig_FIFO_0_High_Priority.FilterActivation 		= 	CAN_FILTER_ENABLE;
  CAN_Filterconfig_FIFO_0_High_Priority.SlaveStartFilterBank 	= 	0;						// Unimportant, SMT32F3 has only one CAN BUS


  /*	CAN FIFO_0 Filter configuration		*/
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterIdHigh 			= 	0x0410 << 5;			// Bit 0 must be 1 (High priority), bit 1 don't care (Both high priority),bit 2-6 must be 00001 (Device id 1), bit 7-10 don't care (Operation)
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterIdLow 				= 	0x0000;
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterMaskIdHigh 		= 	0x05F0 << 5;
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterMaskIdLow 			= 	0x0000;
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterFIFOAssignment 	= 	CAN_FILTER_FIFO1;
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterBank 				= 	1;						// Selects used filter bank as 1 for FIFO1
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterMode 				= 	CAN_FILTERMODE_IDMASK;
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterScale 				= 	CAN_FILTERSCALE_32BIT;	// Use 32 bit, only need single identifier mask
  CAN_Filterconfig_FIFO_1_Low_Priority.FilterActivation 		= 	CAN_FILTER_ENABLE;
  CAN_Filterconfig_FIFO_1_Low_Priority.SlaveStartFilterBank 	= 	0;						// Unimportant, SMT32F3 has only one CAN BUS

  HAL_CAN_ConfigFilter(&hcan, &CAN_Filterconfig_FIFO_0_High_Priority);
  HAL_CAN_ConfigFilter(&hcan, &CAN_Filterconfig_FIFO_1_Low_Priority);

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
