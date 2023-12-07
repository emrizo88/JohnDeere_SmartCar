/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "myprintf.h"
#include "MY_NRF24.h"
#include "nRF24L01.h"
#include "string.h"
#include "math.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

double tOnN = 0.0015; //Neutral
double tONB = 0.001;
double period = 0.02;	//Period

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];
uint8_t TxData[] = {0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF};
uint8_t TxData2[] = {0xFF,0xFF,0xFF,0x00,0x11,0xFF,0xFF,0xFF};
//uint8_t TxData3[] = {0xFF,0xFF,0xFF,0x00,0x4B,0xFF,0xFF,0xFF};

uint8_t TxData4[] = {0x00,0x01,0xFF,0x00,0x01,0xFF,0xFF,0xFF};
uint8_t TxData5[] = {0x00,0x00,0xFF,0x00,0x01,0xFF,0xFF,0xFF};

//uint8_t TxData4[] = {0x01,0x01,0xFF,0x01,0x00,0xFF,0xFF,0xFF};
//uint8_t TxData5[] = {0x00,0x00,0xFF,0x00,0x01,0xFF,0xFF,0xFF};

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t wirelessTaskHandle;
const osThreadAttr_t wirelessTask_attributes = {
		.name = "wirelessTask",
		.stack_size = 128 *8,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
		.name = "servoTask",
		.stack_size = 128*4,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t escTaskHandle;
const osThreadAttr_t escTask_attributes = {
		.name = "escTask",
		.stack_size = 128*4,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
		.name = "canTask",
		.stack_size = 128*8,
		.priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t movimientoCamaraTaskHandle;
const osThreadAttr_t movimientoCamaraTask_attributes = {
		.name = "movimientoCamaraTask",
		.stack_size = 128*4,
		.priority = (osPriority) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void wirelessTask(void *argument);
void servoTask(void *argument);
void escTask(void *argument);
void canTask(void *argument);
void movimientoCamaraTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t RxpipeAddrs = 0x11223344AA;
uint8_t myRxData[100];
uint16_t flag = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM17_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  /* Configuración del Wireless */
  NRF24_begin(GPIOB, GPIO_PIN_2, GPIO_PIN_1, hspi1); // utilizar pines declarados para CE y CSN
  nrf24_DebugUART_Init(huart3);
  NRF24_setAutoAck(false);
  NRF24_setChannel(52);
  NRF24_setPayloadSize(32);
  NRF24_openReadingPipe(0, RxpipeAddrs);
  NRF24_enableDynamicPayloads();
  printRadioSettings();
  NRF24_startListening();
  /* Termina Configuración del Wireless */

//  int16_t accel_dataX;
//  int16_t accel_dataY;
//  int16_t accel_dataZ;
//  int16_t temp;
//  int16_t gyro_dataX;
//  int16_t gyro_dataY;
//  int16_t gyro_dataZ;
//  uint8_t imuA_data[14];
//  uint8_t imuT_data[14];
//  uint8_t imuG_data[14];
//
//  mpu9250_write_reg(28, 0x08);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTask_attributes);
  //servoTaskHandle = osThreadNew(servoTask,NULL, &servoTask_attributes);
  escTaskHandle = osThreadNew(escTask,NULL, &escTask_attributes);
  //canTaskHandle = osThreadNew(canTask,NULL, &canTask_attributes);
  movimientoCamaraTaskHandle = osThreadNew(movimientoCamaraTask,NULL, &movimientoCamaraTask_attributes);
  //imuTaskHandle = osThreadNew(imuTask,NULL, &imuTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 19;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 19;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 65535;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_CE_Pin|SPI_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CE_Pin SPI_CSN_Pin */
  GPIO_InitStruct.Pin = SPI_CE_Pin|SPI_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

void wirelessTask(void *argument){
	uint16_t x_position, y_position, degrees;
	printf("Empezando a Imprimir:\n\r");
	for(;;){
		if(NRF24_available()){
			NRF24_read(myRxData,32);
			myRxData[32] = '\r'; myRxData[32+1] = '\n';
			printf("Coordenadas: %d %d %d %d %d %d\r\n",myRxData[0],myRxData[1],myRxData[2],myRxData[3],myRxData[4],myRxData[5],myRxData[6]);
/*
			if (myRxData[0] == 1){
				x_position = (uint16_t)255 + myRxData[1];
			} else{
				x_position = myRxData[1];
			}
			if (myRxData[2] == 1){
				y_position = (uint16_t)255 + myRxData[3];
			} else{
				y_position = myRxData[3];
			}
			if (myRxData[4] == 1){
				degrees = (uint16_t)255 + myRxData[5];
			} else{
				degrees = myRxData[5];
			}

			flag = 1;
			printf("X: %d Y: %d Angle: %d\r\n", x_position, y_position, degrees);
		}
		*/
	osDelay(150);
	}
}
}

void servoTask(void *argument){
	double dutyCycle3=tONB/period;
	int32_t CH1_DC_IZQ = TIM2->ARR * dutyCycle3;

	uint32_t valor_ccr = 4500;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
	osDelay(2000);
	valor_ccr = 5500;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
	osDelay(2000);
	valor_ccr = 4500;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
	osDelay(2000);
	valor_ccr = CH1_DC_IZQ;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
	osDelay(2000);


}

void escTask(void *argument){
	for(;;){

		  double dutyCycle3=tONB/period;
		  int32_t CH1_DC_IZQ = TIM5->ARR * dutyCycle3;
		  //Seccion donde se controla el motor
		  double pulseWidth= 0.0015;
		  double ccr = 0;
		  ccr = (pulseWidth * htim5.Init.Period )/0.02;
		  htim5.Instance -> CCR4 = ccr;
		  osDelay(400) ;
		  double i=0.00001;
		  while(pulseWidth<0.00183){
			  ccr=(pulseWidth*htim5.Init.Period)/0.02;
			  htim5.Instance -> CCR4=ccr;
			  osDelay(30);
			  pulseWidth += i;
		  }
		  ccr = (pulseWidth * htim5.Init.Period)/0.02;
		  htim5.Instance-> CCR4 = ccr;
		  osDelay(100);
	}
}


void canTask(void *argument){
	for(;;){

		while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK);

		  	osDelay(10);
		  	printf("\n\rCAN ID: %lx", RxHeader.Identifier);

		  	printf(" [%X] ", ((unsigned int)RxHeader.DataLength & 0x0CFEF3A3) >> 16 );
		  	printf(" %02X %02X %02X %02X %02X %02X %02X %02X",RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6],RxData[7]);
		    osDelay(1000);
	}
}

void movimientoCamaraTask(void *argument){
	int actual_target = 0;
	uint32_t valor_ccr = 3900;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);

	for(;;){
		float angulo_final;
		float angulo_error;

		uint32_t valor_pwm;


		// aqui se escriben las coordenadas de los targets
		float x_targets[] = {95, 198, 233, 135};
		float y_targets[] = {24, 56, 135, 165};

		// calcula y guarda el tamano de el array de los targets
		float len_x_target = sizeof(x_targets)/sizeof(x_targets[0]);

	    float x_actual = myRxData[1];
	    float y_actual = myRxData[3];

		// algoritmo para dirigir el carro con la camara
	    float adyacente = x_targets[actual_target] - x_actual; // calcula el cateto adyacente
		float opuesto = y_targets[actual_target] - y_actual; // calcula el cateto opuesto
		float angulo_actual = myRxData[5]; // convierte el angulo recibido a radianes
		float angulo_target0 = atan2(opuesto, adyacente) * 180 / 3.1416; // calculates the desired angle
		if (angulo_target0 < 0) angulo_target0 += 360; // shifts the range from -180-180 to 0-360
		float angulo_target = fmod(angulo_target0, 360); // ensures the angle is within 0-360 range

		// Assuming angulo_actual and angulo_target are float values representing angles in degrees
		if (angulo_target > 0 && angulo_target < 270 && angulo_actual > 0 && angulo_actual < 270){
			if (angulo_actual > angulo_target){
				angulo_final = angulo_actual - angulo_target;
				if (angulo_final <= 23){
					valor_pwm = 3900 - ((fabs(angulo_final)*1950)/23);
					valor_ccr = valor_pwm;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);}
				else {
						valor_pwm = 1950;
						valor_ccr = valor_pwm;
						__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
					}
				}
			if (angulo_actual < angulo_target){
				angulo_final = angulo_target - angulo_actual;
				if (angulo_final <= 23){
					valor_pwm = ((fabs(angulo_final) * 1350)/23)+3900;
					valor_ccr = valor_pwm;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);}
				else {
						valor_pwm = 5250;
						valor_ccr = valor_pwm;
						__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
					}
				}
			}
		else {
			if (angulo_actual > angulo_target){
				angulo_final = (360 - angulo_actual) + (90 - angulo_target);
				if (angulo_final <= 23){
					valor_pwm = 3900 - ((fabs(angulo_final)*1950)/23);
					valor_ccr = valor_pwm;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);}
				else {
						valor_pwm = 1950;
						valor_ccr = valor_pwm;
						__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
					}
				}
			if (angulo_actual < angulo_target){
				angulo_final = (360 - angulo_actual) + (90 - angulo_actual);
				if (angulo_final <= 23){
					valor_pwm = ((fabs(angulo_final) * 1350)/23)+3900;
					valor_ccr = valor_pwm;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);}
				else {
						valor_pwm = 5250;
						valor_ccr = valor_pwm;
						__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, valor_ccr);
					}
				}
			}
		if (x_actual < (x_targets[actual_target] + 15) && x_actual > (x_targets[actual_target] - 15) && y_actual < (y_targets[actual_target] + 15) && y_actual > (y_targets[actual_target] - 15) && actual_target < len_x_target){
					actual_target++;
				}

		}

	osDelay(100);

}

//void imuTask(void *argument){
//	  for(;;)
//	  {
//		  mpu9250_read_reg(59, imuA_data, sizeof(imuA_data));
//		  accel_dataX = ((int16_t)imuA_data[0]<<8) + imuA_data[1];
//		  accel_dataY = ((int16_t)imuA_data[2]<<8) + imuA_data[3];
//		  accel_dataZ = ((int16_t)imuA_data[4]<<8) + imuA_data[5];
//
//		  mpu9250_read_reg(65, imuT_data, sizeof(imuT_data));
//		  temp = ((int16_t)imuT_data[0]<<8) + imuT_data[1];
//
//
//		  mpu9250_read_reg(67, imuG_data, sizeof(imuG_data));
//		  gyro_dataX = ((int16_t)imuG_data[0]<<8) + imuG_data[1];
//		  gyro_dataY = ((int16_t)imuG_data[2]<<8) + imuG_data[3];
//		  gyro_dataZ = ((int16_t)imuG_data[4]<<8) + imuG_data[5];
//
//		  float Xaccel = (float)accel_dataX/8192.0;
//		  float Yaccel = (float)accel_dataY/8192.0;
//		  float Zaccel = (float)accel_dataZ/8192.0;
//
//		  float temperatura = (float)temp / 333.87 + 21;
//
//		  float Xgyro = (float)gyro_dataX * (250.0f/32768.0f);
//		  float Ygyro = (float)gyro_dataY * (250.0f/32768.0f);
//		  float Zgyro = (float)gyro_dataZ * (250.0f/32768.0f);
//		  printf("Acelerometro X: \n\r");
//		  printf("%.2f\r\n", Xaccel);
//		  printf("Acelerometro Y: \n\r");
//		  printf("%.2f\r\n", Yaccel);
//		  printf("Acelerometro Z: \n\r");
//		  printf("%.2f\r\n", Zaccel);
//		  printf("Temperatura: \n\r");
//		  printf("%.2f\r\n", temperatura);
//		  printf("Giroscopio X: \n\r");
//		  printf("%.2f\r\n", Xgyro);
//		  printf("Giroscopio Y: \n\r");
//		  printf("%.2f\r\n", Ygyro);
//		  printf("Giroscopio Z: \n\r");
//		  printf("%.2f\r\n", Zgyro);
//		  osDelay(1000);
//
//	    /* USER CODE END WHILE */
//
//	    /* USER CODE BEGIN 3 */
//	  }
//	  /* USER CODE END 3 */
//	}
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
