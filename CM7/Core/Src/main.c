/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
const uint32_t LEFT_MOTOR_ID = 0x601;
const uint32_t RIGHT_MOTOR_ID = 0x602;
const uint32_t LEFT_READ_MOTOR_ID = 0x581;
const uint32_t RIGHT_READ_MOTOR_ID = 0x582;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Khởi tạo CAN*/

void motor_init_can_interface(void) {
  HAL_FDCAN_Start(&hfdcan1);
}

/**
  * @brief  Set the motor to CAN OPERATION mode 
  * @param motor_id
  * @retval None
  */
void motor_enter_operation_mode(uint32_t motor_id) {
  //Driver enter operating mode-------------------------------
  uint8_t tx_data_op[2];
  tx_data_op[0] = 0x01;
  tx_data_op[1] = motor_id > 1 ? motor_id : 0x00; //First in operation mode cob ID 0000, data 01 XX (XX is the node of the drive, station number 1 is 01, and 00 is sent for all node operations
  FDCAN_TxHeaderTypeDef tx_operation;

  tx_operation.Identifier = 0x000000;
  tx_operation.IdType = FDCAN_STANDARD_ID;
  tx_operation.TxFrameType = FDCAN_DATA_FRAME;
  tx_operation.DataLength = FDCAN_DLC_BYTES_2;
  tx_operation.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_operation.BitRateSwitch = FDCAN_BRS_OFF;
  tx_operation.FDFormat = FDCAN_CLASSIC_CAN;
  tx_operation.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_operation.MessageMarker = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_operation, tx_data_op);
}

/**
  * @brief  Set the motor to CAN SPEED mode
  * @param motor_id
  * @retval None
  */
void motor_enter_speed_mode(uint32_t motor_id) {
  FDCAN_TxHeaderTypeDef tx_transmitter;

  tx_transmitter.Identifier = motor_id > 1 ? motor_id : 0x00;  //servo Id
  tx_transmitter.IdType = FDCAN_STANDARD_ID;
  tx_transmitter.TxFrameType = FDCAN_DATA_FRAME;
  tx_transmitter.DataLength = FDCAN_DLC_BYTES_8;
  tx_transmitter.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_transmitter.BitRateSwitch = FDCAN_BRS_OFF;
  tx_transmitter.FDFormat = FDCAN_CLASSIC_CAN;
  tx_transmitter.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_transmitter.MessageMarker = 0;

  uint8_t tx_speed_mode_data[8];
  //set driver to speed mode data 2F 60 60 00 03 00 00 00-----------------------
  tx_speed_mode_data[0] = 0x2F;
  tx_speed_mode_data[1] = 0x60;
  tx_speed_mode_data[2] = 0x60;
  tx_speed_mode_data[3] = 0x00;
  tx_speed_mode_data[4] = 0x03;
  tx_speed_mode_data[5] = 0x00;
  tx_speed_mode_data[6] = 0x00;
  tx_speed_mode_data[7] = 0x00;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_transmitter, tx_speed_mode_data);
}

/**
  * @brief  Enable the motor
  * @param motor_id
  * @retval None
  */
void motor_enable(uint32_t motor_id) {
  FDCAN_TxHeaderTypeDef tx_transmitter;

  tx_transmitter.Identifier = motor_id > 0 ? motor_id : 0x00;  //servo Id
  tx_transmitter.IdType = FDCAN_STANDARD_ID;
  tx_transmitter.TxFrameType = FDCAN_DATA_FRAME;
  tx_transmitter.DataLength = FDCAN_DLC_BYTES_8;
  tx_transmitter.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_transmitter.BitRateSwitch = FDCAN_BRS_OFF;
  tx_transmitter.FDFormat = FDCAN_CLASSIC_CAN;
  tx_transmitter.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_transmitter.MessageMarker = 0;

  uint8_t tx_enable_data[8];
  //driver enable 2B 40 60 00 0F 00 00 00---------------------------------
  tx_enable_data[0] = 0x2B;
  tx_enable_data[1] = 0x40;
  tx_enable_data[2] = 0x60;
  tx_enable_data[3] = 0x00;
  tx_enable_data[4] = 0x0F;
  tx_enable_data[5] = 0x00;
  tx_enable_data[6] = 0x00;
  tx_enable_data[7] = 0x00;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_transmitter, tx_enable_data);
}
/**
  * @brief  Set motor speed in SPEED mode
  * @param  motor_id: Motor CAN ID
  * @param  speed: The target speed in rpm
  * @param  reversed: True if the motor is in reversed direction
  * @retval None
  */
void motor_set_speed(uint32_t motor_id, uint32_t speed, bool reversed)
{
  FDCAN_TxHeaderTypeDef tx_transmitter;

  tx_transmitter.Identifier = motor_id;
  tx_transmitter.IdType = FDCAN_STANDARD_ID;
  tx_transmitter.TxFrameType = FDCAN_DATA_FRAME;
  tx_transmitter.DataLength = FDCAN_DLC_BYTES_8;
  tx_transmitter.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_transmitter.BitRateSwitch = FDCAN_BRS_OFF;
  tx_transmitter.FDFormat = FDCAN_CLASSIC_CAN;
  tx_transmitter.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_transmitter.MessageMarker = 0;

  uint8_t tx_data[8];

  /*
   * Write target velocity
   * Object 0x60FF
   */
  tx_data[0] = 0x23;
  tx_data[1] = 0xFF;
  tx_data[2] = 0x60;
  tx_data[3] = 0x00;

  /*
   * Convert rpm to driver velocity unit
   * Driver unit: 0.1 count/s
   * Encoder resolution: 10000 count/rev
   */
  uint32_t s = speed * 10000 / 60 / 0.1;

  if (reversed)
  {
    s = (~s) + 1;
  }

  tx_data[4] = s & 0xFF;
  tx_data[5] = (s >> 8) & 0xFF;
  tx_data[6] = (s >> 16) & 0xFF;
  tx_data[7] = s >> 24;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_transmitter, tx_data);
}
/**
  * @brief  Initialize motor driver
  * @retval None
  */
void start_motors(void) {

  motor_enter_operation_mode(1);
  HAL_Delay(100);
  motor_enter_speed_mode(LEFT_MOTOR_ID);
  HAL_Delay(100);
  motor_enter_speed_mode(RIGHT_MOTOR_ID);
  HAL_Delay(100);
  motor_enable(LEFT_MOTOR_ID);
  HAL_Delay(100);
  motor_enable(RIGHT_MOTOR_ID);

}

/* --------------------------------------------------------- */
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
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
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
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* -------------Motor Driver Can------------------------- */
    motor_init_can_interface();
    HAL_Delay(3000);
    start_motors();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	   motor_set_speed(LEFT_MOTOR_ID, 200, false);
	   HAL_Delay(300);
     motor_set_speed(RIGHT_MOTOR_ID, 200, false);
     HAL_Delay(300);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 7;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 12;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 12;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
#ifdef USE_FULL_ASSERT
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
