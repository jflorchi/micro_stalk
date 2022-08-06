/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ENABLE_CAN_TRANSCEIVER() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define LED_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)

#define SET_ERROR(n) errors |= 1UL << n;
#define CLEAR_ERROR(n) errors &= ~(1UL << n);

#define ABS(x) ((x) > 0 ? (x) : -(x))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len);
void processCAN(CAN_HandleTypeDef *hcan, uint32_t mailbox);
int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr);
void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_FilterTypeDef     canfil;
CAN_TxHeaderTypeDef   txHeader;
CAN_RxHeaderTypeDef   rxHeader;
uint8_t               rxBuffer[8];
uint32_t              txMailbox;

uint8_t cruiseOn = 0;
uint16_t setSpeed = 50, lastSpeed = 0, currentSpeed = 0;

uint8_t errors = 0;
uint8_t ignition = 0;
uint32_t lastSeen = 0;

uint8_t bigHopped = 0;
uint32_t lastPressed = 0;
uint8_t modePressed = 0;
uint8_t lastState = 0;
uint8_t stateChanged = 0;
uint8_t controlsPressed[4] = {0x00, 0x00, 0x00, 0x00}; // speedDown, speedUp, distanceDown, distanceUp
uint32_t controlsPressedTime[4] = {0, 0, 0, 0};
uint8_t signalLights[2] = {0x00, 0x00}; // left,

uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

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
  MX_CAN_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  ENABLE_CAN_TRANSCEIVER();
  LED_OFF();
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
	  LED_ON();
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  if (HAL_GetTick() - lastSeen > 500) {
		  ignition = 0;
	  }

	  if (ignition == 1) {
		  LED_ON();
		  HAL_ADC_Start(&hadc);
		  HAL_ADC_PollForConversion(&hadc, 1);
		  uint32_t mediaButton = HAL_ADC_GetValue(&hadc);
		  if (mediaButton >= 3600 && mediaButton <= 4500) { // increase speed
			  if (controlsPressed[0] == 0) {
				  // rising edge
				  controlsPressedTime[0] = HAL_GetTick();
				  controlsPressed[0] = 1;
				  if (cruiseOn == 0) {
					  cruiseOn = 1;
					  setSpeed = lastSpeed;
				  }
			  } else {
				  if (HAL_GetTick() - controlsPressedTime[0] >= 250) {
					  controlsPressedTime[0] = HAL_GetTick();
					  if (cruiseOn == 1) {
						  setSpeed += 5;
						  bigHopped = 1;
					  }
				  }
			  }
		  } else if (mediaButton >= 2600 && mediaButton <= 3500) { // decrease speed
			  if (controlsPressed[1] == 0) {
				  // rising edge
				  controlsPressedTime[1] = HAL_GetTick();
				  controlsPressed[1] = 1;
			  } else {
				  if (HAL_GetTick() - controlsPressedTime[1] >= 250) {
					  controlsPressedTime[1] = HAL_GetTick();
					  if (cruiseOn == 0) {

					  } else {
						  setSpeed -= 5;
						  bigHopped = 1;
					  }
				  }
			  }
		  } else if (mediaButton >= 1600 && mediaButton <= 2500) { // increase follow distance

		  } else if (mediaButton >= 600 && mediaButton <= 1500) { // decrease follow distance

		  } else {
			  // detect falling edge
			  for (uint8_t i = 0; i < 4; i++) {
				  if (controlsPressed[i] == 1) {
					  if (bigHopped == 0) {
						  if (i == 0) {
							  setSpeed += 1;
						  } else if (i == 1) {
							  setSpeed -= 1;
						  } else if (i == 2) {

						  } else if (i == 3) {

						  }
					  } else {
						  bigHopped = 0;
					  }
					  controlsPressed[i] = 0;
					  controlsPressedTime[i] = 0;
				  }
			  }
		  }


		  // Signal light is on if GPIO is low, it's normally high.
		  signalLights[0] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) ? 0x00 : 0xFF;
		  signalLights[1] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) ? 0x00 : 0xFF;
		  sendCAN(0x614, signalLights, 2);

		  uint8_t modeButton = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		  if (modeButton == SET) {
			  if (lastState != modeButton) {
				  if (stateChanged == 0) {
					  if (cruiseOn == 0) {
						  cruiseOn = 1;
					  } else {
//						  setSpeed = currentSpeed;
						  cruiseOn = 0;
					  }
				  }
				  stateChanged = 1;
			  } else {
				  stateChanged = 0;
			  }
		  }
		  lastState = modeButton;

		  //0x1d2 msg PCM_CRUISE
		  PCM_CRUISE_MSG[0] = (cruiseOn << 5) & 0x20;
		  PCM_CRUISE_MSG[5] = (cruiseOn << 7) & 0x80;
		  attachChecksum(0x1D2, 8, PCM_CRUISE_MSG);
		  sendCAN(0x1D2, PCM_CRUISE_MSG, 8);
		  //0x1d3 msg PCM_CRUISE_2
		  PCM_CRUISE_2_MSG[1] = ((0x001 << 7) & 0x80) | 0x28;
		  PCM_CRUISE_2_MSG[2] = setSpeed;
		  attachChecksum(0x1D3, 8, PCM_CRUISE_2_MSG);
		  sendCAN(0x1D3, PCM_CRUISE_2_MSG, 8);

		  HAL_Delay(50);
	  } else {
		  LED_OFF();
		  setSpeed = 50;
		  cruiseOn = 0;
		  HAL_Delay(500);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  canfil.FilterMode = CAN_FILTERMODE_IDLIST;
  canfil.FilterScale = CAN_FILTERSCALE_16BIT;
  canfil.FilterIdHigh = 0x001 << 5;
  canfil.FilterIdLow =  0x002 << 5;
  canfil.FilterMaskIdHigh = 0x001 << 5;
  canfil.FilterMaskIdLow = 0x001 << 5;
  canfil.FilterFIFOAssignment = 0;
  canfil.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &canfil);
  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len) {
	CAN_TxHeaderTypeDef header;

	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	header.DLC = len;
	header.StdId = id;

	uint32_t mailbox;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
		uint8_t res = HAL_CAN_AddTxMessage(&hcan, &header, buffer, &mailbox);
		if (res != HAL_OK) {
			SET_ERROR(0);
		} else {
			CLEAR_ERROR(0);
		}
	} else {
		if (HAL_CAN_IsTxMessagePending(&hcan, txMailbox)) {
			SET_ERROR(1);
		}
	}

	return mailbox;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	processCAN(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	processCAN(hcan, CAN_RX_FIFO1);
}

void processCAN(CAN_HandleTypeDef *hcan, uint32_t mailbox) {
	CAN_RxHeaderTypeDef tmp;
	uint8_t data[8];
	HAL_CAN_GetRxMessage(hcan, mailbox, &tmp, data);
	if (tmp.StdId == 0x001) {
		ignition = 1;
		lastSeen = HAL_GetTick();
		currentSpeed = data[0];
		if (cruiseOn == 0) {
			setSpeed = data[0];
		}
	} else if (tmp.StdId == 0x002) {
		lastSpeed = setSpeed;
		cruiseOn = 0;
	}
}

int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (msg[ii]);
    }
    return checksum;
}

void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg) {
    msg[len -1] = getChecksum(msg, len - 1, id);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
