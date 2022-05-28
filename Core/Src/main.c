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
typedef struct HCSR04 HCSR04;
struct HCSR04 {
	GPIO_TypeDef *TRIG_PORT;
	uint16_t TRIG_PIN;
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint32_t IC_Val1;
	uint32_t IC_Val2;
	uint8_t Is_First_Captured;  // is the first value captured ?
	uint32_t Distance;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
HCSR04 sensor1;
HCSR04 sensor2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HC_SR04_init(){
	sensor1.TRIG_PORT = HC_SR04_1_trig_GPIO_Port;
	sensor1.TRIG_PIN = HC_SR04_1_trig_Pin;
	sensor1.htim = &htim3;
	sensor1.channel = TIM_CHANNEL_2;
	sensor1.IC_Val1 = 0;
	sensor1.IC_Val2 = 0;
	sensor1.Is_First_Captured = 0;
	sensor1.Distance = 0;

	sensor2.TRIG_PORT = HC_SR04_2_trig_GPIO_Port;
	sensor2.TRIG_PIN = HC_SR04_2_trig_Pin;
	sensor2.htim = &htim1;
	sensor1.channel = TIM_CHANNEL_1;
	sensor2.IC_Val1 = 0;
	sensor2.IC_Val2 = 0;
	sensor2.Is_First_Captured = 0;
	sensor2.Distance = 0;
}
void notify(GPIO_PinState PinState){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, PinState);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void delay(TIM_HandleTypeDef *htim, uint16_t time) {
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (__HAL_TIM_GET_COUNTER(htim) < time)
		;
}

//#define TRIG_PIN GPIO_PIN_9
//#define TRIG_PORT GPIOA

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HCSR04 *sensor;
	switch(htim->Instance){
	    case TIM3:
	    	sensor = &sensor1;
	      break;

	    case TIM1:
	    	sensor = &sensor2;
	      break;

	    default:
	}

	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
				{
			if (sensor1.Is_First_Captured == 0) // if the first value is not captured
					{
				sensor1.IC_Val1 = HAL_TIM_ReadCapturedValue(htim,
						TIM_CHANNEL_1); // read the first value
				sensor1.Is_First_Captured = 1; // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else  // if the first is already captured
			{
				sensor1.IC_Val2 = HAL_TIM_ReadCapturedValue(htim,
						TIM_CHANNEL_1); // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				sensor1.Distance = ((sensor1.IC_Val2 - sensor1.IC_Val1)
						* (sensor1.IC_Val2 > sensor1.IC_Val1)
						+ ((0xffff - sensor1.IC_Val1) + sensor1.IC_Val2)
								* (sensor1.IC_Val1 > sensor1.IC_Val2)) * .034
						/ 2;
				//Distance = Difference * .034/2;
				sensor1.Is_First_Captured = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			}
		}
	} else {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // if the interrupt source is channel1
				{
			if (sensor2.Is_First_Captured == 0) // if the first value is not captured
					{
				sensor2.IC_Val1 = HAL_TIM_ReadCapturedValue(htim,
						TIM_CHANNEL_2); // read the first value
				sensor2.Is_First_Captured = 1; // set the first captured as true

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
						TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else  // if the first is already captured
			{
				sensor2.IC_Val2 = HAL_TIM_ReadCapturedValue(htim,
						TIM_CHANNEL_2); // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				sensor2.Distance = ((sensor2.IC_Val2 - sensor2.IC_Val1)
						* (sensor2.IC_Val2 > sensor2.IC_Val1)
						+ ((0xffff - sensor2.IC_Val1) + sensor2.IC_Val2)
								* (sensor2.IC_Val1 > sensor2.IC_Val2)) * .034
						/ 2;

				//Distance = Difference * .034/2;
				sensor2.Is_First_Captured = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
						TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
			}
		}
	}
}
void HCSR04_Read(HCSR04 *sensor) {
	HAL_GPIO_WritePin(sensor->TRIG_PORT, sensor->TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay(sensor->htim, 10);  // wait for 10 us
	HAL_GPIO_WritePin(sensor->TRIG_PORT, sensor->TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
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
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */



	uint8_t data[32];
	uint8_t count[8];
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	uint8_t reset_counter= 14;
	int32_t human_counter = 0;
	uint8_t sensor1_trigger_first = 0;
	uint8_t sensor2_trigger_first = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(100);
		HCSR04_Read(&sensor1);
		HAL_Delay(20);
		HCSR04_Read(&sensor2);
		HAL_Delay(20);
		if(sensor1_trigger_first && !sensor2_trigger_first){
			reset_counter--;
			if(sensor1.Distance > 50 && sensor2.Distance < 30){
				human_counter++;
				notify(GPIO_PIN_SET);
				HAL_UART_Transmit(&huart6, count,
						sprintf(count, "%d\n",1),
						100);
				sensor1_trigger_first = 1;
				sensor2_trigger_first = 1;
			}
			if(reset_counter <= 0){
				reset_counter = 14;
				sensor1_trigger_first = 1;
				sensor2_trigger_first = 1;
			}
		}
		else if(sensor2_trigger_first && !sensor1_trigger_first){
			reset_counter--;
			if(sensor2.Distance > 50 && sensor1.Distance < 30){
				human_counter--;
				notify(GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart6, count,
						sprintf(count, "%d\n",2),
						100);
				sensor1_trigger_first = 1;
				sensor2_trigger_first = 1;
				}
			if(reset_counter <= 0){
				reset_counter = 6;
				sensor1_trigger_first = 1;
				sensor2_trigger_first = 1;
			}
			}
		else if(!sensor1_trigger_first && !sensor2_trigger_first){
			if(sensor1.Distance < 30 && sensor2.Distance > 50){
			sensor1_trigger_first = 1;
			sensor2_trigger_first = 0;
			}
			if(sensor2.Distance < 30 && sensor1.Distance > 50){
			sensor1_trigger_first = 0;
			sensor2_trigger_first = 1;
			}
		}else{
			if(sensor2.Distance > 50 && sensor1.Distance > 50){
				sensor1_trigger_first = 0;
				sensor2_trigger_first = 0;
			}
		}


		HAL_UART_Transmit(&huart2, data,
				sprintf(data, "%d %d state: %d %d %d\n", sensor1.Distance, sensor2.Distance, sensor1_trigger_first,sensor2_trigger_first,human_counter),
				100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|HC_SR04_1_trig_Pin
                          |HC_SR04_2_trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HC_SR04_1_trig_Pin HC_SR04_2_trig_Pin */
  GPIO_InitStruct.Pin = HC_SR04_1_trig_Pin|HC_SR04_2_trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	while (1) {
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
