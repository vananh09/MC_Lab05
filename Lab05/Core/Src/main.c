/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_BUFFER_SIZE  30
enum State{NONE_REQUEST, SEND_REQUEST, WAITING_REPLY, REPLY, CAN_NOT_SEND, RESEND, INIT, SEND_SUCCESS};
uint8_t state = NONE_REQUEST;
uint8_t pre_state = NONE_REQUEST;
uint8_t input = 0;
uint8_t buffer_input[MAX_BUFFER_SIZE];
uint8_t index_buffer_input = 0;
uint8_t buffer_flag = 0;
uint8_t pre_ADC_value = 0;
uint8_t timeout_flag = 0;
int count = 0;
int timer_counter = 0;
unsigned char timer_flag = 0;
const int TIMER_CIRCLE = 10;
int timer1_counter = 0;
unsigned char timer1_flag = 0;
uint8_t segmentNumber[10] = {
		0x40,  // 0
		0x79,  // 1
		0x24,  // 2
		0x30,  // 3
		0x19,  // 4
		0x12,  // 5
		0x02,  // 6
		0x78,  // 7
		0x00,  // 8
		0x10   // 9
};

void display7SEG(uint8_t number){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, ((number>>0)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, ((number>>1)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, ((number>>2)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, ((number>>3)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, ((number>>4)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, ((number>>5)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, ((number>>6)&0x01));
}

// make a timer interrupt for fsm
void setTimer(int duration){
	timer_counter = duration/TIMER_CIRCLE;
	timer_flag = 0;
}

void timer_run(){
	timer_counter--;
	if(timer_counter <= 0){
		timer_flag = 1;
	}
}

unsigned char get_timer_flag(){
	return timer_flag;
}

// make a timer interrupt for display time error control
void setTimer1(int duration){
	timer1_counter = duration/TIMER_CIRCLE;
	timer1_flag = 0;
}

void timer1_run(){
	timer1_counter--;
	if(timer1_counter <= 0){
		timer1_flag = 1;
	}
}


// uart call back in stm32
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		buffer_input[index_buffer_input++] = input;
		if(index_buffer_input == MAX_BUFFER_SIZE) {
			index_buffer_input = 0;
		}
		buffer_flag = 1;
		HAL_UART_Receive_IT(&huart2, &input, 1);
	}
}

// make a function to save the adc value to send again
void get_ADC_value(uint8_t status) {
	//save the previous value
	if(status == CAN_NOT_SEND) {
		uint8_t ADC_value = HAL_ADC_GetValue(&hadc1);
		pre_ADC_value = ADC_value;
	}
	char msg[20];
	HAL_UART_Transmit(&huart2, (void*)msg, sprintf(msg, "!ADC=%d#", pre_ADC_value), HAL_MAX_DELAY);
	// display time
	uint32_t time = HAL_GetTick()/1000;
	HAL_UART_Transmit(&huart2, (void *)msg, sprintf(msg, " -- time: %ld s\n\r", time), HAL_MAX_DELAY);
}

void set_timeout(uint8_t status) {
	if(status == INIT) {
		setTimer(30);
	}
	timeout_flag = 0;
}
// make a fsm machine for read the input
void command_parser_fsm() {
	char request_command[] = "!RST#";
	char reply_command[] = "!OK#";
	uint8_t index = (index_buffer_input - 2 >= 0) ? index_buffer_input - 2 : MAX_BUFFER_SIZE - 2;
	// check if the last input is end line and semi-final input is # then we get 2 case per it
	if(buffer_input[index_buffer_input - 1] == '\r' && buffer_input[index] == '#') {
		switch (buffer_input[index - 1]) {
		case 'T':
			state = SEND_REQUEST;
			for(int i = 2; i < 5; i++) {
				if(buffer_input[index-i] != request_command[4-i]) {
					// check input string with the format request command
					state = pre_state;
					break;
				}
			}
			break;
		case 'K':
			state = REPLY;
			for(int i = 2; i < 4; i++) {
				// check input string with the format reply command
				if(buffer_input[index-i] != reply_command[3-i]) {
					state = pre_state;
					break;
				}
			}
			break;
		default:
			break;
		}
	} else {
		return;
	}
}

//make a fsm uart communication
void uart_communication_fsm () {
	switch (state) {
	case NONE_REQUEST:
		// display led per state
		HAL_GPIO_WritePin(GPIOA, NONE_REQUEST_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, SEND_REQUEST_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, WAITING_REPLY_Pin, 1);
		break;
	case SEND_REQUEST:
		// display led per state and save the pre ADC value if can not send
		HAL_GPIO_WritePin(GPIOA, NONE_REQUEST_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, SEND_REQUEST_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, WAITING_REPLY_Pin, 1);
		get_ADC_value(CAN_NOT_SEND);
		set_timeout(INIT);
		state = WAITING_REPLY;
		break;
	case WAITING_REPLY:
		// display led per state and send again per 3s
		HAL_GPIO_WritePin(GPIOA, NONE_REQUEST_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, SEND_REQUEST_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, WAITING_REPLY_Pin, 0);
		if(get_timer_flag()){
			timeout_flag = 1;
		}
		if(timeout_flag){
			get_ADC_value(RESEND);
			set_timeout(INIT);
		}
		break;
	case REPLY:
		// if get the reply then send success
		set_timeout(SEND_SUCCESS);
		state = NONE_REQUEST;
		break;
	default:
		break;
	}
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &input, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(buffer_flag == 1){
		  command_parser_fsm();
		  buffer_flag = 0;
	  }
	  uart_communication_fsm();
	  if(timer1_flag == 1 && state == WAITING_REPLY) {
		  setTimer1(10);
		  switch(count) {
			  case 0:
				  display7SEG(segmentNumber[count]);
				  count = 1;
				  break;
			  case 1:
				  display7SEG(segmentNumber[count]);
				  count = 2;
				  break;
			  case 2:
				  display7SEG(segmentNumber[count]);
				  count = 0;
				  break;
		  }
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NONE_REQUEST_Pin|SEND_REQUEST_Pin|WAITING_REPLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : NONE_REQUEST_Pin SEND_REQUEST_Pin WAITING_REPLY_Pin */
  GPIO_InitStruct.Pin = NONE_REQUEST_Pin|SEND_REQUEST_Pin|WAITING_REPLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2){
		timer_run();
		timer1_run();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
