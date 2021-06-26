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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "dht.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX 35
#define MAX_HUM 90
#define LOW_HUM 70
DHT_DataTypedef DHT11_Data;
int Temperature, Humidity;
_Bool sw_fan1_fast = false;
_Bool sw_fan1_low = false;
_Bool sw_fan2 = 0;
_Bool sw_fan3 = 0;
_Bool sw_heater = 0;
_Bool sw_pump = 0;
_Bool st_1 = true;
int st_1_count = 0;
int st_2_count = 0;
_Bool st_2 = true;
char buf[50];
int buf_len = 50;
int msec = 0;
int sec = 0;
int count =0;
int count1 = 0;
int count2 = 0;
//int min = 0;
int counter_fast = 0;
int counter_low = 0;
int fan_1_speed = 0;
int speed_fan = 100;
int fan_1_speed_2 = 0;
int fan_period = 150;
void heater(void);
void heat_pump(void);
void fan_1(void);
//sTask SCH_tasks_G[SCH_MAX_TASKS];
//char uart_buf[50];
//int uart_buf_len;


void Tim_out(void){
	//while(DHT11_Data.Humidity == -1 && DHT11_Data.Temperature == -1){
	while(DHT_Read() == 2){
		buf_len = sprintf(buf, "\033[2J]	");
		buf_len = sprintf(buf, " Time out     \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 // LED_ON();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 64000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
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
  HAL_GPIO_WritePin(GPIOC, HEAT_PUMP_Pin|HEATER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FAN_1_Pin|GPIO_PIN_1|FAN_2_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_3_GPIO_Port, FAN_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HEAT_PUMP_Pin HEATER_Pin */
  GPIO_InitStruct.Pin = HEAT_PUMP_Pin|HEATER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_1_Pin PA1 FAN_2_Pin LD2_Pin */
  GPIO_InitStruct.Pin = FAN_1_Pin|GPIO_PIN_1|FAN_2_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_3_Pin */
  GPIO_InitStruct.Pin = FAN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


void fan_1(void){
	if(DHT11_Data.Humidity > MAX_HUM){
		fan_1_speed = speed_fan *(DHT11_Data.Humidity- MAX_HUM + 20)/40;
		count1++;
		if(count1 > fan_period/fan_1_speed){
			count1 = 0;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		}

	}
	if(DHT11_Data.Humidity > LOW_HUM && DHT11_Data.Humidity < MAX_HUM){
		/*
		buf_len = sprintf(buf, "\033[2J]	");
		buf_len = sprintf(buf, "test low\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
		*/
		fan_1_speed_2 = speed_fan *(DHT11_Data.Humidity- LOW_HUM)/40;
		count2++;
		if(count2 > fan_period/fan_1_speed_2){
			count2 = 0;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		}
	}
	if(DHT11_Data.Humidity < LOW_HUM){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	}
}
void heater(void){
	if(sw_fan2){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
	}

	if(sw_heater){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_RESET);
	}
}
void heat_pump(void){
	if(sw_pump){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
	}

	if(sw_fan3){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}

}
enum {HEAT_UP, HEAT_PUMP} eState;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *time) {

	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	Tim_out();
    DHT_GetData(&DHT11_Data, DHT_PORT, DHT_PIN);
	Temperature = DHT11_Data.Temperature;
    Humidity = DHT11_Data.Humidity;
  	//HAL_Delay(3000);

	buf_len = sprintf(buf, "\033[2J]	");
	buf_len = sprintf(buf, "%d percent : %d C     \r\n",Humidity,Temperature);
	HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);

	fan_1();

	switch (eState){
		case HEAT_UP:
			//increase LATD;
			heat_pump();
			heater();
			sw_heater = 1;
			sw_fan2 = 1;
			if(DHT11_Data.Temperature > MAX){
				sw_heater = 0;
				sw_fan2 = 0;
			}
			//heater();
			count++;
			if(count > 30){
				sw_heater = 0;
				sw_fan2 = 0;
				count = 0;
				eState = HEAT_PUMP;
			}

		break;
		case HEAT_PUMP:
				//increase LATD;
				heater();
				heat_pump();
				sw_pump = 1;
				sw_fan3 = 1;
				count++;
				if(count > 30){
					sw_fan3 = 0;
					sw_pump = 0;
					count = 0;
					eState = HEAT_UP;
				}

		break;
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
