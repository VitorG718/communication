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
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	output,
	input
}DIO_ModeTypedef;

typedef enum {
	none, s1, s2, s3, s4, s5, s6, s7, s8
}Button_Typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_INSTRUCTION 		0x1U << 6U
#define DSP_CONTROL 				0x2U << 6U
#define ADDRESS_INSTRUCTION 0x3U << 6U

#define TEST_MODE						(DATA_INSTRUCTION | (0x1U << 3U))
#define NORMAL_MODE					DATA_INSTRUCTION
#define AUTO_ADDRESS				DATA_INSTRUCTION
#define FIXED_ADDRESS				(DATA_INSTRUCTION | (0x1U << 2U))
#define WRITE_DATA					DATA_INSTRUCTION
#define READ_DATA						(DATA_INSTRUCTION | (0x1U << 1U))

#define DSP_ON							0x1U << 3U
#define DSP_OFF             0x0U
#define DSP_DOT							0x1U << 7U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t numbers[10] = {0x3F, 0x06, 0x9B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void DSP_Write_Data(uint8_t data);
void DSP_7SEG_Data(uint8_t address, uint8_t data);
uint8_t* DSP_Read_Data(void);
Button_Typedef DSP_ButtonPressed(void);
void DSP_Init(void);
void DIO_Mode(DIO_ModeTypedef mode) {
	GPIO_InitTypeDef gpio_init = {0};
	
	switch(mode) {
		case output:
			gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
			gpio_init.Pull = GPIO_PULLUP;
			gpio_init.Pin = DIO_Pin;
			gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(DIO_GPIO_Port, &gpio_init);
			break;
		case input:
			gpio_init.Mode = GPIO_MODE_INPUT;
			gpio_init.Pull = GPIO_NOPULL;
			gpio_init.Pin = DIO_Pin;
			HAL_GPIO_Init(DIO_GPIO_Port, &gpio_init);
			break;
	}
}

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DSP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(250);
		DSP_ButtonPressed();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|STB_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : DIO_Pin */
  GPIO_InitStruct.Pin = DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin STB_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void DSP_Write_Data(uint8_t data)
{
	for(uint8_t i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(DIO_GPIO_Port, DIO_Pin, ((data >> i) & 0x1) == 0x1 ? GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}
}

void DSP_Init(void)
{
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	DSP_Write_Data(DSP_CONTROL | DSP_ON | 0x0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	DSP_Write_Data(NORMAL_MODE | WRITE_DATA | FIXED_ADDRESS);
	HAL_Delay(1);
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	
	DSP_7SEG_Data(0x00, numbers[1]);
	DSP_7SEG_Data(0x02, numbers[0]);
	DSP_7SEG_Data(0x04, numbers[0]);
	DSP_7SEG_Data(0x06, numbers[0]);
	DSP_7SEG_Data(0x08, numbers[1]);
	DSP_7SEG_Data(0x0A, numbers[0]);
	DSP_7SEG_Data(0x0C, numbers[0]);
	DSP_7SEG_Data(0x0E, numbers[0]);
}

void DSP_7SEG_Data(uint8_t address, uint8_t data) {
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	DSP_Write_Data(ADDRESS_INSTRUCTION | address);
	HAL_Delay(1);
	DSP_Write_Data(data);
	HAL_Delay(1);
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

uint8_t* DSP_Read_Data(void) {
	uint8_t* data = (uint8_t *) malloc(4 * sizeof(uint8_t));
	uint8_t temp;
	uint8_t msg[14];
	
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	DSP_Write_Data(NORMAL_MODE | READ_DATA | AUTO_ADDRESS);
	HAL_Delay(1);
	
	DIO_Mode(input);
	for(uint8_t i = 0; i < 4; i++) {
		temp = 0x0;
		for(uint8_t j = 0; j < 8; j++) {
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			if(HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_Pin) == 1) {
				temp |= (0x1 << j);
			}
			HAL_Delay(1);
		}
		data[i] = temp;
	}
	HAL_Delay(1);
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
	DIO_Mode(output);
		
	return data;
}

Button_Typedef DSP_ButtonPressed(void)
{
	uint8_t *data = DSP_Read_Data();
	Button_Typedef button = none;
	
	for(uint8_t j = 0; j < 4; j++) {
		switch(data[j]) {
			case 1:
				button = j+1;
				break;
			case 16:
				button = j + 5;
		}
	}
	
	uint8_t msg[] = "Button: none\n";
	if(button != none)
		sprintf(msg, "Button: S%02d\n", button);
	
	HAL_UART_Transmit(&huart2, msg, sizeof(msg)/sizeof(uint8_t), 250);
	
	free(data);
	
	return button;
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
