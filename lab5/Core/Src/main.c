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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	__HAL_RCC_GPIOB_CLK_ENABLE(); // Enable the GPIOB clock in the RCC
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	__HAL_RCC_I2C2_CLK_ENABLE(); // Enable the I2C2 clock in the RCC
	
	I2C2 -> TIMINGR |= (0x1 << 28); // Set PRESC
	I2C2 -> TIMINGR |= (0x13); // Set SCLL
	I2C2 -> TIMINGR |= (0xF << 8); // Set SCLH
	I2C2 -> TIMINGR |= (0x2 << 16); // Set SDADEL
	I2C2 -> TIMINGR |= (0x4 << 20); // Set SCLDEL
	
	GPIO_InitTypeDef initStr11 = {GPIO_PIN_11,
	GPIO_MODE_AF_OD,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL,
	GPIO_AF1_I2C2};
	
	GPIO_InitTypeDef initStr13 = {GPIO_PIN_13,
	GPIO_MODE_AF_OD,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL,
	GPIO_AF5_I2C2};

	GPIO_InitTypeDef initStr14 = {GPIO_PIN_14,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	GPIO_InitTypeDef initStr0 = {GPIO_PIN_0,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	GPIO_InitTypeDef initStrLed = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	HAL_GPIO_Init(GPIOB, &initStr11); // Initialize pin PB11
	HAL_GPIO_Init(GPIOB, &initStr13); // Initialize pin PB13
	HAL_GPIO_Init(GPIOB, &initStr14); // Initialize pin PB14
	HAL_GPIO_Init(GPIOC, &initStr0); // Initialize pin PC0
	HAL_GPIO_Init(GPIOC, &initStrLed); // Initialize LEDs
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Set PB14 High
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set PC0 High
	
	I2C2 -> CR1 |= I2C_CR1_PE; // Enable the I2C2 peripheral
	
	//Begin write sequence to slave
	I2C2 -> CR2 |= (0x69 << 1); // Set slave address of gyro
	I2C2 -> CR2 |= (1 << 16); // Set num of bytes to transmit
	I2C2 -> CR2 &= ~(1 << 10); // Set RD_WRN to indicate write
	I2C2 -> CR2 |= (1 << 13); // Set START
	
	while(1) 
	{
		if (I2C2 -> ISR & (1<<1)) // check TXIS
		{
			I2C2 -> TXDR = 0xF;
			break;
		}
		else if (I2C2 -> ISR & (1<<4)) // check NACKF
		{
			// should never reach or else there is a config error
		}
	}
	while(1)
	{
		if (I2C2 -> ISR & (1<<6)) // check TC for completion
		{ 
			break;
		}
	}
	
	// Begin read sequence to retrieve value from WHO_AM_I
	I2C2 -> CR2 |= (0x69 << 1); // Set slave address of gyro
	I2C2 -> CR2 |= (1 << 16); // Set num of bytes to transmit
	I2C2 -> CR2 |= (1 << 10); // Set RD_WRN to indicate read
	I2C2 -> CR2 |= (1 << 13); // Set START
	volatile int rxdrVal = 0;
	
	while(1) 
	{
		if (I2C2 -> ISR & (1<<2)) // check RXNE
		{
			break;
		}
		else if (I2C2 -> ISR & (1<<4)) // check NACKF
		{
			// should never reach or else there is a config error
		}
	}
	while(1)
	{
		if (I2C2 -> ISR & (1<<6)) // check TC
		{
			rxdrVal = I2C2 -> RXDR; // grab value from RXDR
			break;
		}
	}
	
	I2C2 -> CR2 &= ~(1 << 14); // set STOP
	
	// check if RDXR is expected WHO_AM_I value
	if (rxdrVal == 0xD3)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // toggle green if expected value
	}
	else
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // toggle red if value is not expected
	}
	

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
