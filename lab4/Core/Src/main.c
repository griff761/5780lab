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
void Character_Transmit(void) 
{
	volatile char string[] = "Wrong key pressed\r\n";
	volatile char curr = string[0];
	volatile int index = 0;
	while (curr != 0) 
	{
		if ((USART3->ISR) & USART_ISR_TXE) 
		{
			USART3->TDR = curr;
			++index;
			curr = string[index];
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int mainSimple(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	__HAL_RCC_GPIOB_CLK_ENABLE(); // Enable the GPIOB clock in the RCC
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	__HAL_RCC_USART3_CLK_ENABLE(); // Enable the USART3 clock in the RCC
		// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_10 | GPIO_PIN_11,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	GPIO_InitTypeDef ledInitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
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
	HAL_GPIO_Init(GPIOB, &initStr); // Initialize pins PB10, PB11
	HAL_GPIO_Init(GPIOC, &ledInitStr); // Initialize pin PC6
	GPIOB -> AFR[1] = 0x4400; //GPIO_AFRH_AFSEL10 - (0x3 << 2); // Set AFSEL10 to 0b1100 to enable output to AF0 for PC6
	//GPIOB -> AFR[1] |= 0xC000;//GPIO_AFRH_AFSEL11 - (0x3 << 3); // Set AFSEL7 to 0b1100 to enable output to AF0 for PC7
	
	int baud_divider = HAL_RCC_GetHCLKFreq() / 115200;
	USART3 -> BRR |= baud_divider; // Set baud rate
	
	USART3 -> CR1 |= USART_CR1_RE;
	USART3 -> CR1 |= USART_CR1_TE;
	USART3 -> CR1 |= USART_CR1_UE;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	volatile char transmit = 0;
  while (1)
  {
    /* USER CODE END WHILE */
		if ((USART3 -> ISR) & USART_ISR_RXNE)
		{
			transmit = (USART3 -> RDR);
			if (transmit == 'r') 
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			}
			else if (transmit == 'g')
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			}
			else if (transmit == 'b')
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			}
			else if (transmit == 'o')
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			}
			else 
			{
				Character_Transmit();
			}
			
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
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
	__HAL_RCC_USART3_CLK_ENABLE(); // Enable the USART3 clock in the RCC
		// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_10 | GPIO_PIN_11,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	GPIO_InitTypeDef ledInitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
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
	HAL_GPIO_Init(GPIOB, &initStr); // Initialize pins PB10, PB11
	HAL_GPIO_Init(GPIOC, &ledInitStr); // Initialize pin PC6
	GPIOB -> AFR[1] = 0x4400; //GPIO_AFRH_AFSEL10 - (0x3 << 2); // Set AFSEL10 to 0b1100 to enable output to AF0 for PC6
	//GPIOB -> AFR[1] |= 0xC000;//GPIO_AFRH_AFSEL11 - (0x3 << 3); // Set AFSEL7 to 0b1100 to enable output to AF0 for PC7
	
	int baud_divider = HAL_RCC_GetHCLKFreq() / 115200;
	USART3 -> BRR |= baud_divider; // Set baud rate
	
	USART3 -> CR1 |= USART_CR1_RE;
	USART3 -> CR1 |= USART_CR1_TE;
	USART3 -> CR1 |= USART_CR1_UE;
	
	USART3 -> CR1 |= USART_CR1_RXNEIE;
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	volatile char transmit = 0;
  while (1)
  {

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
