/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

//void Character_Transmit(void) 
//{
//	volatile char string[] = "Wrong key pressed\r\n";
//	volatile char curr = string[0];
//	volatile int index = 0;
//	while (curr != 0) 
//	{
//		if ((USART3->ISR) & USART_ISR_TXE) 
//		{
//			USART3->TDR = curr;
//			++index;
//			curr = string[index];
//		}
//	}
//}
volatile char lastValue = 0;
volatile int flag = 0;
void Character_Transmit(char message[]);
void successHelper(int pin)
{
	switch(pin)
	{
		case GPIO_PIN_6:
			Character_Transmit("red\r\n");
			break;
		case GPIO_PIN_9:
			Character_Transmit("green\r\n");
			break;
		case GPIO_PIN_7:
			Character_Transmit("blue\r\n");
			break;
		case GPIO_PIN_8:
			Character_Transmit("orange\r\n");
			break;
		default:
			Character_Transmit("Unknown, this shouldn't be possible!\r\n");
			break;
	}
}
void ledHelper (int pin, char currValue)
{
	switch(currValue)
	{
		case '1': 
			HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
			Character_Transmit("Turned on LED: ");
			successHelper(pin);
			break;
		
		case '2':
			HAL_GPIO_TogglePin(GPIOC, pin);
			Character_Transmit("Toggled LED: ");
			successHelper(pin);
			break;
		
		case '0':
			HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
			Character_Transmit("Turned off LED: ");
			successHelper(pin);
			break;
		
		default:
			Character_Transmit("Wrong key pressed\r\n");
			flag = 0;
			break;
	}
	Character_Transmit("CMD?\r\n");
}

void USART3_4_IRQHandler(void) 
{
	volatile char currValue = (USART3 -> RDR);
	if(flag) 
	{
		switch(lastValue) 
		{
			case 'r': 
				ledHelper(GPIO_PIN_6, currValue);
				flag = 0;
				break;
			
			case 'g':
				ledHelper(GPIO_PIN_9, currValue);
				flag = 0;
				break;
			
			case 'b':
				ledHelper(GPIO_PIN_7, currValue);
				flag = 0;
				break;
				
			case 'o':
				ledHelper(GPIO_PIN_8, currValue);
				flag = 0;
				break;
				
			default:
				Character_Transmit("Wrong key pressed\r\n");
				flag = 0;
				break;
			
		}
		//Character_Transmit("CMD?\r\n");
	}
	else 
	{
			lastValue = currValue;
			flag = 1;
	}
}

/* USER CODE END 1 */
