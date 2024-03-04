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
#include "stm32f072xb.h"
#include "core_cm0.h"

//Global Variables to keep track of information
char inputChar; //character input from computer
int inputFlag; //an integer equal to 1 if an input has been recieved, 0 if an input has been handled

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
  HAL_Init();
  SystemClock_Config();

  //Enable RCC
	RCC -> APB1ENR |= RCC_APB1ENR_USART3EN; //Enable USART 3
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN; //Enable GPIOB
	
	//Set Up Pins PB10/11 as Alternate Function, push/pull, low speed, no pull-up/down
	GPIOB -> MODER |= (1<<21); //Set PB10 to Alternate Function
	GPIOB -> MODER &= ~(1<<20);
	
	GPIOB -> MODER |= (1<<23); //Set PB11 to Alternate Function
	GPIOB -> MODER &= ~(1<<22);
	
	GPIOB -> OTYPER &= ~(1<<10); //Set PB10 to Push/Pull
	GPIOB -> OTYPER &= ~(1<<11); //Set PB11 to Push/Pull
	
	GPIOB->OSPEEDR &= ~(1<<21); //Set PB10 to low speed
	GPIOB->OSPEEDR &= ~(1<<20);
	
	GPIOB -> OSPEEDR &= ~(1<<23); //Set PB11 to low speed
	GPIOB -> OSPEEDR &= ~(1<<22);
	
	GPIOB -> PUPDR &= ~(1<<21); //Set PB10 to no Pull Up/Down
	GPIOB -> PUPDR &= ~(1<<20);
	
	GPIOB -> PUPDR &= ~(1<<23); //Set PB11 to no Pull Up/Down
	GPIOB -> PUPDR &= ~(1<<22);
	
	//Set up Alternate Function for PB10/11 for AF4
	GPIOB->AFR[1] &= ~(1<<15);
	GPIOB->AFR[1] |= (1<<14);
	GPIOB->AFR[1] &= ~(1<<13);
	GPIOB->AFR[1] &= ~(1<<12);
	
	//Enable USART3
	USART3 -> CR1 |= (1<<2); //Enable RX
	USART3 -> CR1 |= (1<<3); //Enable TX
	
	USART3 -> BRR |= HAL_RCC_GetHCLKFreq()/115200; //Set Baud Rate to 115200 bits/second
	
	NVIC_EnableIRQ(USART3_4_IRQn); // enable USART 3 interrupts
  	NVIC_SetPriority(USART3_4_IRQn, 2);
	
  while (1)
  {
		
  }
}

//USART3 Interrupt Handler
void USART3_4_IRQHandler(void)
{
  inputFlag = 1;
  inputChar = USART3->RDR;
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
