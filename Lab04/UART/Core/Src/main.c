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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void sendChar(char);
void sendString(char*);
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
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC
	
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
	GPIOB -> AFR[1] |= (0x4 << GPIO_AFRH_AFSEL11_Pos) | (0x4 << GPIO_AFRH_AFSEL10_Pos);

	//Enable LEDs (GPIOC Pins 6-9)
	GPIOC -> MODER |= (1<<18); //Set Green LED to Output
	GPIOC -> OTYPER &= (0<<9); //Set Green LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<18); //Set Green LED to Low Speed
	GPIOC -> PUPDR &= (0<<18); //Set Green LED to no Pull up/down
	
	GPIOC -> MODER |= (1<<12); //Set Red LED to Output
	GPIOC -> OTYPER &= (0<<6); //Set Red LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<12); //Set Red LED to Low Speed
	GPIOC -> PUPDR &= (0<<12); //Set Red LED to no Pull up/down
	
	GPIOC -> MODER |= (1<<14); //Set Blue LED to Output
	GPIOC -> OTYPER &= (0<<7); //Set Blue LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<14); //Set Blue LED to Low Speed
	GPIOC -> PUPDR &= (0<<14); //Set Blue to no Pull up/down
	
	GPIOC -> MODER |= (1<<16); //Set Orange LED to Output
	GPIOC -> OTYPER &= (0<<8); //Set Orange LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<16); //Set Orange LED to Low Speed
	GPIOC -> PUPDR &= (0<<16); //Set Orange LED to no Pull up/down
	
	//Enable USART3
	USART3 -> CR1 |= (1<<2); //Enable RX
	USART3 -> CR1 |= (1<<3); //Enable TX
	
	USART3 -> BRR |= HAL_RCC_GetHCLKFreq()/115200; //Set Baud Rate to 115200 bits/second
	
	int LED = 0; //integer to keep track of what LED is being picked
	int secondChar = 0; //integer to keep track if first char has been recieved

	//Enable USART3 before While Loop
	USART3->CR1 |= 1;
	sendString("Awaiting input: \n\r");
  while (1)
  {
		/*//Part 1 Code (single character input)
		//HAL_Delay(100); //Check every second for input character
		if((USART3 -> ISR & (1<<5)) == (1 << 5)){ //check an input has been recieved
			inputChar = USART3 -> RDR;
			switch(inputChar){ //check what the input character is
				case'r':
					GPIOC -> ODR ^= (1<<6); //Toggle Red LED
					break;
				case'b':
					GPIOC -> ODR ^= (1<<7); //Toggle Blue LED
					break;
				case'o':
					GPIOC -> ODR ^= (1<<8); //Toggle Orange LED
					break;
				case'g':
					GPIOC -> ODR ^= (1<<9); //Toggle Green LED
					break;
				default:
					sendString("Error: Incorrect Input \n\r");
					break;
			}
			USART3 -> ISR &= (0<<5);
		}*/
		
		//Part 2 Code (double character input)
		HAL_Delay(100); //Check every 100ms for input character
		while(((USART3 -> ISR & (1<<5)) == (1 << 5))){ //check an input has been recieved
			inputChar = USART3 -> RDR;
			switch(inputChar){ //check what the input character is
				case'r':
					LED = 6;
					break;
				case'b':
					LED = 7;
					break;
				case'o':
					LED = 8;
					break;
				case'g':
					LED = 9;
					secondChar = 1;
					break;
				case '0':
					if(LED<6){ //Check to make sure valid LED was selected
						sendString("Error: First Character must be r, b, o, or g\n\r");
						break;
					}
					GPIOC -> ODR &= (0<<LED); //Turn LED off
					sendString("LED ");
					sendChar(LED);
					sendString("was turned off \n\r");
					USART3 -> ISR &= (0<<5); //Reset input flag
					LED = 0;
					sendString("Awaiting Input: \n\r");//Ask for next input
					break;
				case '1':
					if(LED<6){ //Check to make sure valid LED was selected
						sendString("Error: First Character must be r, b, o, or g \n\r");
						break;
					}
					GPIOC -> ODR |= (1<<LED); //Turn LED on
					sendString("LED ");
					sendChar(LED);
					sendString("was turned on \n\r");
					USART3 -> ISR &= (0<<5); //Reset input flag
					LED = 0;
					sendString("Awaiting Input: \n\r");//Ask for next input
					break;
				case '2':
					if(LED<6){ //Check to make sure valid LED was selected
						sendString("Error: First Character must be r, b, o, or g\n\r");
						break;
					}
					GPIOC -> ODR ^= (1<<LED); //Toggle LED
					sendString("LED ");
					sendChar(LED);
					sendString("was toggled \n\r");
					USART3 -> ISR &= (0<<5); //Reset input flag
					sendString("Awaiting Input: \n\r");//Ask for next input
					LED = 0;
					break;
				default: //Default should only be touched if input was incorrect (besides incorrect LED character)
					sendString("Error: Incorrect Input \n\r");
					USART3 -> ISR &= (0<<5); //Reset input flag
					LED = 0;
					sendString("Awaiting Input: \n\r");//Ask for next input
					break;
			}
		}
  }
}

//USART3 Interrupt Handler
void USART3_4_IRQHandler(void)
{
  inputFlag = 1;
  inputChar = USART3->RDR;
}

//Send characters to Computer
void sendChar(char toSend)
{
  while ((USART3->ISR & (1 << 7)) == 0); 
  USART3->TDR = toSend;
  return;
}

//Send strings to Computer
void sendString(char* toSend)
{
	//Send each character at a time
  for (int i = 0; toSend[i] != '\0'; i++)
  {
    sendChar(toSend[i]);
  }
  return;
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
