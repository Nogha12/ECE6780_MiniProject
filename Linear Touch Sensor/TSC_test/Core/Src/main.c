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


#define TSC_MIN_THRESHOLD       	(1000)
#define TSC_LOW_MAXTHRESHOLD    	(1290)
#define TSC_MEDIUM_MAXTHRESHOLD 	(1270)
#define TSC_HIGH_MAXTHRESHOLD   	(1245)

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  SystemClock_Config();
	
	// 				LED SETUP 					//
	//														//
	// Initialize all of the LED pins in the main function
	// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC9)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	//Set the pins to general-purpose output mode in the MODER register
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER |= (1 << 18);
	//Set the pins to push-pull output type in  the OTYPER register
	GPIOC->OTYPER &= ~(1 << 6);
	GPIOC->OTYPER &= ~(1 << 7);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OTYPER &= ~(1 << 9);
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12));
	GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14));
	GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16));
	GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18));
	// Set to no pull-up/down resistors in the PUPDR register
	GPIOC->PUPDR &= ~((1 << 13) | (1 << 12));
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14));
	GPIOC->PUPDR &= ~((1 << 17) | (1 << 16));
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18));
	// Set LEDs to LOW
	GPIOC->ODR &= ~(1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	GPIOC->ODR &= ~(1 << 8);
	GPIOC->ODR &= ~(1 << 9);
	
	
	// 				TSC SETUP 				  //
	//														//
	// RCC to enable the TSC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_TSCEN;
	
	// RCC to enable the GPIOx peripherals
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Capacitor (sensor): PA2 AF3 ---> Open-Drain
	GPIOA->MODER |= (1 << 5);
	GPIOA->MODER &= ~(1 << 4);
	GPIOA->OTYPER |= (1 << 2);
	GPIOA->AFR[0] &= ~(1 << 11);
	GPIOA->AFR[0] &= ~(1 << 10);
	GPIOA->AFR[0] |= (1 << 9);
	GPIOA->AFR[0] |= (1 << 8);
	
	// Sampling IOs: PA3 AF3, PA7 AF3, PB1 AF3 ----> Push-Pull
	GPIOA->MODER |= (1 << 7);
	GPIOA->MODER &= ~(1 << 6);
	GPIOA->MODER |= (1 << 15);
	GPIOA->MODER &= ~(1 << 14);
	GPIOB->MODER |= (1 << 3);
	GPIOB->MODER &= ~(1 << 2);
	
	GPIOA->OTYPER &= ~(1 << 3);
	GPIOA->OTYPER &= ~(1 << 7);
	GPIOB->OTYPER &= ~(1 << 1);
	
	GPIOA->AFR[0] &= ~(1 << 15);
	GPIOA->AFR[0] &= ~(1 << 14);
	GPIOA->AFR[0] |= (1 << 13);
	GPIOA->AFR[0] |= (1 << 12);
	
	GPIOA->AFR[0] &= ~(1 << 31);
	GPIOA->AFR[0] &= ~(1 << 30);
	GPIOA->AFR[0] |= (1 << 29);
	GPIOA->AFR[0] |= (1 << 28);
	
	GPIOB->AFR[0] &= ~(1 << 7);
	GPIOB->AFR[0] &= ~(1 << 6);
	GPIOB->AFR[0] |= (1 << 5);
	GPIOB->AFR[0] |= (1 << 4);
	
	// Touch sensing controller enable
	TSC->CR |= (1 << 0);
	// Set Acquisition mode to normal
	TSC->CR &= ~(TSC_CR_AM);
	// Charge transfer pulse high to 1 cycle
	TSC->CR &= ~((1 << 28) | (1 << 29) | (1 << 30) | (1 << 31));
	// Charge transfer pulse low to 1 cycle
	TSC->CR &= ~((1 << 24) | (1 << 25) | (1 << 26) | (1 << 27));
	// I/O Default mode float
	TSC->CR |= TSC_CR_IODEF;
	// Max count error interrupt disable
	TSC->IER &= ~(TSC_IER_MCEIE);
	// Max count value to 16383
	TSC->CR |= (TSC_CR_MCV_2 | TSC_CR_MCV_1);
	// Pulse generator prescaler to DIV/64
	TSC->CR |= (TSC_CR_PGPSC_2 | TSC_CR_PGPSC_1);
	// Spread spectrum disabled
	TSC->CR &= ~(TSC_CR_SSE);
	// Spread spectrum deviation to 127
	TSC->CR |= ((1 << 17) | (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23));
	// Spread spectrum prescaler DIV/1
	TSC->CR &= ~(1 << 15);
	// Synchronization pin polarity: falling edge
	TSC->CR &= ~(1 << 3);
	// G1_IO3 channel mode /* TS1 touchkey */
	TSC->IOCCR |= (1 << 2);
	// G1_IO4, G2_IO4, G3_IO3 sampling IOs
	TSC->IOSCR |= (1 << 3) | (1 << 7) | (1 << 10);
	// Disable hysterstesis
	TSC->IOHCR &= (uint32_t)(~(TSC_IOHCR_G1_IO3 | TSC_IOHCR_G1_IO4 | TSC_IOHCR_G2_IO4 | TSC_IOHCR_G3_IO3));

/* (3) Enable end of acquisition IT */
/* (4) Sampling enabled, G2IO4 */
/* (5) Channel enabled, G2IO3 */
/* (6) Enable group, G2 */

	TSC->IER = TSC_IER_EOAIE; /* (3) */
	TSC->IOGCSR |= TSC_IOGCSR_G1E; /* (5) */

	
uint32_t AcquisitionValue = 0;

  while (1)
  {
		/*##-2- Discharge the touch-sensing IOs ##################################*/
    /* Must be done before each acquisition */
		TSC->CR &= ~(1 << 4);
		HAL_Delay(1); /* 1 ms is more than enough to discharge all capacitors */
		TSC->CR |= (1 << 4);
		
		/*##-3- Start the acquisition process ####################################*/
		TSC->CR |= (1 << 1);
		
		// Wait for the completion flag, or an interrupt and read the counter values.
		while ((TSC->ISR & TSC_ISR_EOAF) == TSC_ISR_EOAF)
		{
			// Waiting...
			TSC->ICR = TSC_ICR_EOAIC;
		}
	  if ((TSC->ISR & TSC_ISR_MCEF) == 1)
		{
			// ERROR....
		}
		
		AcquisitionValue = TSC->IOGXCR[1];
		
		if (AcquisitionValue > 1)
		{
			/// TESTyy
			GPIOC->ODR |= (1 << 9);
		}
		
		
		if ((AcquisitionValue > TSC_MIN_THRESHOLD) && (AcquisitionValue < TSC_LOW_MAXTHRESHOLD))
		{
			GPIOC->ODR |= (1 << 6);
			
			if (AcquisitionValue < TSC_MEDIUM_MAXTHRESHOLD)
			{
				GPIOC->ODR |= (1 << 7);
				
				if (AcquisitionValue < TSC_HIGH_MAXTHRESHOLD)
				{
					GPIOC->ODR |= (1 << 8);
				}
				else
				{
					GPIOC->ODR &= ~(1 << 8);
				}
			}
			else
			{
				GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR &= ~(1 << 8);
			}
		}
		else
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR &= ~(1 << 8);
		}
			
    
  }

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
