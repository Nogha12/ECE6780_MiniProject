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
#define TSC_LOW_MAXTHRESHOLD    	(1347)
#define TSC_MEDIUM_MAXTHRESHOLD 	(1325)
#define TSC_HIGH_MAXTHRESHOLD   	(1300)

void SystemClock_Config(void);
void Trans_Character(char ch);
void Trans_String(size_t n, char string[]);
char* itoa(int value, char* result, int base);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  SystemClock_Config();
	
	// 				USART SETUP 				  //
	//														//
	// RCC to enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Set the pins to alternate function mode in the MODER register
	// For PC10 -> MODER10 [10] for bit 21 and 20
	GPIOC->MODER |= (1 << 21);
	GPIOC->MODER &= ~(1 << 20);
	// For PC11 -> MODER11 [10] for bit 23 and 22
	GPIOC->MODER |= (1 << 23);
	GPIOC->MODER &= ~(1 << 22);
	// For PC12 -> MODER12 [10] for bit 25 and 24
	GPIOC->MODER |= (1 << 25);
	GPIOC->MODER &= ~(1 << 24);
	
	// Set alternate function registers: 
	// PC10 -> USART3_TX -> AF1
	// PC11 -> USART3_RX -> AF1
	// PC12 -> USART3_CK -> AF1
	// For SEL10 AF1 [0001] for bit [11:8]
	GPIOC->AFR[1] |= (1 << 8);
	GPIOC->AFR[1] &= ~((1 << 9) | (1 << 10) | (1 << 11));
	// For SEL11 AF1 [0001] for bit [15:12]
	GPIOC->AFR[1] |= (1 << 12);
	GPIOC->AFR[1] &= ~((1 << 13) | (1 << 14) | (1 << 15));
	// For SEL12 AF1 [0001] for bit [19:16]
	GPIOC->AFR[1] |= (1 << 16);
	GPIOC->AFR[1] &= ~((1 << 17) | (1 << 18) | (1 << 19));
	
	
	// RCC to enable the USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the Baud rate for communication to be 115200 bits/second
	USART3->BRR = 69;   // Target 69.444... Error of 0.64%
	
	// Enable the transmitter and receiver hardware
	USART3->CR1 |= (1 << 2);	// Receiver Enable
	USART3->CR1 |= (1 << 3);	// Transmitter Enable
	
	// Enable the receive register not empty interrupt
	USART3->CR1 |= (1 << 5); 	// RXNEIE Enable
	
	// The USART has a peripheral enable/disable bit in its control register
	USART3->CR1 |= (1 << 0);	// USART3 Enable
	
	
	// Initialize all of the LED pins in the main function
	// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC9)
	
	//Set the pins to general-purpose output mode in the MODER register
	// For PC6 -> MODER6 [01] for bit 13 and 12
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	// For PC7 -> MODER7 [01] for bit 15 and 14
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);
	// For PC8 -> MODER8 [01] for bit 17 and 16
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	// For PC9 -> MODER9 [01] for bit 19 and 18
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER |= (1 << 18);
	
	//Set the pins to push-pull output type in  the OTYPER register
	// For PC6 -> OTYPER [0] for bit 6
	GPIOC->OTYPER &= ~(1 << 6);
	// For PC7 -> OTYPER [0] for bit 7
	GPIOC->OTYPER &= ~(1 << 7);
	// For PC8 -> OTYPER [0] for bit 8
	GPIOC->OTYPER &= ~(1 << 8);
	// For PC9 -> OTYPER [0] for bit 9
	GPIOC->OTYPER &= ~(1 << 9);
	
	// Set the pins to low speed in the OSPEEDR register
	// For PC6 -> OSPEEDR [x0] for bit 13 and 12
	GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12));
	// For PC7 -> OSPEEDR [x0] for bit 15 and 14
	GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14));
	// For PC8 -> OSPEEDR [x0] for bit 17 and 16
	GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16));
	// For PC9 -> OSPEEDR [x0] for bit 19 and 18
	GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18));
	
	// Set to no pull-up/down resistors in the PUPDR register
	// FOR PC6 -> PUPDR [00] for bit 13 and 12
	GPIOC->PUPDR &= ~((1 << 13) | (1 << 12));
	// FOR PC7 -> PUPDR [00] for bit 15 and 14
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14));
	// FOR PC8 -> PUPDR [00] for bit 17 and 16
	GPIOC->PUPDR &= ~((1 << 17) | (1 << 16));
	// FOR PC9 -> PUPDR [00] for bit 19 and 18
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18));
	
		
	
	
	// 				TSC SETUP 				  //
	//														//
	// RCC to enable the TSC, GPIOA, GPIOB peripheral clock
	RCC->AHBENR |= RCC_AHBENR_TSCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Channel (sensor): PA2 AF3 ---> Push-Pull
	GPIOA->MODER |= GPIO_MODER_MODER2_1;
	GPIOA->AFR[0] |= 0x00000300;
	GPIOA->OTYPER &= ~(1 << 2);
	// Capacitor Sampling IOs: PA3 AF3, PA7 AF3, PB1 AF3 ----> Open-Drain 
	GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER7_1);
	GPIOA->AFR[0] |= (0x00003000 | 0x30000000);
	GPIOA->OTYPER |= ((1 << 3) | (1 << 7));
	GPIOB->MODER |= GPIO_MODER_MODER1_1;
	GPIOB->AFR[0] |= 0x00000030; 
	GPIOB->OTYPER |= (1 << 1);
	// Acquisition Mode Normal
	TSC->CR &= ~(TSC_CR_AM);
	// Pulse High Length 1 cycle
	TSC->CR &= ~((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28));
	// Pulse Low Length 1 cycle
	TSC->CR &= ~((1 << 27) | (1 << 26) | (1 << 25) | (1 << 24));
	// IO Float mode
	TSC->CR |= TSC_CR_IODEF;
	// Max Count Interrupt disable
	TSC->IER &= ~(1 << 1);
	// Max Count Value: 16383
	TSC->CR |= (1 << 7) | (1 << 6);
	TSC->CR &= ~(1 << 5);
	// Pulse Generator Prescaler: 64
	TSC->CR |= (1 << 14) |(1 << 13);
	TSC->CR &= ~(1 << 12);
	// Spread Spectrum disable
	TSC->CR &= ~(1 << 16);
	// Spread Spectrum Deviation: 127
	TSC->CR |= ((1 << 23) | (1 << 22) | (1 << 21) | (1 << 20) | (1 << 19) | (1 << 18) | (1 << 17));
	// Spread Spectrum Prescaler: 1
	TSC->CR &= ~(1 << 15);
	// Synch Pin Polarity: falling
	TSC->CR &= ~(1 << 3);
	// Channel IOs: G1_IO3 /* TS1 touchkey */
	TSC->IOCCR |= (1 << 2);
	// Sampling IOs: G1_IO4, G2_IO4, G3_IO3
	TSC->IOSCR |= ((1 << 3) | (1 << 7) | (1 << 10));
	// Disable hysterstesis for all IOs
	TSC->IOHCR &= ~((1 << 2) | (1 << 3) | (1 << 7) | (1 << 10));
	// Enable TSC
	TSC->CR |= (1 <<0);
	
	int AcquisitionValue = 0;
	

  while (1)
  {
		// Discharge the touch-sensing IOs
		TSC->CR &= ~(1 << 4);
		HAL_Delay(1);
		TSC->CR |= (1 << 4);
		
		// Enable G1_IO3 as channel
		TSC->IOCCR |= (1 << 2);
		// Enable G1_IO4, , G2_IO4, G3_IO3 as sampling
		TSC->IOSCR |= ((1 << 3) | (1 << 7) | (1 << 10));
		// Enable G1, G2, G3 Analog group
		TSC->IOGCSR |= ((1 << 0) |(1 << 1) | (1 << 2));
		
		// Wait for the completion flag, or an interrupt and read the counter values
		while((TSC->ISR & TSC_ISR_EOAF) == TSC_ISR_EOAF)
		{
			// Clear Flags
			TSC->ICR |= (1 << 0);
			TSC->ICR |= (1 << 1);
		}
		
		// Print to USART
		char s[16];
	  itoa(TSC->IOGXCR[0], s, 10);
		// New Line
		char msg0[] = "\r\n";
		size_t n0 = (sizeof(msg0) / sizeof(msg0[0]));
	  Trans_String(16,s);
	  Trans_String(n0,msg0);	
  	// RESTART acquisition
		TSC->CR |= (1 << 1);
		
		AcquisitionValue = TSC->IOGXCR[0];
		// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC9)
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
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR &= ~(1 << 7);
			}
		}
		else
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR &= ~(1 << 8);
		}
		
		HAL_Delay(100); // SLOW DATA CAPTURE FOR TESTING
		
		
  }

}

char* itoa(int value, char* result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
  
    // Reverse the string
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

void Trans_Character(char ch)
{
	// ------4.9.2 — Blocking Transmission------ //
	
	// Check and wait on the USART status flag that indicates the transmit register is empty
	while(!(USART3->ISR & (1 << 7)))	// Triggers if bit 7 (TXE) is NOT set
	{
		// Empty
	}
	
	// Write the character into the transmit data register
	USART3->TDR = ch;
	
}

/**
  * @brief  Transmits a string on the USART
  * @retval void
  */
void Trans_String(size_t n, char string[])
{
	
	int i;
	// Loop over each element of the array and transmit
	for (i = 0; i < n; i++)
	{
		// Return if the array is terminated
		if (string[i] == 0)
		{
			return;
		}
		
		Trans_Character(string[i]);
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
