/**
  ******************************************************************************
  * @file    tsc.c
  * @author  David Venegas
  * @brief   This file contains function definitions for the linear touch 
						 sensor.
  ******************************************************************************
  */
	
#include "main.h"
#include "tsc.h"

/******************************************************************************/
/*                        TSC Set Up and Initialization                     */
/******************************************************************************/
/* NOTE:                                                                      */
/* Polling based linear touch sensor.																					*/
/*	Pins used:             																										*/
/*  Channel sensor IO - PA2                                        						*/
/*  Capacitor sensing IOs - PA3, PA7, PB1																			*/
/*  One sensor, three keys                                   									*/
/******************************************************************************/

/**
  * @brief Fully initialize the linear touch sensor
  */
	void PollingTSC()
	{
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
	}
	
	/**
  * @brief Enable the touch sensor
  */
	void TSC_Enable()
	{
			TSC->CR |= TSC_CR_TSCE;
	}
	
	/**
  * @brief Start acquisition and retrieve sensing data
  */
	int TSC_acquisition()
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
		
		// RESTART acquisition
		TSC->CR |= TSC_CR_START;
		
		return TSC->IOGXCR[0];
	}