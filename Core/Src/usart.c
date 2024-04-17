/**
  ******************************************************************************
  * @file    usart.c
  * @author  Noah Lomu
  * @brief   This file contains function definitions for communicating via
  *          USART.
  ******************************************************************************
  */
  
#include "main.h"
#include "usart.h"

USART_TypeDef* activeUSART;

/******************************************************************************/
/*                        USART Set Up and Initialization                     */
/******************************************************************************/
/**
  * @brief Fully initialize use of USART 3
  */
void USART3_Initialize(int targetBaud)
{
  auto int clkSpeed;
  
  // Enable clocks
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // enable USART 3 clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIO C clock
  
  // Configure GPIO C pins 10 and 11 to connect to USART 3
  GPIOC->MODER &= ~(GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);
  GPIOC->MODER |= (0x2 << GPIO_MODER_MODER10_Pos) | (0x2 << GPIO_MODER_MODER11_Pos); // alternate mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11); // push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11); // low-speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11); // no pull-up/pull-down
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
  GPIOC->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL10_Pos) | (0x1 << GPIO_AFRH_AFSEL11_Pos); // alternate function 1
  
  // Configure USART 3
  clkSpeed = HAL_RCC_GetHCLKFreq();
  USART3->CR1 |= USART_CR1_RE | USART_CR1_TE; // enable TX and RX
  USART3->CR1 |= USART_CR1_RXNEIE; // enable interrupts from receive register not empty
  USART3->BRR &= 0x0000;
  USART3->BRR |= clkSpeed / targetBaud; // set baud rate clock divisor
}

/**
  * @brief Enable USART 3
  */
void USART3_Enable()
{
  USART3->CR1 |= USART_CR1_UE; // enable USART 3
}

/******************************************************************************/
/*                     USART Transmit and Receive Functions                   */
/******************************************************************************/
/**
  * @brief Transmits a byte via USART 3
  */
void USART3_TxByte(uint8_t data)
{
  while (!(USART3->ISR & USART_ISR_TXE));
  USART3->TDR = data;
  return;
}

/**
  * @brief Transmits a string via USART 3
  */
void USART3_txString(char* data)
{
  for (int i = 0; data[i] != '\0'; i++)
  {
    USART3_TxByte((uint8_t)data[i]);
  }
  return;
}

/**
  * @brief Receives a byte via USART 3
  */
uint8_t USART3_RxByte(void)
{
  while (!(USART3->ISR & USART_ISR_RXNE));
  return USART3->RDR;
}
