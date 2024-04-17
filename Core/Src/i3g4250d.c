/**
  ******************************************************************************
  * @file    i3g4250d.c
  * @author  Noah Lomu
  * @brief   This file contains function definitions for communicating with the
  *          MEMS motion sensor 3-axis digital output gyroscope.
  ******************************************************************************
  */
  
#include "main.h"
#include "i3g4250d.h"

/******************************************************************************/
/*          I3F4250D MEMS Gyroscope and SPI Set Up and Initialization         */
/******************************************************************************/
/* NOTE:                                                                      */
/* To connect the gyroscope via SPI, the following pins are used:             */
/*  SPI serial port clock (SPC) - PB13                                        */
/*  SPI serial data output (SDO) - PB14                                       */
/*  SPI serial data input (SDI) - PB15                                        */
/*  SPI enable (enable low) - PC0                                             */
/*  Programmable interrupt - PC1                                              */
/*  FIFO interrupt - PC2                                                      */
/* This requires the use of SPI 2.                                            */
/* The gyroscope wants the clock to have a high-level idle state,             */
/* so CR1[CPOL] should be set.                                                */
/* The gyroscope latches on clock high, so CR1[CPHA] should be set.           */
/******************************************************************************/
/**
  * @brief Fully initialize communication with the I3G4250D via SPI 2
  */
void I3G4250D_Initialize()
{
  // Enable clocks
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // enable SPI 2 clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable GPIO B clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIO C clock
  
  // Configure GPIO B pins 13, 14, and 15 for SPI 2
  GPIOB->MODER &= ~(GPIO_MODER_MODER13_Msk | GPIO_MODER_MODER14_Msk | GPIO_MODER_MODER15_Msk);
  GPIOB->MODER |= (0x2 << GPIO_MODER_MODER13_Pos) | (0x2 << GPIO_MODER_MODER14_Pos) | 
                  (0x2 << GPIO_MODER_MODER15_Pos); // alternate mode
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15); // push-pull
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13_Msk | GPIO_PUPDR_PUPDR14_Msk | GPIO_PUPDR_PUPDR15_Msk); // no pull-up/pull-down
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13 | GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15); // alternate function 0
  
  // Configure GPIO C pin 0 for gyroscope enable
  GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk);
  GPIOC->MODER |= (0x1 << GPIO_MODER_MODER0_Pos); // output mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0); // push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0); // low-speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0); // no pull-up/pull-down
  
  // Configure SPI 2 as master
  SPI2->CR1 &= ~(SPI_CR1_BR_Msk);
  SPI2->CR1 |= (0x1 << SPI_CR1_BR_Pos); // set baud rate to clock/4
  SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; // clock idle high, rising edge capture
  SPI2->CR1 &= ~(SPI_CR1_LSBFIRST); // MSB first
  SPI2->CR1 |= SPI_CR1_SSM; // software NSS management
  SPI2->CR1 |= SPI_CR1_MSTR; // master configuration
  SPI2->CR2 |= (0x7 << SPI_CR2_DS_Pos); // set data size to 8-bit
  SPI2->CR2 |= SPI_CR2_SSOE; // slave select output enable
  SPI2->CR2 |= SPI_CR2_FRXTH; // Receive buffer not empty at 8 bits
}

/**
  * @brief Enable the gyroscope and check that it is communicating correctly
  */
void I3G4250D_Enable()
{
  auto uint8_t whoAmI;
  
  // Enable SPI to the gyroscope
  GPIOC->ODR |= GPIO_ODR_0; // set pin C0 high when SPI is not ongoing
  SPI2->CR1 |= SPI_CR1_SPE; // enable SPI 2
  
  // Check that commnication is working by reading the WHO_AM_I value
  whoAmI = I3G4250D_ReadRegister(I3G4250D_WHO_AM_I_Addr, 1); // read WHO_AM_I
  if (whoAmI != I3G4250D_WHO_AM_I_Value) // verify that it matches the known WHO_AM_I
  {
    Error_Handler();
    return;
  }
  
  // Enable the X, Y, and Z axes and switch to normal mode
  I3G4250D_WriteToRegister(I3G4250D_CTROL_REG1_Addr, 
                           (I3G4250D_CTRL_REG1_Xen | I3G4250D_CTRL_REG1_Yen | 
                            I3G4250D_CTRL_REG1_Zen | I3G4250D_CTRL_REG1_PD)
                          ); 
}

/******************************************************************************/
/*           I3F4250D MEMS Gyroscope Communication Over SPI Functions         */
/******************************************************************************/
/**
  * @brief Reads data from the I3G4250D gyroscope
  */
int I3G4250D_ReadRegister(uint8_t registerAddr, int bytesToRead)
{
  int data = 0;
  int inByte;
  uint8_t ctrlByte = registerAddr | I3G4250D_SPI_RW | I3G4250D_SPI_MS; // set to read and multiple-byte
  
  if (bytesToRead == 1)
  {
    ctrlByte &= ~(I3G4250D_SPI_MS); // set to single-byte
  } 
  else if (bytesToRead > sizeof(int) || bytesToRead < 1)
  {
    Error_Handler();
    return -1;
  }
  
  GPIOC->ODR &= ~(GPIO_ODR_0); // enable I3G4250D
  
  while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
  *(uint8_t *)&(SPI2->DR) = ctrlByte; // transmit the control byte
  while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  (void)SPI2->DR; // clear byte from read register
  
  for (int i = 0; i < bytesToRead; i++)
  {
    *(uint8_t *)&(SPI2->DR) = 0xFF; // transmit a dummy byte
    while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
    while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
    
    inByte = (uint8_t)SPI2->DR; // read byte from data register
    data |= (inByte & 0xFF) << i*8; // shift byte into data
  }
  
  GPIOC->ODR |= GPIO_ODR_0; // disable I3G4250D
  
  return data;
}

/**
  * @brief Writes data to the I3G4250D gyroscope
  */
void I3G4250D_WriteToRegister(uint8_t registerAddr, uint8_t data)
{
  uint8_t ctrlByte = registerAddr & ~(I3G4250D_SPI_RW | I3G4250D_SPI_MS); // set to write and single byte
  
  GPIOC->ODR &= ~(GPIO_ODR_0); // enable I3G4250D
  
  while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
  *(uint8_t *)&(SPI2->DR) = ctrlByte; // transmit the control byte
  while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  *(uint8_t *)&(SPI2->DR) = data; // transmit the data
  while (!(SPI2->SR & SPI_SR_TXE)); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  GPIOC->ODR |= GPIO_ODR_0; // disable I3G4250D
  
  while (SPI2->SR & SPI_SR_RXNE) (void)SPI2->DR; // clear the receive buffer
  
  return;
}