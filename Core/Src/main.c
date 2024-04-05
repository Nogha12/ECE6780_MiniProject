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

#include "i3g4250d.h"

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

  //int clkSpeed;
  //int targetBaud = 115200;
  
  int returnValue;
  int16_t xValue;
  int16_t yValue;
  int16_t zValue;
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  // Enable clocks
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // timer 3
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable GPIO B clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIO C clock
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // enable SPI 2 clock
  //RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // enable USART 3 clock
  
  // Configure GPIO C pins 6, 7, and 8 for PWM LED Strip signal generation
  /*
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk);
  GPIOC->MODER |= (0x2 << GPIO_MODER_MODER6_Pos) | (0x2 << GPIO_MODER_MODER7_Pos) | 
                  (0x2 << GPIO_MODER_MODER8_Pos); // alternate mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8); // push-pull
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk); // no pull-up/pull-down
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7); // alternate function 0
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL8); // alternate function 0
  */
  
  // Configure GPIO C pins 6, 7, 8, and 9 (LED pins)
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
  GPIOC->MODER |= (0x1 << GPIO_MODER_MODER6_Pos) | (0x1 << GPIO_MODER_MODER7_Pos) | 
                  (0x1 << GPIO_MODER_MODER8_Pos) | (0x1 << GPIO_MODER_MODER9_Pos); // output mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9); // push-pull
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9); // no pull-up/pull-down
  
  /* NOTE:
     To connect the gyroscope via SPI, the following pins are used:
      SPI serial port clock (SPC) - PB13
      SPI serial data output (SDO) - PB14
      SPI serial data input (SDI) - PB15
      SPI enable (enable low) - PC0
      Programmable interrupt - PC1
      FIFO interrupt - PC2
     This requires the use of SPI 2.
     The gyroscope wants the clock to have a high-level idle state,
     so CR1[CPOL] should be set.
     The gyroscope latches on clock high, so CR1[CPHA] should be set.
   */
   
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
    
  // Set the timer 3 prescaler to 80 to get to 10us resolution
  TIM3->PSC &= 0x0;
  TIM3->PSC |= 79;
  
  // Set the timer 3 auto reload point to 125 to get to 800Hz
  TIM3->ARR &= 0x0;
  TIM3->ARR |= 125;
  
  // Configure timer 3 channel 1 and 2
  TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk); // configure as output
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // enable preload
  TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
  TIM3->CCMR1 |= 0x6 << TIM_CCMR1_OC1M_Pos | 0x6 << TIM_CCMR1_OC2M_Pos; // PWM mode 1
  
  // Configure timer 3 channel 3
  TIM3->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk); // configure as output
  TIM3->CCMR2 |= TIM_CCMR2_OC3PE; // enable preload
  TIM3->CCMR1 &= ~(TIM_CCMR2_OC3M_Msk);
  TIM3->CCMR2 |= 0x6 << TIM_CCMR2_OC3M_Pos; // PWM mode 1
  
  // Enable Capture/Compare for channels 1, 2, and 3
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  
  // Set the compare registers to 20%, 60%, and 100%
  TIM3->CCR1 &= 0x0000;
  TIM3->CCR1 |= 25;
  TIM3->CCR2 &= 0x0000;
  TIM3->CCR2 |= 75;
  TIM3->CCR3 &= 0x0000;
  TIM3->CCR3 |= 125;
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  
  //TIM3->CR1 |= TIM_CR1_CEN; // enable timer 3
  
  GPIOC->ODR |= GPIO_ODR_0; // set pin C0 high when SPI is not ongoing
  SPI2->CR1 |= SPI_CR1_SPE; // enable SPI 2
  
  returnValue = I3G4250D_ReadRegister(I3G4250D_WHO_AM_I_Addr, 1); // Read WHO_AM_I
  if (returnValue != I3G4250D_WHO_AM_I_Value) // Verify that the gyroscope matches the known WHO_AM_I
  {
    Error_Handler();
  }
  
  I3G4250D_WriteToRegister(I3G4250D_CTROL_REG1_Addr, 
                           (I3G4250D_CTRL_REG1_Xen | I3G4250D_CTRL_REG1_Yen | 
                            I3G4250D_CTRL_REG1_Zen | I3G4250D_CTRL_REG1_PD)
                          ); // enable X, Y, Z, and normal mode
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // read gyro values value
    xValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_X_L_Addr, 2); // Read X low and high
    yValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_Y_L_Addr, 2); // Read Y low and high
    zValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_Z_L_Addr, 2); // Read Z low and high
    
    if (xValue > 4000) // x positive
    {
      GPIOC->ODR |= GPIO_ODR_8; // Turn on pin C8 (orange LED)
      GPIOC->ODR &= ~(GPIO_ODR_9); // Turn off pin C9 (green LED)
    }
    else if (xValue < -4000) // x negative
    {
      GPIOC->ODR |= GPIO_ODR_9; // Turn on pin C9 (green LED)
      GPIOC->ODR &= ~(GPIO_ODR_8); // Turn off pin C8 (orange LED)
    }
    
    if (yValue > 4000) // y positive
    {
      GPIOC->ODR |= GPIO_ODR_7; // Turn on pin C7 (red LED)
      GPIOC->ODR &= ~(GPIO_ODR_6); // Turn off pin C6 (blue LED)
    }
    else if (yValue < -4000) // y negative
    {
      GPIOC->ODR |= GPIO_ODR_6; // Turn on pin C6 (blue LED)
      GPIOC->ODR &= ~(GPIO_ODR_7); // Turn off pin C7 (red LED)
    }
    
    HAL_Delay(100);
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
  
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
  *(uint8_t *)&(SPI2->DR) = ctrlByte; // transmit the control byte
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  (void)SPI2->DR; // clear byte from read register
  
  for (int i = 0; i < bytesToRead; i++)
  {
    *(uint8_t *)&(SPI2->DR) = 0xFF; // transmit a dummy byte
    while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
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
  
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
  *(uint8_t *)&(SPI2->DR) = ctrlByte; // transmit the control byte
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  *(uint8_t *)&(SPI2->DR) = data; // transmit the data
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmit buffer to be empty
  while (SPI2->SR & SPI_SR_BSY); // wait for SPI to finish
  
  GPIOC->ODR |= GPIO_ODR_0; // disable I3G4250D
  
  while (SPI2->SR & SPI_SR_RXNE) (void)SPI2->DR; // clear the receive buffer
  
  return;
}

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
