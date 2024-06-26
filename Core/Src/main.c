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
#include "usart.h"
#include "tsc.h"

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

const int arrValue = 255;
const int targetBaud = 9600;
const int gyroSensitivity = 2000;

int AcquisitionValue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void USART3_txColorData(LED_Data data);
void LED_Receive_Color(uint8_t data);
void LED_Transmit_Loop();

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
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  // Initialization functions
  I3G4250D_Initialize();
  USART3_Initialize(targetBaud);
	PollingTSC();
  
  // Enable clocks
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // timer 3
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIO C clock
    
  // Set the timer 3 prescaler to 80 to get to 10us resolution for PWM
  TIM3->PSC &= 0x0;
  TIM3->PSC |= 38;
  
  // Set the timer 3 auto reload point to 125 to get to 800Hz for PWM
  TIM3->ARR &= 0x0;
  TIM3->ARR |= arrValue;
  
  // Configure timer 3 channel 1 and 2 for PWM
  TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk); // configure as output
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // enable preload
  TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
  TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos) | (0x6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1
  
  // Configure timer 3 channel 3 for PWM
  TIM3->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk); // configure as output
  TIM3->CCMR2 |= TIM_CCMR2_OC3PE; // enable preload
  TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk);
  TIM3->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // PWM mode 1
  
  // Enable Capture/Compare for channels 1, 2, and 3 for PWM
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  
  // Configure GPIO C pins 6, 7, and 8 for PWM LED Strip signal generation
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER8_Msk);
  GPIOC->MODER |= (0x2 << GPIO_MODER_MODER6_Pos) | (0x2 << GPIO_MODER_MODER7_Pos) | 
                  (0x2 << GPIO_MODER_MODER8_Pos); // alternate mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8); // push-pull
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR8_Msk); // no pull-up/pull-down
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7); // alternate function 0
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL8); // alternate function 0
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  
  // Enable functions
  I3G4250D_Enable();
  USART3_Enable();
	TSC_Enable();
  
  // Enable timer 3 for PWM
  TIM3->CR1 |= TIM_CR1_CEN; // enable timer 3
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    LED_Transmit_Loop();
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
  * @brief Transmits color data via USART 3
  */
void USART3_txColorData(LED_Data data)
{
  USART3_TxByte((uint8_t)data.color);
  USART3_TxByte((uint8_t)data.data);
  return;
}

/**
  * @brief Read data from gyroscope and touch sensor and send PWM values over USART
  */
void LED_Transmit_Loop()
{
  static uint8_t redPWMValue = 0;
  static uint8_t greenPWMValue = 0;
  static uint8_t bluePWMValue = 0;
  
  static int brightnessDivisor = 1;
  
  static LED_Data redData = {RED, NULL};
  static LED_Data greenData = {GREEN, NULL};
  static LED_Data blueData = {BLUE, NULL};

  int16_t xValue;
  int16_t yValue;
  int16_t zValue;
	
	//int AcquisitionValue;
  
  // read gyro values value
  xValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_X_L_Addr, 2); // Read X low and high
  yValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_Y_L_Addr, 2); // Read Y low and high
  zValue = (int16_t)I3G4250D_ReadRegister(I3G4250D_OUT_Z_L_Addr, 2); // Read Z low and high

  if ((xValue > gyroSensitivity) && (redPWMValue < arrValue)) // x positive
  {
    redPWMValue++; // increase red brightness
  }
  else if ((xValue < -gyroSensitivity) && (redPWMValue > 0)) // x negative
  {
    redPWMValue--; // decrease red brightness
  }
  
  if ((yValue > gyroSensitivity) && (greenPWMValue < arrValue)) // y positive
  {
    greenPWMValue++; // increase green brightness
  }
  else if ((yValue < -gyroSensitivity) && (greenPWMValue > 0)) // y negative
  {
    greenPWMValue--; // decrease green brightness
  }
  
  if ((zValue > gyroSensitivity) && (bluePWMValue < arrValue)) // z positive
  {
    bluePWMValue++; // increase blue brightness
  }
  else if ((zValue < -gyroSensitivity) && (bluePWMValue > 0)) // z negative
  {
    bluePWMValue--; // decrease blue brightness
  }
	
	// Touch sensing acquisition
	AcquisitionValue = TSC_acquisition();

  // If sensor value is in the idle range, leave as is
  if ((AcquisitionValue > TSC_IDLE_MINTHRESHOLD) && (AcquisitionValue < TSC_IDLE_MAXTHRESHOLD))
  {
     brightnessDivisor = brightnessDivisor;
  }
  // If sensor value is in the low range -----> Low brightness PWM = 10%
  else if ((AcquisitionValue > TSC_LOW_MINTHRESHOLD) && (AcquisitionValue < TSC_LOW_MAXTHRESHOLD))
  {
     brightnessDivisor = 10;
  }
  // If sensor value is in the medium range ----> Medium brightness PWM = 50%
  else if ((AcquisitionValue > TSC_MEDIUM_MINTHRESHOLD) && (AcquisitionValue < TSC_MEDIUM_MAXTHRESHOLD))
  {
     brightnessDivisor = 2;
  }
  // If sensor value is in the high range ----> High brightness PWM = 100%
  else if ((AcquisitionValue > TSC_HIGH_MINTHRESHOLD) && (AcquisitionValue < TSC_HIGH_MAXTHRESHOLD))
  {
     brightnessDivisor = 1;
  }
  
  // Assign PWM values to data to send over USART 3
  redData.data = redPWMValue / brightnessDivisor;
  greenData.data = greenPWMValue / brightnessDivisor;
  blueData.data = bluePWMValue / brightnessDivisor;
  
  // Send color data over USART 3
  USART3_txColorData(redData);
  USART3_txColorData(greenData);
  USART3_txColorData(blueData);
}

/**
  * @brief Receives PWM values over USART and sets the colors' brightness values
  *        and denoises the signal by only allowing values within 5 of the previous
  */
void LED_Process_Color(uint8_t data)
{
  static Color activeColor = NULL;
  static uint8_t redPWMValue = 0;
  static uint8_t greenPWMValue = 0;
  static uint8_t bluePWMValue = 0;
  
  if (activeColor == NULL)
  {
    switch (data)
    {
      case RED:
        activeColor = RED;
        break;
      case GREEN:
        activeColor = GREEN;
        break;
      case BLUE:
        activeColor = BLUE;
        break;
      default:
        activeColor = NULL;
    }
    return;
  }
  else
  {
    switch (activeColor)
    {
      case RED:
        if (data < redPWMValue + 5 || data > redPWMValue - 5)
        {
          redPWMValue = data;
        }
        activeColor = NULL;
        break;
      case GREEN:
        if (data < greenPWMValue + 5 || data > greenPWMValue - 5)
        {
          greenPWMValue = data;
        }
        activeColor = NULL;
        break;
      case BLUE:
        if (data < bluePWMValue + 5 || data > bluePWMValue - 5)
        {
          bluePWMValue = data;
        }
        activeColor = NULL;
        break;
      default:
        activeColor = NULL;
    }
  }
  
  TIM3->CCR1 = redPWMValue;
  TIM3->CCR2 = greenPWMValue;
  TIM3->CCR3 = bluePWMValue;
  
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
