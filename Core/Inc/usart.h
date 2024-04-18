/**
  ******************************************************************************
  * @file    usart.h
  * @author  Noah Lomu
  * @brief   This file contains definitions and function prototypes to be used
  *          with the MEMS motion sensor 3-axis digital output gyroscope.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H
#define __USART_H

/****************  Function prototypes (setup/initialization)  ****************/
void USART3_Initialize(int targetBaud);
void USART3_Enable();

/*********************  Function prototypes (read/write)  *********************/
void USART3_TxByte(uint8_t data);
void USART3_txString(char* data);
uint8_t USART3_RxByte();

#endif /* __USART_H */