/**
  ******************************************************************************
  * @file    tsc.h
  * @author  David Venegas
  * @brief   This file contains definitions and function prototypes for the 
						 linear touch sensor.
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSC_H
#define __TSC_H

/************************  Sensing value definitions  *************************/
#define TSC_MIN_THRESHOLD       	(1000)
#define TSC_LOW_MAXTHRESHOLD    	(1347)
#define TSC_MEDIUM_MAXTHRESHOLD 	(1325)
#define TSC_HIGH_MAXTHRESHOLD   	(1300)

/****************  Function prototypes (setup/initialization)  ****************/
void PollingTSC();
void TSC_Enable();

/*********************  Function prototypes (Polling Acquisition Mode)  *********************/
int TSC_acquisition();

#endif /* __TSC_H */