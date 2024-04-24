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
//#define TSC_MIN_THRESHOLD       	(1150)
//#define TSC_MAX_THRESHOLD       	(1300)
//#define TSC_LOW_MAXTHRESHOLD    	(1340)
//#define TSC_MEDIUM_MAXTHRESHOLD 	(1325)
//#define TSC_HIGH_MAXTHRESHOLD   	(1300)
#define TSC_IDLE_MINTHRESHOLD     (1310)
#define TSC_IDLE_MAXTHRESHOLD     (1330)
#define TSC_LOW_MINTHRESHOLD     (1248)
#define TSC_LOW_MAXTHRESHOLD     (1253)
#define TSC_MEDIUM_MINTHRESHOLD     (1290)
#define TSC_MEDIUM_MAXTHRESHOLD     (1393)
#define TSC_HIGH_MINTHRESHOLD     (1241)
#define TSC_HIGH_MAXTHRESHOLD     (1246)

/****************  Function prototypes (setup/initialization)  ****************/
void PollingTSC();
void TSC_Enable();

/*********************  Function prototypes (Polling Acquisition Mode)  *********************/
int TSC_acquisition();

#endif /* __TSC_H */