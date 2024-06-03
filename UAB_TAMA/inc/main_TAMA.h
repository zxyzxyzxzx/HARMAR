/*
******************************************************************************
** Filename:  main_TAMA.h
**
** Description:
**       This file provides all generic definitions applicable to
**       the TAMA system using the UAB module.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef MAIN_TAMA_H_
#define MAIN_TAMA_H_

/*
** Uncomment this #define if doing development debug
** allowing UART1 to be used as a debug port.
*/
#define TAMA_DEV_DEBUG

/* Major/Minor values MUST be changed for each release */
#define   UAB_TAMA_MAJOR_VERSION 3
#define   UAB_TAMA_MINOR_VERSION 5

/* TAMA-specific Current Sense Speed Control (CSSC) defines */
#define  TAMA_NOMINAL_PWM        60.0f  /* 60% PWM */
#define  TAMA_PWM_SCALE_FACTOR   2.2f   /* Testing Determined Scale Factor */

#endif /* MAIN_TAMA_H_ */
