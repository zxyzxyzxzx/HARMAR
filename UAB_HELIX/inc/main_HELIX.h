/*
******************************************************************************
** Filename:  main_HELIX.h
**
** Description:
**       This file provides all generic definitions applicable to
**       the HELIX system using the UAB module.
**
** Copyright (C) 2015 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef MAIN_HELIX_H_
#define MAIN_HELIX_H_

/*
** Uncomment this #define if doing development debug
** allowing UART1 to be used as a debug port.
*/
#define HELIX_DEV_DEBUG

/* Major/Minor values MUST be changed for each release */
#define   UAB_HELIX_MAJOR_VERSION 1
#define   UAB_HELIX_MINOR_VERSION 7

/* HELIX-specific Current Sense Speed Control (CSSC) defines */
#define  HELIX_NOMINAL_PWM        85.0f  /* xx67% PWM default 85*/
#define  HELIX_PWM_SCALE_FACTOR   1.5f   /* Testing Determined Scale Factor was 2.8,default 1.5 */

#endif /* MAIN_HELIX_H_ */
