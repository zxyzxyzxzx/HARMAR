/*
******************************************************************************
** Filename:  utils.h
**
** Description:
**       This file defines common utility functions (delay, time, etc.)
**       and their associated definitions.
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef UTILS_H_
#define UTILS_H_

/* DelayMs() time selection */
#define SYS_TICK_BASED    0
#define BUSY_WAIT_BASED   1

extern volatile uint32_t DelayCnt;

/* Function Prototypes */
void delayUsec( int usec_delay );
void delayMs(uint32_t ms, uint8_t mode, uint8_t enable_intr);

#endif /* UTILS_H_ */
