/*
******************************************************************************
** Filename:  speed.h
**
** Description:
**       This file contains #defines and function prototypes
**       that pertain to the Current Sense Speed Control (CSSC)
**       functionality on the Universal Access Board(UAB).
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef SPEED_H_
#define SPEED_H_

extern uint8_t UabAdjustTargetPwmDelay;
extern float   UabNominalPwm;
extern float   UabPwmScaleFactor;

void uabPwmAdjust( uint8_t direction );
void uabCsscBegin( uint32_t adcNum );
void uabCsscCheck( uint32_t adcNum );
void uabCsscEnd(void);

#endif /* SPEED_H_ */
