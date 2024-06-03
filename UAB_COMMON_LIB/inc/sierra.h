/*
******************************************************************************
** Filename:  sierra.h
**
** Description:
**       This file provides all function prototypes specific
**       to the SIERRA platform lift system
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef SIERRA_H_
#define SIERRA_H_

/* Function Prototypes */
void uabRampBoardPioGet(uint8_t *pio);
void uabRampBoardPioSet(uint8_t pio);
int  uabRampBoardRegsGet(uint8_t *reg);
int  uabLeftsideRampRaise(void);
int  uabLeftsideRampLower(void);
int  uabRightsideRampRaise(void);
int  uabRightsideRampLower(void);
int  uabSierraRampBoardInit(void);
void uabSierraAdjustRamps(uint8_t platform_position);
void uabSierraPlatformPositionInit(void);
void uabSierraPlatformCheck(void);

#endif /* SIERRA_H_ */
