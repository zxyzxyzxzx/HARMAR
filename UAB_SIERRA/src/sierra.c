/*
******************************************************************************
** Filename:  sierra.c
**
** Description:
**       This file contains code that is specific to the
**       SIERRA Platform Lift system.  
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include "LPC122x.h"
#include "main.h"
#include "utils.h"
#include "uab.h"
#include "alarms.h"
#include "uart.h"

/*
******************************************************************************
** Function:     uabRampBoardPioGet
**
** Description:  Gets the Sierra Ramp Board PIO Register contents
**               using UART1 Serial port communication.
**
** Input  Parms: *pio   Ptr to callers Ramp PIO Register
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabRampBoardPioGet(uint8_t *pio)
{
    uint8_t done = FALSE;
    uint8_t retry  = 0;

    while ((!done) && (retry++ < 3))
    {
        UART1_Count = 0;
        UART1_Status = 0;
        UART1_Buffer[0] = 0;
        UART1_XmitBuf[0] = 'I';
        UART1_XmitBuf[1] = 'P';
        UARTSend(UART1, UART1_XmitBuf, 2);
        delayMs(5, BUSY_WAIT_BASED, TRUE);
        if (UART1_Status == 0)
        {
            if (UART1_Count == 1)
            {
                *pio = UART1_Buffer[0];
                done = TRUE;
            }
        }
    }
    if (!done)
    {
        /* Ramp Board UART Comm Failure */
        uabMajorFaultHandler(UAB_MAJOR_FAULT_RAMP_BOARD_UART_COMM);
        /* no return from Major Fault Handler */
    }
}

/*
******************************************************************************
** Function:     uabRampBoardPioSet
**
** Description:  Sets the Sierra Ramp Board PIO Register value
**               using UART1 Serial port communication.
**
** Input  Parms: pio   Ramp PIO Register value
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabRampBoardPioSet(uint8_t pio)
{
    UART1_XmitBuf[0] = 'O';
    UART1_XmitBuf[1] = pio;
    UART1_XmitBuf[2] = 'P';
    UARTSend(UART1, UART1_XmitBuf, 3);
    delayMs(15, BUSY_WAIT_BASED, TRUE);
}

/*
******************************************************************************
** Function:     uabRampBoardRegsGet
**
** Description:  Gets all five Sierra Ramp Board PIO Registers
**               using UART1 Serial port communication.
**
** Input  Parms: *reg   Ptr to callers buffer for 5 Ramp PIO Registers
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
int uabRampBoardRegsGet(uint8_t *reg)
{
    uint8_t i;
    int retVal = NOT_OK;

    UART1_Count = 0;
    UART1_Status = 0;
    UART1_Buffer[0] = 0;
    UART1_XmitBuf[0] = 'R';
    for (i=1; i<6; i++)
    {
        UART1_Buffer[i] = 0;
        UART1_XmitBuf[i] = (i-1);
    }
    UART1_XmitBuf[6] = 'P';
    UARTSend(UART1, UART1_XmitBuf, 7);
    delayMs(20, BUSY_WAIT_BASED, TRUE);
    if (UART1_Status == 0)
    {
        if (UART1_Count == 5 )
        {
            for (i=0; i<5; i++)
               reg[i] = UART1_Buffer[i];
            retVal = OK;
        }
    }
   return(retVal);
}

/*
******************************************************************************
** Function:     uabLeftsideRampRaise
**
** Description:  Raises the Sierra Leftside Ramp.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: SIERRA_RAMP_NOT_RAISED
**               SIERRA_RAMP_RAISED
**
******************************************************************************
*/
int uabLeftsideRampRaise(void)
{
    int retVal = SIERRA_RAMP_RAISED;
    uint8_t        rampPio;
    uint8_t        done;
    uint8_t        rampOverloadLeft;
    static uint8_t rampOverloadDelayLeft = 0;
    static uint8_t rampOverloadCntLeft   = 0;
    uint8_t        rampTimeoutLeft;
    static uint8_t rampTimeoutCntLeft    = 0;
    uint8_t        eData[2];

    if (UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED)
    {
        if (rampOverloadDelayLeft > 0)
        {
            rampOverloadDelayLeft--;
            return(SIERRA_RAMP_NOT_RAISED);
        }

        uabRampBoardPioGet(&rampPio);
        if ((rampPio & SIERRA_LEFTSIDE_RAMP_POSITION_BIT) == 0)    /* 0 = RAISED */
        {
            UabSierraLeftsideRampPosition = SIERRA_RAMP_RAISED;
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            return(retVal);
        }

        if ((rampPio & SIERRA_RAISE_RAMP_BIT) == 0)
        {
            rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
            uabRampBoardPioSet(rampPio);
            rampPio |= (SIERRA_RAISE_RAMP_BIT);
            uabRampBoardPioSet(rampPio);
        }
        rampPio |= (SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
        uabRampBoardPioSet(rampPio);
        delayMs(100, SYS_TICK_BASED, TRUE);

        done = FALSE;
        rampTimeoutLeft = 0;
        rampOverloadLeft = FALSE;
        do
        {
            uabRampBoardPioGet(&rampPio);
            if ((rampPio & SIERRA_RAMP_OVERLOAD_BIT) == 0)     /* 0 = OVERLOAD */
                rampOverloadLeft = TRUE;
            else if ((rampPio & SIERRA_LEFTSIDE_RAMP_POSITION_BIT) == 0) /* 0 = RAISED */
                done = TRUE;
            else
                delayMs(100, SYS_TICK_BASED, TRUE);
        }
        while( (!done) && (!rampOverloadLeft) && (rampTimeoutLeft++ < SIERRA_RAMP_TIMEOUT) );
        rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
        uabRampBoardPioSet(rampPio);

        if (rampOverloadLeft)
        {
            UabSierraLeftsideRampPosition = SIERRA_RAMP_UNKNOWN;
            retVal = SIERRA_RAMP_NOT_RAISED;
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultSet(SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            if (rampOverloadCntLeft++ < 3)
            {
                rampOverloadDelayLeft = 100;
                uabRampBoardPioGet(&rampPio);
                if (rampPio & SIERRA_RAISE_RAMP_BIT)
                {
                    rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
                    uabRampBoardPioSet(rampPio);
                    rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
                    uabRampBoardPioSet(rampPio);
                }
                rampPio |= (SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);
            }
            else
            {
                rampOverloadCntLeft = 0;
                uabRampBoardPioGet(&rampPio);
                if (rampPio & SIERRA_RAISE_RAMP_BIT)
                {
                    rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
                    uabRampBoardPioSet(rampPio);
                    rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
                    uabRampBoardPioSet(rampPio);
                }
                rampPio |= (SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);


                delayMs((SIERRA_RAMP_TIMEOUT * 100), SYS_TICK_BASED, TRUE);  /* 4 Second Delay */
                rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);

                /* Leftside Ramp Current Overload */
                uabMajorFaultHandler(UAB_MAJOR_FAULT_LEFTSIDE_RAMP_CURRENT_OVERLOAD);
                /* no return from Major Fault Handler */
            }
        }
        else if (rampTimeoutLeft >= SIERRA_RAMP_TIMEOUT)
        {
            UabSierraLeftsideRampPosition = SIERRA_RAMP_UNKNOWN;
            retVal = SIERRA_RAMP_NOT_RAISED;
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            uabFaultSet(SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            if (rampTimeoutCntLeft++ >= 3)
            {
                rampTimeoutCntLeft = 0;

                /* Leftside Ramp Raise Timeout*/
                uabMajorFaultHandler(UAB_MAJOR_FAULT_LEFTSIDE_RAMP_RAISE_TIMEOUT);
                /* no return from Major Fault Handler */
            }
        }
        else
        {  /* done */
            rampOverloadCntLeft = 0;
            rampTimeoutCntLeft  = 0;
            UabSierraLeftsideRampPosition = SIERRA_RAMP_RAISED;
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultClr(SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
        }

        eData[0] = 1;  /* LEFTSIDE_RAMP */
        eData[1] = (uint8_t)(UabSierraLeftsideRampPosition);
        uabEvent(UAB_RAMP_CHANGE_EVENT,
                 &eData, sizeof(eData) );
    }
    return(retVal);
}

/*
******************************************************************************
** Function:     uabLeftsideRampLower
**
** Description:  Lowers the Sierra Leftside Ramp.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: SIERRA_RAMP_UNKNOWN
**               SIERRA_RAMP_LOWERED
**
******************************************************************************
*/
int uabLeftsideRampLower(void)
{
    int retVal = SIERRA_RAMP_LOWERED;
    uint8_t  rampTimeout;
    uint8_t  rampPio;
    uint8_t  obstrFault;
    uint8_t  i;
    uint32_t pioa;
    uint32_t pio;
    uint8_t  eData[2];

	if (UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED)
	{
		for (i=0; i < 3; i++)
		{
			pioa = (uint32_t)(LPC_GPIO0->PIN);
			delayMs(10, BUSY_WAIT_BASED, TRUE);
			pio = (uint32_t)(LPC_GPIO0->PIN);
			if ( (pioa & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH) ==
                             (pio  & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH) )
			  break;
		}

		if (pio & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH)
		{
            uabFaultSet(SIERRA_UPSIDE_RAMP_OB_FAULT,
                        SIERRA_UPSIDE_RAMP_OB_FAULT_NUM);
            return(SIERRA_RAMP_UNKNOWN);
		}
	}

	uabRampBoardPioGet(&rampPio);
	if (rampPio & SIERRA_RAISE_RAMP_BIT)
	{
		rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
		uabRampBoardPioSet(rampPio);
		rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
		uabRampBoardPioSet(rampPio);
	}
	rampPio |= (SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
	uabRampBoardPioSet(rampPio);

	rampTimeout = 0;
	obstrFault = FALSE;
	do
	{
            if (UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED)
            {
                for (i=0; i < 3; i++)
                {
                        pioa = (uint32_t)(LPC_GPIO0->PIN);
                        delayMs(10, BUSY_WAIT_BASED, TRUE);
                        pio = (uint32_t)(LPC_GPIO0->PIN);
                        if ( (pioa & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH) ==
                             (pio  & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH) )
                          break;
                }

                if (pio & SIERRA_UPSIDE_RAMP_OBSTR_SWITCH)
                {
                    uabFaultSet(SIERRA_UPSIDE_RAMP_OB_FAULT,
                                 SIERRA_UPSIDE_RAMP_OB_FAULT_NUM);
                    obstrFault = TRUE;
                    retVal = SIERRA_RAMP_UNKNOWN;
                }
                else
                {
                    uabFaultClr(SIERRA_UPSIDE_RAMP_OB_FAULT,
                                 SIERRA_UPSIDE_RAMP_OB_FAULT_NUM);
                    delayMs(100, SYS_TICK_BASED, TRUE);
                }
            }
            else
                delayMs(100, SYS_TICK_BASED, TRUE);
	}
	while( (!obstrFault) && (++rampTimeout < SIERRA_RAMP_TIMEOUT) );
	rampPio &= ~(SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT);
	uabRampBoardPioSet(rampPio);
	if (obstrFault)
	   UabSierraLeftsideRampPosition = SIERRA_RAMP_UNKNOWN;
	else
	   UabSierraLeftsideRampPosition = SIERRA_RAMP_LOWERED;

	eData[0] = 1;  /* LEFTSIDE_RAMP */
    eData[1] = (uint8_t)(UabSierraLeftsideRampPosition);
    uabEvent(UAB_RAMP_CHANGE_EVENT,
             &eData, sizeof(eData) );

    return(retVal);
}

/*
******************************************************************************
** Function:     uabRightsideRampRaise
**
** Description:  Raises the Sierra Rightside Ramp.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: SIERRA_RAMP_NOT_RAISED
**               SIERRA_RAMP_RAISED
**
******************************************************************************
*/
int uabRightsideRampRaise(void)
{
    int retVal = SIERRA_RAMP_RAISED;
    uint8_t        rampPio;
    uint8_t        done;
    uint8_t        rampOverloadRight;
    static uint8_t rampOverloadDelayRight = 0;
    static uint8_t rampOverloadCntRight   = 0;
    uint8_t        rampTimeoutRight;
    static uint8_t rampTimeoutCntRight    = 0;
    uint8_t        eData[2];

    if ( UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED )
    {
        if (rampOverloadDelayRight > 0)
        {
            rampOverloadDelayRight--;
            return(SIERRA_RAMP_NOT_RAISED);
        }

        uabRampBoardPioGet(&rampPio);
        if ((rampPio & SIERRA_RIGHTSIDE_RAMP_POSITION_BIT) == 0)
        {
            UabSierraRightsideRampPosition = SIERRA_RAMP_RAISED;
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            return(retVal);
        }

        if ((rampPio & SIERRA_RAISE_RAMP_BIT) == 0)
        {
            rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
            uabRampBoardPioSet(rampPio);
            rampPio |= (SIERRA_RAISE_RAMP_BIT);
            uabRampBoardPioSet(rampPio);
        }
        rampPio |= (SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
        uabRampBoardPioSet(rampPio);
        delayMs(100, SYS_TICK_BASED, TRUE);

        done = FALSE;
        rampTimeoutRight = 0;
        rampOverloadRight = FALSE;
        do
        {
            uabRampBoardPioGet(&rampPio);
            if ( (rampPio & SIERRA_RAMP_OVERLOAD_BIT) == 0)     /* 0 = OVERLOAD */
                rampOverloadRight = TRUE;
            else if ((rampPio & SIERRA_RIGHTSIDE_RAMP_POSITION_BIT) == 0) /* 0 = RAISED */
                done = TRUE;
            else
                delayMs(100, SYS_TICK_BASED, TRUE);
        }
        while( (!done) && (!rampOverloadRight) && (++rampTimeoutRight < SIERRA_RAMP_TIMEOUT) );
        rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
        uabRampBoardPioSet(rampPio);

        if (rampOverloadRight)
        {
            UabSierraRightsideRampPosition = SIERRA_RAMP_UNKNOWN;
            retVal = SIERRA_RAMP_NOT_RAISED;
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultSet(SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            if (rampOverloadCntRight++ < 3)
            {
                rampOverloadDelayRight = 100;
                uabRampBoardPioGet(&rampPio);
                if (rampPio & SIERRA_RAISE_RAMP_BIT)
                {
                    rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
                    uabRampBoardPioSet(rampPio);
                    rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
                    uabRampBoardPioSet(rampPio);
                }
                rampPio |= (SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);
            }
            else
            {
                rampOverloadCntRight = 0;
                uabRampBoardPioGet(&rampPio);
                if (rampPio & SIERRA_RAISE_RAMP_BIT)
                {
                    rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
                    uabRampBoardPioSet(rampPio);
                    rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
                    uabRampBoardPioSet(rampPio);
                }
                rampPio |= (SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);
                delayMs((SIERRA_RAMP_TIMEOUT * 100), SYS_TICK_BASED, TRUE); /* 4 Second Delay */
                rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
                uabRampBoardPioSet(rampPio);

                /* Rightside Ramp Current Overload */
                uabMajorFaultHandler(UAB_MAJOR_FAULT_RIGHTSIDE_RAMP_CURRENT_OVERLOAD);
                /* no return from Major Fault Handler */
            }
        }
        else if (rampTimeoutRight >= SIERRA_RAMP_TIMEOUT)
        {
            UabSierraRightsideRampPosition = SIERRA_RAMP_UNKNOWN;
            retVal = SIERRA_RAMP_NOT_RAISED;
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
            uabFaultSet(SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            if (rampTimeoutCntRight++ >= 3)
            {
                rampTimeoutCntRight = 0;

                /* Righttside Ramp Raise Timeout*/
                uabMajorFaultHandler(UAB_MAJOR_FAULT_RIGHTSIDE_RAMP_RAISE_TIMEOUT);
                /* no return from Major Fault Handler */
            }
        }
        else
        {  /* done */
            rampOverloadCntRight = 0;
            rampTimeoutCntRight  = 0;
            UabSierraRightsideRampPosition = SIERRA_RAMP_RAISED;
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM);
            uabFaultClr(SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT,
                         SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM);
        }

        eData[0] = 2;  /* RIGHTSIDE_RAMP */
        eData[1] = (uint8_t)(UabSierraRightsideRampPosition);
        uabEvent(UAB_RAMP_CHANGE_EVENT,
                 &eData, sizeof(eData) );
    }
    return(retVal);
}

/*
******************************************************************************
** Function:     uabRightsideRampLower
**
** Description:  Lowers the Sierra Rightside Ramp.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: SIERRA_RAMP_UNKNOWN
**               SIERRA_RAMP_LOWERED
**
******************************************************************************
*/
int uabRightsideRampLower(void)
{
    int retVal = SIERRA_RAMP_LOWERED;
    uint8_t  rampTimeout;
    uint8_t  rampPio;
    uint8_t  obstrFault;
    uint8_t  i;
    uint32_t pioa;
    uint32_t pio;
    uint8_t  eData[2];

    if (UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED)
    {
    	for (i=0; i < 3; i++)
        {
            pioa = (uint32_t)(LPC_GPIO0->PIN);
            delayMs(10, BUSY_WAIT_BASED, TRUE);
            pio = (uint32_t)(LPC_GPIO0->PIN);
            if ( (pioa & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH) ==
                 (pio  & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH) )
              break;
        }

        if (pio & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH)
        {
            uabFaultSet(SIERRA_DOWNSIDE_RAMP_OB_FAULT,
                         SIERRA_DOWNSIDE_RAMP_OB_FAULT_NUM);
            return(SIERRA_RAMP_UNKNOWN);
        }
    }

    uabRampBoardPioGet(&rampPio);
    if (rampPio & SIERRA_RAISE_RAMP_BIT)
    {
        rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
        uabRampBoardPioSet(rampPio);
        rampPio &= ~(SIERRA_RAISE_RAMP_BIT);
        uabRampBoardPioSet(rampPio);
    }
    rampPio |= (SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
    uabRampBoardPioSet(rampPio);

    rampTimeout = 0;
    obstrFault = FALSE;
    do
    {
        if (UabSierraPlatformPosition == SIERRA_PLATFORM_UNFOLDED)
        {
            for (i=0; i < 3; i++)
            {
                pioa = (uint32_t)(LPC_GPIO0->PIN);
                delayMs(10, BUSY_WAIT_BASED, TRUE);
                pio = (uint32_t)(LPC_GPIO0->PIN);
                if ( (pioa & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH) ==
                     (pio  & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH) )
                  break;
            }

            if (pio & SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH)
            {
                uabFaultSet(SIERRA_DOWNSIDE_RAMP_OB_FAULT,
                             SIERRA_DOWNSIDE_RAMP_OB_FAULT_NUM);
                obstrFault = TRUE;
                retVal = SIERRA_RAMP_UNKNOWN;
            }
            else
            {
                uabFaultClr(SIERRA_DOWNSIDE_RAMP_OB_FAULT,
                             SIERRA_DOWNSIDE_RAMP_OB_FAULT_NUM);
                delayMs(100, SYS_TICK_BASED, TRUE);
            }
        }
        else
            delayMs(100, SYS_TICK_BASED, TRUE);
    }
    while( (!obstrFault) && (++rampTimeout < SIERRA_RAMP_TIMEOUT) );
    rampPio &= ~(SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT);
    uabRampBoardPioSet(rampPio);
    if (obstrFault)
       UabSierraRightsideRampPosition = SIERRA_RAMP_UNKNOWN;
    else
       UabSierraRightsideRampPosition = SIERRA_RAMP_LOWERED;

    eData[0] = 2;  /* RIGHTSIDE_RAMP */
    eData[1] = (uint8_t)(UabSierraRightsideRampPosition);
    uabEvent(UAB_RAMP_CHANGE_EVENT,
             &eData, sizeof(eData) );

    return(retVal);
}

/*
******************************************************************************
** Function:     uabSierraRampBoardInit
**
** Description:  Initializes the Sierra Ramp Board.
**               Configures SC18IM700 device for 9600 Baud communication
**               to/from the UART1 Serial Port on the UAB.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: OK
**               NOT_OK
**
******************************************************************************
*/
int uabSierraRampBoardInit(void)
{
    int retVal = OK;
    uint8_t i;

    UART1_XmitBuf[0]  = 'W';
    UART1_XmitBuf[1]  = 4;  /* Reg 4 */
    UART1_XmitBuf[2]  = 0x00;  /* Force Outputs to 0 */
    UART1_XmitBuf[3]  = 2;  /* Reg 2 */
    UART1_XmitBuf[4]  = SIERRA_SC18IM700_CFG2_INIT;
    UART1_XmitBuf[5]  = 3;  /* Reg 3 */
    UART1_XmitBuf[6]  = SIERRA_SC18IM700_CFG3_INIT;
    UART1_XmitBuf[7]  = 4;     /* Reg 4 */
    UART1_XmitBuf[8]  = 0x00;  /* Reforce Outputs to 0 */
    UART1_XmitBuf[9]  = 'P';
    UARTSend(UART1, UART1_XmitBuf, 10);
    delayMs(20, BUSY_WAIT_BASED, TRUE);

    UART1_Count = 0;
    UART1_Status = 0;
    UART1_Buffer[0] = 0;
    UART1_XmitBuf[0] = 'R';
    for (i=1; i<6; i++)
    {
        UART1_Buffer[i] = 0;
        UART1_XmitBuf[i] = (i-1);
    }
    UART1_XmitBuf[6] = 'P';
    UARTSend(UART1, UART1_XmitBuf, 7);
    delayMs(20, BUSY_WAIT_BASED, TRUE);
    if (UART1_Status == 0)
    {
        if (UART1_Count == 5 )
        {
            if  ( (UART1_Buffer[0] != SIERRA_SC18IM700_BRG0_INIT) ||
                  (UART1_Buffer[1] != SIERRA_SC18IM700_BRG1_INIT) ||
                  (UART1_Buffer[2] != SIERRA_SC18IM700_CFG2_INIT) ||
                  (UART1_Buffer[3] != SIERRA_SC18IM700_CFG3_INIT) )
            {
                retVal = NOT_OK;
            }
        }
        else
        {
            retVal = NOT_OK;
        }
    }
    else
    {
        retVal = NOT_OK;
    }
    return(retVal);
}

/*
******************************************************************************
** Function:     uabSierraAdjustRamps
**
** Description:  Based on platform position(FOLDED/UNFOLDED) and the
**               current position of the platform on the track, call
**               functions to Raise or Lower both the Leftside and
**               Rightside ramps.
**
** Input  Parms: platform_position:  FOLDED or UNFOLDED
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabSierraAdjustRamps(uint8_t platform_position)
{
    uint32_t pio0;
    uint32_t pio2;
    int top_bottom;

    if (platform_position == SIERRA_PLATFORM_UNFOLDED)
    {
        pio0 = (uint32_t)(LPC_GPIO0->PIN);
        pio2 = (uint32_t)(LPC_GPIO2->PIN);
        top_bottom = uabCheckTopBottom(pio0, pio2);
        if (top_bottom == UAB_TOP_LEFT)
        {
            if (UabSierraPlatformMounting == SIERRA_LEFTSIDE_UP)
            {
                if (UabSierraLeftsideRampPosition != SIERRA_RAMP_LOWERED)
                  uabLeftsideRampLower();
                if (UabSierraRightsideRampPosition != SIERRA_RAMP_RAISED)
                  uabRightsideRampRaise();
            }
            else
            {
                if (UabSierraRightsideRampPosition != SIERRA_RAMP_LOWERED)
                  uabRightsideRampLower();
                if (UabSierraLeftsideRampPosition != SIERRA_RAMP_RAISED)
                  uabLeftsideRampRaise();
            }
        }
        else if (top_bottom == UAB_BOTTOM_RIGHT)
        {
            if (UabSierraPlatformMounting == SIERRA_LEFTSIDE_UP)
            {
                if (UabSierraRightsideRampPosition != SIERRA_RAMP_LOWERED)
                  uabRightsideRampLower();
                if (UabSierraLeftsideRampPosition != SIERRA_RAMP_RAISED)
                  uabLeftsideRampRaise();
            }
            else
            {
                if (UabSierraLeftsideRampPosition != SIERRA_RAMP_LOWERED)
                  uabLeftsideRampLower();
                if (UabSierraRightsideRampPosition != SIERRA_RAMP_RAISED)
                  uabRightsideRampRaise();
            }
        }
        else
        {
            if (UabSierraLeftsideRampPosition != SIERRA_RAMP_RAISED)
              uabLeftsideRampRaise();
            if (UabSierraRightsideRampPosition != SIERRA_RAMP_RAISED)
              uabRightsideRampRaise();
        }
    }
    else
    {
        if (UabSierraLeftsideRampPosition != SIERRA_RAMP_LOWERED)
          uabLeftsideRampLower();
        if (UabSierraRightsideRampPosition != SIERRA_RAMP_LOWERED)
          uabRightsideRampLower();
    }
}

uint8_t  prevPlatformPosition;

/*
******************************************************************************
** Function:     uabSierraPlatformPositionInit
**
** Description:  Initializes global varisbles indicating
**                    Platform Mounting:  Leftside or Rightside
**                    Platform Position:  Folded or Unfolded
**               based on the Ramp Board PIO register contents.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabSierraPlatformPositionInit(void)
{
    uint8_t rampPio;
    uint8_t eData[1];

    uabRampBoardPioGet(&rampPio);
    if (rampPio & SIERRA_PLATFORM_MOUNTING_BIT)
      UabSierraPlatformMounting = SIERRA_LEFTSIDE_UP;
    else
      UabSierraPlatformMounting = SIERRA_RIGHTSIDE_UP;

    if (rampPio & SIERRA_PLATFORM_POSITION_UNFOLDED_BIT)
    {
        UabSierraPlatformPosition = SIERRA_PLATFORM_UNFOLDED;
        uabSierraAdjustRamps(SIERRA_PLATFORM_UNFOLDED);
    }
    else
    {
        UabSierraPlatformPosition = SIERRA_PLATFORM_FOLDED;
        uabSierraAdjustRamps(SIERRA_PLATFORM_FOLDED);
    }
    eData[0] = UabSierraPlatformPosition;
    uabEvent(UAB_PLATFORM_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    prevPlatformPosition = UabSierraPlatformPosition;
}

/*
******************************************************************************
** Function:     uabSierraPlatformCheck
**
** Description:  This function is called every 10ms scan to check that
**               the platform position (Folded vs. Unfolded) has not changed.
**               The function attempts to de-bounce the Folded/Unfolded signal
**               by requiring 5 consecutive reads of the PIO pin.
**               If the Folded/Unfolded state has changed, both the Leftside
**               and Rightside ramps are Lowered or Raised as needed.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
uint8_t UnfoldedTime = 0;
uint8_t FoldedTime   = 0;

void uabSierraPlatformCheck(void)
{
    uint8_t rampPio;
    uint8_t currPlatformPosition;
    uint8_t raise;
    uint8_t eData[1];

    uabRampBoardPioGet(&rampPio);
    if (rampPio & SIERRA_PLATFORM_POSITION_UNFOLDED_BIT)
    {
        if (UnfoldedTime < 5)
            UnfoldedTime++;
        FoldedTime = 0;
        currPlatformPosition = SIERRA_PLATFORM_UNFOLDED;
    }
    else
    {
        if (FoldedTime < 5)
            FoldedTime++;
        UnfoldedTime = 0;
        currPlatformPosition = SIERRA_PLATFORM_FOLDED;
    }
    if ( (currPlatformPosition == SIERRA_PLATFORM_FOLDED) &&
         (prevPlatformPosition == SIERRA_PLATFORM_UNFOLDED) )
    {
        if (FoldedTime >= 5)
        {
            uabMotorOff();
            UabMotionState = stopped;
            PlatformRampChangeDelay = 50;
            UabSierraPlatformPosition = SIERRA_PLATFORM_FOLDED;
            uabSierraAdjustRamps(SIERRA_PLATFORM_FOLDED);
            eData[0] = UabSierraPlatformPosition;
            uabEvent(UAB_PLATFORM_CHANGE_EVENT,
                     &eData, sizeof(eData) );
            prevPlatformPosition = currPlatformPosition;
        }
    }
    else
    {
        if ( (currPlatformPosition == SIERRA_PLATFORM_UNFOLDED) &&
             (prevPlatformPosition == SIERRA_PLATFORM_FOLDED) )
        {
            if (UnfoldedTime >= 5)
            {
                uabMotorOff();
                UabMotionState = stopped;
                PlatformRampChangeDelay = 50;
                UabSierraPlatformPosition = SIERRA_PLATFORM_UNFOLDED;
                uabSierraAdjustRamps(SIERRA_PLATFORM_UNFOLDED);
                eData[0] = UabSierraPlatformPosition;
                uabEvent(UAB_PLATFORM_CHANGE_EVENT,
                         &eData, sizeof(eData) );
                prevPlatformPosition = currPlatformPosition;
            }
        }
        else
        {
            if ( (currPlatformPosition == SIERRA_PLATFORM_UNFOLDED) &&
                 (prevPlatformPosition == SIERRA_PLATFORM_UNFOLDED) )
            {
               if  ( (UabMotionState != stopped) &&
                     (UabMotionState != start_down_holdoff) &&
                     (UabMotionState != start_up_holdoff) )
               {
                   raise = FALSE;
                   if (rampPio & SIERRA_RIGHTSIDE_RAMP_POSITION_BIT)  /* 1 = Not Raised */
                   {
                       UabSierraRightsideRampPosition = SIERRA_RAMP_NOT_RAISED;
                       raise = TRUE;
                   }
                   if (rampPio & SIERRA_LEFTSIDE_RAMP_POSITION_BIT)  /* 1 = Not Raised */
                   {
                       UabSierraLeftsideRampPosition = SIERRA_RAMP_NOT_RAISED;
                       raise = TRUE;
                   }

                   if (raise)
                   {
                       uabMotorOff();
                       UabMotionState = stopped;
                       PlatformRampChangeDelay = 50;
                       uabSierraAdjustRamps(SIERRA_PLATFORM_UNFOLDED);
                   }
               }
            }
        }
    }
}

