/*
******************************************************************************
** Filename:  utils.c
**
** Description:
**       This file contains common utility functions (delays, etc.).
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/
#include "LPC122x.h"
#include "main.h"
#include "utils.h"
#include "wdt.h"

/*
******************************************************************************
** Function:     delayUsec
**
** Description:  Busy-wait delay for up to 2000 usec.
**               Based on use of 24 MHz system clock.
**
** Input  Parms: int usec_delay  (10-2000) usec
**               Must be a multiple of 10 usec.
**               Do not use usec delay greater than 2000 (i.e. 2 ms)
**               Do not use for accurate time measurement.
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/

/* for Release Loads */
#define DELAY_ADJUST 230
/* For Debug */
//#define DELAY_ADJUST 100

void delayUsec( int usec_delay )
{
    int i;
    uint32_t startt;
    uint32_t endt;

    for (i=0; i<(usec_delay/10); i++)
    {
        startt = SysTick->VAL;
        do
        {
            endt = SysTick->VAL;
        }
        while ((startt - endt) < DELAY_ADJUST); /* based on testing with scope */
    }
}

/*
******************************************************************************
** Function:     delayMs
**
** Description:  Delay specified number of ms.
**               Both modes (SYS_TICK_BASED(0) or BUSY_WAIT_BASED(1)) will
**               block execution of the 10 ms scan loop.
**               This delayMs() function should be used only in init or when
**               it is known that the motor is entering a "stopped" state.
**
**               The WDT is punched every 500 ms while in the delay loop,
**
** Input  Parms: uint32_t ms    Number of ms to delay
**               uint8_t mode   SYS_TICK_BASED(0) or BUSY_WAIT_BASED(1)
**               uint8_t enable_intr TRUE or FALSE
**                       Indicates whether all interrupts should be enabled
**                       or disabled when wdtFeed() is called.
**
** Output Parms: None
**
** Global Vars:  uint32_t SysTicks;
**               uint32_t DelayCnt;
**
** Return Value: None
**
******************************************************************************
*/
void delayMs(uint32_t ms, uint8_t mode, uint8_t enable_intr)
{
    uint32_t   i;
    static int wdtCnt = 0;

    if (mode == SYS_TICK_BASED)
    {
        DelayCnt = ms/10;
        while (DelayCnt > 0)
        {
            if ( ++wdtCnt > 50 )
            {
                wdtCnt = 0;
                wdtFeed(TRUE);
            }
        }
    }
    else
    {
        for(i=0; i<ms; i++)
        {
            if ( ++wdtCnt > 500 )
            {
                wdtCnt = 0;
                wdtFeed(enable_intr);
            }
           (void)delayUsec(1000);
        }
    }
}
