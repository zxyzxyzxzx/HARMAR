/*
******************************************************************************
** Filename:  wdt.c
**
** Description:
**       This file contains functions to handle the Watchdog Timer.
**       The Watchdog Timer must be punched/fed every 2 Seconds or a
**       processor reset will occur.
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include "LPC122x.h"
#include "wdt.h"
/*
******************************************************************************
** Function:     wdtFeed
**
** Description:  Punch/feed the Watchdog Timer to prevent 2 Second
**               Watchdog timeout causing processor to reset.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void wdtFeed(uint8_t enable_intr)
{
    __disable_irq();   /* Mandatory Disable interrupts */
    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    if (enable_intr)
      __enable_irq();
}
/*
******************************************************************************
** Function:     wdtEnable
**               wdtDisable
**
** Description:  Enable or Disable the Watchdog Timer.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void wdtEnable(void)
{
    /* Enable watchdog  */
    LPC_WWDT->MOD |= WDT_MOD_WDEN;
    wdtFeed(1);
}

void wdtDisable(void)
{
    /* Disable watchdog  */
    LPC_WWDT->MOD &= ~WDT_MOD_WDEN;
    wdtFeed(1);
}

/*
******************************************************************************
** Function:     wdtInit
**
** Description:  Initialize the Watchdog Timer HW registers to expect a
**               2 Second punch/feed interval and to perform a reset if
**               punch does not occur.
**               Configured to use the 12Mhz Internal Oscillator for timing.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void wdtInit(void)
{
    /* Select Clock Src Osc  IRC or WDT  Using IRC 12 Mhz */
    LPC_WWDT->CLKSEL &= ~WDT_CLKSEL_MASK;
    LPC_WWDT->CLKSEL  =  WDT_CLKSEL_IRCOSC;
    //LPC_WWDT->WDCLKSEL  = WDT_CLKSEL_WDOSC;

    /* make sure IRC and/or WDT clock powered */
    LPC_SYSCON->PDRUNCFG &= ~(1<<0);
    //LPC_SYSCON->PDRUNCFG &= ~(1<<6);

    /* Enable WDT clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);

    /* Config WDT mode to reset uc */
    LPC_WWDT->MOD |= WDT_MOD_WDRESET;

    /* config timeout value */
    LPC_WWDT->TC = WDT_TIMEOUT_2_SEC;

    /* config Reset and Warning interrupt compare values */
    LPC_WWDT->WINDOW = WDT_TIMEOUT_2_SEC;  /* 0x00FFFFFF */
    LPC_WWDT->WARNINT = 0;
    // LPC_WWDT->WARNINT = WDT_WDWARNINT_MAX; 0x3FF

    wdtFeed(1);
}
