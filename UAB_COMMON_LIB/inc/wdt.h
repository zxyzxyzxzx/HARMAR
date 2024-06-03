/*
******************************************************************************
** Filename:  wdt.h
**
** Description:
**       This file defines internal Windowed Watchdog Timer(WWDT) HW register
**       bit mappings.
** 
**       The Watchdog Timer is configured to reset the microprocessor if
**       not punched every 2 Seconds. 
**       The Watchdog Timer is configured to use the Internal 12Mhz Osc and 
**       not the Watchdog Osc.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef WDT_H_
#define WDT_H_

#define WDT_IRC_OSC_CLK  12000000UL     /* Internal osc freq (12Mhz) */

/* Macro defines for WDT Control register  */
#define WDT_MOD_WDEN	      (1<<0)   /* WDT enable bit */
#define WDT_MOD_WDRESET       (1<<1)   /* WDT reset enable bit */
#define WDT_MOD_WDTOF	      (1<<2)   /* WDT time out flag bit */
#define WDT_MOD_WDINT	      (1<<3)   /* WDT interrupt flag bit */
#define WDT_MOD_WDPROTECT     (1<<4)   /* WDT update mode bit */
#define WDT_MOD_WDLOCKCLK     (1<<5)   /* WDT clock lock bit */
#define WDT_MOD_WDLOCKDP      (1<<6)   /* WDT deep pd disable bit */
#define WDT_MOD_WDLOCKEN      (1<<7)   /* WDT enable lockout bit */
#define WDT_MOD_MASK	      (uint32_t)(0x000000FF) /* WDT mode reg mask */

/* Macro defines for WDT Clock Source Selection register */
#define WDT_CLKSEL_WDLOCK   ((1UL<<31))    /* Clock source lock bit */
#define WDT_CLKSEL_MASK     (uint32_t)(0x80000003)   /* register mask */

/* Watchdog timer constant register mask */
#define WDT_TC_MASK	      (uint32_t)(0x00FFFFFF)
/* Watchdog feed sequence register mask */
#define WDT_FEED_MASK       (uint32_t)(0x000000FF)
/* Watchdog warning interrupt register mask */
#define WDT_WARNINT_MASK    (uint32_t)(0x000003FF)
/* Watchdog window register mask */
#define WDT_WINDOW_MASK     (uint32_t)(0x00FFFFFF)

#define WDT_CLKSEL_IRCOSC   0   /* Clock source: IRC Osc (default)*/
#define WDT_CLKSEL_WDOSC    1   /* Clock source: Watchdog osc */

#define WDT_MODE_INT_ONLY   0   /* generate interrupt only */
#define WDT_MODE_RESET      1   /* generate interrupt & reset uc */

#define WDT_TIMEOUT_TICKVAL 0  /* timeout in absolute value */
#define WDT_TIMEOUT_USVAL   1  /* timeout in microsecond value*/

/* WDT Timeout minimum value */
#define WDT_TIMEOUT_MIN	     ((uint32_t)(0x000000FF))
/* WDT Timeout maximum value */
#define WDT_TIMEOUT_MAX	     ((uint32_t)(0x00FFFFFF))
/* SLCB WDT Timeout is set at ~2 Seconds */
#define WDT_TIMEOUT_2_SEC    ((uint32_t)(0x00555555))

/* WDT Timeout default value */
#define WDT_TIMEOUT_DEFAULT  ((uint32_t)(0x0000FFFF))

/* Function Prototypes */
void wdtFeed(uint8_t enable_intr);
void wdtEnable(void);
void wdtDisable(void);
void wdtInit(void);

#endif /* WDT_H_ */

