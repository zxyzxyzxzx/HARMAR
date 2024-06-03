/*
******************************************************************************
** Filename:  compadc.c
**
** Description:
**       This file contains functions used to read the LPC122x
**       Comparator 0 and Analog-to-Digital Converter(ADC)
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include "LPC122x.h"
#include "main.h"
#include "utils.h"
#include "uab.h"
#include "compadc.h"

/*
******************************************************************************
** Function:     uabVoltsConvert
**
** Description:  Based on the Comparator 0 input selected and the 
**               current step (0-31) being read, convert the step value to a
**               voltage based on reference voltage for that specific input.
**
** Input  Parms: uint32_t  step        Comparator 0 step (0-31)
**               uint32_t  volts_sel   Selects which Comparator 0 Input
**
** Output Parms: None
**
** Return Value: float voltage 
** 
******************************************************************************
*/
float uabVoltsConvert( uint32_t step, uint32_t volts_sel)
{
    float volts;

    switch(volts_sel)
    {
        case CMP_LADDER_OUT_VOLTS:
            volts = (float)((CMP_REF_V/CMP_STEP) * step);
            break;

        case CMP_CHARGE_VOLTS:
            volts = (float)((CHARGE_REF_V/CMP_STEP) * step);
            break;

        case CMP_SENSE12_VOLTS:
            volts = (float)((SENSE12_REF_V/CMP_STEP) * step);
            break;

        case CMP_MOTOR_A_VOLTS:
        case CMP_MOTOR_B_VOLTS:
            volts = (float)((MOTOR_REF_V/CMP_STEP) * step);
            break;

        case ADC_BATTERY_VOLTS:
            volts = (float)((BATTERY_REF_V/ADC_STEP) * step);
            break;

        case ADC_CURR_SENSE_VOLTS:
            volts = (float)((CURR_SENSE_REF_V/ADC_STEP) * step);
            break;

        default:
            volts = 0.0;
            break;
    }

    return(volts);
}

/*
******************************************************************************
** Function:     uabAdcRead
**
** Description:  Get the value of the current ADC conversion.
**               The value returned is the ADC step from 0-1023.
**
** Input  Parms: uint32_t  adcNum  ADC Input channel to read (1 or 2)
**
** Output Parms: None
**
** Return Value: uint32_t ADC Step (0-1023) 
** 
******************************************************************************
*/
uint32_t uabAdcRead( uint32_t adcNum )
{
  uint32_t regVal, adcData;

  LPC_ADC->CR &= 0xFFFFFF00;
  LPC_ADC->CR |= (1 << 24) | (1 << adcNum); /* Select AD# and start A/D conv */

  while ( 1 ) /* wait until end of A/D convert */
  {
     regVal = *(volatile unsigned long *)(LPC_ADC_BASE
                        + ADC_OFFSET + ADC_INDEX * adcNum);
     /* read result of A/D conversion */
     if ( regVal & ADC_DONE )
       break;
  }

  LPC_ADC->CR &= 0xF8FFFFFF;    /* stop ADC now */
  if ( regVal & ADC_OVERRUN )   /* if overrun, return zero */
  {
      return ( 0 );
  }
  adcData = ( regVal >> 6 ) & 0x3FF;
  return ( adcData );           /* return A/D conversion value */
}

/*
******************************************************************************
** Function:     uabCompVoltsRead
**
** Description:  Read the current voltage of a Comparator 0 input.
**
** Input  Parms: uint32_t  cmpSel  Comparator 0 Input channel to read.
**
** Output Parms: None
**
** Return Value: float voltage for selected input 
** 
******************************************************************************
*/
float uabCompVoltsRead(uint32_t cmpSel)
{
    int step;
    float volts;
    volatile uint32_t reg1;
    volatile uint32_t reg2;
    uint8_t step1;
    uint8_t step2;

    LPC_ACOMP->CMP &= ~CMP_SEL_MASK;
    switch (cmpSel)
    {
        case CMP_MOTOR_B_VOLTS:
            LPC_ACOMP->CMP |= CMP_MOTOR_B_VOLTS;
        break;
        case CMP_MOTOR_A_VOLTS:
            LPC_ACOMP->CMP |= CMP_MOTOR_A_VOLTS;
            break;
        case CMP_SENSE12_VOLTS:
            LPC_ACOMP->CMP |= CMP_SENSE12_VOLTS;
            break;
        case CMP_CHARGE_VOLTS:
            LPC_ACOMP->CMP |= CMP_CHARGE_VOLTS;
            break;
        default:
            cmpSel = CMP_LADDER_OUT_VOLTS;
            break;
    }

    for (step = CMP_STEP; step >= 0; step--)
    {
        LPC_ACOMP->VLAD = ((step << 1) | (1<<0) | (1<<6));
        delayUsec(100);
        reg1 = LPC_ACOMP->CMP;
        reg1 &= (1<<21);
        if (reg1)
        {
           step1 = (uint8_t)step;
           reg2 = LPC_ACOMP->CMP;
           reg2 &= (1<<21);
           if (reg2)
           {
               step2 = (uint8_t)step;
               if (step1 == step2)
               {
                   volts = uabVoltsConvert(step, cmpSel);
                   break;
               }
           }
        }
    }
    if (step < 0)
    {
       volts = uabVoltsConvert(0, cmpSel);
    }

    return(volts);
}

/*
******************************************************************************
** Function:     uabAdcVoltsRead
**
** Description:  Read the current voltage of an ADC input.
**
** Input  Parms: uint32_t  adcSel  ADC Input channel to read.
**
** Output Parms: None
**
** Return Value: float voltage for selected input 
** 
******************************************************************************
*/
float uabAdcVoltsRead(uint32_t adcSel)
{
    uint32_t step;
    float volts;

    if (adcSel == ADC_BATTERY_SEL)
    {
        step = uabAdcRead( ADC_BATTERY_SEL );
        volts = uabVoltsConvert(step, ADC_BATTERY_VOLTS);
        if (step == 1)
           volts = 0.0;
    }
    else
    {
        step = uabAdcRead( ADC_CURR_SENSE_SEL );
        volts = uabVoltsConvert(step, ADC_CURR_SENSE_VOLTS);
        if (step == 1)
           volts = 0.0;
    }

    return(volts);
}

/*
******************************************************************************
** Function:     uabAdcAmpsRead
**
** Description:  Read the current sense input to get the voltage.
**               Convert the voltage reading to current reading in amps
**               based on hardware provided conversion algorithm.
**
** Input  Parms: None
**
** Output Parms: None
**
** Return Value: float amps for current sense 
** 
******************************************************************************
*/
float uabAdcAmpsRead(void)
{
    uint32_t step;
    float volts;
    float amps;

    step = uabAdcRead( ADC_CURR_SENSE_SEL );
    if (step <= 1)
      volts = 0.0;
    else
      volts = uabVoltsConvert(step, ADC_CURR_SENSE_VOLTS);

    amps = (volts - 0.429) / 52.523 / 0.002;

    return(amps);
}

/*
******************************************************************************
** Function:     uabCompAdcInit
**
** Description:  Initialize the LPC122x PIO Ports needed for 
**               Comparator 0 and ADC special functions based on
**               input channels needed for UAB application.
**
** Input  Parms: None
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabCompAdcInit(void)
{
    uint32_t i;

    /* Enable Power To ADC and CMP */
    LPC_SYSCON->PDRUNCFG &= ~((1<<4) | (1<<15));

    /* Enable AHB Clock to the ADC an CMP */
    LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<14) | (1<<20));

    /* Init Comparator 0 Inputs */
    LPC_IOCON->PIO0_19  &= ~0x97; /* Analog and No Pull Up */
    LPC_IOCON->PIO0_19  |= 0x02;  /* ACMP0 0 IN */
    LPC_IOCON->PIO0_20  &= ~0x97; /* Analog and No Pull Up */
    LPC_IOCON->PIO0_20  |= 0x02;  /* ACMP0 1 IN */
    LPC_IOCON->PIO0_21  &= ~0x97; /* Analog and No Pull Up */
    LPC_IOCON->PIO0_21  |= 0x02;  /* ACMP0 2 IN */
    LPC_IOCON->PIO0_22  &= ~0x97; /* Analog and No Pull Up */
    LPC_IOCON->PIO0_22  |= 0x02;  /* ACMP0 3 IN */

    /* Init ADC AD0 and AD1 Inputs */
    LPC_IOCON->R_PIO0_30  &= ~0x97; /* AD0 I/O config */
    LPC_IOCON->R_PIO0_30  |= 0x03;  /* AD0 IN */
    LPC_IOCON->R_PIO0_31  &= ~0x97; /* AD1 I/O config */
    LPC_IOCON->R_PIO0_31  |= 0x03;  /* AD1 IN */

    i = ((SystemCoreClock/ADC_CLK)-1) & 0xFF;
    LPC_ADC->CR = ( 0x01 << 0 ) | /* SEL=1,select channel 0~7 on ADC0 */
                  ( i << 8 )    | /* CLKDIV = (24 MHz / 8 MHz) - 1 */
                  ( 0 << 16 )   | /* BURST = 0, no BURST, software controlled */
                  ( 0 << 24 )   | /* START = 0 A/D conversion stops */
                  ( 0 << 27 );    /* EDGE = 0 (trigger A/D conversion) */

    LPC_ACOMP->CMP  |= (1<<0);  /* Enable ACMP0 Only */
    LPC_ACOMP->VLAD |= ((1<<0) | (1<<6));  /* En ACMP0 Volt Ladder (VCC 3.3) */
}

