/*
******************************************************************************
** Filename:  speed.c
**
** Description:
**       This file contains code to support Current Sense Speed Control (CSSC)
**       on the Universal Access Board (UAB)
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

//#define SPEED_DBG    /* Never enable for SIERRA */

#include "LPC122x.h"
#include "main.h"
#include "uab.h"
#include "compadc.h"

#if defined(SPEED_DEBUG)
#include "uart.h"
#include "small_printf.h"
#endif

#define  ADJUST_TARGET_PWM_DELAY    20  /* 20 x 10ms  = every 200 ms */
#define  ADJUST_PWM_DELAY            4  /* 4 x 10 ms  = every 40 ms */
#define  PWM_MAX                    98  /* PWM Max Rate = 98% */
#define  PWM_MIN                    25  /* PWM Min Rate = 25% */
#define  PWM_STEP                    1  /* Inc/Dec PWM by 1% each 40 ms */

/* Global Variables */

uint8_t  UabAdjustTargetPwmDelay = 0;
uint8_t  UabAdjustPwmDelay = 0;
uint8_t  UabTargetPwm      = 0;
float    UabCsscAmps       = 0.0;
float    TmpTargetPwm      = 0.0;

/*
** The following global variables are system-specific
** and are assigned values during UAB Initialization.
*/
float    UabNominalPwm;
float    UabPwmScaleFactor;

/*
******************************************************************************
** Function:     uabPwmAdjust
**
** Description:  Adjust the PWM rate of the motor in the Up/Left
**               or Down/Right direction based on the current sense
**               reading at time function is called.
**
** Input  Parms: direction   UP/LEFT  or DOWN/RIGHT
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabPwmAdjust( uint8_t direction )
{
    /* Using Current Sense Speed Control to change UabPwm */
    if (UabAdjustTargetPwmDelay == 0)
    {
        UabAdjustTargetPwmDelay = ADJUST_TARGET_PWM_DELAY;
        UabAdjustPwmDelay = 0;
        TmpTargetPwm = UabNominalPwm +
                       (UabPwmScaleFactor * UabCsscAmps);
        UabTargetPwm = (int8_t)TmpTargetPwm;
        if (UabTargetPwm > PWM_MAX)
        {
#if defined(SPEED_DEBUG)
             printf("FORCED TARGET PWM %2d%%  CS: %3.1f  TPWM: %2d%%\n",
                    PWM_MAX, UabCsscAmps, UabTargetPwm);
#endif
             UabTargetPwm = PWM_MAX;
        }
        else
        {
            if (UabTargetPwm < PWM_MIN)
            {
#if defined(SPEED_DEBUG)
                 printf("FORCED TARGET PWM %2d%%  CS: %3.1f  TPWM: %2d%%\n",
                        PWM_MIN, UabCsscAmps, UabTargetPwm);
#endif
                 UabTargetPwm = PWM_MIN;
            }
            else
            {
#if defined(SPEED_DEBUG)
                printf("CS: %3.1f  TPWM: %2d%%  PWM: %2d%%\n",
                       UabCsscAmps, UabTargetPwm, UabPwm);
#endif
            }
        }
    }
    else
    {
        UabAdjustTargetPwmDelay--;
    }

    if (UabAdjustPwmDelay == 0)
    {
        UabAdjustPwmDelay = ADJUST_PWM_DELAY;  /* every 40 ms */
        if (UabPwm < UabTargetPwm)
            UabPwm += PWM_STEP;
        else if (UabPwm > UabTargetPwm)
            UabPwm -= PWM_STEP;
        else
            UabPwm = UabTargetPwm;
        uabPwmChange(direction, UabPwm);   /* Change real PWM */
    }
    else
    {
        UabAdjustPwmDelay--;
    }
}

/*
******************************************************************************
** Function:     uabCsscBegin
**
** Description:  Every 10 ms execution loop, this function is called
**               to start the pulses required to read the current sense
**               properly from the circuit on the UAB.
**
** Input  Parms: adcNum  ADC Input channel number
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabCsscBegin( uint32_t adcNum )
{
    uint32_t regVal;
    int      i;

    if (UabMotorIsOff)
      return;

    if (UabPwm >= 5)
    {
        LPC_CT32B0->MR[1] = (UabPwm * 60) - 240;  /* 10 usec before */

        LPC_ADC->CR &= 0xF8FFFF00;
        LPC_ADC->CR |= (1 << 24) | (1<<26) | (1 << adcNum);  /* HW Trigger via CT32B0 MR 1 */

        /* Pitch first few readings */
        for (i=0; i<2; i++)
        {
            while (1) /* wait until end of A/D convert */
            {
                regVal = *(volatile unsigned long *)(LPC_ADC_BASE
                               + ADC_OFFSET + ADC_INDEX * adcNum);
                /* read result of A/D conversion */
                if ( regVal & ADC_DONE )
                  break;
            }
        }
    }
}

/*
******************************************************************************
** Function:     uabCsscCheck
**
** Description:  Every 10 ms execution loop, while the carriage is
**               is in the Cruise Up or Cruise Down motion state, the
**               the current sense amps are read and used by the motion
**               code to adjust the PWM based on that current sense reading.
**
**               A check for System Overload is also performed based on the
**               current sense value read.
**
** Input  Parms: adcNum  ADC Input channel number
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabCsscCheck( uint32_t adcNum )
{
    uint32_t regVal;
    uint32_t adcStep;
    float    volts;
    float    curr_sense_amps;

    if (UabMotorIsOff)
      return;

    if (UabPwm >= 5)
    {
        while ( 1 ) /* wait until end of A/D convert */
        {
            regVal = *(volatile unsigned long *)(LPC_ADC_BASE
                          + ADC_OFFSET + ADC_INDEX * adcNum);
            /* read result of A/D conversion */
            if ( regVal & ADC_DONE )
              break;
        }

        adcStep = ( regVal >> 6 ) & 0x3FF;
        if (adcStep <= 1)
            volts = 0.0;
        else
            volts = uabVoltsConvert(adcStep, ADC_CURR_SENSE_VOLTS);
        UabCsscAmps = (volts - 0.429) / 52.523 / 0.002;

        if ( ((UabMotionState == cruise_up_left) ||
              (UabMotionState == cruise_down_right)) &&
              (UabPwm > 25) )
        {
            /* Check for System Overload */
            curr_sense_amps = ((CURR_SENSE_AMPS_MAX * ((float)(UabPwm)))/100) + 0.75;
            if ( UabCsscAmps > curr_sense_amps )
            {
                uabMotorOff();
                UabSystemOverLoadAmps = UabCsscAmps;
                uabFaultSet(SYSTEM_OVERLOAD_FAULT,
                            SYSTEM_OVERLOAD_FAULT_NUM);
#if defined(SPEED_DEBUG)
                printf("SOL: Act: %3.1f  SC: %3.1f PWM: %2d%%\n",
                        UabCsscAmps, curr_sense_amps, UabPwm);
#endif
            }
            else
            {
#if defined(SPEED_DEBUG)
                if (UabFaults & SYSTEM_OVERLOAD_FAULT)
                  printf("SOL: Cleared\n");
#endif
                uabFaultClr(SYSTEM_OVERLOAD_FAULT,
                            SYSTEM_OVERLOAD_FAULT_NUM);
            }
        }
    }
}

/*
******************************************************************************
** Function:     uabCsscEnd
**
** Description:  Turn off Cssc until next 10 ms scan.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabCsscEnd(void)
{
    LPC_ADC->CR &= 0xF8FFFFFF;    /* stop ADC now */
    LPC_CT32B0->MR[1] = 0;
}
