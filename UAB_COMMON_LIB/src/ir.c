/*
 ******************************************************************************
 ** Filename:  ir.c
 **
 ** Description:
 **       This file contains the interrupt service routine and other functions
 **       to decode the data patterns received by the IR Detectors.
 **       The IR detectors decode the IR signal sent from either of
 **       two IR Remote Control transmitters and pass the signal to the
 **       IR_IN PIO pin on the LPC122x.
 **
 **       The IR functions must gather and decode a 26-bit data pattern.
 **       This 26-bit data pattern indicates whether the IR Remote transmitter
 **       UP or DOWN button is being pressed for any of the 4 possible
 **       IR Unit #s (0-3).
 **
 **       See UAB Software Design Spec (Agile DOC0003413) for
 **       more details on IR input signal processing.
 **
 ** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
 ******************************************************************************
 */

#include "LPC122x.h"
#include "main.h"
#include "uab.h"
#include "ir.h"

/* IR Global Variables */
uabIrState_t UabIrState = irOff;
uint32_t UabIr0UnitNum = 0xFF;
uint32_t UabIr1UnitNum = 0xFF;
uint32_t IrPwp[26];
uint8_t IrPwpProcessed = TRUE;
uint8_t IrIntrDetected = FALSE;
uint8_t IrLearning = 0;

/*
 ******************************************************************************
 ** Function:     PIOINT1_IRQHandler
 **
 ** Description:  PIO Port 1 Interrupt Handler.
 **               Only IR_IN signal (PIO1_6) is enabled for interrupt handling.
 **               The IR signal repeats every 200 ms as long as the
 **               IR remote pushbutton is pressed.
 **               This Interrupt Handler is invoked on the first low-going edge
 **               of the 26 pulses comprising the IR pattern. This ISR times
 **               the width in usec of each high and low portion of the signal
 **               and saves for processing within the 10 ms execution loop.
 **
 **               Note: Processing of this ISR takes around 20 ms to complete.
 **                     You get a IR interrupt about every 200 ms, so having
 **                     one scan every 200 ms be greater than the normal 10 ms
 **                     realtime scan is acceptable.
 **
 ** Input  Parms: None
 **
 ** Output Parms: None
 **
 ** Return Value: None
 **
 ******************************************************************************
 */
void PIOINT1_IRQHandler(void)
{
    int i = 0;
    int j = 0;
    uint32_t start_time;
    uint32_t stop_time;

    __disable_irq();

    if (LPC_GPIO1->MIS & IR_IN)
    {
        /* Check if previous IR pattern processed during 10ms scan */
        if (IrPwpProcessed)
        {
            while ((i < 26) && (j < 40000))
            {
                SysTick->VAL = 0;
                start_time = SysTick->VAL;
                while ((LPC_GPIO1->PIN & IR_IN)== 0 );
                stop_time = SysTick->VAL;
                IrPwp[i++] = start_time - stop_time;

                SysTick->VAL = 0;
                start_time = SysTick->VAL;
                while (LPC_GPIO1->PIN & IR_IN)
                {
                    if (j++ >= 40000)
                        break;
                }
                stop_time = SysTick->VAL;
                IrPwp[i++] = start_time - stop_time;
            }
            IrPwpProcessed = FALSE;
            IrIntrDetected = TRUE;
        }
        LPC_GPIO1->IC |= IR_IN;
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
    __enable_irq();
}

/*
 ******************************************************************************
 ** Function:     uabIrInit
 **
 ** Description:  Initializes the IR Input port (PIO1_6).
 **               Enable interrupts for only this PIO1_6 IR input.
 **
 ** Input  Parms: None
 **
 ** Output Parms: None
 **
 ** Return Value: None
 **
 ******************************************************************************
 */
void uabIrInit(void)
{
    int i;

    /* Clear Pulse Width Period (IrPwp) data */
    for (i = 0; i < 26; i++)
        IrPwp[i] = 0;

    /* Enable IR Interrupt */
    NVIC_EnableIRQ(EINT1_IRQn);
    LPC_GPIO1 ->IE |= IR_IN;
}

/*
 ******************************************************************************
 ** Function:     uabIrPatternGet
 **
 ** Description:  Decode the usec pulse times returned by the IR Interrupt
 **               Handler into a 26-bit data pattern for comparison to
 **               determine if the UP/DOWN IR remote control button is being
 **               pressed.
 **
 **               See UAB Software Design Spec (Agile DOC0003413) for
 **               more details on IR input signal processing.
 **
 ** Input  Parms: None
 **
 ** Output Parms: None
 **
 ** Return Value: uint32_t  26-bit IR Data Pattern
 **                         (32-bits but upper 6 bits always set to 1)
 **
 ******************************************************************************
 */
uint32_t uabIrPatternGet(void)
{
    int i, j;
    uint32_t pat;
    int bit;

    /* Divide by 24 (24 MHz system clock) to get pulse width period in usec */
    for (i = 0; i < 26; i++)
        IrPwp[i] /= 24;

    i = 0;
    j = 0;
    bit = 25;
    pat = 0xFC000000; /* Only 26-bit pattern..set upper 6-bits to 1 */
    while (i < 26)
    {
        /* if pulse width is <500 usec or >1900 usec..invalid pulse..toss */
        if ((IrPwp[i] < 500) || (IrPwp[i] > 1900))
            break;

        /* 
         ** The following code detects when a pulse is low or high for more
         ** than one normal pulse width (750 usec).
         ** Any pulse width >1200 usec is treated as 2 low or high pulses in
         ** a row. The maximum number of high or low pulse in a row is 2.
         */
        if (j == 0)
        {
            j = 1;
            if (IrPwp[i] > 1200)
                bit -= 2;
            else
                bit--;
        }
        else
        {
            j = 0;
            if (IrPwp[i] > 1200)
            {
                pat |= (1 << bit);
                bit--;
                pat |= (1 << bit);
                bit--;
            }
            else
            {
                pat |= (1 << bit);
                bit--;
            }
        }
        i++;
    }

    return (pat);
}

/*
 ******************************************************************************
 ** Function:     uabIrStateGet
 **
 ** Description:  Based on the 26-bit IR pattern passed into this function,
 **               determine if the pattern matches any of the acceptable
 **               UP or DOWN patterns for any of the four possible IR Unit#s.
 **               During the IR Learning process, the function sets the
 **               IR Unit#s being detected by the UP/DOWN pattern matching.
 **               During normal execution, if the user is pressing an IR UP
 **               or DOWN button, an IR state flag is returned which is used
 **               by the UP/DOWN motion control (armrest switch or IR button)
 **               to control the movement of the carriage on the stair lift.
 **
 **               See UAB Software Design Spec (Agile DOC0003413) for
 **               more details on IR input signal processing.
 **
 ** Input  Parms: None
 **
 ** Output Parms: None
 **
 ** Return Value: uabIrState_t   IR UP/DOWN button state
 **                                       irOff
 **                                       irUpLeft
 **                                       irDownRight
 **                                       irInvalid
 **
 ******************************************************************************
 */
uabIrState_t uabIrStateGet(uint32_t ir_pattern)
{
    uabIrState_t ir_state = irInvalid;

    /*
     ** The IR pattern cases were decoded directly from the signal
     ** received from the IR Remote transmitters set for each Unit# (0-3).
     */
    switch (ir_pattern)
    {
        case IR_UNIT0_UP_A:
        case IR_UNIT0_UP_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 0;
                    break;
                case 2:
                    UabIr1UnitNum = 0;
                    break;
                default:
                    if ((UabIr0UnitNum == 0) || (UabIr1UnitNum == 0))
                      ir_state = irUpLeft;
                    break;
            }
            break;

        case IR_UNIT1_UP_A:
        case IR_UNIT1_UP_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 1;
                    break;
                case 2:
                    UabIr1UnitNum = 1;
                    break;
                default:
                    if ((UabIr0UnitNum == 1) || (UabIr1UnitNum == 1))
                        ir_state = irUpLeft;
                    break;
            }
            break;

        case IR_UNIT2_UP_A:
        case IR_UNIT2_UP_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 2;
                    break;
                case 2:
                    UabIr1UnitNum = 2;
                    break;
                default:
                    if ((UabIr0UnitNum == 2) || (UabIr1UnitNum == 2))
                        ir_state = irUpLeft;
                    break;
            }
            break;

        case IR_UNIT3_UP_A:
        case IR_UNIT3_UP_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 3;
                    break;
                case 2:
                    UabIr1UnitNum = 3;
                    break;
                default:
                    if ((UabIr0UnitNum == 3) || (UabIr1UnitNum == 3))
                        ir_state = irUpLeft;
                    break;
            }
            break;

        case IR_UNIT0_DOWN_A:
        case IR_UNIT0_DOWN_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 0;
                    break;
                case 2:
                    UabIr1UnitNum = 0;
                    break;
                default:
                    if ((UabIr0UnitNum == 0) || (UabIr1UnitNum == 0))
                        ir_state = irDownRight;
                    break;
            }
            break;

        case IR_UNIT1_DOWN_A:
        case IR_UNIT1_DOWN_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 1;
                    break;
                case 2:
                    UabIr1UnitNum = 1;
                    break;
                default:
                    if ((UabIr0UnitNum == 1) || (UabIr1UnitNum == 1))
                        ir_state = irDownRight;
                    break;
            }
            break;

        case IR_UNIT2_DOWN_A:
        case IR_UNIT2_DOWN_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 2;
                    break;
                case 2:
                    UabIr1UnitNum = 2;
                    break;
                default:
                    if ((UabIr0UnitNum == 2) || (UabIr1UnitNum == 2))
                        ir_state = irDownRight;
                    break;
            }
            break;

        case IR_UNIT3_DOWN_A:
        case IR_UNIT3_DOWN_B:
            switch (IrLearning)
            {
                case 1:
                    UabIr0UnitNum = 3;
                    break;
                case 2:
                    UabIr1UnitNum = 3;
                    break;
                default:
                    if ((UabIr0UnitNum == 3) || (UabIr1UnitNum == 3))
                        ir_state = irDownRight;
                    break;
            }
            break;

        default:
            break;
    }

    return (ir_state);
}

