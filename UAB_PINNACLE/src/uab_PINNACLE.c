/*
******************************************************************************
** Filename:  uab_PINNACLE.c
**
** Description:
**       This file contains all of the functions for the monitoring/control
**       of the PINNACLE stair lift system via the UAB module.
**       The UAB major functions are as follows:
**
**             1.  Motion Control of the carriage UP/DOWN the stair lift.
**                 This is mainly handled by a Motion State Machine in 
**                 the uabMotionSet() function.
**
**             2.  System/Safety fault detection and handling.
**                 Alarm/Fault prioritization and reporting via
**                 armrest LED and Beeper is done by functions in alarms.c.
**
**             3.  Charging circuit monitoring/control.
**                 This is done in the function uabChargeCheck().
**
**             4.  Misc functions to assist in debug during development
**                 and production test. 
**                 Debug enables are run-time controlled not compile-time.
**
**       See the UAB Software Design Spec (Agile DOC0003413) for
**       more details on the Motion State Machine description and diagrams 
**       and detailed description of the charging circuit function.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include <string.h>
#include "LPC122x.h"
#include "main.h"
#include "main_PINNACLE.h"
#include "utils.h"
#include "uab.h"
#include "alarms.h"
#include "compadc.h"
#include "ir.h"
#include "wdt.h"
#include "uart.h"

#if defined(PINNACLE_DEV_DEBUG)  /* Debug Only */
#include "small_printf.h"
#endif

/* UAB GLobal Variables */
uint8_t                 UabMotorIsOff     = FALSE;
uint8_t                 UabInstMode       = FALSE;
uint8_t                 UabTripDone       = FALSE;
uint8_t                 UabMajorFault     = FALSE;
uint8_t                 UabMajorFaultInProgress = FALSE;
uint8_t                 UabUpDownReleased = FALSE;
uint32_t                UabSystemId;
uint32_t                UabFaults         = NO_FAULTS;
uint32_t                UabDebugMode      = 0;

uabMotionState_t       UabMotionState    = stopped;
uabBrakeState_t        UabBrakeState;  /* for SW tracking */
uabUpDownLeftRightSwitchState_t UabUpDownLeftRightSwitchState = switch_off;

/* PWM Control Variables */
int8_t UabBegPwm;
int8_t UabEndPwm;
int8_t UabPwmStep;
int8_t UabRampInterval;
int8_t UabPwm;

/* Charger Circuit Variables */
uint8_t  UabOffCharge        = FALSE;
uint8_t  UabOffchargeFlag    = FALSE;
uint8_t  UabChargePercent    = 0;
int32_t  UabChargeTimer      = 0;
uint8_t  UabChargeInProgress = FALSE;
int32_t  UabChargePeriod     = 0;
uint32_t UabChargeDuration   = 0;
float    SaveBatteryVolts    = 0.0f;
float    SaveChargeVolts     = 0.0f;

float    UabSense12Volts     = 0.0f;
uint32_t UabMajorFaultCause  = 0;
float    UabBatteryVolts     = 0.0f;
float    UabSystemOverLoadAmps = 0.0f;
float    UabSystemOverLoadScaledAmps = 0.0f;
float    UabRunawayAmps = 0.0f;
uint8_t  UabMDriveState  = UAB_DISABLED;
uint8_t  UabChargerState = UAB_DISABLED;
uint8_t  UabChargerInitState = UAB_DISABLED;
uint8_t  UabInstUpDownState;
uint8_t  UabUpDownState;
uint8_t  UabIrUpDownState;
uint8_t  UabInitComplete = FALSE;
uint8_t  UabCommEnabled  = FALSE;
uint8_t  UabLed = 0;
uint8_t  SeqNum = 1;

/*
******************************************************************************
** Function:     uabEvent
**
** Description:  Post an event/update message that is to be sent to DiagGUI
**               into the next available Xmit FIFO buffer.
**
** Input  Parms: event           Event/Update message ID
**               *event_data     Ptr to caller's message to be sent to DiagGUI
**               event_data_len  Length in bytes of event data part of message.
**                               (does not include 8-byte message header)
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabEvent(uint8_t event, void *event_data, uint8_t event_data_len)
{
    int i;
    uint8_t chksum;
    uint8_t *event_data_ptr;
    uint8_t xmitMsg[MAX_XMIT_MSG_SIZE];

    if ((UabCommEnabled) || (event == UAB_INIT_COMPLETE_EVENT))
    {
    	event_data_ptr = (uint8_t *)(event_data);
    	if (event_data_len > (MAX_XMIT_MSG_SIZE - 8))
    	{
            XmitErrDataLen++;
            return;
    	}
    	memset(xmitMsg, 0, MAX_XMIT_MSG_SIZE);
        if (event < UAB_EVENTS_MAX)
    	  xmitMsg[0] = UAB_EVENT_OPCODE;   /* 'E' 0x45 */
        else
    	  xmitMsg[0] = UAB_UPDATE_OPCODE;  /* 'U' 0x55 */
    	xmitMsg[1] = event;
    	xmitMsg[2] = event_data_len;
        /* Checksum [0]+[1]+[2] */
    	xmitMsg[3] = xmitMsg[0] + xmitMsg[1] + xmitMsg[2]; 
    	xmitMsg[4] = 0;
    	xmitMsg[5] = 0;
    	xmitMsg[6] = SeqNum;
    	chksum = 0;
        /* Store event data in the message */
    	for (i = 0; i < event_data_len; i++)
    	{
    	    xmitMsg[8+i] = *event_data_ptr;
            chksum += *event_data_ptr++;
    	}
        /* Add checksum byte */
        xmitMsg[7] = chksum;

        /* Load DiagGUI message into next avail Xmit FIFO buffer */
    	xmitFifoBufLoad(xmitMsg,(8 + event_data_len), SeqNum);
        /* Adjust Sequence # for next message */
        if (SeqNum  < 255)
           SeqNum++;
        else
           SeqNum = 1;
    }
}

/*
******************************************************************************
** Function:     uabInstModeCheck   (Pinnacle/Sierra Only)
**
** Description:  Based on detected combination of switch settings,
**               determine if UAB is in Installation Mode.
**
** Input  Parms: uint32_t  pio0    Current reading of PIO0 Register
**
** Output Parms: None
**
** Global Vars:  uint8_t UabInstMode  (TRUE or FALSE)
**
** Return Value: None
** 
******************************************************************************
*/
void uabInstModeCheck(uint32_t  pio0)
{
    if ( (pio0 & FOOTREST_UP_LEFT_SWITCH) &&
         (pio0 & FOOTREST_DOWN_RIGHT_SWITCH)  &&
         (pio0 & FOOTREST_UNDER_SWITCH) &&
         (pio0 & SEAT_SWITCH) &&
         (pio0 & UP_SWITCH) &&
         (pio0 & DOWN_SWITCH) )
    {
        if (UabInstMode == FALSE)
        {
            UabInstMode = TRUE;
            uabEvent(UAB_INST_MODE_CHANGE_EVENT,
                      &UabInstMode, sizeof(UabInstMode) );
        }
        UabInstMode = TRUE;
    }
    else
    {
        if (UabInstMode)
        {
            UabInstMode = FALSE;
            uabEvent(UAB_INST_MODE_CHANGE_EVENT,
                      &UabInstMode, sizeof(UabInstMode));
        }
        UabInstMode = FALSE;
    }
}

/*
******************************************************************************
** Function:     uabFaultSet
**               uabFaultClr
**
** Description:  Set/Clr a UAB Minor Fault which is reported by
**               the Alarm/Fault Handler code.
**
**               Note:  The compiler warning messages generated when
**               these functions are compiled are OK.
**
** Input  Parms: fault_bit  Bit representing minor fault.
**               fault_num  Minor Fault number included as part of
**                          DiagGUI message.
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
#define FAULT_CLR 0
#define FAULT_SET 1

void uabFaultSet(uint32_t fault_bit, uint8_t fault_num)
{
    uint8_t faultData[6];
    uint32_t bvu;
    uint32_t curru;

    if ( UabInitComplete &&
         ((UabFaults & fault_bit) == 0) )
    {
        UabFaults |= fault_bit;
    	memset(faultData, 0, sizeof(faultData));
        faultData[0] = fault_num;
        faultData[1] = FAULT_SET;
        if ( (fault_num == LOW_BATTERY_FAULT_NUM) ||
             (fault_num == VERY_LOW_BATTERY_FAULT_NUM) ||
             (fault_num == SUPER_LOW_BATTERY_FAULT_NUM) )
        {
            bvu = *((uint32_t *)(&UabBatteryVolts));
            faultData[2] = (uint8_t)((bvu >> 24) & 0xFF);
            faultData[3] = (uint8_t)((bvu >> 16) & 0xFF);
            faultData[4] = (uint8_t)((bvu >>  8) & 0xFF);
            faultData[5] = (uint8_t)(bvu & 0xFF);
        }
        else 
        {
            if (fault_num == SYSTEM_OVERLOAD_FAULT_NUM) 
            {
                curru = *((uint32_t *)(&UabSystemOverLoadAmps));
                faultData[2] = (uint8_t)((curru >> 24) & 0xFF);
                faultData[3] = (uint8_t)((curru >> 16) & 0xFF);
                faultData[4] = (uint8_t)((curru >>  8) & 0xFF);
                faultData[5] = (uint8_t)(curru & 0xFF);
            }
        }
        uabEvent(UAB_FAULT_CHANGE_EVENT,
                  &faultData, sizeof(faultData));
    }
    UabFaults |= fault_bit;
}

void uabFaultClr(uint32_t fault_bit, uint8_t fault_num)
{
    uint8_t faultData[6];

    if ( UabInitComplete &&
         (UabFaults & fault_bit) )
    {
        memset(faultData, 0, sizeof(faultData));
        UabFaults &= ~fault_bit;
        faultData[0] = fault_num;
        faultData[1] = FAULT_CLR;
        uabEvent(UAB_FAULT_CHANGE_EVENT,
                  &faultData, sizeof(faultData));
    }
    UabFaults &= ~fault_bit;
}


void uabMajorFaultChargeDisplay( uint8_t offcharge )
{
    uint8_t xmitMsg[MAX_XMIT_MSG_SIZE];

    memset(xmitMsg, 0, MAX_XMIT_MSG_SIZE);
    xmitMsg[0] = UAB_EVENT_OPCODE;   /* 'E' 0x45 */
    xmitMsg[1] = UAB_CHARGER_STATE_CHANGE_EVENT;
    xmitMsg[2] = 1;  /* Data Len */
    /* Checksum [0]+[1]+[2] */
    xmitMsg[3] = xmitMsg[0] + xmitMsg[1] + xmitMsg[2];
    xmitMsg[4] = 0;
    xmitMsg[5] = 0;
    xmitMsg[6] = 3;  /* dummy Seq # */
    xmitMsg[7] = 0;
    xmitMsg[8] = 0;  /* Not Charging */
    if (!offcharge)
    {
        xmitMsg[7] = 1;  /* Charging */
        xmitMsg[8] = 1;
    }

    UART0_Count     = 0;
    UART0_Buffer[0] = 0;
    UART0_Buffer[1] = 0;
    UARTSend(UART0, xmitMsg, 9);
    delayMs(30, BUSY_WAIT_BASED, TRUE);
}


void uabMajorFaultVoltsDisplay( uint8_t *eData )
{
    uint8_t xmitMsg[MAX_XMIT_MSG_SIZE];
    uint8_t chksum;
    int i;

    memset(xmitMsg, 0, MAX_XMIT_MSG_SIZE);
    xmitMsg[0] = UAB_UPDATE_OPCODE;  /* 'U' 0x55 */
    xmitMsg[1] = UAB_VOLTS_CURRENT_UPDATE_EVENT;
    xmitMsg[2] = 15;  /* Data Len */
    /* Checksum [0]+[1]+[2] */
    xmitMsg[3] = xmitMsg[0] + xmitMsg[1] + xmitMsg[2];
    xmitMsg[4] = 0;
    xmitMsg[5] = 0;
    xmitMsg[6] = 2;  /* dummy Seq # */
    chksum = 0;
    /* Store event data in the message */
    for (i = 0; i < 15; i++)
    {
        xmitMsg[8+i] = *eData;
        chksum += *eData++;
    }
    /* Add checksum byte */
    xmitMsg[7] = chksum;

    UART0_Count     = 0;
    UART0_Buffer[0] = 0;
    UART0_Buffer[1] = 0;
    UARTSend(UART0, xmitMsg, 23);
    delayMs(30, BUSY_WAIT_BASED, TRUE);
}

/*
******************************************************************************
** Function:     uabMajorFaultHandler
**
** Description:  This function is called whenever a Major Fault is detected.
**               The Motor is turned off and the Brake is engaged.
**               The Charging circuit is Disabled.
**               The Beeper is turned OFf and LED is set to RED.
**               Then a forever while loop is entered, which punches the WDT.
**               This function never returns...
**               A system reset/reboot via software or via power-cycle of UAB
**               is required to recover the system.
**
**               The Major Fault message is repeated every minute.
**               In addition, the UAB beeper will beep and the on-board
**               CPU LED will blink a number of times every minute indicating
**               which Major Fault has occurred in case no DiagGUI comm exists.
**
** Input  Parms: cause:  Major Fault cause number
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/

#define MAJ_FAULT_TEN_MINUTE 57000   /* 10 Minutes - code exec time */

void uabMajorFaultHandler(uint8_t cause)
{
    uint8_t xmitMsg[MAX_XMIT_MSG_SIZE];
    int i;
    static uint32_t displayVoltsCntr = 200;
    static uint8_t  causeCntr        = 0;
    static uint32_t minuteCntr       = MAJ_FAULT_TEN_MINUTE;
    static uint8_t  doBeepOn         = FALSE;
    static uint8_t  doBeepOnCntr     = 0;
    static uint8_t  doBeepOff        = FALSE;
    static uint8_t  doBeepOffCntr    = 0;

    uabMotorOff();
    alarms_beep_off();
    alarms_light_led(lcs_RED);
    uabCpuLedOff();

    UabMajorFaultInProgress = TRUE;

    memset(xmitMsg, 0, MAX_XMIT_MSG_SIZE);
    xmitMsg[0] = UAB_EVENT_OPCODE;   /* 'E' 0x45  */
    xmitMsg[1] = UAB_MAJOR_FAULT_EVENT;
    xmitMsg[2] = 1;  /* Data Len */
    /* Checksum [0]+[1]+[2] */
    xmitMsg[3] = xmitMsg[0] + xmitMsg[1] + xmitMsg[2];
    xmitMsg[4] = 0;
    xmitMsg[5] = 0;
    xmitMsg[6] = 1;      /* dummy Seq # */
    xmitMsg[7] = cause;  /* Checksum */
    xmitMsg[8] = cause;  /* Data */

    delayMs(2000, BUSY_WAIT_BASED, TRUE);

    while (1)
    {
        wdtFeed(TRUE);

        if (++minuteCntr >= MAJ_FAULT_TEN_MINUTE)
        {
            minuteCntr = 0;
            for (i=0; i<2; i++)
            {
                UART0_Count     = 0;
                UART0_Buffer[0] = 0;
                UART0_Buffer[1] = 0;
                UARTSend(UART0, xmitMsg, 9);
                delayMs(25, BUSY_WAIT_BASED, TRUE);
            }
#if defined(PINNACLE_DEV_DEBUG)  /* Debug Only */
        printf("Major Fault:  Cause %d\n", cause);
#endif
            causeCntr = cause - 1;
            doBeepOn = TRUE;
            wdtFeed(TRUE);
            uabCpuLedOn();
            alarms_beep_on();
        }

        if (doBeepOn)
        {
            if (++doBeepOnCntr >= 50)
            {
                doBeepOn     = FALSE;
                doBeepOnCntr = 0;
                doBeepOff    = TRUE;
                uabCpuLedOff();
                alarms_beep_off();
            }
        }

        if (doBeepOff)
        {
            if (++doBeepOffCntr >= 50)
            {
                doBeepOff     = FALSE;
                doBeepOffCntr = 0;
                if (causeCntr > 0)
                {
                    causeCntr--;
                    doBeepOn = TRUE;
                    uabCpuLedOn();
                    alarms_beep_on();
                }
            }
        }

        if (cause != UAB_MAJOR_FAULT_12V_SUPPLY)
        {
           uabChargeCheck();
           alarms_light_led(lcs_RED);
        }

        if (++displayVoltsCntr > 200)
        {
            displayVoltsCntr = 0;
            uabMajorFaultChargeDisplay((uint8_t)UabOffCharge);
            uabVoltsCurrSenseDisplay();
        }

        delayMs(10, BUSY_WAIT_BASED, TRUE);
    }  /* Loop until board is reset */
}

/*
******************************************************************************
** Function:     uab12vCheck   
**
** Description:  This function verifies the 12V Supply voltage is 
**               valid (9.5 V - 13.0 V).
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: OK or NOT_OK
** 
******************************************************************************
*/

#define SENSE_12V_VOLTS_MIN    9.5f
#define SENSE_12V_VOLTS_MAX   13.0f

int uab12vCheck(void)
{
    float sense12_volts;

    sense12_volts = uabCompVoltsRead(CMP_SENSE12_VOLTS);
    UabSense12Volts = sense12_volts;
    if ( (sense12_volts < SENSE_12V_VOLTS_MIN) ||
         (sense12_volts > SENSE_12V_VOLTS_MAX) )
      return(NOT_OK);

    return(OK);
}

/*
******************************************************************************
** Function:     uabLowBatteryCheck   
**
** Description:  This function verifies the Battery Voltage is not in the
**               Low Battery range of >21.0 V to <22.0 V.
**               5 consecutive 10 ms readings are required to set/clear a
**               Low Battery System Fault.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults (Set/Clr LOW_BATTERY_FAULT)
**
** Return Value: None
** 
******************************************************************************
*/

#define LOW_BATTERY_VOLTS        22.0f
#define LOW_BATTERY_VOLTS_MIN    21.0f

void uabLowBatteryCheck(void)
{
    static float   bat_volts[5];
    static uint8_t ind = 0;
    uint8_t        cnt = 0;

    if (ind < 5)
    {
       bat_volts[ind] = uabAdcVoltsRead(ADC_BATTERY_SEL);
       ind++;
    }
    else
    {
        cnt = 0;
        for (ind=0; ind < 5; ind++)
        {
            if ( (bat_volts[ind] < LOW_BATTERY_VOLTS) &&
                 (bat_volts[ind] > LOW_BATTERY_VOLTS_MIN) )
                cnt++;
        }
        ind = 0;
        if (cnt == 5)
        {
            UabBatteryVolts = bat_volts[4];
            uabFaultSet(LOW_BATTERY_FAULT, LOW_BATTERY_FAULT_NUM);
        }
        else
        {
            cnt = 0;
            for(ind=0; ind < 5; ind++)
            {
                if (bat_volts[ind] > LOW_BATTERY_VOLTS)
                    cnt++;
            }
            ind = 0;
            if (cnt == 5)
            {
                UabBatteryVolts = bat_volts[4];
                uabFaultClr(LOW_BATTERY_FAULT, LOW_BATTERY_FAULT_NUM);
                UabTripDone = FALSE;
            }
        }
    }
}

/*
******************************************************************************
** Function:     uabVeryLowBatteryCheck   
**
** Description:  This function verifies the Battery Voltage is not in the
**               Very Low Battery range of >18.0 V to <21.0 V.
**               5 consecutive 10 ms readings are required to set/clr a
**               Very Low Battery System Fault.
**               The Very Low Battery fault will not clear until the
**               battery voltage goes above 23.0 V.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults    (Set/Clr VERY_LOW_BATTERY_FAULT)
**               uint8_t  UabTripDone  Flag to indicate carriage is at
**                                      the end of track when a Very Low
**                                      Battery voltage fault exists.
** Return Value: OK or NOT_OK
** 
******************************************************************************
*/
#define VERY_LOW_BATTERY_VOLTS          21.0f
#define VERY_LOW_BATTERY_VOLTS_REENABLE 23.0f

int uabVeryLowBatteryCheck(void)
{
    int            retVal = OK;
    static float   bat_volts[5];
    static uint8_t ind = 0;
    uint8_t        cnt = 0;

    if (ind < 5)
    {
        bat_volts[ind] = uabAdcVoltsRead(ADC_BATTERY_SEL);
        ind++;
        if (UabFaults & VERY_LOW_BATTERY_FAULT)
          retVal = NOT_OK;
    }
    else
    {
        cnt = 0;
        for (ind=0; ind < 5; ind++)
        {
            if (bat_volts[ind] < VERY_LOW_BATTERY_VOLTS)
                cnt++;
        }
        ind = 0;
        if (cnt == 5)
        {
            UabBatteryVolts = bat_volts[4];
            uabFaultSet(VERY_LOW_BATTERY_FAULT, VERY_LOW_BATTERY_FAULT_NUM);
            retVal = NOT_OK;
        }
        else
        {
            cnt = 0;
            for (ind=0; ind < 5; ind++)
            {
                if (bat_volts[ind] > VERY_LOW_BATTERY_VOLTS_REENABLE)
                   cnt++;
            }
            ind = 0;
            if (cnt == 5)
            {
                UabBatteryVolts = bat_volts[4];
                uabFaultClr(VERY_LOW_BATTERY_FAULT, VERY_LOW_BATTERY_FAULT_NUM);
                UabTripDone = FALSE;
            }
            else
                retVal = NOT_OK;
        }
    }
    return(retVal);
}

/*
******************************************************************************
** Function:     uabSuperLowBatteryCheck   
**
** Description:  This function verifies the Battery Voltage is not in the
**               Super Low Battery range of <18.0 V.
**               5 consecutive 10 ms readings are required to set/clr a
**               Super Low Battery System Fault.
**               The Super Low Battery fault will not clear until the
**               battery voltage goes above 23.0 V.
**
**               When a Super Low Battery voltage (<18.0 V) is detected,
**               the Motor is immediately shut off and the brake engaged
**               the state of the motion state machine is not changed until
**               the UP/DOWN armrest switch or the IR Remote UP/DOWN button
**               is released...then the motion state changes to STOPPED.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults      (Set/Clr SUPER_LOW_BATTERY_FAULT)
**               uint8_t  UabMotorIsOff  Flag to prevent Runaway fault from
**                                        posting when Super Low Battery fault
**                                        exists.
** Return Value: OK or NOT_OK
** 
******************************************************************************
*/
#define SUPER_LOW_BATTERY_VOLTS          18.0f
#define SUPER_LOW_BATTERY_VOLTS_REENABLE 23.0f

int uabSuperLowBatteryCheck(void)
{
    int            retVal = OK;
    static float   bat_volts[5];
    static uint8_t ind = 0;
    uint8_t        cnt = 0;

    if (ind < 5)
    {
        bat_volts[ind] = uabAdcVoltsRead(ADC_BATTERY_SEL);
        ind++;
        if (UabFaults & SUPER_LOW_BATTERY_FAULT)
          retVal = NOT_OK;
    }
    else
    {
        cnt = 0;
        for (ind=0; ind < 5; ind++)
        {
            if (bat_volts[ind] < SUPER_LOW_BATTERY_VOLTS)
                cnt++;
        }
        ind = 0;
        if (cnt == 5)
        {
            uabMotorOff();

            UabBatteryVolts = bat_volts[4];
            uabFaultSet(SUPER_LOW_BATTERY_FAULT, SUPER_LOW_BATTERY_FAULT_NUM);
            retVal = NOT_OK;
        }
        else
        {
            cnt = 0;
            for (ind=0; ind < 5; ind++)
            {
                if (bat_volts[ind] > SUPER_LOW_BATTERY_VOLTS_REENABLE)
                   cnt++;
            }
            ind = 0;
            if (cnt == 5)
            {
                UabBatteryVolts = bat_volts[4];
                uabFaultClr(SUPER_LOW_BATTERY_FAULT, SUPER_LOW_BATTERY_FAULT_NUM);
                UabTripDone = FALSE;
            }
            else
                retVal = NOT_OK;
        }
    }
    return(retVal);
}

/*
******************************************************************************
** Function:     uabSystemOverloadCheck   
**
** Description:  This function checks for a set/clr of a System Overload fault.
**               Per the System Requirements, a System Overload fault occurs
**               when:
**                      PINNACLE Current Sense is > 20.0 Amps (scaled)
**
**               Since we do not operate at a full 100% PWM rate,
**               these values must be scaled down based on the max PWM rate
**               we are using for proper System Overload fault detection.
**               
**               We do 100 reads of the ADC for current sense each 10ms,
**               if a single read ADC step is greater than the associated
**               max current sense ADC step, a System Overload fault occurs.
**
**               When a System Overload fault is detected,
**               the Motor is immediately shut off and the brake engaged
**               the state of the motion state machine is not changed until
**               the UP/DOWN armrest switch or the IR Remote UP/DOWN button
**               is released...then the motion state changes to STOPPED.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults      (Set/Clr SYSTEM_OVERLOAD_FAULT)
**               uint8_t  UabMotorIsOff  Flag to prevent Runaway fault from
**                                        posting when System Overload fault
**                                        exists.
** Return Value: None
** 
******************************************************************************
*/

/* Number of ADC readings each 10ms loop for current sense */
#define CS_LOOP_MAX 100

/* Scaled factors based on the max PWM rate allowed */
#define PINNACLE_CURR_SENSE_AMPS_MAX           20.0f
#define PINNACLE_CURR_SENSE_SCALE_FACTOR_DOWN  0.64f   /* Max PWM 60% */
#define PINNACLE_CURR_SENSE_SCALE_FACTOR_UP    0.94f   /* Max PWM 95% */

void uabSystemOverloadCheck(void)
{
    float curr_sense_amps;
    float volts;
    float amps;
    uint32_t step;
    uint32_t max_step = 0;
    int i;
    static uint32_t SolClearDelay = 0;

    if (UabMotorIsOff)
    {
        if (SolClearDelay > 0)
            SolClearDelay--;
        else
            uabFaultClr(SYSTEM_OVERLOAD_FAULT, SYSTEM_OVERLOAD_FAULT_NUM);
        return;
    }

    for (i=0; i < CS_LOOP_MAX; i++)
    {
        step = uabAdcRead( ADC_CURR_SENSE_SEL );
        if (step > max_step)
          max_step = step;
    }

    if (max_step <= 1)
      volts = 0.0;
    else
      volts = uabVoltsConvert(max_step, ADC_CURR_SENSE_VOLTS);
    amps = (volts - 0.429) / 52.523 / 0.002;

    if (amps > 15.0)
    {
        curr_sense_amps = PINNACLE_CURR_SENSE_AMPS_MAX; /* Non-scaled */
        if (UabUpDownLeftRightSwitchState == switch_up_left)
        {
            curr_sense_amps = PINNACLE_CURR_SENSE_AMPS_MAX *
                              PINNACLE_CURR_SENSE_SCALE_FACTOR_UP;
        }
        else
        {
            if (UabUpDownLeftRightSwitchState == switch_down_right)
            {
                curr_sense_amps = PINNACLE_CURR_SENSE_AMPS_MAX *
                                  PINNACLE_CURR_SENSE_SCALE_FACTOR_DOWN;
            }
        }

        if ( amps > curr_sense_amps )
        {
            uabMotorOff();
            UabSystemOverLoadAmps = amps;
            UabSystemOverLoadScaledAmps = curr_sense_amps;
            uabFaultSet(SYSTEM_OVERLOAD_FAULT, SYSTEM_OVERLOAD_FAULT_NUM);
#if defined(PINNACLE_DEV_DEBUG)
            printf("SOL: ActAmps: %3.1f  ScaledAmps: %3.1f  PWM: %2d%%\n",
                   UabSystemOverLoadAmps,
                   UabSystemOverLoadScaledAmps,
                   UabPwm);
#endif
            SolClearDelay = 200;
        }
        else
        {
#if defined(PINNACLE_DEV_DEBUG)
            if (UabFaults & SYSTEM_OVERLOAD_FAULT)
              printf("SOL: Clr\n");
#endif
            uabFaultClr(SYSTEM_OVERLOAD_FAULT, SYSTEM_OVERLOAD_FAULT_NUM);
            SolClearDelay = 0;
        }
    }
}

/*
******************************************************************************
** Function:     uabRunawayCheck   (Pinnacle/Sierra Only)
**
** Description:  This function checks for a set/clr of a Runaway fault.
**               Per the System Requirements, a Runaway fault occurs
**               when:
**                      Pinnacle:  Current Sense is < 1.8 Amps
**                      & Sierra   when carriage is moving at full cruise
**                                 PWM rate (95%) in the UP direction.
**
**               If a Runaway fault is detected, a Major Fault will occur
**               shutting off the motor, engaging the brake, and disabling 
**               the charging circuit. A system reset/reboot will be required
**               to recover.
**                 
**               If a System Overload or Super Low Battery fault is currently
**               being detected, then the runaway fault check is skipped.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults      (Set/Clr RUNAWAY_FAULT)
**               uint8_t  UabMotorIsOff  Flag to prevent Runaway fault from
**                                        posting when System Overload or
**                                        Super Low Battery faults exist.
**
** Return Value: OK or NOT_OK
** 
******************************************************************************
*/
#define RUNAWAY_AMPS_MAX  1.8f

int uabRunawayCheck(void)
{
    float curr_sense_amps;

    if (UabMotorIsOff)
       return(OK);   /* Skip Runaway Check */

    if (UabMotionState == cruise_up_left)
    {
        curr_sense_amps = uabAdcAmpsRead();
        if (curr_sense_amps < RUNAWAY_AMPS_MAX)
        {
            UabRunawayAmps = curr_sense_amps;
#if defined(PINNACLE_DEV_DEBUG)
            printf("RA: Amps: %3.1f\n", UabRunawayAmps);
#endif
            return(NOT_OK);
        }
        /* Never cleared since Major Fault will happen */
    }

    return(OK);
}

/*
******************************************************************************
** Function:     uabDiagLed
**
** Description:  This function turns the Diag LED
**               RED, GREEN, AMBER, and OF
**
** Input  Parms: uint8 LED Color
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/

void uabDiagLed( uint8_t led_color )
{
    switch (led_color)
    {
        case lcs_RED:
            //--- LED color is RED
            LPC_IOCON ->PIO0_29 &= ~(1 << 9);
            LPC_GPIO0 ->OUT |= RED_LED_BIT;
            LPC_GPIO0 ->OUT &= ~(GREEN_LED_BIT);
            break;

        case lcs_GREEN:
            //--- LED color is GREEN
            LPC_GPIO0 ->OUT |= GREEN_LED_BIT;
            LPC_GPIO0 ->OUT &= ~(RED_LED_BIT);
            break;

        case lcs_AMBER:
            //--- LED color is AMBER
            LPC_IOCON ->PIO0_29 |= (1 << 9);
            LPC_GPIO0 ->OUT |= GREEN_LED_BIT;
            LPC_GPIO0 ->OUT |= RED_LED_BIT;
            break;

        case lcs_OFF:
            //--- LED is not illuminated
            LPC_GPIO0 ->OUT &= ~(RED_LED_BIT);
            LPC_GPIO0 ->OUT &= ~(GREEN_LED_BIT);
            break;

        default:
            break;
    }
}


/*
******************************************************************************
** Function:     uabCpuLedOn
**               uabCpuLedOff
**
** Description:  This function turns the on-board CPU LED ON/OFF
**
** Input  Parms: None
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabCpuLedOn(void)
{
    LPC_GPIO0->OUT |= CPU_DIAG_LED_BIT;
}

void uabCpuLedOff(void)
{
    LPC_GPIO0->OUT &= ~(CPU_DIAG_LED_BIT);
}

/*
******************************************************************************
** Function:     uabMDriveDisable
**               uabMDriveEnable
**
** Description:  These functions Enable/Disable the Motor Drive (Mdrive)
**               PIO output pin (PIO2_0).  In order for motor movement to 
**               occur, MDrive must be Enabled.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabMDriveDisable(void)
{
    LPC_GPIO2->OUT |= (DISABLE_MDRIVE_BIT);   /* Disable Mdrive */
    UabMDriveState = UAB_DISABLED;
}

void uabMDriveEnable(void)
{
    LPC_GPIO2->OUT &= ~(DISABLE_MDRIVE_BIT);   /* Enable Mdrive */
    UabMDriveState = UAB_ENABLED;
}

/*
******************************************************************************
** Function:     uabChargerDisable
**               uabChargerEnable
**
** Description:  These functions Enable/Disable the Charger
**               PIO output pin (PIO1_2). The Charger Enable/Disable is 
**               controlled by the uabChargeCheck() function during normal
**               operation.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabChargerDisable(void)
{
    LPC_GPIO1->OUT &= ~CHARGER_BIT;   /* Set Charger Bit LO (Disable) */
    UabChargerState = UAB_DISABLED;
}

void uabChargerEnable(void)
{
    LPC_GPIO1->OUT |= CHARGER_BIT;    /* Set Charger Bit HI (Enable) */
    UabChargerState = UAB_ENABLED;
}

/*
******************************************************************************
** Function:     uabChargerInitDisable
**               uabChargerInitEnable
**
** Description:  These functions Enable/Disable the Charger Init
**               PIO output pin (PIO1_0). The Charger Init must be Enabled
**               for the charging circuit to function.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabChargerInitDisable(void)
{
    LPC_GPIO1->OUT |= CHARGER_INIT_BIT;  /* Set Charger Init Bit HI (Dis) */
    UabChargerInitState = UAB_DISABLED;
}

void uabChargerInitEnable(void)
{
    LPC_GPIO1->OUT &= ~(CHARGER_INIT_BIT);  /* Set Charger Init Bit LO (En) */
    UabChargerInitState = UAB_ENABLED;
}

/*
******************************************************************************
** Function:     uabChargeCheck
**
** Description:  This function monitors/controls the UAB Charging circuit.
**               The charging cycle is based on a 2 Second period/window.
**               The charger circuit is enabled at the start of each
**               2 Second charging window. The Charger Voltage and the
**               Battery Voltage are both read after the charger is enabled.
**               Thus, at the start of each 2 Second charge window the 
**               charger is enabled for at least a short time.
**               
**               If the Charger Voltage is below 20.0 Volts, then an 
**               Off-Charge fault is posted, and the charger is disabled
**               until the start of the next 2 Second window. This normally
**               occurs when the carriage is not in an On-Charge position
**               (on the 33V power strips) at the end of the track.
**               
**               If the Charger Voltage is below the (Battery Voltage-2 Volts)
**               or the carriage is in motion UP/DOWN, the charger is disabled
**               until the start of the next 2 Second window.
**
**               If charging is enabled, determine what the charge duration
**               should be based on the current Battery Voltage reading:
**
**                   <=24.0          Charge Duration 95%  (1.9 Sec)
**                    >24.0 - <27.4  Charge Duration 85%  (1.7 Sec)
**                   >=27.4 - <27.7  Charge Duration 25%  (500 msec)
**                   >=27.7 - <28.0  Charge Duration  5%  (100 msec)
**                   >=28.0          Charge Duration  0   (Charging Disabled)
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t UabFaults      (Set/Clr OFFCHARGE_FAULT)
**
** Return Value: None
** 
******************************************************************************
*/
#define OFFCHARGE_VOLTS_MIN  20.0f

void uabChargeCheck(void)
{
    static float    charge_volts;
    static float    bat_volts;
    static uint32_t charge_duration = 0;
    static uint8_t  first_pass = TRUE;
    static uint8_t  charge1 = TRUE;
    static uint8_t  doWait = FALSE;
    static uint32_t waitTime;
    static uint8_t  firstTime = TRUE;
    static uint8_t  offchargeReported = FALSE;

    uint8_t eData[1];

    /* Initialize 2 Second Charge Cycle/Window */
    if (first_pass)
    {
        first_pass = FALSE;
        waitTime = 0;
        return;
    }

    /* 
    ** Check if 2 Second window has completed
    ** and reset flags for next 2 Second cycle.
    */
    if (++waitTime > (UAB_CHARGE_PERIOD + 2))
    {
        waitTime = 0;
        UabChargeTimer = 0;
        doWait = FALSE;
        return;
    }

    /* Just wait for 2 Second cycle to complete */
    if (doWait)
       return;

    /*
    ** At the start of 2 Second cycle, read the battery and charger voltages.
    ** NOTE!
    **   The Charger must be enabled to properly read the charger voltage.
    **   and a few usec delay is required after enabling the charger
    **   before the charge voltage should be read.
    */
    if (++UabChargeTimer == 1)
    {
        bat_volts = uabAdcVoltsRead(ADC_BATTERY_SEL);
        uabChargerEnable();
        delayUsec(20);
        charge_volts = uabCompVoltsRead(CMP_CHARGE_VOLTS);
        SaveBatteryVolts = bat_volts;
        SaveChargeVolts = charge_volts;
    }

    if (UabChargeTimer >= 1)
    {
        /* Check that the charger voltage is >20.0 Volts */
        if (charge_volts < OFFCHARGE_VOLTS_MIN)
        {
            if ((UabOffCharge == FALSE) || (firstTime))
            {
                firstTime = FALSE;
                UabOffCharge = TRUE;
                if (!UabMajorFaultInProgress)
                {
                    eData[0] = 0;    /* Not Charging */
                    uabEvent(UAB_CHARGER_STATE_CHANGE_EVENT,
                             &eData, sizeof(eData));
                }
            }

            /* 
            ** If charger voltage is <20.0 Volts,
            ** then an Off-Charge fault is set if the carriage is stopped.
            ** The charger is disabled and the wait for next 2 Sec cycle
            ** flag is set
            */
            UabOffCharge = TRUE;
            if (!UabMajorFaultInProgress)
            {
                if ( UabMotionState != stopped )
                {
                    uabFaultClr(OFFCHARGE_FAULT,
                                OFFCHARGE_FAULT_NUM);
                    UabOffchargeFlag = FALSE;
                    offchargeReported = FALSE;
                }
                else
                {
                    UabFaults |= OFFCHARGE_FAULT;
                    if ((UabOffchargeFlag)&& (!offchargeReported))
                    {
                        offchargeReported = TRUE;
                        UabFaults &= ~(OFFCHARGE_FAULT);
                        uabFaultSet(OFFCHARGE_FAULT,
                                    OFFCHARGE_FAULT_NUM);
                        UabOffchargeFlag = FALSE;
                    }
                }
            }
            UabChargeDuration = 0;
            UabChargePercent = 0;
            uabChargerDisable();
            UabChargeInProgress = FALSE;
            doWait = TRUE;
            return;
        }

        /*
        ** If charger voltage is >20.0 Volts,
        ** then an the carriage is in an On-Charge position and
        ** the Off-charge fault is cleared.
        ** The charger is still enabled at this point.
        */
        if ( (UabOffCharge) || (firstTime) )
        {
            firstTime = FALSE;
            UabOffCharge = FALSE;
            if (!UabMajorFaultInProgress)
            {
                eData[0] = 1;   /* Charging */;
                uabEvent(UAB_CHARGER_STATE_CHANGE_EVENT,
                         &eData, sizeof(eData));
            }
        }
        UabOffCharge = FALSE;
        if (!UabMajorFaultInProgress)
        {
            uabFaultClr(OFFCHARGE_FAULT,
                        OFFCHARGE_FAULT_NUM);
        }
        UabOffchargeFlag = FALSE;
        offchargeReported = FALSE;

        /* Is a charge cycle currently in progress */
        if (UabChargeInProgress)
        {
            /*
            ** If charger voltage is < (Battery Voltage - 2.0 Volts),
            ** then charging is disabled and the wait for next 2 Sec cycle
            ** flag is set
            */
            if (charge_volts < (bat_volts - 2.0f))
            {
                if (charge1)
                {
                    charge1 = FALSE;
                    if (!UabMajorFaultInProgress)
                    {
                        eData[0] = 2;   /* Not Charging: Charge V < Batt V - 2 */
                        uabEvent(UAB_CHARGER_STATE_CHANGE_EVENT,
                                 &eData, sizeof(eData));
                    }
                }
                UabOffCharge = TRUE;
                UabChargeDuration = 0;
                UabChargePercent  = 0;
                uabChargerDisable();
                UabChargeInProgress = FALSE;
                doWait = TRUE;
                return;
            }
            else
            {
                charge1 = TRUE;
            }

            /*
            ** If the carriage is in motion,
            ** then charging must be disabled and the wait for next 2 Sec
            ** cycle flag is set
            */
            if (!UabMajorFaultInProgress)
            {
                if (UabMotionState != stopped)
                {
                    UabChargeDuration = 0;
                    UabChargePercent  = 0;
                    uabChargerDisable();
                    UabChargeInProgress = FALSE;
                    doWait = TRUE;
                    return;
                }
            }

            /*
            ** If we have met the charge duration time (95%, 85%, 25%, 5%) of
            ** the current 2 Second cycle, then disable charging for the
            ** remainder of the 2 Second cycle.
            */
            if (charge_duration++ > UabChargeDuration)
            {
                uabChargerDisable();
            }

            /*
            ** If we have completed the 2 Sec cycle, 
            ** make sure charging is disabled and 
            ** charge in progress flag is set to FALSE to 
            ** prepare for the start of next 2 Sec cycle
            */
            if (++UabChargePeriod >= UAB_CHARGE_PERIOD)
            {
                uabChargerDisable();
                UabChargeInProgress = FALSE;
                doWait = TRUE;
                return;
            }
        }
        else
        {
            /* 
            ** No charge is currently in progress but the charger is 
            ** enabled so determine new value for charge duration and
            ** start a new 2 Sec Cycle.
            ** If Battery Voltage is >28.0 Volts, then no charging is
            ** required and the charger is disabled until Battery Voltage
            ** falls below 28 Volts, then charging resumes at next 2 Sec
            ** cycle. 
            */
            UabChargePeriod  = 0;
            charge_duration  = 0;
            UabChargeInProgress = TRUE;
            if (bat_volts <= 24.0)
            {
                /* 1.9 Sec Charge Duration (95%) */
                UabChargeDuration =
                            (UAB_CHARGE_PERIOD * CHARGE_DURATION_95)/100;
                UabChargePercent = CHARGE_DURATION_95;
            }
            else
            {
                if ( (bat_volts > 24.0) && (bat_volts < 27.4) )
                {
                    /* 1.7 Sec Charge Duration (85%) */
                    UabChargeDuration =
                            (UAB_CHARGE_PERIOD * CHARGE_DURATION_85)/100;
                    UabChargePercent = CHARGE_DURATION_85;
                }
                else
                {
                    if ( (bat_volts >= 27.4) && (bat_volts < 27.7) )
                    {
                        /* 500 msec Charge Duration (25%) */
                        UabChargeDuration =
                              (UAB_CHARGE_PERIOD * CHARGE_DURATION_25)/100;
                        UabChargePercent = CHARGE_DURATION_25;
                    }
                    else
                    {
                        if ( (bat_volts >= 27.7) && (bat_volts < 28.0) )
                        {
                            /* 100 msec Charge Duration (5%) */
                            UabChargeDuration =
                                 (UAB_CHARGE_PERIOD * CHARGE_DURATION_5)/100;
                            UabChargePercent = CHARGE_DURATION_5;
                        }
                        else
                        { 
                            /* Charging not required Charge Duration 0 */
                            UabChargeDuration = 0;
                            UabChargePercent  = 0;
                            UabChargeInProgress = FALSE;
                            uabChargerDisable();
                            doWait = TRUE;
                            return;
                        }
                    }
                }
            }
        }
    }
}

/*
******************************************************************************
** Function:     uabVoltsCurrSenseDisplay
**
** Description:  This function sends the Charger Votlage, Battery Voltage,
**               Current Sense (Amps) via a message to the DiagGUI.
**
**               During normal opearation, the charger voltage and battery 
**               voltage are the values read at the start of the last 2 Sec
**               charge cycle.  
**
**               For production test, the test program MUST enable the
**               charger, before requesting a reading of the charger voltage
**               and battery voltage.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabVoltsCurrSenseDisplay(void)
{
    float curr_sense_amps;
    float bat_volts;
    float charge_volts;
    uint32_t curru;
    uint32_t bvu;
    uint32_t cvu;

    uint8_t eData[15];

    charge_volts = SaveChargeVolts;
    curr_sense_amps = uabAdcAmpsRead();
    bat_volts = uabAdcVoltsRead(ADC_BATTERY_SEL);

    cvu = *((uint32_t *)(&charge_volts));
    eData[0] = (uint8_t)((cvu >> 24) & 0xFF);
    eData[1] = (uint8_t)((cvu >> 16) & 0xFF);
    eData[2] = (uint8_t)((cvu >>  8) & 0xFF);
    eData[3] = (uint8_t)(cvu & 0xFF);
    bvu = *((uint32_t *)(&bat_volts));
    eData[4] = (uint8_t)((bvu >> 24) & 0xFF);
    eData[5] = (uint8_t)((bvu >> 16) & 0xFF);
    eData[6] = (uint8_t)((bvu >>  8) & 0xFF);
    eData[7] = (uint8_t)(bvu & 0xFF);
    curru = *((uint32_t *)(&curr_sense_amps));
    eData[8]  = (uint8_t)((curru >> 24) & 0xFF);
    eData[9]  = (uint8_t)((curru >> 16) & 0xFF);
    eData[10] = (uint8_t)((curru >>  8) & 0xFF);
    eData[11] = (uint8_t)(curru & 0xFF);
    
    /* Send along Motion State, PWM %, and Charge % as part of 2 Sec update */
    eData[12] = (uint8_t)(UabMotionState);
    if (UabPwm > 0)
      eData[13] = (uint8_t)(UabPwm);
    else
      eData[13] = 0;
    eData[14] = UabChargePercent;

    if (!UabMajorFaultInProgress)
    {
        uabEvent(UAB_VOLTS_CURRENT_UPDATE_EVENT,
                 &eData, sizeof(eData));
    }
    else
    {
        uabMajorFaultVoltsDisplay( eData );
    }
}

/*
******************************************************************************
** Function:     uabMotorVoltsDisplay
**
** Description:  This function displays the current Motor A, Motor B, and
**               12V Supply voltage readings via the UART0 Debug Comm port.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabMotorVoltsDisplay(void)
{
#if 0
    float motor_a_volts;
    float motor_b_volts;
    float sense12_volts;

    motor_a_volts = uabCompVoltsRead(CMP_MOTOR_A_VOLTS);
    motor_b_volts = uabCompVoltsRead(CMP_MOTOR_B_VOLTS);
    sense12_volts = uabCompVoltsRead(CMP_SENSE12_VOLTS);
#endif
}

/*
******************************************************************************
** Function:     uabInstUpDownSwitchStateGet   (Pinnacle/Sierra Only)
**
** Description:  This function reads the state UP/DOWN/OFF of the
**               Installation UP/DOWN Switch.
**
** Input  Parms: pio0 (INST_DOWN_SWITCH bit)
**               pio2 (INST_UP_SWITCH bit)
**
** Output Parms: None
**
** Return Value: Switch State:  switch_up_left
**                              switch_down_right
**                              switch_off
** 
******************************************************************************
*/
uabUpDownLeftRightSwitchState_t uabInstUpDownSwitchStateGet(uint32_t pio0,
                                                            uint32_t pio2)
{
    uint32_t up;
    uint32_t down;

    up   = (pio2 & INST_UP_SWITCH);
    down = (pio0 & INST_DOWN_SWITCH);

    UabUpDownReleased = FALSE;
    if ( (up == 0) && (down) )
    {
        UabInstUpDownState = switch_up_left;
    	return(switch_up_left);
    }

    if ( (up) && (down == 0) )
    {
        UabInstUpDownState = switch_down_right;
        return(switch_down_right);
    }
    UabUpDownReleased = TRUE;

    UabInstUpDownState = switch_off;
    return(switch_off);
}

/*
******************************************************************************
** Function:     uabUpDownLeftRightSwitchStateGet
**
** Description:  This function reads the state 
**                     For Pinnacle/Sierra:   UP/DOWN/OFF
**                     For Tama:       LEFT/RIGHT/OFF
**               of the armrest motion control UP-LEFT/DOWN-RIGHT switch or 
**               the IR Remote Transmitter UP-LEFT/DOWN-RIGHT button.
**
**               The pressing of the armrest motion control switch
**               UP-LEFT/DOWN-RIGHT takes precedence over the IR Remote
**               Transmitter button if they are being pressed at same time.
**
**               If the user changes direction by pressing the armrest switch
**               or the IR button, a mandatory 1 Sec delay in movement is
**               imposed unless 3 Secs have elapsed.
**
**               If the user has just released the armrest UP-LEFT/DOWN-RIGHT
**               switch and then immediately presses the IR Remote transmitter 
**               UP-LEFT/DOWN-RIGHT button, there is a mandatory 15 Sec delay
**               before movement is allowed.
**
** Input  Parms: pio0  Pinnacle (UP_SWITCH   bit
**                               DOWN_SWITCH bit)
**               pio0  Tama     (LEFT_SWITCH  bit
**                               RIGHT_SWITCH bit)
**
** Output Parms: None
**
** Global Vars:  UabUpDownReleased  Flag indicating UP-LEFT/DOWN-RIGHT switch
**                                   has been fully released and now in 
**                                   switch_off state.
**               UabIrState         Current state of IR Remote Transmitter
**                                   UP-LEFT/DOWN-RIGHT pushbutton.
**
** Return Value: Switch State:  switch_up_left
**                              switch_down_right
**                              switch_off
** 
******************************************************************************
*/
#define STOP_TIME_MAX   300   /*  300 x 10ms =  3 Sec */
#define UD_LR_DELAY     100   /*  100 x 10ms =  1 Sec */
#define IR_UD_LR_DELAY 1000   /* 1000 x 10-20ms = ~15 Sec */

uabUpDownLeftRightSwitchState_t uabUpDownLeftRightSwitchStateGet(uint32_t pio0)
{
    uint32_t up_left;
    uint32_t down_right;
    static uint16_t stop_time        = 0;
    static uint16_t ir_delay         = 0;
    static uint16_t up_left_delay    = 0;
    static uint16_t down_right_delay = 0;

    /* 
    ** Read the UP-LEFT/DOWN-RIGHT switches.
    */
    up_left    = pio0 & UP_SWITCH;
    down_right = pio0 & DOWN_SWITCH;

    UabUpDownReleased = FALSE;
    if ( (down_right == 0) && (up_left) )
    {
        stop_time     = STOP_TIME_MAX;
        up_left_delay = UD_LR_DELAY;
        ir_delay      = IR_UD_LR_DELAY;
        if (down_right_delay > 0)
        {
            down_right_delay--;
            UabUpDownState   = switch_off;
            UabIrUpDownState = switch_off;
            return(switch_off);
        }
        else
        {
            UabUpDownState   = switch_down_right;
            UabIrUpDownState = switch_off;
            return(switch_down_right);
        }
    }

    if ( (down_right) && (up_left == 0) )
    {
        stop_time        = STOP_TIME_MAX;
        down_right_delay = UD_LR_DELAY;
        ir_delay         = IR_UD_LR_DELAY;
        if (up_left_delay > 0)
        {
            up_left_delay--;
            UabUpDownState   = switch_off;
            UabIrUpDownState = switch_off;
            return(switch_off);
        }
        else
        {
            UabUpDownState   = switch_up_left;
            UabIrUpDownState = switch_off;
            return(switch_up_left);
        }
    }

    if ( (down_right == 0) && (up_left == 0) )
    {
        return(switch_invalid);
    }

    if (down_right && up_left)
    {
        if (ir_delay > 0)
            ir_delay--;
        else
        {
            if (UabIrState == irUpLeft)
            {
                stop_time        = STOP_TIME_MAX;
                down_right_delay = UD_LR_DELAY/2;
                if (up_left_delay > 0)
                 {
                     up_left_delay--;
                     UabUpDownState   = switch_off;
                     UabIrUpDownState = switch_off;
                     return(switch_off);
                 }
                 else
                 {
                	 UabUpDownState   = switch_off;
                	 UabIrUpDownState = switch_up_left;
                     return(switch_up_left);
                 }
            }

            if (UabIrState == irDownRight)
            {
                stop_time     = STOP_TIME_MAX;
                up_left_delay = UD_LR_DELAY/2;
                if (down_right_delay > 0)
                {
                    down_right_delay--;
                    UabUpDownState   = switch_off;
                    UabIrUpDownState = switch_off;
                    return(switch_off);
                }
                else
                {
                    UabUpDownState   = switch_off;
                    UabIrUpDownState = switch_down_right;
                    return(switch_down_right);
                }
            }
        }
    }
    if (stop_time > 0)
    {
        stop_time--;
    }
    else
    {
        down_right_delay = 0;
        up_left_delay    = 0;
    }
    UabUpDownReleased = TRUE;

    UabUpDownState   = switch_off;
    UabIrUpDownState = switch_off;
    return(switch_off);
}

/*
******************************************************************************
** Function:     uabBrakeSet     (Pinnacle/Sierra Only)
**
** Description:  This function engages/releases the brake on the motor. 
**
** Input  Parms: uint32_t on   0  Release Brake
**                             1  Engage Brake
**
** Output Parms: None
**
** Global Vars:  UabBrakeState   (brake_off, brake_on) for SW tracking.
**
** Return Value: None
** 
******************************************************************************
*/
void uabBrakeOff(void)
{
    LPC_GPIO2->OUT &= ~(BRAKE_BIT);
    if (UabBrakeState == brake_on)
    {
        UabBrakeState = brake_off;
        uabEvent(UAB_BRAKE_CHANGE_EVENT,
                  &UabBrakeState, sizeof(UabBrakeState));
    }
    UabBrakeState = brake_off;
}

void uabBrakeOn(void)
{
    LPC_GPIO2->OUT |= (BRAKE_BIT);
    if (UabBrakeState == brake_off)
    {
        UabBrakeState = brake_on;
        uabEvent(UAB_BRAKE_CHANGE_EVENT,
                  &UabBrakeState, sizeof(UabBrakeState));
    }
    UabBrakeState = brake_on;
}

/*
******************************************************************************
** Function:     uabMotorOff
**
** Description:  This function turns the motor off by turning off the
**               FET control signals (AHI,BHI,ALI,BLI).
**               PWM output is turned off.
**               For Pinnacle, the brake is also engaged.
**
** Input  Parms: None 
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabMotorOff(void)
{
    /* On Pinnacle/Sierra, engage the Brake */
    if (UabBrakeState == brake_off)
    {
        uabBrakeOn();  /* Engage Brake */
        delayMs(200,SYS_TICK_BASED, TRUE);
    }

    LPC_CT32B0->TCR = 0x0;          /* Disable timer */
    /* Turn off PWM */
    LPC_IOCON->PIO2_4 &= ~0x1F;     /* Set ALI as normal PIO Output */
    LPC_IOCON->PIO2_6 &= ~0x1F;     /* Set BLI as normal PIO Output */

    /* Turn Off all FETs */
    LPC_GPIO2->OUT |= (ALI_BIT | AHI_BIT | BLI_BIT | BHI_BIT);

    UabMotorIsOff = TRUE;
    UabPwm = 0;
}

/*
******************************************************************************
** Function:     uabMotorOn
**
** Description:  This function turns the motor on by setting the needed
**               FET control signals (AHI,BHI,ALI,BLI).
**               PWM output (ALI or BLI) is enabled at the rate specified 
**               in UabPwm global variable.
**               For Pinnacle, the brake is also released.
**
**               UAB_DOWN_DYN_BRAKE (Dynamic Braking) case is currently
**               not used to control the motor.
**
** Input  Parms: uint32_t direction   UAB_DOWN_RIGHT
**                                    UAB_UP_LEFT
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabMotorOn(uint32_t direction)
{
    switch (direction)
    {
        case UAB_UP_LEFT:
            LPC_GPIO2->OUT |= (AHI_BIT | BLI_BIT); /* Make sure AHI and BLI are High */
            LPC_IOCON->PIO2_6 |= 0x3;       /* Change BLI as CT32B0_MAT2 Output */

            /* 24,000 KHz/4Khz = 6000 x Dur/100 = Dur * 60 */
            LPC_CT32B0->MR[2] = (UabPwm * 60);      /* BLI PWM */

            LPC_CT32B0->TCR = 0x2;                   /* Reset timer */

            LPC_GPIO2->OUT &= ~(AHI_BIT | BHI_BIT);  /* Set AHI and BHI Low */
            LPC_GPIO2->OUT |= (ALI_BIT);             /* Set ALI High */

            LPC_CT32B0->TCR = 0x1;                   /* Enable timer */
            break;

        case UAB_DOWN_RIGHT:
            LPC_GPIO2->OUT |= (BHI_BIT | ALI_BIT); /* Make sure BHI and ALI are High */
            LPC_IOCON->PIO2_4 |= 0x3;       /* Change ALI as CT32B0_MAT0 Output */

            /* 24,000 KHz/4Khz = 6000 x Dur/100 = Dur * 60 */
            LPC_CT32B0->MR[0] = (UabPwm * 60);      /* ALI PWM */

            LPC_CT32B0->TCR = 0x2;                   /* Reset timer */

            LPC_GPIO2->OUT &= ~(AHI_BIT | BHI_BIT);  /* Set AHI and BHI Low */
            LPC_GPIO2->OUT |= (BLI_BIT);             /* Set BLI High */

            LPC_CT32B0->TCR = 0x1;                  /* Enable timer */
            break;

        case UAB_DOWN_DYN_BRAKE:
            LPC_GPIO2->OUT |= (AHI_BIT | BLI_BIT); /* Make sure AHI and BLI are High */
            LPC_IOCON->PIO2_6 |= 0x3;       /* Change BLI as CT32B0_MAT2 Output */

            /* 24,000 KHz/4Khz = 6000 x Dur/100 = Dur * 60 */
            LPC_CT32B0->MR[2] = (UabPwm * 60);      /* BLI PWM */
            LPC_CT32B0->TCR = 0x2;                   /* Reset timer */

            LPC_GPIO2->OUT |= (AHI_BIT | BHI_BIT);   /* Set AHI and BHI High */
            LPC_GPIO2->OUT &= ~(ALI_BIT);            /* Set ALI Low */

            LPC_CT32B0->TCR = 0x1;                   /* Enable timer */
            break;
    }

    if (UabBrakeState == brake_on)
    {
        delayMs(50, SYS_TICK_BASED, TRUE);
        uabBrakeOff();   /* Release Brake */
    }

    UabMotorIsOff = FALSE;
}


/*
******************************************************************************
** Function:     uabPwmInit
**
** Description:  This function initializes the LPC122x 32-bit Counter 0
**               to provide the Pulse Width Modulation (PWM) needed for
**               controlling the speed of the motor on the stair lift.
**               The PWM period/cycle is 4KHz.
**               The PWM duration controls the speed of the motor in either
**               the UP-LEFT or DOWN-RIGHT directions.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabPwmInit(void)
{
    LPC_IOCON->PIO2_4 &= ~0x1F;         /* Set ALI as normal PIO Output */
    LPC_IOCON->PIO2_6 &= ~0x1F;         /* Set BLI as normal PIO Output */

    LPC_CT32B0->TCR   = 0x02;           /* reset & Disable timer */
    LPC_CT32B0->PR    = 0x00;           /* set pre-scaler to zero */
    LPC_CT32B0->IR    = 0xFF;           /* reset all interrupts */
    LPC_CT32B0->MR[0] = 0;              /* PWM Duration % for DOWN (0-95) */
    LPC_CT32B0->MR[1] = 0;              /* HW Trigger to ADC */
    LPC_CT32B0->MR[2] = 0;              /* PWM Duration % for UP   (0-95) */
    LPC_CT32B0->MR[3] = UAB_PWM_PERIOD; /* Period = 24M/4K = 6000 - 1 = 5999 */
    LPC_CT32B0->MCR   = 0x00000400;     /* Reset timer on match MR3 */

    LPC_CT32B0->EMR  = 0x000003F7;      /* EM 0-2, EMC 0-2 Toggle */
    LPC_CT32B0->PWMC = 0x0000000F;      /* PWM Enabled on MR0 - MR3 */
}

/*
******************************************************************************
** Function:     uabPwmChange
**
** Description:  This function changes the PWM duration as requested.
**               This is used to speed up or slow down the motor.
**
** Input  Parms: uint32_t direction   UAB_DOWN_RIGHT
**                                    UAB_UP_LEFT
**               uint32_t duration  PWM Duration % (0 - 95)
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void uabPwmChange(uint32_t direction, uint32_t duration)
{
    if (direction == UAB_DOWN_RIGHT)
    {
        LPC_CT32B0->MR[0] = (duration * 60);  /* ALI */
    }
    else
    {
        LPC_CT32B0->MR[2] = (duration * 60);  /* BLI */
    }
}

/*
******************************************************************************
** Function:     uabPioGet
**
** Description:  This function reads and debounces the three LPC122x
**               PIO ports (PIO0-PIO2) during each 10ms realtime loop.
**               All other functions use the PIO0-PIO2 values read by
**               this function.
**
** Input  Parms:
**
** Output Parms: uint32_t* pio0_ptr   Callers uint32 for PIO0 contents
**               uint32_t* pio1_ptr   Callers uint32 for PIO1 contents
**               uint32_t* pio2_ptr   Callers uint32 for PIO2 contents
**
** Return Value: OK     if no switch bounce between 10ms reads
**               NOT_OK if switch bounce between 10ms reads
**
******************************************************************************
*/
int uabPioGet(uint32_t *pio0_ptr,
              uint32_t *pio1_ptr,
              uint32_t *pio2_ptr)
{
    static uint32_t prev_pio0 = PIO0_SWITCH_DEFAULT;
    static uint32_t prev_pio2 = PIO2_SWITCH_DEFAULT;

    *pio0_ptr = (uint32_t)(LPC_GPIO0->PIN);
    *pio1_ptr = (uint32_t)(LPC_GPIO1->PIN);
    *pio2_ptr = (uint32_t)(LPC_GPIO2->PIN);

    /*
    ** Debounce any switch change
    **   PIO0: 0,3-9,14,15
    **   PIO2: 1,2,14,15
    */
    if ( ((*pio0_ptr & PIO0_SWITCH_ALL) != prev_pio0) ||
         ((*pio2_ptr & PIO2_SWITCH_ALL) != prev_pio2) )
    {
        prev_pio0 = (*pio0_ptr & PIO0_SWITCH_ALL);
        prev_pio2 = (*pio2_ptr & PIO2_SWITCH_ALL);
        return(NOT_OK);
    }
    return(OK);
}

/*
******************************************************************************
** Function:     uabFaultsGet
**
** Description:  This function reads all Safety Switches for faults.
**
** Input  Parms: uint32_t pio0   Current PIO0 port register
**               uint32_t pio2   Current PIO2 port register
**
** Output Parms: None
**
** Return Value: uint32_t  Current Safety Switch faults
**                           NO_FAULTS                0
**                           OB_UP_FAULT          (1<<0)
**                           OB_DOWN_FAULT        (1<<1)
**                           FR_UP_LEFT_FAULT     (1<<2)
**                           FR_DOWN_RIGHT_FAULT  (1<<3)
**                           FR_UNDER_FAULT       (1<<4)
**                           FR_MAJOR_FAULT       (1<<5)  (NOT USED) 
**                           SEAT_FAULT           (1<<6)
**                           KEY_OFF_FAULT        (1<<7)
**
******************************************************************************
*/
uint32_t uabFaultsGet(uint32_t pio0, uint32_t pio2)
{
    uint32_t faults;
    static uint32_t prev_faults = 0;
    uint32_t bit;
    int      i;

    faults = 0;

    if ( ((pio2 & OBSTR_UP_SWITCH) == 0) ||
         ((prev_faults & OB_UP_FAULT) && (!UabUpDownReleased)) )
        faults |= OB_UP_FAULT;

    if ( ((pio2 & OBSTR_DOWN_SWITCH) == 0) ||
         ((prev_faults & OB_DOWN_FAULT) && (!UabUpDownReleased)) )
        faults |= OB_DOWN_FAULT;

    if ( (pio0 & FOOTREST_UNDER_SWITCH) ||
         ((prev_faults & FR_UNDER_FAULT) && (!UabUpDownReleased)) )
        faults |= FR_UNDER_FAULT;

    if (pio0 & KEY_SWITCH)
        faults |= KEY_OFF_FAULT;

    if ( (pio0 & FOOTREST_UP_LEFT_SWITCH) ||
         ((prev_faults & FR_UP_LEFT_FAULT) && (!UabUpDownReleased)) )
        faults |= FR_UP_LEFT_FAULT;

    if ( (pio0 & FOOTREST_DOWN_RIGHT_SWITCH) ||
         ((prev_faults & FR_DOWN_RIGHT_FAULT) && (!UabUpDownReleased)) )
        faults |= FR_DOWN_RIGHT_FAULT;

    if ( (pio0 & SEAT_SWITCH) ||
         ((prev_faults & SEAT_FAULT) && (!UabUpDownReleased)) )
        faults |= SEAT_FAULT;

    /* For Pinnacle/Sierra, if in Installation Mode, disregard indicated faults */
    if (UabInstMode)
    {
        faults &= ~(KEY_OFF_FAULT | SEAT_FAULT |
                    FR_DOWN_RIGHT_FAULT | FR_UP_LEFT_FAULT | FR_UNDER_FAULT);
    }

    /* (Pinnacle/Sierra) if pushing UP switch, disregard indicated faults */
    if (UabUpDownLeftRightSwitchState == switch_up_left)
    {
        faults &= ~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT | FR_UNDER_FAULT);
    }
    else
    {
        /* (Pinnacle/Sierra) if pushing DOWN switch, disregard indicated faults */
        if (UabUpDownLeftRightSwitchState == switch_down_right)
        {
            faults &= ~(FR_UP_LEFT_FAULT | OB_UP_FAULT);
        }
    }

    /* Clear any existing faults that have cleared and Set new faults */
    if (prev_faults != faults)
    {
        for(i=0; i<8; i++)
        {
            bit = (1<<i);
            if ( (prev_faults & bit) != (faults & bit) )
            {
                switch(bit)
                {
                    case OB_UP_FAULT:
                        if (faults & bit)
                          uabFaultSet(OB_UP_FAULT,
                                       OB_UP_FAULT_NUM);
                        else
                          uabFaultClr(OB_UP_FAULT,
                                       OB_UP_FAULT_NUM);
                        break;

                    case OB_DOWN_FAULT:
                        if (faults & bit)
                          uabFaultSet(OB_DOWN_FAULT,
                                       OB_DOWN_FAULT_NUM);
                        else
                          uabFaultClr(OB_DOWN_FAULT,
                                       OB_DOWN_FAULT_NUM);
                        break;

                    case FR_UP_LEFT_FAULT:
                        if (faults & bit)
                          uabFaultSet(FR_UP_LEFT_FAULT,
                                       FR_UP_LEFT_FAULT_NUM);
                        else
                          uabFaultClr(FR_UP_LEFT_FAULT,
                                       FR_UP_LEFT_FAULT_NUM);
                        break;

                    case FR_DOWN_RIGHT_FAULT:
                        if (faults & bit)
                          uabFaultSet(FR_DOWN_RIGHT_FAULT,
                                       FR_DOWN_RIGHT_FAULT_NUM);
                        else
                          uabFaultClr(FR_DOWN_RIGHT_FAULT,
                                       FR_DOWN_RIGHT_FAULT_NUM);
                        break;

                    case FR_UNDER_FAULT:
                        if (faults & bit)
                          uabFaultSet(FR_UNDER_FAULT,
                                       FR_UNDER_FAULT_NUM);
                        else
                          uabFaultClr(FR_UNDER_FAULT,
                                       FR_UNDER_FAULT_NUM);
                        break;

                    case SEAT_FAULT:
                        if (faults & bit)
                          uabFaultSet(SEAT_FAULT,
                                       SEAT_FAULT_NUM);
                        else
                          uabFaultClr(SEAT_FAULT,
                                       SEAT_FAULT_NUM);
                        break;

                    case KEY_OFF_FAULT:
                        if (faults & bit)
                          uabFaultSet(KEY_OFF_FAULT,
                                       KEY_OFF_FAULT_NUM);
                        else
                          uabFaultClr(KEY_OFF_FAULT,
                                       KEY_OFF_FAULT_NUM);
                        break;

                    default:
                        break;
                }
            }
        }
    }

    prev_faults = faults;

    return(faults);
}

/*
******************************************************************************
** Function:     uabCheckStopDownRight
**
** Description:  This function checks if the DOWN-RIGHT STOP switch
**               at the end of track is being detected.
**
**
** Input  Parms: uint32_t pio2   Current PIO2 port register
** Output Parms: None
**
** Return Value: 1  DOWN-RIGHT STOP switch is activated
**               0  DOWN-RIGHT STOP switch is not activated
**
******************************************************************************
*/
int uabCheckStopDownRight(uint32_t pio2)
{
    if ( pio2 & STOP_DOWN_RIGHT_SWITCH )
      return(1);
    else
      return(0);
}

/*
******************************************************************************
** Function:     uabCheckStopUpLeft
**
** Description:  This function checks if the UP-LEFT STOP switch
**               at the end of track is being detected.
**
**
** Input  Parms: uint32_t pio0   Current PIO0 port register
** Output Parms: None
**
** Return Value: 1  UP-LEFT STOP switch is activated
**               0  UP-LEFT STOP switch is not activated
**
******************************************************************************
*/
int uabCheckStopUpLeft(uint32_t pio0)
{
    if ( pio0 & STOP_UP_LEFT_SWITCH )
      return(1);
    else
      return(0);
}

/*
******************************************************************************
** Function:     uabCheckLimit
**
** Description:  This function checks if the FINAL/LIMIT switch
**               is being detected at top or bottom.
**
** Input  Parms: uint32_t pio0   Current PIO0 port register
**
** Output Parms: None
**
** Return Value: 1  FINAL/LIMIT switch is activated
**               0  FINAL/LIMIT switch is not activated
**
******************************************************************************
*/
int uabCheckLimit(uint32_t pio0, uint32_t pio2)
{
    if ( (pio0 & STOP_UP_LEFT_SWITCH) &&
         (pio2 & STOP_DOWN_RIGHT_SWITCH) )
    {
        if (UabInstMode == FALSE)
        {
            /* Major Fault! STOP UP and STOP DOWN switches both Detected */
            uabMajorFaultHandler(UAB_MAJOR_FAULT_CONF_STOP_UP_AND_DOWN_DETECTED);
            /* will not return */
        }
    }

    if ( (pio0 & STOP_LIMIT_SWITCH) &&
         ((pio0 & STOP_UP_LEFT_SWITCH) == 0) &&
         ((pio2 & STOP_DOWN_RIGHT_SWITCH) == 0) )
    {
        if (UabInstMode == FALSE)
        {
            /* Major Fault! STOP UP and STOP DOWN switches both Not Detected */
            uabMajorFaultHandler(UAB_MAJOR_FAULT_CONF_STOP_UP_AND_DOWN_NOT_DETECTED);
            /* will not return */
        }
    }

    if (pio0 & STOP_LIMIT_SWITCH)
      return(1);
        
    return(0);
}

/*
******************************************************************************
** Function:     uabCheckTopBottom
**
** Description:  This function checks if the FINAL/LIMIT switch
**               at bottom or RIGHT end of track is being detected.
**
** Input  Parms: uint32_t pio0   Current PIO0 port register
**               uint32_t pio2   Current PIO2 port register
**
** Output Parms: None
**
** Return Value: 1  Bottom-RIGHT FINAL/LIMIT switch is activated
**               0  Bottom-RIGHT FINAL/LIMIT switch is not activated
**
******************************************************************************
*/
int uabCheckTopBottom(uint32_t pio0, uint32_t pio2)
{
    if ( uabCheckLimit(pio0, pio2) == 0 )
      return(UAB_MIDTRACK);

    if (pio2 & STOP_DOWN_RIGHT_SWITCH)
      return(UAB_BOTTOM_RIGHT);

    if (pio0 & STOP_UP_LEFT_SWITCH)
      return(UAB_TOP_LEFT);

    return(UAB_MIDTRACK);
}

void uabTopStop(void)
{
    uabMotorOff();
    UabTripDone = TRUE;
    UabMotionState = stopped;
}

void uabBottomStop(void)
{
    uabMotorOff();
    UabTripDone = TRUE;
    UabMotionState = stopped;
}

/*
******************************************************************************
** Function:     uabMotionSet
**
** Description:  This function implements the Motion State Machine.
**               The Motion State Machine (see UAB Software Design Spec -
**               DOC0003413 for Motion State Diagram and Description)
**               consists of 7 states (as defined in uabMotionState_t):
**
**                         stopped
**                         soft_start_down_right
**                         cruise_down_right
**                         soft_stop_down_right
**                         soft_start_up_left
**                         cruise_up_left
**                         soft_stop_up_left
**
**               up/down    apply to Pinnacle/Sierra
**               left/right apply to Tama
**
**               The general flow of the motion state machine is as follows:
**
**                    1.  Check to see if any conflicting Safety Switches
**                        are being reported by the alarm/fault handler,
**                        if so generate Major Fault which requires system
**                        reset/reboot to recover.
**
**                    2.  Read the current armrest motion control switch
**                        or the IR Remote control button to determine if
**                        UP-LEFT, DOWN-RIGHT, or OFF motion is being
**                        requested by the user.
**
**                    3.  Read the current Safety Switch faults.
**
**                    4.  Enter the motion state machine with all the
**                        information gathered in #2 and #3 and at start
**                        of each state, check if UP-LEFT/DOWN-RIGHT STOP
**                        switch and (for Pinnacle) UP/DOWN FINAL/LIMIT
**                        switch are being detected.
**
**                    5.  Execute the motion state machine based on current
**                        state as stored in UabMotionState global variable.
**                        For each state other than "stopped", the PWM rate
**                        of the motor is varied to implement "soft start",
**                        "soft-stop", and "cruise" motor control of the
**                        carriage UP-LEFT/DOWN-RIGHT on the track.
**
** Input  Parms: uint32_t pio0   Current PIO0 port register
**               uint32_t pio2   Current PIO2 port register
**
** Output Parms: None
**
** Global Vars:  UabMajorFault
**               UabInstMode
**               UabSystem
**               UabDebugMode
**               UabUpDownLeftRightSwitchState
**               UabMotionState
**               UabTripDone        (flag that indicates end of track)
**
**               PWM Rate Global Variables
**               UabPwm
**               UabBegPwm
**               UabEndPwm
**               UabPwmStep
**               UabRampInterval
**
** Return Value: None
** 
******************************************************************************
*/

/* Soft Start UP PWM values */
#define SOFT_START_UP_BEG_PWM                40
#define SOFT_START_UP_END_PWM                95
#define INST_SOFT_START_UP_END_PWM           50
#define SOFT_START_UP_PWM_STEP                5
#define SOFT_START_UP_PWM_INTERVAL           12

/*  Soft Start DOWN PWM values */
#define SOFT_START_DOWN_BEG_PWM              15
#define PINNACLE_SOFT_START_DOWN_END_PWM     60
#define INST_SOFT_START_DOWN_END_PWM         50
#define SOFT_START_DOWN_PWM_STEP              5
#define SOFT_START_DOWN_PWM_INTERVAL         12

/*  Cruise UP and Soft Stop UP PWM values */
#define PINNACLE_SLOW_CRUISE_UP_PWM          40
#define SOFT_STOP_UP_PWM_STEP                10
#define SOFT_STOP_UP_PWM_INTERVAL             1
#define SOFT_STOP_UP_PWM_INTERVAL_A           4

/* Cruise DOWN and Soft Stop DOWN PWM values */
#define PINNACLE_SLOW_CRUISE_DOWN_PWM        15
#define SOFT_STOP_DOWN_PWM_STEP               5
#define SOFT_STOP_DOWN_PWM_INTERVAL           4
#define SOFT_STOP_DOWN_PWM_INTERVAL_A         4
#define SOFT_STOP_DOWN_PWM_INTERVAL_B         2

void uabMotionSet(uint32_t pio0, uint32_t pio2)
{
    uint32_t faults;
    int top_bottom;
    static uabUpDownLeftRightSwitchState_t prevUabIrUpDownState = switch_off;
    uint8_t eData[2];

    /*
    ** Check for alarm/fault handler has set UabMajorFault due
    ** to conflicting safety switches and we are not in Installation Mode,
    ** post a major alarm which requires system reset/reboot to recover.
    */
    if ((UabMajorFault) && (UabInstMode == FALSE))
    {
        uabMajorFaultHandler(UabMajorFaultCause);
        /* Will not return */
    }

    /*
    ** Get the current state of the armrest motion control switch or
    ** the IR Remote Control pushbutton state.
    ** If in Installation mode, get the UP/DOWN state of the
    ** Installation switch.
    */

    if (UabInstMode)
    {
        UabUpDownLeftRightSwitchState =
            uabInstUpDownSwitchStateGet(pio0, pio2);
    }
    else
    {
        UabUpDownLeftRightSwitchState =
            uabUpDownLeftRightSwitchStateGet(pio0);

        /* Send IR UP/DOWN DiagGUI msg */
        if ( (UabUpDownState == switch_off) &&
             (UabIrUpDownState != prevUabIrUpDownState) )
        {
            eData[0] = IR_UP_SWITCH_NUM;
            eData[1] = 0;
            if (UabIrUpDownState == switch_up_left)
                eData[1] = 1;
            uabEvent(UAB_SWITCH_CHANGE_EVENT,
                     &eData, sizeof(eData) );

            eData[0] = IR_DOWN_SWITCH_NUM;
            eData[1] = 0;
            if (UabIrUpDownState == switch_down_right)
                eData[1] = 1;
            uabEvent(UAB_SWITCH_CHANGE_EVENT,
                     &eData, sizeof(eData) );

            prevUabIrUpDownState = UabIrUpDownState;
        }
    }

    /* If Invalid Switch is returned, treat as a Major Fault */
    if ( (UabUpDownLeftRightSwitchState == switch_invalid) &&
         (UabInstMode == FALSE) )
    {
        /*  Do nothing if both switches pressed at same time */
        return;
    }

    /* Get current Safety Switch faults */
    faults = uabFaultsGet(pio0, pio2);

    /* Execute Motion State Machine */
    switch(UabMotionState)
    {
        case stopped:  /* Motor is Off and Brake is set on entry */

            /*
            ** Check if at Top/Left end of track (see comment in function).
            ** If there are no other safety faults other than
            ** Footrest UP-LEFT or Obstruction UP, then down movement
            ** is allowed. Check if user is pressing DOWN switch on armrest
            ** or on IR Remote Control. If so, adjust for "soft start"
            ** PWM rate, turn on motor, release brake, and transition
            ** state machine to soft_start_down_right state.
            */
            top_bottom = uabCheckTopBottom(pio0, pio2);
            if ( top_bottom == UAB_TOP_LEFT )
            {
                if ( (faults & (~(FR_UP_LEFT_FAULT | OB_UP_FAULT))) == 0 )
                {
                    if (UabUpDownLeftRightSwitchState == switch_down_right)
                    {
                        UabBegPwm = SOFT_START_DOWN_BEG_PWM;
                        if (UabInstMode)
                          UabEndPwm = INST_SOFT_START_DOWN_END_PWM;
                        else
                          UabEndPwm = PINNACLE_SOFT_START_DOWN_END_PWM;
                        UabPwmStep      = SOFT_START_DOWN_PWM_STEP;
                        UabRampInterval = SOFT_START_DOWN_PWM_INTERVAL;
                        UabPwm = UabBegPwm;
                        uabMotorOn(UAB_DOWN_RIGHT);
                        UabMotionState = soft_start_down_right;
                    }
                }
            }
            else
            {
                /*
                ** Check if at Bottom/Right end of track
                ** (see comment in function).
                ** If there are no other safety faults other than
                ** Footrest DOWN-RIGHT or Obstruction DOWN, then
                ** up movement is allowed. Check if user is pressing
                ** UP switch on armrest or on IR Remote Control.
                ** If so, adjust for "soft start" PWM rate,
                ** turn on motor, release brake, and transition
                ** state machine to soft_start_up_left state.
                */
            	if ( top_bottom == UAB_BOTTOM_RIGHT )
                {
                    if ( (faults & (~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT))) == 0 )
                    {
                        if (UabUpDownLeftRightSwitchState == switch_up_left)
                        {
                            UabBegPwm       = SOFT_START_UP_BEG_PWM;
                            if (UabInstMode)
                              UabEndPwm     = INST_SOFT_START_UP_END_PWM;
                            else
                              UabEndPwm     = SOFT_START_UP_END_PWM;
                            UabPwmStep      = SOFT_START_UP_PWM_STEP;
                            UabRampInterval = SOFT_START_UP_PWM_INTERVAL;
                            UabPwm = UabBegPwm;
                            uabMotorOn(UAB_UP_LEFT);
                            UabMotionState = soft_start_up_left;
                        }
                    }
                }
                else
                {
                    /*
                    ** In mid-track and user is pressing DOWN-RIGHT switch
                    ** on armrest or on IR Remote Control, and there are no
                    ** other safety faults other than Footrest UP-LEFT or
                    ** Obstruction UP, then down movement is allowed.
                    ** Adjust for "soft start" PWM rate, turn on motor,
                    ** release brake, and transition state machine to
                    ** soft_start_down_right state.
                    */
                    if (UabUpDownLeftRightSwitchState == switch_down_right)
                    {
                        if ( (faults & (~(FR_UP_LEFT_FAULT | OB_UP_FAULT))) == 0 )
                        {
                            UabBegPwm   = SOFT_START_DOWN_BEG_PWM;
                            if (UabInstMode)
                              UabEndPwm = INST_SOFT_START_DOWN_END_PWM;
                            else
                            {
                                UabEndPwm = PINNACLE_SOFT_START_DOWN_END_PWM;
                            }
                            UabPwmStep      = SOFT_START_DOWN_PWM_STEP;
                            UabRampInterval = SOFT_START_DOWN_PWM_INTERVAL;
                            UabPwm = UabBegPwm;
                            uabMotorOn(UAB_DOWN_RIGHT);
                            UabMotionState = soft_start_down_right;
                        }
                    }
                    else
                    {
                        /*
                        ** In mid-track and user is pressing UP-LEFT switch
                        ** on armrest or on IR Remote Control,and there are no
                        ** other safety faults other than Footrest DOWN-RIGHT
                        ** or Obstruction DOWN, then up movement is allowed.
                        ** Adjust for "soft start" PWM rate, turn on motor,
                        ** release brake, and transition state machine to
                        ** soft_start_up_left state.
                        */
                        if (UabUpDownLeftRightSwitchState == switch_up_left)
                        {
                            if ( (faults & (~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT))) == 0 )
                            {
                                UabBegPwm       = SOFT_START_UP_BEG_PWM;
                                if (UabInstMode)
                                   UabEndPwm = INST_SOFT_START_UP_END_PWM;
                                else
                                   UabEndPwm     = SOFT_START_UP_END_PWM;
                                UabPwmStep      = SOFT_START_UP_PWM_STEP;
                                UabRampInterval = SOFT_START_UP_PWM_INTERVAL;
                                UabPwm = UabBegPwm;
                                uabMotorOn(UAB_UP_LEFT);
                                //UabMotionState = cruise_up_left;
                                UabMotionState = soft_start_up_left;
                            }
                            /*
                            ** For any other conditions, the motion
                            ** state machine remains in the "stopped" state.
                            */
                        }
                    }
                }
            }
            break;

        case soft_start_up_left:
            /*
            ** If in mid-track and cruise PWM rate has not been reached,
            ** and user is pressing UP-LEFT switch on armrest or on
            ** IR Remote Control, and there are no other safety faults other
            ** than Footrest DOWN-RIGHT or Obstruction DOWN, then up movement
            ** is allowed.
            */
            if (UabUpDownLeftRightSwitchState == switch_up_left)
            {
                if ( (faults & (~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT))) == 0 )
                {
                    if ( uabCheckTopBottom(pio0, pio2) == UAB_TOP_LEFT )
                    {
                        /*
                        ** If we are at Top/Left end of track, then stop the
                        ** motor, engage the brake, and trasition motion
                        ** state machine to "stopped" state.
                        */
                        uabTopStop();
                    }
                    else
                    {
                        /*
                        ** If not at Top/Left end of track but
                        ** UP-LEFT STOP switch is being detected,
                        ** then adjust PWM rate for "soft-stop" and transition
                        ** state machine to "soft_stop_up_left" state.
                        */
                        if (uabCheckStopUpLeft(pio0))
                        {
                            UabEndPwm   = PINNACLE_SLOW_CRUISE_UP_PWM;
                            UabPwmStep      = SOFT_STOP_UP_PWM_STEP;
                            UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL;
                            UabMotionState = soft_stop_up_left;
                        }
                        else
                        {
                            /*
                            ** In mid-track, continue to increase/adjust
                            ** PWM rate for soft start up/left.
                            ** If cruise PWM rate has been reached,
                            ** transition motion the state machine
                            ** to "cruise_up_left" state.
                            */
                            UabRampInterval--;
                            if (UabRampInterval == 0)
                            {
                                if (UabPwm < UabEndPwm)
                                {
                                    UabPwm += UabPwmStep;
                                    UabRampInterval = SOFT_START_UP_PWM_INTERVAL;
                                    uabPwmChange(UAB_UP_LEFT, UabPwm);
                                }
                                else
                                {
                                    UabMotionState = cruise_up_left;
                                }
                            }
                        }
                    }
                }
                else
                {
                    /*
                    ** There are Safety faults...stop motor, engage brake and
                    ** transition motion state machine to "stopped" state.
                    */
                    uabMotorOff();
                    UabMotionState = stopped;
                }
            }
            else
            {
                /*
                ** User has released armrest UP switch or IR Remote UP button.
                ** Adjust PWM rate for soft stop up/left and transition the
                ** state machine to "soft_stop_up_left" state.
                */
                UabEndPwm       = 10;     /* assumes in mid track */
                UabPwmStep      = SOFT_STOP_UP_PWM_STEP;
                UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL_A;
                UabMotionState = soft_stop_up_left;
            }
            break;

        case cruise_up_left:
            /*
            ** If in mid-track and at cruise PWM rate, and user is pressing
            ** UP-LEFT switch on armrest or on IR Remote Control, and there
            ** are no other safety faults other than Footrest DOWN-RIGHT or
            ** Obstruction DOWN, then up movement continues.
            */
            if ( (faults & (~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT))) == 0 )
            {
                if (UabUpDownLeftRightSwitchState == switch_up_left)
                {
                    /* Same comment as in soft_start_up_left case */
                    if ( uabCheckTopBottom(pio0, pio2) == UAB_TOP_LEFT )
                    {
                        uabTopStop();
                    }
                    else
                    {
                        /* Same comment as in soft_start_up_left case */
                        if (uabCheckStopUpLeft(pio0))
                        {
                            UabEndPwm   = PINNACLE_SLOW_CRUISE_UP_PWM;
                            UabPwmStep      = SOFT_STOP_UP_PWM_STEP;
                            UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL;
                            UabMotionState = soft_stop_up_left;
                        }
                    }
                    /* Continue Up movement at Cruise PWM rate */
                }
                else
                {
                    /*
                    ** User has released armrest UP switch or IR Remote UP
                    ** button. Adjust PWM rate for soft stop up/left and
                    ** transition the state machine to "soft_stop_up_left"
                    ** state.
                    */
                    UabEndPwm       = 10;     /* assumes in mid track */
                    UabPwmStep      = SOFT_STOP_UP_PWM_STEP;
                    UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL_A;
                    UabMotionState = soft_stop_up_left;
                }
            }
            else
            {
                /* 
                ** There are Safety faults...stop motor, engage brake, and
                ** transition motion state machine to "stopped" state.
                */
                uabMotorOff();
                UabMotionState = stopped;
            }
            break;

        case soft_stop_up_left:
            /*
            ** Once in this state, even if user releases armrest UP switch
            ** or IR Remote UP button, movement of the motor will continue
            ** at decreasing soft stop PWM rate to end of track.
            ** If safety faults other than Footrest DOWN-RIGHT or
            ** Obstruction DOWN are being detected, stop the motor and
            ** engage the brake immediately.
            */
            /* Same comment as in soft_start_up_left case */
            if ( uabCheckTopBottom(pio0, pio2) == UAB_TOP_LEFT )
            {
                uabTopStop();
            }
            else
            {
                if  ((faults & (~(FR_DOWN_RIGHT_FAULT | OB_DOWN_FAULT))) == 0)
                {
                    /* Same comment as in soft_start_up_left case */
                    if (uabCheckStopUpLeft(pio0))
                    {
                        if (UabUpDownLeftRightSwitchState != switch_up_left)
                        {
                            uabMotorOff();
                            UabMotionState = stopped;
                            return;
                        }

                        if (UabPwm > UabEndPwm)
                        {
                            UabEndPwm   = PINNACLE_SLOW_CRUISE_UP_PWM;
                        }
                    }

                    /* Adjust/decrease PWM Rate until End PWM Rate */
                    UabRampInterval--;
                    if (UabRampInterval == 0)
                    {
#if 0
                        printf("A: %d  %d  %d\n",
                                 UabPwm, UabEndPwm, UabRampInterval);
#endif
                        if (UabPwm > UabEndPwm)
                        {
                            UabPwm -= UabPwmStep;
                            if (uabCheckStopUpLeft(pio0))
                                UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL;
                            else
                                UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL_A;
                            uabPwmChange(UAB_UP_LEFT, UabPwm);
                        }
                        else
                        {
                            if (uabCheckStopUpLeft(pio0) == 0)
                            {
                                uabMotorOff();
                                UabMotionState = stopped;
                            }
                            else
                            {
                                /*  allow motor to reach FINAL */
                                UabRampInterval = SOFT_STOP_UP_PWM_INTERVAL;
                                UabEndPwm = PINNACLE_SLOW_CRUISE_UP_PWM;
                                UabPwm = UabEndPwm;
                                uabPwmChange(UAB_UP_LEFT, UabPwm);
                            }
                        }
                    }
                }
                else
                {
                    /*
                    ** There are Safety faults...stop motor, engage brake and
                    ** transition motion state machine to "stopped" state.
                    */
                    uabMotorOff();
                    UabMotionState = stopped;
                }
            }
            break;

        /*
        ** !!! NOTE !!!
        **
        ** For all of the DOWN-RIGHT motion states, the same code comments
        ** as for UP-LEFT motion states apply. The only difference is the
        ** direction and the values used for PWM rates.
        ** Code comments are not repeated for DOWN-RIGHT motion states.
        */
        case soft_start_down_right:
            if (UabUpDownLeftRightSwitchState == switch_down_right)
            {
                if ( (faults & (~(FR_UP_LEFT_FAULT | OB_UP_FAULT))) == 0 )
                {
                    if ( uabCheckTopBottom(pio0, pio2) == UAB_BOTTOM_RIGHT )
                    {
                        uabBottomStop();
                    }
                    else
                    {
                        if (uabCheckStopDownRight(pio2))
                        {
                            UabEndPwm     = PINNACLE_SLOW_CRUISE_DOWN_PWM;
                            UabPwmStep      = SOFT_STOP_DOWN_PWM_STEP;
                            UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL;
                            UabMotionState = soft_stop_down_right;
                        }
                        else
                        {
                            UabRampInterval--;
                            if (UabRampInterval <= 0)
                            {
                                if (UabPwm < UabEndPwm)
                                {
                                    UabPwm += UabPwmStep;
                                    UabRampInterval = SOFT_START_DOWN_PWM_INTERVAL;
                                    uabPwmChange(UAB_DOWN_RIGHT, UabPwm);
                                }
                                else
                                {
                                    UabMotionState = cruise_down_right;
                                }
                            }
                        }
                    }
                }
                else
                {
                    uabMotorOff();
                    UabMotionState = stopped;
                }
            }
            else
            {
                UabEndPwm       = 5;     /* assumes in mid track */
                UabPwmStep      = SOFT_STOP_DOWN_PWM_STEP;
                UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL_A;
                UabMotionState = soft_stop_down_right;
            }
            break;

        case cruise_down_right:
            if ( (faults & (~(FR_UP_LEFT_FAULT | OB_UP_FAULT))) == 0 )
            {
                if (UabUpDownLeftRightSwitchState == switch_down_right)
                {
                    if ( uabCheckTopBottom(pio0, pio2) == UAB_BOTTOM_RIGHT )
                    {
                        uabBottomStop();
                    }
                    else
                    {
                        if (uabCheckStopDownRight(pio2))
                        {
                            UabEndPwm = PINNACLE_SLOW_CRUISE_DOWN_PWM;
                            UabPwmStep = SOFT_STOP_DOWN_PWM_STEP;
                            if (UabIrState == irDownRight)
                               UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL_B;
                            else
                               UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL;
                            UabMotionState = soft_stop_down_right;
                        }
                    }
                }
                else
                {
                    UabEndPwm       = 5;     /* assumes in mid track */
                    UabPwmStep      = SOFT_STOP_DOWN_PWM_STEP;
                    UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL_A;
                    UabMotionState  = soft_stop_down_right;
                }
            }
            else
            {
                uabMotorOff();
                UabMotionState = stopped;
            }
            break;

        case soft_stop_down_right:
            if ( uabCheckTopBottom(pio0, pio2) == UAB_BOTTOM_RIGHT )
            {
                uabBottomStop();
            }
            else
            {
                if  ((faults & (~(FR_UP_LEFT_FAULT | OB_UP_FAULT))) == 0 )
                {
                    if (uabCheckStopDownRight(pio2))
                    {
                        if (UabUpDownLeftRightSwitchState != switch_down_right)
                        {
                            uabMotorOff();
                            UabMotionState = stopped;
                            return;
                        }

                        if (UabPwm > UabEndPwm)
                        {
                            UabEndPwm = PINNACLE_SLOW_CRUISE_DOWN_PWM;
                        }
                    }

                    UabRampInterval--;
                    if (UabRampInterval == 0)
                    {
#if 0
                        printf("B: %d  %d  %d  \n",
                                  UabPwm, UabEndPwm, UabRampInterval);
#endif
                        if (UabPwm > UabEndPwm)
                        {
                            UabPwm -= UabPwmStep;
                            if (uabCheckStopDownRight(pio2))
                            {
                                if (UabIrState == irDownRight)
                                  UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL_B;
                                else
                                  UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL;
                            }
                            else
                                UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL_A;
                            uabPwmChange(UAB_DOWN_RIGHT, UabPwm);
                        }
                        else
                        {
                            if (uabCheckStopDownRight(pio2) == 0)
                            {
                                uabMotorOff();
                                UabMotionState = stopped;
                            }
                            else
                            {
                                UabRampInterval = SOFT_STOP_DOWN_PWM_INTERVAL;
                                UabEndPwm = PINNACLE_SLOW_CRUISE_DOWN_PWM;
                                UabPwm = UabEndPwm;
                                uabPwmChange(UAB_DOWN_RIGHT, UabPwm);
                            }
                        }
                    }
                }
                else
                {
                    uabMotorOff();
                    UabMotionState = stopped;
                }
            }
            break;

        default:
            break;
    }
}

/*
******************************************************************************
** Function:     uabCheckSwitch
**
** Description:  This function updates any change
**               in any switch connected to the LPC122x PIO pins to DiagGUI
**
** Input  Parms: uint32_t pio0       Current  PIO0 port contents
**               uint32_t pio2       Current  PIO2 port contents
**               uint32_t prev_pio0  Previous PIO0 port contents
**               uint32_t prev_pio2  Previous PIO2 port contents
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabCheckSwitch(uint32_t pio0,      uint32_t pio2,
                     uint32_t prev_pio0, uint32_t prev_pio2)
{
    uint8_t eData[2];

    if ( (pio2      & OBSTR_UP_SWITCH) !=
         (prev_pio2 & OBSTR_UP_SWITCH) )
    {
        eData[0] = OB_UP_SWITCH_NUM; 
        if ((pio2 & OBSTR_UP_SWITCH) == 0)
          eData[1] = 1;         //OBSTR UP Detected
        else
          eData[1] = 0;         //OBSTR UP Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio2      & OBSTR_DOWN_SWITCH) !=
         (prev_pio2 & OBSTR_DOWN_SWITCH) )
    {
        eData[0] = OB_DOWN_SWITCH_NUM; 
        if ((pio2 & OBSTR_DOWN_SWITCH) == 0)
          eData[1] = 1;         //OBSTR DOWN Detected
        else
          eData[1] = 0;         //OBSTR DOWN Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & FOOTREST_UNDER_SWITCH) !=
         (prev_pio0 & FOOTREST_UNDER_SWITCH) )
    {
        eData[0] = FR_UNDER_SWITCH_NUM; 
        if (pio0 & FOOTREST_UNDER_SWITCH)
          eData[1] = 1;         //FR UNDER Detected
        else
          eData[1] = 0;         //FR UNDER Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & STOP_LIMIT_SWITCH) !=
         (prev_pio0 & STOP_LIMIT_SWITCH) )
    {
        eData[0] = STOP_LIMIT_SWITCH_NUM; 
        if (pio0 & STOP_LIMIT_SWITCH)
          eData[1] = 1;         //STOP LIMIT Detected
        else
          eData[1] = 0;         //STOP LIMIT Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & STOP_UP_LEFT_SWITCH) !=
         (prev_pio0 & STOP_UP_LEFT_SWITCH) )
    {
        eData[0] = STOP_UP_LEFT_SWITCH_NUM; 
        if (pio0 & STOP_UP_LEFT_SWITCH)
          eData[1] = 1;         //STOP UP LEFT Detected
        else
          eData[1] = 0;         //STOP UP LEFT Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio2      & STOP_DOWN_RIGHT_SWITCH) !=
         (prev_pio2 & STOP_DOWN_RIGHT_SWITCH) )
    {
        eData[0] = STOP_DOWN_RIGHT_SWITCH_NUM; 
        if (pio2 & STOP_DOWN_RIGHT_SWITCH)
          eData[1] = 1;         //STOP DOWN RIGHT Detected
        else
          eData[1] = 0;         //STOP DOWN_RIGHT Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & FOOTREST_UP_LEFT_SWITCH) !=
         (prev_pio0 & FOOTREST_UP_LEFT_SWITCH) )
    {
        eData[0] = FR_UP_LEFT_SWITCH_NUM; 
        if (pio0 & FOOTREST_UP_LEFT_SWITCH)
          eData[1] = 1;         //FR UP LEFT Detected
        else
          eData[1] = 0;         //FR UP LEFT Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & FOOTREST_DOWN_RIGHT_SWITCH) !=
         (prev_pio0 & FOOTREST_DOWN_RIGHT_SWITCH) )
    {
        eData[0] = FR_DOWN_RIGHT_SWITCH_NUM; 
        if (pio0 & FOOTREST_DOWN_RIGHT_SWITCH)
          eData[1] = 1;         //FR DOWN RIGHT Detected
        else
          eData[1] = 0;         //FR DOWN RIGHT Off
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & UP_SWITCH) !=
         (prev_pio0 & UP_SWITCH) )
    {
        eData[0] = UP_LEFT_SWITCH_NUM; 
        if (pio0 & UP_SWITCH)
          eData[1] = 0;         //UP_LEFT Off
        else
          eData[1] = 1;         //UP_LEFT On
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & DOWN_SWITCH) !=
         (prev_pio0 & DOWN_SWITCH) )
    {
        eData[0] = DOWN_RIGHT_SWITCH_NUM; 
        if (pio0 & DOWN_SWITCH)
          eData[1] = 0;         //DOWN_RIGHT Off
        else
          eData[1] = 1;         //DOWN_RIGHT On
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if (UabInstMode)
    {
        if ( (pio0      & INST_DOWN_SWITCH) !=
             (prev_pio0 & INST_DOWN_SWITCH) )
        {
            eData[0] = INST_DOWN_SWITCH_NUM;
            if (pio0 & INST_DOWN_SWITCH)
              eData[1] = 0;         //INST DOWN Off
            else
              eData[1] = 1;         //INST DOWN On
            uabEvent(UAB_SWITCH_CHANGE_EVENT,
                      &eData, sizeof(eData) );
        }

        if ( (pio2      & INST_UP_SWITCH) !=
             (prev_pio2 & INST_UP_SWITCH) )
        {
            eData[0] = INST_UP_SWITCH_NUM;
            if (pio2 & INST_UP_SWITCH)
              eData[1] = 0;         //INST UP Off
            else
              eData[1] = 1;         //INST UP On
            uabEvent(UAB_SWITCH_CHANGE_EVENT,
                      &eData, sizeof(eData) );
        }
    }

    if ( (pio0      & KEY_SWITCH) !=
         (prev_pio0 & KEY_SWITCH) )
    {
        eData[0] = KEY_SWITCH_NUM; 
        if (pio0 & KEY_SWITCH)
          eData[1] = 0;         //KEY Off
        else
          eData[1] = 1;         //KEY On
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }

    if ( (pio0      & SEAT_SWITCH)!=
         (prev_pio0 & SEAT_SWITCH) )
    {
        eData[0] = SEAT_SWITCH_NUM; 
        if (pio0 & SEAT_SWITCH)
          eData[1] = 0;         //SEAT Not OK
        else
          eData[1] = 1;         //SEAT OK
        uabEvent(UAB_SWITCH_CHANGE_EVENT,
                  &eData, sizeof(eData) );
    }
}

/*
******************************************************************************
** Function:     uabInit
**
** Description:  This Initialization function configures all the the
**               LPC122x PIO Port pins that are specific to the UAB.
**               It also enables MDrive and Charger Init pins, sets up
**               32-bit COunter 0 on LPC122x for PWM, shuts off the motor,
**               and engages the brake.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  UabSystem
**
** Return Value: None
**
******************************************************************************
*/
void uabInit(void)
{
    LPC_IOCON->PIO2_4 &= ~0x1F;          /* Set ALI as PIO Output */
    LPC_IOCON->PIO2_4 |= (1<<9);         /* Set ALI HIGH Drive */

    LPC_IOCON->PIO2_5 &= ~0x1F;          /* Set AHI as PIO Output */
    LPC_IOCON->PIO2_5 |= (1<<9);         /* Set AHI HIGH Drive */

    LPC_IOCON->PIO2_6 &= ~0x1F;          /* Set BLI as PIO Output */
    LPC_IOCON->PIO2_6 |= (1<<9);         /* Set BLI HIGH Drive */

    LPC_IOCON->PIO2_7 &= ~0x1F;          /* Set BHI as PIO Output */
    LPC_IOCON->PIO2_7 |= (1<<9);         /* Set BHI HIGH Drive */

    LPC_IOCON->PIO2_0 &= ~0x1F;          /* Set DisableMDrive as PIO Output */

    LPC_IOCON->PIO0_27 &= ~0x7F;         /* Set Tilt Switch #1 as input. PULL-UP OFF !!! */
    LPC_IOCON->PIO0_23 &= ~0x7F;         /* Set Tilt Switch #2 as input. PULL-UP OFF !!! */

    LPC_IOCON->PIO2_3  &= ~0x1F;         /* Set Brake as PIO Output */

    LPC_IOCON->PIO0_28 &= ~0x1F;         /* Set GREEN LED as PIO Output */
    LPC_IOCON->PIO0_28 |= (1<<9);        /* Set GREEN LED HIGH Drive */

    LPC_IOCON->PIO0_29 &= ~0x1F;         /* Set RED LED as PIO Output */
    LPC_IOCON->PIO0_29 |= (1<<9);        /* Set RED LED HIGH Drive */

    LPC_IOCON->PIO0_24   &= ~0x1F;       /* Set CPU Diag LED as PIO Output */

    LPC_IOCON->R_PIO1_0  &= ~0x1F;
    LPC_IOCON->R_PIO1_0  |= 1;           /* Set Charge Init as PIO Output */

    LPC_IOCON->PIO1_2    &= ~0x1F;       /* Set Charge Enable as PIO Output */
    LPC_IOCON->PIO0_16   &= ~0x1F;       /* Set Beeper as PIO Output */

    LPC_GPIO0->DIR |= ((1<<24) | (1<<16) | (1<<28) | (1<<29));  /* LEDs & Beep Out */

    LPC_GPIO1->DIR |= ((1<<0) | (1<<2));  /* Charge Init and Enable as PIO Out */

    /* Dis MDriver(PIO2_0), Brake (PIO2_3) and Motor Cntl (PIO2_4-7) Out */
    LPC_GPIO2->DIR |= ((1<<0)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));

    LPC_GPIO2->DIR |= ((1<<8)|(1<<9)|(1<<10)|(1<<11));  /* Spare TPs */

    uabMDriveEnable();
    uabChargerInitEnable();

    /* Disable Charging circuit initially */
    uabChargerDisable();

    /* Init PWM */
    uabPwmInit();

    uabBrakeOn();
    uabMotorOff();
}

