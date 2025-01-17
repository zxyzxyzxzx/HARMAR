/*
******************************************************************************
** Filename:  main_HELIX.c
**
** Description:
**       This is the UAB main program for the HELIX system.
**       It is called by the LPC122x startup code after LPC122x is
**       initialized (clocks, default interrupt handlers,
**       RAM memory variables and stacks).
**
**       This main() function consists basically of only two functionalities:
**               1. UAB Initialization
**               2. UAB 10 ms real-time Execution loop.
**
**       There is no Operating system, no tasks, no heap handling(malloc/free).
**       and no inter-module communication via messaging. All monitoring and
**       control of the stair lift system is completed in the 10 ms scan.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifdef __USE_CMSIS
#include "LPC122x.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include <string.h>
#include "main.h"
#include "main_HELIX.h"
#include "utils.h"
#include "uab.h"
#include "speed.h"
#include "wdt.h"
#include "compadc.h"
#include "iap.h"
#include "alarms.h"
#include "uart.h"

#if defined(HELIX_DEV_DEBUG)  /* Debug Only */
#include "small_printf.h"
#endif

/* 10ms Ticks */
volatile uint32_t SysTicks = 0;
volatile uint32_t DelayCnt = 0;

/* Local Variables */
static uint32_t UabPioCnt = 0;
static uint8_t  UpdateDelay = 0;

/*
******************************************************************************
** Function:     SysTick_Handler (ISR)
**
** Description:  SysTick 10ms Interrupt Service Routine.
**               SysTicks increments every 10ms.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t SysTicks;
**               uint32_t DelayCnt;
**
** Return Value: None
** 
******************************************************************************
*/
void SysTick_Handler(void)
{
    SysTicks++;  /* 10ms per SysTick */
    if (DelayCnt > 0)
      DelayCnt--;
}

/*
******************************************************************************
** Function:     uabCommInit
**
** Description:  Establishes DiagGUI communications to external computer
**               via UAB UART0 Serial port. This comm init message contains
**               the following information:
**
**                    UAB System ID  (PINNACLE,SIERRA,TAMA, etc)
**                    SW Version Major.Minor for System ID
**                    HW Version of UAB (4 chars)
**                    Serial Number of UAB (12 chars)
**                    IR0 Unit Number
**                    IR1 Unit Number
**
**               If the DiagGUI acknowledges (sends ACK) reception
**               of this comm init message, then DiagGUI communication
**               is enabled.  Otherwise, the UAB will execute with no
**               further messages sent to the DiagGUI computer.
**               The UAB must be reset to attempt to re-establish
**               DiagGUI communication.
**
** Input  Parms: None
**
** Output Parms: None
**
** Global Vars:
**
** Return Value: None
******************************************************************************
*/
void uabCommInit( void )
{
    uint8_t  initData[22];
    uint32_t buff[UAB_INFO_PAGE_UINT32_MAX];
    int      rc;    
    uint8_t  done;

    initData[0] = UabSystemId;
    initData[1] = UAB_HELIX_MAJOR_VERSION;
    initData[2] = UAB_HELIX_MINOR_VERSION;
    initData[3] = UabTrackOrientation;
    uabInfoPageGet(buff);

    /* 4 Char UAB HW Version */
    initData[4]  = (uint8_t)((buff[UAB_HW_VER_OFFSET] >> 24) & 0xFF);
    initData[5]  = (uint8_t)((buff[UAB_HW_VER_OFFSET] >> 16) & 0xFF);
    initData[6]  = (uint8_t)((buff[UAB_HW_VER_OFFSET] >>  8) & 0xFF);
    initData[7]  = (uint8_t)((buff[UAB_HW_VER_OFFSET]        & 0xFF));

    /* 12 Char UAB Serial # */
    initData[8]  = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET] >> 24) & 0xFF);
    initData[9]  = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET] >> 16) & 0xFF);
    initData[10] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET] >>  8) & 0xFF);
    initData[11] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET]        & 0xFF));

    initData[12] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+1] >> 24) & 0xFF);
    initData[13] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+1] >> 16) & 0xFF);
    initData[14] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+1] >>  8) & 0xFF);
    initData[15] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+1]        & 0xFF));

    initData[16] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+2] >> 24) & 0xFF);
    initData[17] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+2] >> 16) & 0xFF);
    initData[18] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+2] >>  8) & 0xFF);
    initData[19] = (uint8_t)((buff[UAB_SERIAL_NUM_OFFSET+2]        & 0xFF));

    /* No RF Unit #s */
    initData[20] = 0;
    initData[21] = 0;
    
    uabCpuLedOn();

    UART0_Count     = 0;
    UART0_Buffer[0] = 0;
    UART0_Buffer[1] = 0;
    uabEvent(UAB_INIT_COMPLETE_EVENT,
             initData, sizeof(initData));
    xmitFifoBufSend();

    done = FALSE;
    do
    {
        rc = waitForAck();
        if (rc == 0)
        {
            uabCpuLedOff();
            UabCommEnabled = TRUE;
            done = TRUE;
        }
        else
        {
            if (rc == 1)
                done = TRUE;
            else
                delayUsec(1000);
        }
    }
    while ( (!done) );
}

void uabRfInit(void)
{
    ;
}

/*
******************************************************************************
** Function:     main
**
** Description:  UAB Main Program 
**               
**               This main() function consists basically of the following:
**                      1. UAB Initialization
**                      2. UAB 10 ms real-time Execution loop.
**
**               All monitoring/control of the stair lift system is completed
**               in the 10ms real-time execution loop.
**
** Input  Parms: None
** Output Parms: None
**
** Global Vars:  uint32_t SysTicks;
**               uint32_t DelayCnt;
**
** Return Value: None
** `
******************************************************************************
*/
int main(void)
{
    uint32_t pio0, pio1, pio2;
    uint32_t led_cnt = 0;
    uint32_t pio0_switch_bits;
    uint32_t pio1_switch_bits;
    uint32_t pio2_switch_bits;
    uint32_t prev_pio0_switch_bits = HELIX_PIO0_SWITCH_DEFAULT;
    uint32_t prev_pio1_switch_bits = HELIX_PIO1_SWITCH_DEFAULT;
    uint32_t prev_pio2_switch_bits = HELIX_PIO2_SWITCH_DEFAULT;
    uint8_t csscCntr = 0;
    static uint8_t prevTrackOrien = 0xFF;

#if defined(HELIX_DEV_DEBUG)
    static uint32_t buff[UAB_INFO_PAGE_UINT32_MAX];
#endif

    /* Enable Immediately needed LPC Clocks - GPIO, ICON, CT32B0 */
    LPC_SYSCON->SYSAHBCLKCTRL |= 0xE0010200UL;

    UabSystemId = UAB_SYSTEM_ID_HELIX_RIGHT; //LEFT SYSTEM
    //UabSystemId = UAB_SYSTEM_ID_HELIX_LEFT;  //RIGHT SYSTEM

    if (UabSystemId == UAB_SYSTEM_ID_HELIX_RIGHT)
        UabTrackOrientation = UAB_RIGHT_IS_UP;
    else
        UabTrackOrientation = UAB_LEFT_IS_UP;

    /* CSSC Speed Control : HELIX Specific */
    UabNominalPwm     = HELIX_NOMINAL_PWM;
    UabPwmScaleFactor = HELIX_PWM_SCALE_FACTOR;

    /* 
    ** Initialize all of the LPC122x Port PIO pins
    ** Enable Mdrive
    ** Enable ChargerInit
    ** Set up CT32B0 Timer for PWM
    ** Turn Off the Motor and set the brake
    */
    uabInit();

    /* Configure SysTick for 10 ms based on the 24 MHz system clock */
    SysTick_Config(SystemCoreClock / 100);  /* 10 ms */

    /* Init WDT timeout for 2 Seconds and Enable */
    wdtInit();
    wdtEnable();

    /* Initialize UART0 as DiagGUI Comm port */
    UARTInit(UART0, UART_115200_BAUD);  /* UART0 Interrupt Enabled after this call */

#if defined(HELIX_DEV_DEBUG)
    UARTInit(UART1, UART_115200_BAUD);  /* UART1 Intr Enabled after this call */
#endif

     /* Initialize LPC122x Comparator 0, 1, and ADC */
    uabCompAdcInit();

    /* Initialize RF Input Pins. */
    uabRfInit();

    /* Allow Info Page NVRAM some time to stabilize after reset before using */
    delayMs(500, BUSY_WAIT_BASED, TRUE);

    /* Initialize Info Page (NVRAM..persists over a reset) */
    uabInfoPageInit(UabSystemId,
                    UAB_HELIX_MAJOR_VERSION,
                    UAB_HELIX_MINOR_VERSION);

    /* Attempt to establish DiagGUI communication */
    uabCommInit();

    /* System Fault:  12V Supply check */
    if (uab12vCheck() != OK)
        uabMajorFaultHandler(UAB_MAJOR_FAULT_12V_SUPPLY);
        /* will not return */

    /* Sequence the LEDs R-Y-G-OFF (for diag check) */
    alarms_light_led(lcs_RED);
    delayMs(1000, BUSY_WAIT_BASED, TRUE);
    alarms_light_led(lcs_AMBER);
    delayMs(1000, BUSY_WAIT_BASED, TRUE);
    alarms_light_led(lcs_GREEN);
    delayMs(1000, BUSY_WAIT_BASED, TRUE);
    alarms_light_led(lcs_OFF);

    /* Beep the beeper (diag check) and also indicates Init is complete */
    alarms_beep_on();
    delayMs(750, BUSY_WAIT_BASED, TRUE);
    alarms_beep_off();

    /* Read the PIO Ports and debounce */
    (void)uabPioGet(&pio0, &pio1, &pio2);
#if defined(HELIX_DEV_DEBUG)
    printf("0x%08X  0x%08X  0x%08X\n", pio0, pio1, pio2);
    pio0 = 0; pio1 = 0; pio2 = 0;
    delayMs(750, BUSY_WAIT_BASED, TRUE);
    (void)uabPioGet(&pio0, &pio1, &pio2);
    printf("0x%08X  0x%08X  0x%08X\n", pio0, pio1, pio2);
#endif

    UabInitComplete = TRUE;

#if defined(HELIX_DEV_DEBUG)
    printf("SW Version: %d.%d\n",
            UAB_HELIX_MAJOR_VERSION,
            UAB_HELIX_MINOR_VERSION);
    if (UabTrackOrientation == UAB_LEFT_IS_UP)
        printf("LEFT is UP\n");
    else
        printf("RIGHT is UP\n");
#endif

    /* Enable 10 ms tick interrupt */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* 10 ms real-time execution loop */
    while (1)
    {
        wdtFeed(TRUE);  /* punch WDT */

        /* System Fault:  12V Supply check */
        if (uab12vCheck() != OK)
            uabMajorFaultHandler(UAB_MAJOR_FAULT_12V_SUPPLY);
            /* will not return */

        /* Every 250 ms Secs toggle the on-board GREEN CPU LED On/Off */
        if ((SysTicks % 25) == 0)
        {
            if (led_cnt == 0)
            {
                uabCpuLedOn(); 
                led_cnt = 1;
            }
            else
            {
                uabCpuLedOff(); 
                led_cnt = 0;
            }
        }

       /* Read and Debounce PIO Ports on LPC122x */
        if ( (uabPioGet(&pio0, &pio1, &pio2) == OK) ||
             (UabPioCnt > UAB_DEBOUNCE_MAX) )
        {
            UabPioCnt = 0;
            pio0_switch_bits = (pio0 & HELIX_PIO0_SWITCH_ALL);
            pio1_switch_bits = (pio1 & HELIX_PIO1_SWITCH_ALL);
            pio2_switch_bits = (pio2 & HELIX_PIO2_SWITCH_ALL);

            /* If any switch bits have changed..call DiagGUI event func */
            if ( (pio0_switch_bits != prev_pio0_switch_bits) ||
                 (pio1_switch_bits != prev_pio1_switch_bits) ||
                 (pio2_switch_bits != prev_pio2_switch_bits) )
            {
                uabCheckSwitchHelix(pio0_switch_bits,
                                    pio1_switch_bits,
                                    pio2_switch_bits,
                                    prev_pio0_switch_bits,
                                    prev_pio1_switch_bits,
                                    prev_pio2_switch_bits);
                prev_pio0_switch_bits = pio0_switch_bits;
                prev_pio1_switch_bits = pio1_switch_bits;
                prev_pio2_switch_bits = pio2_switch_bits;
            }

            /* Are we in Installation Mode */
            uabInstModeCheck(pio0);
            if (UabInstMode == FALSE)
            {   /* Not in Installation Mode...do all fault checks */

                /* System Fault:  Runaway check */
                if (uabRunawayCheck() != OK)
                    uabMajorFaultHandler(UAB_MAJOR_FAULT_RUNAWAY);
                    /* will not return */

                /*
                ** System Fault:  System Overload check
                ** If System Overload, don't allow motor to move
                ** otherwise check Battery Voltage
                */
                uabSystemOverloadCheck();
                if ((UabFaults & SYSTEM_OVERLOAD_FAULT) == 0)
                {
                    /*
                    ** System Fault:  SuperLow Battery check
                    ** If SuperLow Bat fault, don't allow motor to move
                    ** otherwise check Very Low Battery Voltage
                    */
                    if (uabSuperLowBatteryCheck() == OK)
                    {
                        /*
                        ** System Fault:  VeryLow Battery check
                        ** If VeryLow Bat fault, allow motor to move
                        ** until top/bottom end of track detected.
                        ** then no more movement allowed.
                        ** otherwise check Low Battery Voltage
                        */
                        if (uabVeryLowBatteryCheck() == OK)
                          /*
                          ** System Fault:  Low Battery check
                          ** If Low Bat fault, allow motor to move
                          ** but warn user via Beeper and Amber LED
                          */
                          uabLowBatteryCheck();

                        if (UabFaults & VERY_LOW_BATTERY_FAULT)
                        {
                            if (UabTripDone == FALSE)
                            {
                                /*
                                ** If VLB fault,
                                ** Call Motion Control with current
                                ** Port PIO switch readings until
                                ** end of track detected.
                                */
                                uabMotionSet(pio0_switch_bits,
                                              pio2_switch_bits);
                            }
                        }
                        else
                        {
                            /*
                            ** Normal execution...no VLB fault
                            ** Call Motion Control with current
                            ** Port PIO switch readings.
                            */
                            uabMotionSet(pio0_switch_bits,
                                          pio2_switch_bits);
                        }
                    }
                }
            }
            else
            {   /* Call Motion Control when in Installation Mode */
                uabMotionSet(pio0_switch_bits, pio2_switch_bits);
            }

            /* Every 10ms scan, do charging circuit control */
            uabChargeCheck();

            /*
            ** Every 10ms scan, pass all System/Safety faults
            ** whether set or clear, to the alarm/fault handler.
            ** The alarm/fault handler will prioritize the faults
            ** and post the highest priority to the LEDs on the
            ** armrest and will pulse the beeper.
            */
            alarms_fault_handler(UabFaults);
        }
        else
        {
            UabPioCnt++;   /* PIO Read Debounce counter */
        }

#if defined(HELIX_DEV_DEBUG)
        int i;
        uint8_t cmd;
        static char prevChar = ' ';

        /* 
        ** Process any single character debug cmds entered via the
        ** UART1 Debug Comm port.  Cannot be done on SIERRA!!!
        */
        if (UART1_Count == 1)
        {
            cmd = UART1_Buffer[UART1_Count-1];
            if (cmd == '0')
                UabDebugMode = 0;
            else if (cmd == '1')
                UabDebugMode = 1;

            if (UabDebugMode != 0)
            {
                switch (cmd)
                {
                    case '9':
                        uabMajorFaultHandler(9);
                        break;

                    case 'g':
                        (void)uabPioGet(&pio0, &pio1, &pio2);
                        printf("PIO0: 0x%08X  PIO1: 0x%08X  PIO2: 0x%08X\n",
                               pio0, pio1, pio2);
                        break;

                    case 'b':
                        uabBrakeOn();          /* Engage Brake */
                        break;

                    case 'o':
                        uabBrakeOff();          /* Release Brake */
                        break;

                    case 'R':
                        prevChar = 'R';
                        break;

                    case 'L':
                        prevChar = 'L';
                        break;

                    case '$':
                        if (prevChar == 'L')
                        {
                            UabPwm = 50;
                            uabMotorOn(UAB_UP_LEFT);
                        }
                        else if (prevChar == 'R')
                        {
                            UabPwm = 50;
                            uabMotorOn(UAB_DOWN_RIGHT);
                        }
                        prevChar = ' ';
                        break;

                    case 's':
                    case 'S':
                        uabMotorOff();
                        UabPwm = 0;
                        prevChar = ' ';
                        break;

                    case 'p':
                        printf("PWM: %d%%\n", UabPwm);
                        break;

                    case 'm':
                        uabMotorVoltsDisplay();
                        break;

                    case 'e':
                        __disable_irq();
                        eraseInfoPage(0, 0);
                        __enable_irq();
                        break;
                        
                    case 'P':
                        readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
                        for (i = 0; i < UAB_INFO_PAGE_UINT32_MAX; i++)
                        {
                            if ((i % 8) == 0)
                                printf("\n%02X:  %08X ", i , buff[i]);
                            else
                                printf("%08X ", buff[i]);
                        }
                        printf("\n");
                        break;

                    case 'W':
                        __disable_irq();
                        eraseInfoPage(0, 0);
                        __enable_irq();
                        readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
                        buff[UAB_IR0_UNIT_OFFSET]     = 0;
                        buff[UAB_IR1_UNIT_OFFSET]     = 0;
                        buff[UAB_HW_VER_OFFSET]       = 0x20203141;
                        buff[UAB_SERIAL_NUM_OFFSET]   = 0x20202020;
                        buff[UAB_SERIAL_NUM_OFFSET+1] = 0x53313233;
                        buff[UAB_SERIAL_NUM_OFFSET+2] = 0x34353637;
                        __disable_irq();
                        writeInfoPage32( (uint32_t*) INFO_LOCATION, buff,
                                         UAB_INFO_PAGE_BYTES_MAX );
                        __enable_irq();
                        break;

                    case 'j':
                        alarms_beep_on();
                        break;

                    case 'k':
                        alarms_beep_off();
                        break;

                    default:
                        prevChar = ' ';
                        break;
                }
            }
        }
        UART1_Count = 0;
#endif

        /* Adjust Current Sense Speed Control (CSSC) */
        csscCntr = 0;
        uabCsscBegin(ADC_CURR_SENSE_SEL);
        while ( (SysTick->VAL > 50000) &&
                (csscCntr < 10) )
        {
            uabCsscCheck(ADC_CURR_SENSE_SEL);
            csscCntr++;
        }
        uabCsscEnd();

        /* Messages will only be sent to the DiagGUI if comm enabled */
        if (UabCommEnabled)
        {
            /* Check for ACK/NAK from DiagGUI for the last message sent */
            if (WaitForAck)
              waitForAck();
            else
            {
                /*
                ** Send update of voltages and current sense amps
                ** to DiagGUI every 2 Secs.
                */
                if (UpdateDelay > 0)
                  UpdateDelay--;
                else
                {
                    /* If Track Orientation changes...post update to DiagGUI */
                    if (prevTrackOrien != UabTrackOrientation)
                    {
                        uabEvent(UAB_TRACK_ORIEN_CHANGE_EVENT,
                                 &UabTrackOrientation,
                                 sizeof(UabTrackOrientation));
                        prevTrackOrien = UabTrackOrientation;
                    }
                    UpdateDelay = 200;  /* 200 x 10 ms = 2 Sec */
                    uabVoltsCurrSenseDisplay();
                }
                /* Send a message to DiagGUI every 10ms then wait for ACK/NAK */
                xmitFifoBufSend();
            }
        }

        /* Wait for next 10ms SysTick */
        __WFI();
        
    }  /* while loop never exits */
}

