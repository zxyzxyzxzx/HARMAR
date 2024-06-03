/*
******************************************************************************
** Filename:  alarms.h
**
** Description:
**       This file defines typedefs used by the Alarm/Fault Handler.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef ALARMS_H_
#define ALARMS_H_

#define DUTY_20      20
#define DUTY_50      50
#define DUTY_100    100

//--- 100 10ms clock ticks equals 2 seconds
#define ONE_SECOND        100

//--- 200 10ms clock ticks equals 2 seconds
#define TWO_SECONDS       200

//--- 300 10ms clock ticks equals 3 seconds
#define THREE_SECONDS     300

//--- 400 10ms clock ticks equals 4 seconds
#define FOUR_SECONDS      400

//--- 500 10ms clock ticks equals 10 seconds
#define FIVE_SECONDS      500

//--- 1000 10ms clock ticks equals 10 seconds
#define TEN_SECONDS      1000

//--- 3000 10ms clock ticks equals 30 seconds
#define THIRTY_SECONDS   3000

//---  6000 10ms clock ticks equals 10 minutes
#define ONE_MINUTE       6000

//--- 60000 10ms clock ticks equals 10 minutes
#define TEN_MINUTES     60000

#define UP_DOWN_MAJOR_FAULT         (1<<14)

typedef enum
{
          lss_NO_FAULT               =  0,
          lss_OB_UP_SAFETY           =  1,
          lss_OB_DOWN_SAFETY         =  2,
          lss_FR_UP_SAFETY           =  3,
          lss_FR_DOWN_SAFETY         =  4,
          lss_FR_UNDER_SAFETY        =  5,
          lss_SEAT_SAFETY            =  6,
          lss_UP_DOWN_MAJOR          =  7,
          lss_AT_REST                =  8,
          lss_OBSTRUCTED             =  9,
          lss_OVERLOAD               = 10,
          lss_OVERLOAD_IN_MOTION     = 11,
          lss_LOW_POWER              = 12,
          lss_LOW_POWER_IN_MOTION    = 13,
          lss_OFF_CHARGE             = 14,
          lss_IN_MOTION              = 15,
          lss_SLEEP_MODE             = 16,
          lss_SUPER_LOW_POWER        = 17,
          lss_VERY_LOW_POWER         = 18,
          lss_MAJOR_FAULT            = 19,
          lss_RUNAWAY_FAULT          = 20,
          lss_KEY_OFF                = 21,
          lss_MOTION_SWITCH_KEY_OFF  = 22,
          lss_INITIAL_STATE          = 23,
          lss_SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT   = 24,
          lss_SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD  = 25,
          lss_SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT  = 26,
          lss_SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD = 27
} lift_status_select;


typedef enum
{
  lcs_OFF         =  0,
  lcs_RED         =  1,
  lcs_GREEN       =  2,
  lcs_AMBER       =  3,
} led_color_select;

/* Function Prototypes */
void alarms_fault_handler( uint32_t   alarm_faults );
void alarms_update_alarm_state( uint32_t   *alarm_faults,
                                int         which_alarm,
                                int         alarm_value );
void alarms_light_led(uint8_t led_color);
void alarms_beep_on(void);
void alarms_beep_off(void);
void alarms_toggle_motion(void);

float alarms_get_charge_voltage(void);
void  alarms_set_voltage( float new_volts );

uabMotionState_t alarms_get_motion_state(void);

#endif /* ALARMS_H_ */
