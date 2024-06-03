/*
******************************************************************************
** Filename:  alarms.c
**
** Description:
**       This file contains all functions needed for Alarm/Fault handling.
**       The functions in alarms.c are only responsible for reporting the
**       faults that set/clear to the user via the armrest multi-color LED
**       and via the on-board Beeper. Detection of faults/alarms occurs
**       via other functions as part of the 10ms scan loop.
**       Functions in alarms.c are responsible for prioritizing the
**       fault conditions and reporting only the highest priority fault.
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include "LPC122x.h"
#include "main.h"
#include "uab.h"
#include "alarms.h"
#include "ir.h"

//--- PRIVATE FUNCTION PROTOTYPES --------------------------------------------
static void alarms_safety_process( lift_status_select lss,
                                   uint32_t fault);

static void alarms_beep_once(           lift_status_select   which_state,
                                        uint32_t             active_fault,
                                        int                  duty_cycle,
                                        uint16_t             cycle_duration );
static void alarms_blink_led(           lift_status_select   which_state,
                                        led_color_select     led_color,
                                        int                  duty_cycle,
                                        uint16_t             cycle_duration,
                                        uint16_t             cycle_interval );
static void alarms_pulse_beep_duration( lift_status_select   which_state,
                                        uint32_t             active_fault,
                                        int                  duty_cycle,
                                        uint16_t             cycle_duration );
static void alarms_pulse_beep(          lift_status_select   which_state,
                                        int                  duty_cycle );

static lift_status_select   alarms_get_lift_state(  uint32_t     alarm_fault  );
static uint8_t              alarms_faults_serviced( uint32_t     fault_event  );

static void alarms_manage_fault_service( lift_status_select   which_state );
static void alarms_pulse_off_charge(     lift_status_select   which_state,
                                         int                  duty_cycle,
                                         uint16_t             cycle_duration,
                                         uint16_t             cycle_interval );
static void alarms_reset_alarm_state(    uint32_t             which_fault     );
static void alarms_major_fault_service(  uint32_t             alarm_fault_flag  );

static void       alarms_reset_alarm(void);
static uint8_t    alarms_switch_check(void);
static void       alarms_safety_led(void);


//--- L O C A L   V A R I A B L E S ------------------------------------------
lift_status_select   alarm_state          = NO_FAULTS;
lift_status_select   previous_alarm_state = lss_INITIAL_STATE;
uint16_t             alarm_counter;
uabMotionState_t    previous_motion_state = stopped;

uabUpDownLeftRightSwitchState_t   previous_switch_state = switch_off;

//--- based on priority, active_fault is the last
//--- fault condition reported by the system
lift_status_select  alarm_active;

//--- bitwise variable that tracks the active faults
uint32_t             faults_serviced = 0;

//--- bitwise variable that tracks the previous
//--- instance of the faults bitwise variable
uint32_t             previous_faults;

void alarms_safety_process( lift_status_select lss, uint32_t fault)
{
    //--- Safety LED check determines LED light
    alarms_safety_led();

     //--- If this fault has not been serviced...
     if ( alarms_faults_serviced( fault ) == 0 )
       //--- This fault to beep once per instance
       alarms_beep_once( lss,
                         fault,
                         DUTY_50,
                         ONE_SECOND );
     else
     {
       //--- Fault already serviced - beep OFF
       alarms_beep_off();
     }

}
//----------------------------------------------------------------------------/
//
// alarms_fault_handler()
//
// Fault handler examines the bitwise alarm_fault variable
// to determine the faults that are active. Based on the
// alarm fault bits, the fault status is reported via the
// beeper and the LED.
//
// Inputs:
//   alarm_fault   32 bit fault handler variable
//
// Outputs:
//   None
//
// Notes:
//   Each of the bit positions in the alarm fault
//   variable indicates a unique fault type. Multiple
//   faults occurring at the same time are possible in
//   which case only the highest priority fault is
//   indicated.
//----------------------------------------------------------------------------
void alarms_fault_handler( uint32_t   alarm_fault )
{
  uint8_t    fault_idx;
  uint32_t   bit_mask = 1;

  //--- if the fault status has changed from the previous sample
  if ( previous_faults != alarm_fault )
  {
    //--- The last fault (bit position 19)
    for ( fault_idx = 0; fault_idx <= 19; fault_idx++ )
    {
      //--- If the fault is clear now...
      if ( ( alarm_fault & bit_mask ) == 0 )
      {
        //--- and was previously set...
        if ( ( previous_faults & bit_mask ) == bit_mask )
          //--- reset the alarm flags for the next fault trigger
          alarms_reset_alarm_state( bit_mask );
      }

      //--- if the previous faults bit is clear...
      if ( ( previous_faults & bit_mask ) == 0 )
      {
        //--- and now the fault is set...
        if ( ( alarm_fault & bit_mask ) == bit_mask )
          //--- Clear the fault serviced flag
          //--- in case the fault was serviced before
          //--- the fault was cleared
          faults_serviced &= ~bit_mask;
      }

      //--- advance to the next fault bit position in the variable
      bit_mask = bit_mask << 1;
    }

    //--- capture the set of faults as the previous values
    previous_faults = alarm_fault;
  }

  //--- alarm fault parameter used to select the
  //--- active alarm for beep and LED display
  alarm_active = alarms_get_lift_state( alarm_fault );

  //--- The alarm that is active dictates if and how the
  //--- beep is sounded and dictate if an how the LED
  //--- is illuminated
  switch( alarm_active )
  {
    case lss_AT_REST:
      //--- Fault status NO FAULT
      alarms_light_led( lcs_GREEN );
      alarms_beep_off();
      break;

    case lss_UP_DOWN_MAJOR:
      //--- Fault status UP direction Down Direction conflict
      //--- Turn off Beep - no beep for Major Fault
      alarms_beep_off();

      //--- Major Fault - light LED RED
      alarms_light_led( lcs_RED );

      //--- Report the Major fault instance
      alarms_major_fault_service( alarm_fault );
      break;

    case lss_FR_UNDER_SAFETY:
      //--- UNDER SAFETY alone is a Minor Fault
      alarms_light_led( lcs_AMBER );

      //--- if the Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
        //--- Turn the beeper on
        alarms_beep_on();

      //--- If this fault has not been serviced...
      else if ( alarms_faults_serviced( FR_UNDER_FAULT ) == 0 )
        //--- This fault to beep once per instance
        alarms_beep_once( lss_FR_UNDER_SAFETY,
                          FR_UNDER_FAULT,
                          DUTY_50,
                          ONE_SECOND );
      else
      {
        //--- Turn off the Beeper
        alarms_beep_off();

        //--- For consistency this is called for UNDER
        //--- SAFETY Fault even though this fault cannot
        //--- be associated with a Major Fault
        alarms_major_fault_service( alarm_fault );
      }
      break;

    case lss_OB_UP_SAFETY:
      //--- Safety LED check determines LED light
      alarms_safety_led();

      //--- if the Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
        //--- Turn the beeper on
        alarms_beep_on();

      //--- If this fault has not been serviced...
      else if ( alarms_faults_serviced( OB_UP_FAULT ) == 0 )
        //--- This fault to beep once per instance
        alarms_beep_once( lss_OB_UP_SAFETY,
                          OB_UP_FAULT,
                          DUTY_50,
                          ONE_SECOND );
      else
      {
        //--- Fault already serviced - beep OFF
        alarms_beep_off();

        //--- Check for major faults associated with
        //--- this minor fault
        alarms_major_fault_service( alarm_fault );
      }
      break;

    case lss_OB_DOWN_SAFETY:
      //--- Safety LED check determines LED light
      alarms_safety_led();

      //--- if the Motion Switch is pressed
      if ( alarms_switch_check() == 1 )
        //--- Turn the beeper on
        alarms_beep_on();

      //--- If this fault has not been serviced...
      else if ( alarms_faults_serviced( OB_DOWN_FAULT ) == 0 )
        //--- This fault to beep once per instance
        alarms_beep_once( lss_OB_DOWN_SAFETY,
                          OB_DOWN_FAULT,
                          DUTY_50,
                          ONE_SECOND );
      else
      {
        //--- Fault already serviced - beep OFF
        alarms_beep_off();

        //--- Check for major faults associated with
        //--- this minor fault
        alarms_major_fault_service( alarm_fault );
      }
      break;

    case lss_FR_UP_SAFETY:
        //--- Safety LED check determines LED light
         alarms_safety_led();

         //--- if the Motion Switch is pressed...
         if ( alarms_switch_check() == 1 )
           //--- Turn the beeper on
           alarms_beep_on();

         //--- If this fault has not been serviced...
         else if ( alarms_faults_serviced( FR_UP_LEFT_FAULT ) == 0 )
           //--- This fault to beep once per instance
           alarms_beep_once( lss_FR_UP_SAFETY,
                             FR_UP_LEFT_FAULT,
                             DUTY_50,
                             ONE_SECOND );
         else
         {
           //--- Fault already serviced - beep OFF
           alarms_beep_off();

           //--- Check for major faults associated with
           //--- this minor fault
           alarms_major_fault_service( alarm_fault );
         }
         break;

    case lss_FR_DOWN_SAFETY:
      //--- Safety LED check determines LED light
      alarms_safety_led();

      //--- if Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
        //--- Turn the beeper on
        alarms_beep_on();

      //--- If this fault has not been serviced...
      else if ( alarms_faults_serviced( FR_DOWN_RIGHT_FAULT ) == 0 )
        //--- This fault to beep once per instance
        alarms_beep_once( lss_FR_DOWN_SAFETY,
                          FR_DOWN_RIGHT_FAULT,
                          DUTY_50,
                          ONE_SECOND );
      else
      {
        //--- Fault already serviced - beep OFF
        alarms_beep_off();

        //--- Check for major faults associated with
        //--- this minor fault
        alarms_major_fault_service( alarm_fault );
      }
      break;

    case lss_SEAT_SAFETY:
      //--- Minor Fault LED is Amber
      alarms_light_led( lcs_AMBER );

      //--- if the Motion Switch has been pressed...
      if ( alarms_switch_check() == 1 )
        //--- Turn the beeper on
        alarms_beep_on();

      //--- If this fault has not been serviced...
      else if ( alarms_faults_serviced( SEAT_FAULT ) == 0 )
        //--- This fault to beep once per instance
        alarms_beep_once( lss_SEAT_SAFETY,
                          SEAT_FAULT,
                          DUTY_50,
                          ONE_SECOND );
      else
      {
        //--- Fault already serviced - beep OFF
        alarms_beep_off();

        //--- Check for major faults associated with
        //--- this minor fault
        alarms_major_fault_service( alarm_fault );
      }
      break;

    case lss_MAJOR_FAULT:
      //--- Major Fault LED is RED
      alarms_light_led( lcs_RED );

      //--- No Beep For Major Fault
      alarms_beep_off();
      break;

    case lss_RUNAWAY_FAULT:
      //--- Major Fault LED is RED
      alarms_light_led( lcs_RED );

      //--- No Beep For Major Fault
      alarms_beep_off();
      break;

    case lss_OVERLOAD:
      //--- Minor fault is Amber
      alarms_light_led( lcs_AMBER );

      //--- If the Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
        //--- beep at 50% duty cycle while moving
        alarms_pulse_beep( alarm_state,
                           DUTY_50 );
      else
      {
        //--- If this fault has not been serviced...
        if ( alarms_faults_serviced( SYSTEM_OVERLOAD_FAULT ) == 0 )
        {
          //--- Carriage is not moving. Beep 30 seconds.
          alarms_pulse_beep_duration( lss_OVERLOAD,
                                      SYSTEM_OVERLOAD_FAULT,
                                      DUTY_50,
                                      THIRTY_SECONDS );
        }
        else
          alarms_beep_off();
      }
      break;

    case lss_LOW_POWER:
      //--- if the Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
      {
        //--- Blink LED in AMBER at 20 percent duty cycle
        //--- Duration of blink is one second
        alarms_blink_led( alarm_state,
                          lcs_AMBER,
                          DUTY_20,
                          ONE_SECOND,
                          ONE_SECOND );

        //--- Beep is sounds at 50% duty cycle
        alarms_pulse_beep( alarm_state,
                           DUTY_50 );
      }
      else
      {
        //--- Motion switch is not pressed, LED is OFF
        alarms_light_led( lcs_OFF );

        //--- Beep is not sounded
        alarms_beep_off();
      }
      break;

    case lss_VERY_LOW_POWER:
      //--- For a Very Low Power Event LED is OFF
      alarms_light_led( lcs_OFF );

      //--- Beep is not sounded
      alarms_beep_off();
      break;

    case lss_SUPER_LOW_POWER:
      //--- For a Super Low Power Event LED is OFF
      alarms_light_led( lcs_OFF );

      //--- Beep is not sounded
      alarms_beep_off();
      break;

    case lss_OFF_CHARGE:
      //--- if the Motion Switch is pressed...
      if ( alarms_switch_check() == 1 )
      {
        //--- LED is turned to GREEN
        alarms_light_led( lcs_GREEN );

        //--- No beep
        alarms_beep_off();
      }
      else
      {
        //--- If the carriage is in an OFF Charge
        //--- position
        alarms_pulse_off_charge( alarm_state,
                                 DUTY_20,
                                 ONE_MINUTE,
                                 TEN_MINUTES );
      }
      break;

    case lss_KEY_OFF:
      alarms_light_led( lcs_OFF );
      alarms_beep_off();
      break;

    case lss_MOTION_SWITCH_KEY_OFF:
        //--- If this fault has not been serviced...
        if ( alarms_faults_serviced( KEY_OFF_FAULT ) == 0 )
        {
          //--- Carriage is not moving. Beep 30 seconds.
          alarms_pulse_beep_duration( lss_MOTION_SWITCH_KEY_OFF,
                                      KEY_OFF_FAULT,
                                      DUTY_100,
                                      THREE_SECONDS );
          alarms_light_led( lcs_RED );

        }
        break;

    case lss_SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT:
        alarms_safety_process(lss_SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT,
                              SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT);
        break;

    case lss_SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD:
        alarms_safety_process(lss_SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD,
                              SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT);
        break;

    case lss_SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT:
        alarms_safety_process(lss_SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT,
                              SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT);
        break;

    case lss_SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD:
        alarms_safety_process(lss_SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD,
                              SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT);
        break;

    default:
      break;
  }

  alarm_counter++;
}


//----------------------------------------------------------------------------
//
// alarms_safety_led()
//
// For Safety switches on the foot rest and the carriage,
// if illogical combinations of contacts being set occur
// simultaneously, an Uab Major Fault flag is set to
// indicate a Major fault. If the flag is set, the LED
// is set to RED otherwise the LED is set to Amber.
//
// Input:
//   None
//
// Output:
//   None
//
// Return:
//   None
//
// Note:
//   This module is called to determine a detected Minor Fault
//   should be upgraded to a Major Fault due to multiple
//   Minor Fault conditions occurring simulataneously
//
//----------------------------------------------------------------------------
static void alarms_safety_led(void)
{
  //--- If a Major Fault has been detected...
  if (UabMajorFault == TRUE )
    //--- ...illuminate the LED in RED
    alarms_light_led( lcs_RED );
  else
    //--- ...Minor Fault illuminate in Amber
    alarms_light_led( lcs_AMBER );
}


//----------------------------------------------------------------------------
//
// alarms_switch_check()
//
// Looking for a motion switch state transition between
//    switch_off AND any other state
// as an indication that the switch transitioned from
//    switch OFF to switch not OFF
// OR
//    switch not OFF to switch OFF
//
// Input:
//   None
//
// Output:
//   None
//
// Return:
//   Motion status indicates whether motion switch is active
//
//----------------------------------------------------------------------------
static uint8_t   alarms_switch_check()
{
  //--- motion switch status returned to caller
  uint8_t   motion_status;

  //--- If the switch state is off Motion Switch is OFF...
  if ( UabUpDownLeftRightSwitchState == switch_off )
  {
    //--- Motion switch is not active
    motion_status = 0;

    //--- If the motion switch was previous Not OFF...
    if ( previous_switch_state != switch_off )
    {
      //--- capture the new motion switch  state
      previous_switch_state = switch_off;

      //--- reset the counter to the beginning
      alarm_counter = 0;

      //--- Start possible cyclic sequence with beep OFF
      alarms_beep_off();
    }
  }
  else
  {
    //--- Motion switch status is active
    motion_status = 1;

    //--- If previous switch state is OFF...
    if ( previous_switch_state == switch_off )
      //--- capture the new switch state
      previous_switch_state = UabUpDownLeftRightSwitchState;
  }

  //--- return the motion switch status to the caller.
  return motion_status;
}


//----------------------------------------------------------------------------
//
// alarms_serviced()
//
// For alarms and beep sounds that continue for some amount of time
// then stop, this function handles whether the fault event has
// been serviced. This is done to prevent alarm and beep to occur
// again after a possible counter overflow. Once the alarm beep event
// has been serviced, the event will not be reported again until
// a new instance of the event type is detected
//
// Input:
//   fault event   bitwise variable that indicates the active event
//                 when a given event fault bit is active, the
//                 fault event has not yet been serviced
//
// Output:
//   None
//
// Return:
//   Status that indicates whether the event has been serviced.
//
//----------------------------------------------------------------------------
static uint8_t  alarms_faults_serviced( uint32_t   fault_event )
{
  uint8_t   alarms_status;

  //--- if the fault event bit is set in the faults serviced flags...
  if ( ( faults_serviced & fault_event ) == fault_event )
    //--- The event has already been serviced - do not perform again
    alarms_status = 1;
  else
    //--- The event has not yet been serviced.
    alarms_status = 0;

  return alarms_status;
}


//----------------------------------------------------------------------------
//
// alarms_light_led()
//
//   This function controls the illumination of the Diag LED
//   according to the value of the input parameter
//
//   Input:
//     led_color : [RED, GREEN, AMBER, OFF]
//
//   Output:
//     None
//
//   Return:
//     None
//
//----------------------------------------------------------------------------

void alarms_light_led( uint8_t led_color )
{
    if (UabLed != led_color)
    {
        UabLed = led_color;
        if (UabInitComplete)
            uabEvent(UAB_LED_CHANGE_EVENT,
                     &UabLed, sizeof(UabLed) );
    }
    uabDiagLed(led_color);
}


//----------------------------------------------------------------------------
//
// alarms_blink_led()
//
// This function controls the blinking of the LED
// according to the led_color parameter
//
//   Input:
//
//
//----------------------------------------------------------------------------
static void alarms_blink_led( lift_status_select   which_state,
                              led_color_select     led_color,
                              int                  duty_cycle,
                              uint16_t             cycle_duration,
                              uint16_t             cycle_interval )
{
  if ( previous_alarm_state != which_state )
  {
    alarm_counter = 0;
    alarms_light_led( lcs_AMBER );
    previous_alarm_state = which_state;
  }
  else
  {
           if ( alarm_counter >= cycle_interval )
      alarm_counter = 0;

    if ( alarm_counter < cycle_duration )
           {
      if ( alarm_counter%100 <= duty_cycle )
        alarms_light_led( lcs_AMBER );
      else
        alarms_light_led( lcs_OFF );
    }
  }
}


//----------------------------------------------------------------------------
//
// alarms_pulse_off_charge()
//
// This function handles the LED and beep indications
// unique to the carriage OFF Charge condition.
//
// Inputs:
//   which state      identifies the current active fault event
//   duty cycle       portion of period in which LED/Beep is active
//                    a duty cycle of 50 percent over a one ("1") second
//                    interval results in indicator active half of
//                    the period
//   cycle duration   period duration over which the duty cycle is
//                    implemented
//   cycle interval   period of repeat sequence. OFF Charge sequence
//                    repeats on a cycle interval of ten ("10") minutes
//
//
//----------------------------------------------------------------------------
static void alarms_pulse_off_charge( lift_status_select   which_state,
                                     int                  duty_cycle,
                                     uint16_t             cycle_duration,
                                     uint16_t             cycle_interval )
{
  //--- If the active fault event has changed...
  if ( previous_alarm_state != which_state )
  {
    //--- clear the alarm counter - this is a new event
    alarm_counter = 0;

    //--- capture the new fault event
    previous_alarm_state = which_state;
  }
  else
  {
    //--- If the cycle interval has been reached...
    if ( alarm_counter >= cycle_interval )
      //--- reset the alarm counter - this will cause the
      //--- alarm sequence to repeat
      alarm_counter = 0;

    //--- If the alarm counter is less than 30 seconds
    if ( alarm_counter < THIRTY_SECONDS )
    {
      //--- Set the LED to GREEN
      alarms_light_led( lcs_GREEN );

      //--- The beep is OFF
      alarms_beep_off();
    }

    //--- After thirty seconds of the active OFF CHARGE event
    else if ( alarm_counter < cycle_duration )
    {
      //--- if the alarm counter is less than the duty cycle...
      if ( alarm_counter%100 <= duty_cycle )
      {
        UabOffchargeFlag = TRUE;
        alarms_light_led( lcs_AMBER );
        alarms_beep_on();
      }
      else
      {
        //--- alarm counter is greater than the duty cycle
        alarms_light_led( lcs_OFF );
        alarms_beep_off();
      }
    }
    else
    {
      alarms_light_led( lcs_GREEN );
    }
  }
}


//----------------------------------------------------------------------------
//
// alarms_pulse_beep_duration()
//
// This function is for a beep active for a fixed non-repeating
// period of time
//
// Inputs:
//   which_state       currently active fault state
//   active fault      bitwise flag of currently active faults
//   duty cycle        percent of period that indicator is active
//   cycle duration    duration of fault indication
//
// Outputs:
//   None
//
// Return:
//   None
//
//
//----------------------------------------------------------------------------
static void alarms_pulse_beep_duration( lift_status_select   which_state,
                                        uint32_t             active_fault,
                                        int                  duty_cycle,
                                        uint16_t             cycle_duration )
{
  //--- If the fault event has changed...
  if ( previous_alarm_state != which_state )
  {
    //--- if the fault event "which state" is already set, then ignore
    //--- if this active fault has been serviced previously...
    if ( ( active_fault & faults_serviced ) != active_fault )
    {
      //--- Then this is a new instance of this fault type
      alarm_counter = 0;

      //--- Sound the beep
      alarms_beep_on();

      //--- Capture the current fault event
      previous_alarm_state = which_state;

      //--- fault event has occurred again. Clear the
      //--- fault serviced flag in order for alert to
      //--- occur due to recurrence of this fault.
      faults_serviced &= ~active_fault;
    }
  }
  else
  {
    //--- If the alarm count has not reached the duration
    if ( alarm_counter < cycle_duration )
    {
      //--- If the alarm counter is less than the duty cycle
      if ( alarm_counter%100 <= duty_cycle )
        //--- ...beep is ON
        alarms_beep_on();
      else
        //---...otherwise beep is OFF
        alarms_beep_off();
    }
    else
    {
      //--- If the active fault has not been servied
      if ( ( faults_serviced & active_fault ) != active_fault )
        //--- set faults serviced flag to indicate fault has
        //--- been serviced
        alarms_manage_fault_service( which_state );

      //--- Turn Beep OFF to prevent persistent beep beyond duration
      alarms_beep_off();
    }
  }
}

//----------------------------------------------------------------------------
//
// alarms_manage_fault_service()
//
// Some of the beeps are for a specific period of time after which
// the beep does not sound again until the fault event is cleared
// and the fault is set again.
//
// This function is called when the duration parameter has been
// met and the fault is then set to serviced.
//
// Inputs:
//   which_state   currently active fault event
//
// Outputs:
//   None:
//
// Return:
//   None
//
//----------------------------------------------------------------------------
static void alarms_manage_fault_service( lift_status_select which_state )
{

  switch( which_state )
  {
    case lss_UP_DOWN_MAJOR:
      //--- clear the UP DOWN Fault bit
      //--- in the faults serviced variable
      faults_serviced |= UP_DOWN_MAJOR_FAULT;
      break;

    case lss_OB_UP_SAFETY:
      //--- clear the Obstruction UP Fault bit
      //--- in the faults serviced variable
      faults_serviced |= OB_UP_FAULT;
      break;

    case lss_OB_DOWN_SAFETY:
      //--- clear Obstruction UP Fault bit in faults serviced variable
      faults_serviced |= OB_DOWN_FAULT;
      break;

    case lss_FR_UNDER_SAFETY:
      //--- clear Foot Rest Under Switch bit in faults serviced variable
      faults_serviced |= FR_UNDER_FAULT;
      break;

    case lss_FR_UP_SAFETY:
      //--- clear Foot Rest Up Left Switch bit in faults serviced variable
      faults_serviced |= FR_UP_LEFT_FAULT;
      break;

    case lss_FR_DOWN_SAFETY:
      //--- clear Foot Rest Down Right Switch bit in faults serviced variable
      faults_serviced |= FR_DOWN_RIGHT_FAULT;
      break;

    case lss_OVERLOAD:
      //--- clear Over Load bit in faults serviced variable
      faults_serviced |= SYSTEM_OVERLOAD_FAULT;
      break;

    case lss_MOTION_SWITCH_KEY_OFF:
      //--- clear Motion Switch Key OFF bit in faults serviced variable
      faults_serviced |= KEY_OFF_FAULT;
      break;

    case lss_SEAT_SAFETY:
      //--- clear Seat Fault bit in faults serviced variable
      faults_serviced |= SEAT_FAULT;
      break;

    default:
      break;
  }
}


//----------------------------------------------------------------------------
//
// alarms_pulse_beep()
//
// This function controls sounding the beeper at the
// duty cycle.
//
// Inputs:
//   which_state    currently active state
//   duty_cycle     percent duty period indicator is active
//
//----------------------------------------------------------------------------
static void alarms_pulse_beep( lift_status_select   which_state,
                               int                  duty_cycle )
{
  //--- If the fault event has changed...
  if ( previous_alarm_state != which_state )
  {
    //---clear the alarm counter
    alarm_counter = 0;

    //--- Beep is ON
    alarms_beep_on();

    //--- capture the fault event
    previous_alarm_state = which_state;
  }
  else
  {
    //--- not a new fault event.
    //--- If the counter is less than the duty cycle...
    if ( alarm_counter%100 <= duty_cycle )
      //---...Beep is ON
      alarms_beep_on();
    else
      //---...Beep is OFF
      alarms_beep_off();
  }
}


//----------------------------------------------------------------------------
//
// alarms_beep_once()
//
// Sound the beep once
//
// Inputs:
//   which_state      currently active fault event
//   active_fault     bitwise active fault variable
//   duty_cycle       percent over period beep is active
//   cycle_duration   duration for sounding of single beep
//
//
//----------------------------------------------------------------------------
static void alarms_beep_once( lift_status_select   which_state,
                              uint32_t             active_fault,
                              int                  duty_cycle,
                              uint16_t             cycle_duration )
{
  //--- If the fault event has changed...
  if ( previous_alarm_state != which_state )
  {
    //--- If the fault event has not been serviced...
    if ( ( active_fault & faults_serviced ) != active_fault )
    {
      //--- Then this is a new instance of this fault type
      alarm_counter = 0;

      //--- Sound the beep
      alarms_beep_on();

      //--- Capture the fault event
      previous_alarm_state = which_state;

      //--- fault event has occurred again. Clear the
      //--- fault serviced flag in order for alert to
      //--- occur due to recurrence of this fault.
      faults_serviced &= ~active_fault;
    }
  }
  else
  {
    //--- If the alarm counter is less than the duty cycle
    if ( alarm_counter < duty_cycle )
      //--- Beep is ON
      alarms_beep_on();
    else
    {
      //--- If the fault has not bee serviced...
      if ( ( faults_serviced & active_fault ) != active_fault )
      //--- manage the servicing of the fault
        alarms_manage_fault_service( which_state );

      //--- Beep is OFF
      alarms_beep_off();
    }
  }
}

//----------------------------------------------------------------------------
//
// alarms_beep_on()
//
//   Set the BEEP BIT to sound the Beeper.
//
// Inputs:
//   None
//
// Outputs:
//   None
//
// Return:
//   None
//
//----------------------------------------------------------------------------
void alarms_beep_on(void)
{
    LPC_GPIO0->OUT |= BEEP_BIT;
}


//----------------------------------------------------------------------------
//
// alarms_beep_off()
//
//   Clear the BEEP BIT to turn OFF the Beeper.
//
// Inputs:
//   None
//
// Outputs:
//   None
//
// Return:
//   None
//
//----------------------------------------------------------------------------
void alarms_beep_off(void)
{
    LPC_GPIO0->OUT &= ~BEEP_BIT;
}

//----------------------------------------------------------------------------
//
// alarms_reset_alarm_state()
//
// This function resets the alarms for the given fault event.
//
// Input:
//   which_fault   one of the defined fault events
//
//
//----------------------------------------------------------------------------
static void alarms_reset_alarm_state( uint32_t   which_fault )
{
  switch( which_fault)
  {
    case OFFCHARGE_FAULT:
      //--- If the active fault is OFF CHARGE
      if ( alarm_active == lss_OFF_CHARGE )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case SYSTEM_OVERLOAD_FAULT:
      //--- If the active fault is OVERLOAD
      if ( alarm_active == lss_OVERLOAD )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case KEY_OFF_FAULT:
      //--- If the active fault is MOTION SWITCH KEY OFF
      if ( alarm_active == lss_MOTION_SWITCH_KEY_OFF )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case OB_UP_FAULT:
      //--- If the active fault is OB UP SAFETY
      if ( alarm_active == lss_OB_UP_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case OB_DOWN_FAULT:
      //--- If the active fault is OB DOWN SAFETY
      if ( alarm_active == lss_OB_DOWN_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case FR_UP_LEFT_FAULT:
      //--- If the active fault is Foot Rest Up SAFETY
      if ( alarm_active == lss_FR_UP_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case FR_DOWN_RIGHT_FAULT:
      //--- If the active fault is Foot Rest Down SAFETY
      if ( alarm_active == lss_FR_DOWN_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case FR_UNDER_FAULT:
      //--- If the active fault is Foot Rest Under SAFETY
      if ( alarm_active == lss_FR_UNDER_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case SEAT_FAULT:
      //--- If the active fault is Seat SAFETY
      if ( alarm_active == lss_SEAT_SAFETY )
        //--- reset the alarm
        alarms_reset_alarm();
      break;

    case SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT:
        if (alarm_active == lss_SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT)
            alarms_reset_alarm();
        break;

    case SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT:
        if (alarm_active == lss_SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD)
            alarms_reset_alarm();
        break;

    case SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT:
        if (alarm_active == lss_SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT)
            alarms_reset_alarm();
        break;

    case SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT:
        if (alarm_active == lss_SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD)
            alarms_reset_alarm();
        break;

    default:
      break;
  }
}

//----------------------------------------------------------------------------
//
// alarms_reset_alarm()
//
// If the alarms is active when the reset is received,
// reset the counter and the previous alarm state
//
// Inputs:
//   None
//
// Outputs:
//   None
//
// Return:
//   None
//
//
//----------------------------------------------------------------------------
static void alarms_reset_alarm(void)
{
  alarm_counter = 0;
  previous_alarm_state = lss_INITIAL_STATE;
}



//----------------------------------------------------------------------------
//
// alarms_get_lift_state()
//
// The function examines the alarms_fault parameter which contains
// all the bits representing active faults. Combinations of bits
// are examined to determine which faults are active. The severity
// of the faults is implemented in the order in which the bitwise
// combinations are compared with fault status values. The higher
// priority faults are handled first.
//
// Inputs:
//   alarm_fault    bitwise active fault variable
//                  bit value is assigned for each of the identified
//                  fault types.
//
// Output:
//   None
//
// Return:
//   None
//
//----------------------------------------------------------------------------
static lift_status_select  alarms_get_lift_state( uint32_t     alarm_fault )
{
  //--- local alarm status based on active fault events
  lift_status_select   lift_state = lss_NO_FAULT;

  //--- If OB UP and OB DOWN are Both Set - Major Fault
  if ( ( alarm_fault & ( OB_UP_FAULT | OB_DOWN_FAULT ) ) ==
                       ( OB_UP_FAULT | OB_DOWN_FAULT ) )
    lift_state = lss_UP_DOWN_MAJOR;

  //--- If OB UP and FR DOWN RIGHT are Both Set - Major Fault
  else if ( ( alarm_fault & ( OB_UP_FAULT | FR_DOWN_RIGHT_FAULT ) ) ==
                            ( OB_UP_FAULT | FR_DOWN_RIGHT_FAULT ) )
    lift_state = lss_UP_DOWN_MAJOR;

  //--- If OB DOWN and FR UP LEFT are Both Set - Major Fault
  else if ( ( alarm_fault & ( OB_DOWN_FAULT | FR_UP_LEFT_FAULT ) ) ==
                            ( OB_DOWN_FAULT | FR_UP_LEFT_FAULT ) )
    lift_state = lss_UP_DOWN_MAJOR;


  //--- MAJOR FAULT Foot Rest UP and Down Both Active...
  else if ( ( alarm_fault & ( FR_UP_LEFT_FAULT | FR_DOWN_RIGHT_FAULT ) ) ==
                            ( FR_UP_LEFT_FAULT | FR_DOWN_RIGHT_FAULT ) )
    lift_state = lss_UP_DOWN_MAJOR;

  //--- if MAJOR FAULT alone active...
  else if ( ( alarm_fault & FR_MAJOR_FAULT ) == FR_MAJOR_FAULT )
    lift_state = lss_MAJOR_FAULT;

  //--- if RUNAWAY FAULT alone active...
  else if ( ( alarm_fault & RUNAWAY_FAULT ) == RUNAWAY_FAULT )
    lift_state = lss_RUNAWAY_FAULT;

  //--- if System Overload fault active...
  else if ( ( alarm_fault & SYSTEM_OVERLOAD_FAULT ) == SYSTEM_OVERLOAD_FAULT )
    lift_state = lss_OVERLOAD;

  //--- if Very Low Battery fault active...
  else if ( ( alarm_fault & VERY_LOW_BATTERY_FAULT ) == VERY_LOW_BATTERY_FAULT )
    lift_state = lss_VERY_LOW_POWER;

  //--- If Super Low Battery Fault active...
  else if ( ( alarm_fault & SUPER_LOW_BATTERY_FAULT ) == SUPER_LOW_BATTERY_FAULT )
    lift_state = lss_SUPER_LOW_POWER;

  //--- If LOW Battery and OFF Charge Faults are active at the same time
  else if ( ( alarm_fault &  ( LOW_BATTERY_FAULT | OFFCHARGE_FAULT ) ) ==
                             ( LOW_BATTERY_FAULT | OFFCHARGE_FAULT ) )
  {
    //--- If the Motion Switch is pressed...
    if ( alarms_switch_check() == 1 )
      //--- lift state is LOW POWER while in motion
      lift_state = lss_LOW_POWER;
    else
      //--- OFF CHARGE has higher priority than LOW POWER
      lift_state = lss_OFF_CHARGE;
  }
  //--- If the Seat Fault alone is active...
  else if ( ( alarm_fault & SEAT_FAULT ) == SEAT_FAULT )
    lift_state = lss_SEAT_SAFETY;

  //--- If the Foot Rest Under contact alone is set...
  else if ( ( alarm_fault & ( FR_UP_LEFT_FAULT |
                              FR_UNDER_FAULT |
                              FR_DOWN_RIGHT_FAULT ) ) == FR_UNDER_FAULT )
    lift_state = lss_FR_UNDER_SAFETY;

  //--- If the Obstruction in Up direction alone is set...
  else if ( ( alarm_fault & OB_UP_FAULT ) == OB_UP_FAULT )
    lift_state = lss_OB_UP_SAFETY;

  //--- If the Obstruction in Down direction alone is set...
  else if ( ( alarm_fault & OB_DOWN_FAULT ) == OB_DOWN_FAULT )
    lift_state = lss_OB_DOWN_SAFETY;

  //--- If the Foot Rest Up direction alone is set...
  else if ( ( alarm_fault & FR_UP_LEFT_FAULT ) == FR_UP_LEFT_FAULT )
    lift_state = lss_FR_UP_SAFETY;

  //--- If the Foot Rest Down direction alone is set...
  else if ( ( alarm_fault & FR_DOWN_RIGHT_FAULT ) == FR_DOWN_RIGHT_FAULT )
    lift_state = lss_FR_DOWN_SAFETY;

  else if ( ( alarm_fault & SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT ) == SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT )
    lift_state = lss_SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT;

  else if ( ( alarm_fault & SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT ) == SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT )
     lift_state = lss_SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD;

  else if ( ( alarm_fault & SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT ) == SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT )
     lift_state = lss_SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT;

  else if ( ( alarm_fault & SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT ) == SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT )
     lift_state = lss_SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD;

  //--- If the OFF Charge Fault is set...
  else if ( ( alarm_fault & OFFCHARGE_FAULT ) == OFFCHARGE_FAULT )
    lift_state = lss_OFF_CHARGE;

  //--- If the Low Battery Fault is set...
  else if ( ( alarm_fault & LOW_BATTERY_FAULT ) == LOW_BATTERY_FAULT )
    lift_state = lss_LOW_POWER;

  //--- If no faults are set...
  else if ( ( alarm_fault & 0x0FFF ) == 0 )
    lift_state = lss_AT_REST;

  //--- This state placed last will supersede any other
  //--- fault active designation because the this fault
  //--- is tested for last. This is the highest priority
  //--- alarm due to if the key is OFF other alarms can't
  //--- be active.
  if ( ( alarm_fault & KEY_OFF_FAULT ) == KEY_OFF_FAULT )
  {
    //--- If the Motion Switch is pressed...
    if ( alarms_switch_check() == 1 )
    {
      //--- activate Up Down Switch while KEY OFF
      lift_state = lss_MOTION_SWITCH_KEY_OFF;
    }
    else
      lift_state = lss_KEY_OFF;
  }
  return lift_state;
}

//----------------------------------------------------------------------------
//
// alarms_major_fault_service()
//
// If any of these faults have occurred, a major fault is identified.
// Setting of the Major Fault flag is deferred until after the event
// has been serviced.
//
// Inputs:
//   alarm_fault_flag    bitwise active fault variable
//
// Outputs:
//   None
//
// Return:
//
//
// Notes:
//   If a Major Fault is detected, the extern global variable
//   UabMajorFault is set to TRUE. Note that this variable
//   name does does not comply with our coding standard but
//   matches the variable defined in Uab.c
//
//----------------------------------------------------------------------------
static void alarms_major_fault_service(uint32_t alarm_fault_flag )
{
  //--- If installation mode is active...
  if ( UabInstMode == FALSE )
  {
    //--- Check for invalid combinations of items being set
    //--- If Up and Down Foot rest contacts are both active
    if ( ( alarm_fault_flag & ( FR_UP_LEFT_FAULT | FR_DOWN_RIGHT_FAULT ) ) ==
                              ( FR_UP_LEFT_FAULT | FR_DOWN_RIGHT_FAULT ) )
    {
        UabMajorFault = TRUE;
        UabMajorFaultCause = UAB_MAJOR_FAULT_CONF_FRUP_FRDOWN;
    }
    //--- If Obstruction UP and Obstruction Down are both active
    else if ( ( alarm_fault_flag & ( OB_UP_FAULT | OB_DOWN_FAULT ) ) ==
                                   ( OB_UP_FAULT | OB_DOWN_FAULT ) )
    {
        UabMajorFault = TRUE;
        UabMajorFaultCause = UAB_MAJOR_FAULT_CONF_OBUP_OBDOWN;
    }
    //--- If Obstruction UP and Foot Rest Down Right are both active
    else if ( ( alarm_fault_flag & ( OB_UP_FAULT | FR_DOWN_RIGHT_FAULT ) ) ==
                                   ( OB_UP_FAULT | FR_DOWN_RIGHT_FAULT ) )
    {
        UabMajorFault = TRUE;
        UabMajorFaultCause = UAB_MAJOR_FAULT_CONF_FRDOWN_OBUP;
    }
    //--- If Obstruction Down and Foot Rest Up Left are both active
    else if ( ( alarm_fault_flag & ( FR_UP_LEFT_FAULT | OB_DOWN_FAULT ) ) ==
                                   ( FR_UP_LEFT_FAULT | OB_DOWN_FAULT ) )
    {
        UabMajorFault = TRUE;
        UabMajorFaultCause = UAB_MAJOR_FAULT_CONF_FRUP_OBDOWN;
    }
    else
        UabMajorFault = FALSE;
  }
}
