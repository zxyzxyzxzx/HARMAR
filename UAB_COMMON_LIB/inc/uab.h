/*
******************************************************************************
** Filename:  uab.h
**
** Description:
**       This file defines LPC1225 PIO register bit defines and typedefs
**       that pertain to their use on Universal Access Board(UAB).
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef UAB_H_
#define UAB_H_

/* System ID */
#define UAB_SYSTEM_ID_PINNACLE      1
#define UAB_SYSTEM_ID_ALPINE        2
#define UAB_SYSTEM_ID_SIERRA        3
#define UAB_SYSTEM_ID_TAMA          4
#define UAB_SYSTEM_ID_HELIX_RIGHT   5
#define UAB_SYSTEM_ID_HELIX_LEFT    6

/* Motion Control Switch */
#define UAB_OFF                  0
#define UAB_UP_LEFT              1
#define UAB_DOWN_RIGHT           2
#define UAB_DOWN_DYN_BRAKE       3

/* PIO Switch Bits */

#define OBSTR_UP_SWITCH                  (1<<14)  /* PIO2_14 */
#define OBSTR_DOWN_SWITCH                (1<<15)  /* PIO2_15 */
/* HELIX */
#define OBSTR_LEFT_SWITCH                (1<<14)  /* PIO2_14 */
#define OBSTR_RIGHT_SWITCH               (1<<15)  /* PIO2_15 */

#define STOP_UP_LEFT_SWITCH              (1<<15)  /* PIO0_15 */
#define STOP_LIMIT_SWITCH                 (1<<0)  /* PIO0_0  */
#define STOP_DOWN_RIGHT_SWITCH            (1<<1)  /* PIO2_1  */

#define INST_UP_SWITCH                    (1<<2)  /* PIO2_2  */
#define INST_DOWN_SWITCH                  (1<<9)  /* PIO0_9  */
/* HELIX */
#define INST_LEFT_SWITCH                  (1<<2)  /* PIO2_2  */
#define INST_RIGHT_SWITCH                 (1<<9)  /* PIO0_9  */

#define FOOTREST_UP_LEFT_SWITCH           (1<<8)  /* PIO0_8  */
#define SIERRA_UPSIDE_RAMP_OBSTR_SWITCH   (1<<8)  /* PIO0_8  */
#define FOOTREST_DOWN_RIGHT_SWITCH        (1<<7)  /* PIO0_7  */
#define SIERRA_DOWNSIDE_RAMP_OBSTR_SWITCH (1<<7)  /* PIO0_7  */
#define FOOTREST_UNDER_SWITCH             (1<<6)  /* PIO0_6  Not on HELIX */

#define UP_SWITCH                         (1<<5)  /* PIO0_5  */
#define DOWN_SWITCH                       (1<<4)  /* PIO0_4  */
/* PIO0_4 and PIO0_5 Swapped on TAMA and HELIX */
#define LEFT_SWITCH                       (1<<4)  /* PIO0_4  */
#define RIGHT_SWITCH                      (1<<5)  /* PIO0_5  */

/* HELIX */
#define RF_LEFT_SWITCH                    (1<<6)  /* PIO0_6 */
#define RF_RIGHT_SWITCH                   (1<<3)  /* PIO0_3 */

#define SEAT_SWITCH                       (1<<3)  /* PIO0_3  Not on HELIX */
#define HELIX_SEAT_SWITCH                 (1<<6)  /* PIO1_6 */

#define KEY_SWITCH                       (1<<14)  /* PIO0_14 */


#define PIO0_SWITCH_ALL               0xC3F9
#define PIO2_SWITCH_ALL               0xC006
#define PIO0_SWITCH_DEFAULT           0x3E36
#define PIO2_SWITCH_DEFAULT           0xF0FC

/*  Use these for HELIX */
#define HELIX_PIO0_SWITCH_ALL         (STOP_UP_LEFT_SWITCH          |  /* 15 L */ \
                                       KEY_SWITCH                   |  /* 14 L */ \
                                       INST_RIGHT_SWITCH            |  /*  9 H */ \
                                       FOOTREST_UP_LEFT_SWITCH      |  /*  8 L */ \
                                       FOOTREST_DOWN_RIGHT_SWITCH   |  /*  7 L */ \
                                       RF_LEFT_SWITCH               |  /*  6 H */ \
                                       RIGHT_SWITCH                 |  /*  5 H */ \
                                       LEFT_SWITCH                  |  /*  4 H */ \
                                       RF_RIGHT_SWITCH)                /*  3 H */
#define HELIX_PIO0_SWITCH_DEFAULT     0x3E7E

#define HELIX_PIO1_SWITCH_ALL         (HELIX_SEAT_SWITCH)              /*  6 L */
#define HELIX_PIO1_SWITCH_DEFAULT     0x003A

#define HELIX_PIO2_SWITCH_ALL         (OBSTR_RIGHT_SWITCH           |  /* 15 H */ \
                                       OBSTR_LEFT_SWITCH            |  /* 14 H */ \
                                       INST_LEFT_SWITCH             |  /*  2 H */ \
                                       STOP_DOWN_RIGHT_SWITCH)         /*  1 L */
#define HELIX_PIO2_SWITCH_DEFAULT     0xF0FC

/* UAB Switch Numbers for DiagGUI */
#define UP_LEFT_SWITCH_NUM          1
#define DOWN_RIGHT_SWITCH_NUM       2
#define INST_UP_SWITCH_NUM          3
#define INST_DOWN_SWITCH_NUM        4
#define OB_UP_SWITCH_NUM            5
#define OB_DOWN_SWITCH_NUM          6
#define FR_UP_LEFT_SWITCH_NUM       7
#define FR_DOWN_RIGHT_SWITCH_NUM    8
#define FR_UNDER_SWITCH_NUM         9
#define SEAT_SWITCH_NUM             10
#define KEY_SWITCH_NUM              11
#define STOP_UP_LEFT_SWITCH_NUM     12
#define STOP_DOWN_RIGHT_SWITCH_NUM  13
#define STOP_LIMIT_SWITCH_NUM       14
/* IR Switches */
#define IR_UP_SWITCH_NUM            20
#define IR_DOWN_SWITCH_NUM          21
/* HELIX RF Switches */
#define RF_LEFT_SWITCH_NUM          20
#define RF_RIGHT_SWITCH_NUM         21
/* Sierra Switches */
#define UPSIDE_RAMP_OB_SWITCH_NUM   30
#define DOWNSIDE_RAMP_OB_SWITCH_NUM 31

/* UAB Fault Bits */
#define NO_FAULTS                          0
#define OB_UP_FAULT                    (1<<0)
#define OB_DOWN_FAULT                  (1<<1)
#define FR_UP_LEFT_FAULT               (1<<2)
#define FR_DOWN_RIGHT_FAULT            (1<<3)
#define FR_UNDER_FAULT                 (1<<4)
#define FR_MAJOR_FAULT                 (1<<5)
#define SEAT_FAULT                     (1<<6)
#define KEY_OFF_FAULT                  (1<<7)
#define OFFCHARGE_FAULT                (1<<8)
#define LOW_BATTERY_FAULT              (1<<9)
#define VERY_LOW_BATTERY_FAULT         (1<<10)
#define RUNAWAY_FAULT                  (1<<11)
#define SYSTEM_OVERLOAD_FAULT          (1<<12)
#define SUPER_LOW_BATTERY_FAULT        (1<<13)

/* Sierra Fault Bits */
#define SIERRA_UPSIDE_RAMP_OB_FAULT     FR_UP_LEFT_FAULT
#define SIERRA_DOWNSIDE_RAMP_OB_FAULT   FR_DOWN_RIGHT_FAULT
#define SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT   (1<<16)
#define SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT  (1<<17)
#define SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT  (1<<18)
#define SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT (1<<19)

#define SWITCH_FAULTS (OB_UP_FAULT          |  \
                       OB_DOWN_FAULT        |  \
                       FR_UP_LEFT_FAULT     |  \
                       FR_DOWN_RIGHT_FAULT  |  \
                       FR_UNDER_FAULT       |  \
                       FR_MAJOR_FAULT       |  \
                       SEAT_FAULT           |  \
                       KEY_OFF_FAULT)

/* UAB Fault Nums for DiagGUI */
#define NO_FAULT_NUM                          0
#define OB_UP_FAULT_NUM                       1
#define OB_DOWN_FAULT_NUM                     2
#define FR_UP_LEFT_FAULT_NUM                  3
#define FR_DOWN_RIGHT_FAULT_NUM               4
#define FR_UNDER_FAULT_NUM                    5
#define SEAT_FAULT_NUM                        6
#define KEY_OFF_FAULT_NUM                     7
#define OFFCHARGE_FAULT_NUM                   8
#define LOW_BATTERY_FAULT_NUM                 9
#define VERY_LOW_BATTERY_FAULT_NUM           10
#define SYSTEM_OVERLOAD_FAULT_NUM            11
#define SUPER_LOW_BATTERY_FAULT_NUM          12
/* 13 - 29 Future Use */

/* Sierra Fault Nums for DiagGUI */
#define SIERRA_UPSIDE_RAMP_OB_FAULT_NUM                 30
#define SIERRA_DOWNSIDE_RAMP_OB_FAULT_NUM               31
#define SIERRA_LEFTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM   32
#define SIERRA_LEFTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM  33
#define SIERRA_RIGHTSIDE_RAMP_RAISED_TIMEOUT_FAULT_NUM  34
#define SIERRA_RIGHTSIDE_RAMP_RAISED_OVERLOAD_FAULT_NUM 35

/* PIO Bit Defines */
#define PROCESS_TIME_BIT        (1<<1)  /* PIO1_1 */
#define DISABLE_MDRIVE_BIT      (1<<0)  /* PIO2_0 */
#define BRAKE_BIT               (1<<3)  /* PIO2_3 */
#define CHARGER_BIT             (1<<2)  /* PIO1_2 */
#define CHARGER_INIT_BIT        (1<<0)  /* PIO1_0 */
#define BEEP_BIT                (1<<16) /* PIO0_16 */
#define TRACK_ORIENTATION_BIT   (1<<27) /* PIO0_27 */

#define ALI_BIT                 (1<<4)  /* PIO2_4 */
#define AHI_BIT                 (1<<5)  /* PIO2_5 */
#define BLI_BIT                 (1<<6)  /* PIO2_6 */
#define BHI_BIT                 (1<<7)  /* PIO2_7 */

#define CPU_DIAG_LED_BIT        (1<<24) /* PIO0_24 */
#define GREEN_LED_BIT           (1<<28) /* PIO0_28 */
#define RED_LED_BIT             (1<<29) /* PIO0_29 */

#define UAB_BEEP_OFF     0
#define UAB_BEEP_ON      1

#define UAB_CHARGER_OFF  0
#define UAB_CHARGER_ON   1

#define UAB_DISABLED     0
#define UAB_ENABLED      1

#define UAB_MIDTRACK     0
#define UAB_BOTTOM_RIGHT 1
#define UAB_TOP_LEFT     2

#define UAB_DEBOUNCE_MAX 3

/* Track Orientation for TAMA */
#define UAB_RIGHT_IS_UP  0
#define UAB_LEFT_IS_UP   1

/* Charging Circuit Defines */
#define UAB_CHARGE_PERIOD       200   /*  2 Sec:  200 x  10ms */
#define CHARGE_DURATION_95       95
#define CHARGE_DURATION_85       85
#define CHARGE_DURATION_25       25
#define CHARGE_DURATION_5         5

#define UAB_PWM_PERIOD (6000 - 1) /* 4K period = 24M/4K = 6000 - 1 = 5999 */

/* Non-scaled max Current Sense value per System Requirements Spec */
#define CURR_SENSE_AMPS_MAX  25.0f

/* Defines for Sierra/Ramp Board */
#define SIERRA_RAMP_OVERLOAD_BIT               (1<<0)
#define SIERRA_PLATFORM_MOUNTING_BIT           (1<<1)
#define SIERRA_RIGHTSIDE_RAMP_POSITION_BIT     (1<<2)
#define SIERRA_LEFTSIDE_RAMP_POSITION_BIT      (1<<3)
#define SIERRA_PLATFORM_POSITION_UNFOLDED_BIT  (1<<4)
#define SIERRA_RAISE_RAMP_BIT                  (1<<5)
#define SIERRA_RIGHTSIDE_RAMP_DRIVE_ENABLE_BIT (1<<6)
#define SIERRA_LEFTSIDE_RAMP_DRIVE_ENABLE_BIT  (1<<7)

#define SIERRA_RIGHTSIDE_UP      0
#define SIERRA_LEFTSIDE_UP       1

#define SIERRA_PLATFORM_FOLDED   0
#define SIERRA_PLATFORM_UNFOLDED 1

#define SIERRA_RAMP_UNKNOWN      0
#define SIERRA_RAMP_RAISED       1
#define SIERRA_RAMP_NOT_RAISED   2 
#define SIERRA_RAMP_LOWERED      3

#define SIERRA_RAMP_TIMEOUT      40  /* 4 Sec = 40 x 100ms loop */

#define SIERRA_SC18IM700_BRG0_INIT  0xF0  /* 9600 Baud */
#define SIERRA_SC18IM700_BRG1_INIT  0x02  /* 9600 Baud */
#define SIERRA_SC18IM700_CFG2_INIT  0x55  /* PIO 0-3 are Inputs */
#define SIERRA_SC18IM700_CFG3_INIT  0xA9  /* PIO 4 Input
                                             PIO 5-7 Output PushPull */

/* Major Faults for DiagGUI */
#define UAB_MAJOR_FAULT_NONE                                0
/* Runaway */
#define UAB_MAJOR_FAULT_RUNAWAY                             1
/* 12V Supply */
#define UAB_MAJOR_FAULT_12V_SUPPLY                          2
/*
** Conflicting Switches  FootRest UP and FootRest DOWN
** or on TAMA FootRest Under Obstruction
*/
#define UAB_MAJOR_FAULT_CONF_FRUP_FRDOWN                    3
#define UAB_MAJOR_FAULT_TAMA_FRUNDER                        3

/* Conflicting Switches  Obstruction UP and Obstruction DOWN */
#define UAB_MAJOR_FAULT_CONF_OBUP_OBDOWN                    4
/* Conflicting Switches  FootRest DOWN and Obstruction UP */
#define UAB_MAJOR_FAULT_CONF_FRDOWN_OBUP                    5
/* Conflicting Switches  FootRest UP and Obstruction DOWN */
#define UAB_MAJOR_FAULT_CONF_FRUP_OBDOWN                    6
/* Conflicting Switches:  STOP UP and STOP DOWN switches both Detected */
#define UAB_MAJOR_FAULT_CONF_STOP_UP_AND_DOWN_DETECTED      7
/* Conflicting Switches:  STOP UP and STOP DOWN switches both Not Detected */
#define UAB_MAJOR_FAULT_CONF_STOP_UP_AND_DOWN_NOT_DETECTED  8

/* Helix Fault:  Final limit switch detected */
#define UAB_MAJOR_FAULT_CONF_HELIX_LIMIT_SWITCH_DETECTED    9

/* 10 - 29 Future */

/* Ramp Board UART Comm Failure    (SIERRA) */
#define UAB_MAJOR_FAULT_RAMP_BOARD_INIT                    30
/* Ramp Board UART Comm Failure    (SIERRA) */
#define UAB_MAJOR_FAULT_RAMP_BOARD_UART_COMM               31
/* Leftside Ramp Current Overload  (SIERRA) */
#define UAB_MAJOR_FAULT_LEFTSIDE_RAMP_CURRENT_OVERLOAD     32
/* Leftside Ramp Raise Timeout     (SIERRA) */
#define UAB_MAJOR_FAULT_LEFTSIDE_RAMP_RAISE_TIMEOUT        33
/* Rightside Ramp Current Overload (SIERRA) */
#define UAB_MAJOR_FAULT_RIGHTSIDE_RAMP_CURRENT_OVERLOAD    34
/* Rightside Ramp Raise Timeout    (SIERRA) */
#define UAB_MAJOR_FAULT_RIGHTSIDE_RAMP_RAISE_TIMEOUT       35

#define UAB_EVENT_OPCODE 				    0x45  /* 'E' */
#define UAB_UPDATE_OPCODE 				    0x55  /* 'U' */

/* Events */
#define UAB_INIT_COMPLETE_EVENT             0x01
#define UAB_FAULT_CHANGE_EVENT              0x02
#define UAB_MAJOR_FAULT_EVENT               0x03
#define UAB_SWITCH_CHANGE_EVENT             0x04
#define UAB_LED_CHANGE_EVENT                0x05
#define UAB_IR_LEARN_EVENT                  0x06
#define UAB_CHARGER_STATE_CHANGE_EVENT      0x07
#define UAB_INST_MODE_CHANGE_EVENT          0x08
#define UAB_BRAKE_CHANGE_EVENT              0x09
#define UAB_PLATFORM_CHANGE_EVENT           0x0A
#define UAB_TRACK_ORIEN_CHANGE_EVENT        0x0B
#define UAB_RAMP_CHANGE_EVENT               0x0C

#define UAB_EVENTS_MAX                      0x20

/* Update Events */
#define UAB_VOLTS_CURRENT_UPDATE_EVENT      0x20

/* UAB Motion State Machine States */
typedef enum
{
    stopped,
    soft_start_down_right,
    cruise_down_right,
    soft_stop_down_right,
    start_down_holdoff,
    soft_start_up_left,
    cruise_up_left,
    soft_stop_up_left,
    start_up_holdoff 
}uabMotionState_t;

/* Motion Control Switch States */
typedef enum
{
    switch_off,
    switch_up_left,
    switch_down_right,
    switch_invalid
}uabUpDownLeftRightSwitchState_t;

/* Brake States */
typedef enum
{
    brake_off,
    brake_on
}uabBrakeState_t;

extern float    UabSense12Volts;
extern uint32_t UabMajorFaultCause;

/* Externs for UAB GLobal Variables */
extern uint8_t                 UabSierraPlatformMounting;
extern uint8_t                 UabSierraPlatformPosition;
extern uint8_t                 UabSierraLeftsideRampPosition;
extern uint8_t                 UabSierraRightsideRampPosition;
extern uint8_t                 UabSierraStartDownHoldoff;
extern uint8_t                 UabSierraStartUpHoldoff;
extern uint8_t                 UabMotorIsOff;
extern uint8_t                 UabInstMode;
extern uint8_t                 UabTripDone;
extern uint8_t                 UabMajorFault;
extern uint8_t                 UabUpDownReleased;
extern uint32_t                UabSystemId;
extern uint32_t                UabFaults;
extern uint32_t                UabDebugMode;
extern uabMotionState_t        UabMotionState;
extern uabBrakeState_t         UabBrakeState;
extern uabUpDownLeftRightSwitchState_t UabUpDownLeftRightSwitchState;
extern int8_t                  UabPwm;
extern uint8_t                 UabChargePercent;
extern uint8_t                 UabInitComplete;
extern uint8_t                 UabCommEnabled;
extern uint8_t                 UabTrackOrientation;
extern uint8_t                 UabLed;
extern uint8_t                 UabOffchargeFlag;
extern float                   UabSystemOverLoadAmps;
extern uint8_t                 PlatformRampChangeDelay;
extern uint8_t                 UabChargeLed;


/* Function Prototypes */
void uabEvent(uint8_t event, void *event_data, uint8_t event_data_len);
void uabFaultSet(uint32_t fault_bit, uint8_t fault_num);
void uabFaultClr(uint32_t fault_bit, uint8_t fault_num);
void uabInstModeCheck(uint32_t  pio0);
void uabMajorFaultHandler(uint8_t cause);
int  uab12vCheck(void);
void uabLowBatteryCheck(void);
int  uabVeryLowBatteryCheck(void);
int  uabSuperLowBatteryCheck(void);
void uabSystemOverloadCheck(void);
int  uabRunawayCheck(void);
void uabCpuLedOn(void);
void uabCpuLedOff(void);
void uabMDriveDisable(void);
void uabMDriveEnable(void);
void uabChargerDisable(void);
void uabChargerEnable(void);
void uabChargerInitDisable(void);
void uabChargerInitEnable(void);
void uabChargeCheck(void);
void uabVoltsCurrSenseDisplay(void);
void uabMotorVoltsDisplay(void);
uabUpDownLeftRightSwitchState_t uabUpDownLeftRightSwitchStateGet(uint32_t pio0);
void uabBrakeOff(void);
void uabBrakeOn(void);
void uabMotorOff(void);
void uabMotorOn(uint32_t direction);
void uabPwmInit(void);
void uabPwmChange(uint32_t direction, uint32_t duration);
int  uabPioGet(uint32_t *pio0_ptr, uint32_t *pio1_ptr, uint32_t *pio2_ptr);
uint32_t uabFaultsGet(uint32_t pio0, uint32_t pio2);
int  uabCheckStopDownRight(uint32_t pio2);
int  uabCheckStopUpLeft(uint32_t pio0);
int  uabCheckTopBottom(uint32_t pio0, uint32_t pio2);
void uabTopStop(void);
void uabBottomStop(void);
void uabMotionSet(uint32_t pio0, uint32_t pio2);
void uabCheckSwitch(uint32_t pio0,      uint32_t pio2,
                    uint32_t prev_pio0, uint32_t prev_pio2);
void uabCheckSwitchHelix(uint32_t pio0,
                         uint32_t pio1,
                         uint32_t pio2,
                         uint32_t prev_pio0,
                         uint32_t prev_pio1,
                         uint32_t prev_pio2);
void uabInit(void);
void uabDiagLed(uint8_t led_color);


#endif /* UAB_H_ */
