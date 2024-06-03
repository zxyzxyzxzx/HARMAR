/*
******************************************************************************
** Filename:  compadc.h
**
** Description:
**       This file defines mappings associated with the internal LPC1225
**       Comparator (COMP/CMP) and the Analog-to-Digital Converter (ADC).
**       Only Comparator 0 is used.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef COMPADC_H_
#define COMPADC_H_

/* Comparator 0 Input Select bits/mask */
#define CMP_SEL_MASK          ((1<<10) | (1<<9) | (1<<8))
#define CMP_LADDER_OUT_VOLTS  0
#define CMP_MOTOR_B_VOLTS     (1<<8)
#define CMP_MOTOR_A_VOLTS     (1<<9)
#define CMP_SENSE12_VOLTS     ((1<<8) | (1<<9))
#define CMP_CHARGE_VOLTS      (1<<10)

/* ADC Input Selects */
#define ADC_CURR_SENSE_VOLTS  1
#define ADC_BATTERY_VOLTS     2

/* Incremental Steps for Comparator and ADC */
#define CMP_STEP           31        /* 32-Step 0-31 */
#define ADC_STEP           1023      /* 1024-Step 0-1023 */

/* Reference Voltages determined by Hardware Design */
#define VCC33               3.30f
#define CMP_REF_V           3.30f
#define CHARGE_REF_V       36.30f
#define SENSE12_REF_V      13.70f
#define MOTOR_REF_V        33.33f
#define BATTERY_REF_V      36.30f
#define CURR_SENSE_REF_V    3.30f

/* ADC reg and bit defines */
#define ADC_CLK                 8000000    /* set to ADC 8Mhz */
#define ADC_OFFSET              0x10       /* index to starting addr of DR0~DR7 register in ADC */
#define ADC_INDEX               4
#define ADC_DONE                0x80000000
#define ADC_OVERRUN             0x40000000
#define ADC_ADINT               0x00010000

#define ADC_CURR_SENSE_SEL      0   /* ADC IN #0 */
#define ADC_BATTERY_SEL         1   /* ADC IN #1 */

/* Function Prototypes */
float uabVoltsConvert( uint32_t step, uint32_t volts_sel);
uint32_t uabAdcRead( uint32_t adcNum );
float uabCompVoltsRead(uint32_t cmpSel);
float uabAdcVoltsRead(uint32_t adcSel);
float uabAdcAmpsRead(void);
void  uabCompAdcInit(void);

#endif /* COMPADC_H_ */
