/*
******************************************************************************
** Filename:  ir.h
**
** Description:
**       This file defines mappings associated with the Infrared Remote(IR)
**       control data patterns and unit #s.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef IR_H_
#define IR_H_

/* IR 26-bit Data patterns decoded to indicate Up/Down button pressings */ 
#define IR_UNIT0_UP_A   0xFD2D3354
#define IR_UNIT0_UP_B   0xFD4D3354
#define IR_UNIT1_UP_A   0xFD2D2B54
#define IR_UNIT1_UP_B   0xFD4D2B54
#define IR_UNIT2_UP_A   0xFD2CD354
#define IR_UNIT2_UP_B   0xFD4CD354
#define IR_UNIT3_UP_A   0xFD2CCB54
#define IR_UNIT3_UP_B   0xFD4CCB54
#define IR_UNIT0_DOWN_A 0xFD2D34CA
#define IR_UNIT0_DOWN_B 0xFD4D34CA
#define IR_UNIT1_DOWN_A 0xFD2D2CCA
#define IR_UNIT1_DOWN_B 0xFD4D2CCA
#define IR_UNIT2_DOWN_A 0xFD2CD4CA
#define IR_UNIT2_DOWN_B 0xFD4CD4CA
#define IR_UNIT3_DOWN_A 0xFD2CCCCA
#define IR_UNIT3_DOWN_B 0xFD4CCCCA

/* IR Unit Numbers */
#define IR_UNIT0        0
#define IR_UNIT1        1
#define IR_UNIT2        2
#define IR_UNIT3        3
#define IR_UNIT_NOT_SET 0x000000FF
#define IR_UNIT_UNKNOWN 0xFFFFFFFF

/* PIO Port bit for IR Input */
#define IR_IN (1<<6)

/* IR Motion control states */
typedef enum
{
    irOff,
    irUpLeft,
    irDownRight,
    irInvalid
}uabIrState_t;

/* Externs for Global Variables */
extern uabIrState_t UabIrState;
extern uint32_t UabIr0UnitNum;
extern uint32_t UabIr1UnitNum;
extern uint32_t IrPwp[26];
extern uint8_t  IrPwpProcessed;
extern uint8_t  IrIntrDetected;
extern uint8_t  IrLearning;

/* Function Prototypes */
void PIOINT1_IRQHandler (void);
void uabIrInit(void);
uint32_t uabIrPatternGet(void);
uabIrState_t uabIrStateGet(uint32_t ir_pattern);

#endif /* IR_H_ */
