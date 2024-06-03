/*
******************************************************************************
** Filename:  uart.h
**
** Description:
**       This file defines UART HW register bit mappings, UART Port#s,
**       and default Baud Rate for UART0 and UART1 comm ports.
** 
**       printf() is defined to use the floating point printf function
**       from the lib_small_printf_m0 library. This is necessary since
**       much of the debug output displays Volts or Amps.
**       printf() is supported for only UART11 Debug port.
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef UART_H_ 
#define UART_H_

/* UART Build Options */
#define RS485_ENABLED   0
#define TX_INTERRUPT    0   /* 0 if TX uses polling, 1 interrupt driven. */
#define MODEM_TEST      0

/* UART Register Bit Defines */
#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)

#define IIR_PEND        0x01
#define IIR_RLS         0x03
#define IIR_RDA         0x02
#define IIR_CTI         0x06
#define IIR_THRE        0x01

#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

#define BUFSIZE         0x40   /* Max UART buffer size is 64 bytes */

#define UART0           0
#define UART1           1

#define UART_9600_BAUD     9600
#define UART_115200_BAUD 115200

#define ACK                0x81
#define NAK                0x84

#define MAX_XMIT_MSG_SIZE  40   /* bytes */
#define MAX_XMIT_BUFS      40

typedef struct
{
  uint8_t  inuse;
  uint8_t  length;
  uint8_t  seq;
  uint8_t  msg[MAX_XMIT_MSG_SIZE];
}XmitFifoBufType;

/* Extern UART Global Variables */
extern volatile uint32_t UART0_Status;
extern volatile uint8_t  UARTT0_xEmpty;
extern volatile uint8_t  UART0_Buffer[BUFSIZE];
extern volatile uint32_t UART0_Count;
extern uint8_t UART0_XmitBuf[BUFSIZE];

extern volatile uint32_t UART1_Status;
extern volatile uint8_t  UART1_TxEmpty;
extern volatile uint8_t  UART1_Buffer[BUFSIZE];
extern volatile uint32_t UART1_Count;
extern uint8_t UART1_XmitBuf[BUFSIZE];

extern uint8_t XmitErrDataLen;
extern uint8_t XmitErrFifoOveflow;
extern uint8_t XmitMsgNum;
extern uint8_t XmitSendDelay;
extern uint8_t XmitMsgRetryCnt;
extern uint8_t XmitFifoBufNum;
extern XmitFifoBufType XmitFifoBuf[MAX_XMIT_BUFS];
extern uint8_t WaitForAck;
extern uint8_t WaitForAckTimeout;

/* External functions */
extern int putchar (char c);
#define printf(...) func_printf_float (putchar, __VA_ARGS__)

/* Function Prototypes */
void UARTInit(uint32_t portNum, uint32_t Baudrate);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UARTSend(uint32_t portNum, uint8_t *BufferPtr, uint32_t Length);
void xmitFifoBufSend(void);
int  waitForAck(void);
void xmitFifoBufLoad(uint8_t *msg, uint8_t len, uint8_t seq);

#endif /* UART_H_ */

