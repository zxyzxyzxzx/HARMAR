/*
******************************************************************************
** Filename:  uart.c
**
** Description:
**       This file contains code to support UART0 and UART1 on the
**       microprocessor.  This code was taken directly from a supplied
**       example of UART driver code for LPC122x microprocessors.
**
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include <string.h>
#include "LPC122x.h"
#include "main.h"
#include "uab.h"
#include "uart.h"
#include "small_printf.h"

//#define UART_DEBUG   /* Never Enable for SIERRA */

/* UART Global Variables */
volatile uint32_t UART0_Status = 0;
volatile uint8_t  UART0_TxEmpty = 1;
volatile uint8_t  UART0_Buffer[BUFSIZE];
volatile uint32_t UART0_Count = 0;
uint8_t UART0_XmitBuf[BUFSIZE];

volatile uint32_t UART1_Status = 0;
volatile uint8_t  UART1_TxEmpty = 1;
volatile uint8_t  UART1_Buffer[BUFSIZE];
volatile uint32_t UART1_Count = 0;
uint8_t UART1_XmitBuf[BUFSIZE];

uint8_t XmitErrDataLen     = 0;
uint8_t XmitErrFifoOveflow = 0;

uint8_t XmitMsgNum         = 0;
uint8_t XmitSendDelay      = 0;
uint8_t XmitMsgRetryCnt    = 0;
uint8_t XmitFifoBufNum     = 0;
XmitFifoBufType XmitFifoBuf[MAX_XMIT_BUFS];

uint8_t WaitForAck = FALSE;
uint8_t WaitForAckTimeout = 0;

uint32_t AckCntr     = 0;
uint32_t NakCntr     = 0;
uint32_t OtherCntr   = 0;
uint8_t  NakRetryCnt = 0;
uint8_t  NoCommCnt   = 0;

/*
******************************************************************************
** Function:     UART0_IRQHandler
**
** Description:  UART0 Interrupt Handler
**               UART 0 is used as a Debug comm port to PC/Laptop.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void UART0_IRQHandler(void)
{
    uint8_t IIRValue;
    uint8_t LSRValue;
    uint8_t Dummy = Dummy;

    IIRValue = LPC_UART0->IIR;
    
    IIRValue >>= 1;     /* skip pending bit in IIR */
    IIRValue &= 0x07;   /* check bit 1~3, interrupt identification */
    if (IIRValue == IIR_RLS)    /* Receive Line Status */
    {
        LSRValue = LPC_UART0->LSR;
        /* Receive Line Status */
        if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
        {
            /* There are errors or break interrupt */
            /* Read LSR will clear the interrupt */
            UART0_Status = LSRValue;
            Dummy = LPC_UART0->RBR;
            /* Dummy read on RX to clear interrupt, then bail out */
            return;
        }
        if (LSRValue & LSR_RDR)     /* Receive Data Ready */
        {
            /* If no error on RLS, normal ready, save into the data buf */
            /* Note: read RBR will clear the interrupt */
            UART0_Buffer[UART0_Count++] = LPC_UART0->RBR;
            if (UART0_Count == BUFSIZE)
            {
                UART0_Count = 0;    /* buffer overflow */
            } 
        }
    }
    else if (IIRValue == IIR_RDA) /* Receive Data Available */
    {   /* Receive Data Available */
        UART0_Buffer[UART0_Count++] = LPC_UART0->RBR;
        if (UART0_Count == BUFSIZE)
        {
            UART0_Count = 0;       /* buffer overflow */
        }
    }
    else if (IIRValue == IIR_CTI) 
    {   /* Character Time-out indicator */
        UART0_Status |= 0x100;     /* Bit 9 as the CTI error */
    }
    else if (IIRValue == IIR_THRE)  /* THRE, transmit holding reg empty */
    {   /* THRE interrupt */
        LSRValue = LPC_UART0->LSR;  /* Check status in the LSR to see if
                                       valid data in U0THR or not */
        if (LSRValue & LSR_THRE)
        {
            UART0_TxEmpty = 1;
        }
        else
        {
            UART0_TxEmpty = 0;
        }
    }
    return;
}

/*
******************************************************************************
** Function:     UART1_IRQHandler
**
** Description:  UART1 Interrupt Handler
**               UART 1 is currently not used.
**               May be used in future to provide communication link
**               to the Ramp Board on a Sierra stair lift system.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void UART1_IRQHandler(void)
{
    uint8_t IIRValue;
    uint8_t LSRValue;
    uint8_t Dummy = Dummy;

    IIRValue = LPC_UART1->IIR;

    IIRValue >>= 1;        /* skip pending bit in IIR */
    IIRValue &= 0x07;      /* check bit 1~3, interrupt identification */
    if (IIRValue == IIR_RLS)     /* Receive Line Status */
    {
        LSRValue = LPC_UART1->LSR;
        /* Receive Line Status */
        if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
        {
            /* There are errors or break interrupt */
            /* Read LSR will clear the interrupt */
            UART1_Status = LSRValue;
            Dummy = LPC_UART1->RBR;   /* Dummy read on RX to clear
                                         interrupt, then bail out */
            return;
        }
        if (LSRValue & LSR_RDR)     /* Receive Data Ready */
        {
            /* If no error on RLS, normal ready, save into the data buf. */
            /* Note: read RBR will clear the interrupt */
            UART1_Buffer[UART1_Count++] = LPC_UART1->RBR;
            if (UART1_Count == BUFSIZE)
            {
                UART1_Count = 0;     /* buffer overflow */
            }
        }
    }
    else if (IIRValue == IIR_RDA) /* Receive Data Available */
    {   /* Receive Data Available */
        UART1_Buffer[UART1_Count++] = LPC_UART1->RBR;
        if (UART1_Count == BUFSIZE)
        {
            UART1_Count = 0;         /* buffer overflow */
        }
    }
    else if (IIRValue == IIR_CTI)
    {   /* Character Time-out indicator */
        UART1_Status |= 0x100;       /* Bit 9 as the CTI error */
    }
    else if (IIRValue == IIR_THRE)  /* THRE, transmit holding reg empty */
    {   /* THRE interrupt */
        LSRValue = LPC_UART1->LSR;  /* Check status in the LSR to see if
                                       valid data in U0THR or not */
        if (LSRValue & LSR_THRE)
        {
            UART1_TxEmpty = 1;
        }
        else
        {
            UART1_TxEmpty = 0;
        }
    }
    return;
}

/*
******************************************************************************
** Function:     UARTInit
**
** Description:  Initializes UART0 or UART1 port.
**                 
**               UART0 configured as:
**                  DiagGUI Comm UART0 RXD on PIO0_1 pin
**                  DiagGUI Comm UART0 TXD on PIO0_2 pin
**                  Baud Rate: 15200
**                  8-bits
**                  No Parity
**                  1 Stop Bit
**                 
**                  Only RX interrupts.
**                  
**               UART1 on SIERRA
**                  UAB Ramp Board Comm UART1 RXD on PIO2_12 pin
**                  UAB Ramp Board Comm UART1 TXD on PIO2_13 pin
**                  Baud Rate: 9600
**                  8-bits
**                  No Parity
**                  1 Stop Bit
**
**                  Only RX interrupts.
**
**               UART1 on PINNACLE & TAMA
**                  Dev Debug Comm UART1 RXD on PIO2_12 pin
**                  Dev Debug Comm UART1 TXD on PIO2_13 pin
**                  Baud Rate: 115200
**                  8-bits
**                  No Parity
**                  1 Stop Bit
**
**                  Only RX interrupts.
**
**
** Input  Parms: uint32_t portNum  (0 or 1)
**               uint32_t baudrate (9600 or 115200)
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void UARTInit(uint32_t portNum, uint32_t baudrate)
{
    uint32_t Fdiv;
    uint32_t regVal;
    uint32_t i;

    if ( portNum == 0 )
    {
        UART0_TxEmpty = 1;
        UART0_Count = 0;
        for ( i = 0; i < BUFSIZE; i++ )
        {
            UART0_Buffer[i] = 0;
        }
        for (i = 0; i < MAX_XMIT_BUFS; i++)
           memset(&XmitFifoBuf[i], 0, sizeof(XmitFifoBufType));

        NVIC_DisableIRQ(UART0_IRQn);

        /* UAB Debug UART0 is on PIO0_1 and PIO0_2 */
        LPC_IOCON->PIO0_1 &= ~0x07;    /* UART0 I/O config */
        LPC_IOCON->PIO0_1 |= 0x02;     /* UART0 RXD LOC0 */
        LPC_IOCON->PIO0_2 &= ~0x07;
        LPC_IOCON->PIO0_2 |= 0x02;     /* UART0 TXD LOC0 */

        /* Enable UART 0 clock */
        LPC_SYSCON->PRESETCTRL |= (0x1<<2);
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<12);
        LPC_SYSCON->UART0CLKDIV = 0x1;     /* divided by 1 */

        LPC_UART0->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
        regVal = LPC_SYSCON->UART0CLKDIV;
        Fdiv = ((SystemCoreClock/regVal)/16)/baudrate ; /* baud rate */

        LPC_UART0->DLM = Fdiv / 256;
        LPC_UART0->DLL = Fdiv % 256;
        LPC_UART0->LCR = 0x03;          /* DLAB = 0 */
        LPC_UART0->FDR = 0x10;          /* set to default value: 0x10 */
        LPC_UART0->FCR = 0x07;          /* Enable and reset TX and RX FIFO. */

        /* Read to clear the line status. */
        regVal = LPC_UART0->LSR;

        /* Ensure a clean start, no data in either TX or RX FIFO. */
        while ((LPC_UART0->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT));
        while ( LPC_UART0->LSR & LSR_RDR )
        {
            regVal = LPC_UART0->RBR;      /* Dump data from RX FIFO */
        }

        /* Enable the UART Interrupt */
        NVIC_EnableIRQ(UART0_IRQn);

#if TX_INTERRUPT
        LPC_UART0->IER = IER_RBR | IER_THRE | IER_RLS;  /* En UART RX/TX intr */
#else
        LPC_UART0->IER = IER_RBR | IER_RLS;     /* Enable UART RX intr */
#endif
    }
    else
    {
        UART1_TxEmpty = 1;
        UART1_Count   = 0;
        for ( i = 0; i < BUFSIZE; i++ )
        {
            UART1_Buffer[i] = 0;
        }  
        NVIC_DisableIRQ(UART1_IRQn);

        /* UAB Ramp Board UART1 is on PIO2_12 and PIO2_13 */
        LPC_IOCON->PIO2_12 &= ~0x07;    /* UART1 I/O config */
        LPC_IOCON->PIO2_12 |= 0x03;     /* UART RXD LOC1 */
        LPC_IOCON->PIO2_13 &= ~0x07;
        LPC_IOCON->PIO2_13 |= 0x03;     /* UART TXD LOC1 */

        /* Enable UART 1 clock */
        LPC_SYSCON->PRESETCTRL |= (0x1<<3);
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<13);
        LPC_SYSCON->UART1CLKDIV = 0x1;     /* divided by 1 */

        LPC_UART1->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
        regVal = LPC_SYSCON->UART1CLKDIV;
        Fdiv = ((SystemCoreClock/regVal)/16)/baudrate ; /* baud rate */

        LPC_UART1->DLM = Fdiv / 256;
        LPC_UART1->DLL = Fdiv % 256;
        LPC_UART1->LCR = 0x03;          /* DLAB = 0 */
        LPC_UART1->FDR = 0x10;          /* set to default value: 0x10 */
        LPC_UART1->FCR = 0x07;          /* Enable and reset TX and RX FIFO. */

        /* Read to clear the line status. */
        regVal = LPC_UART1->LSR;

        /* Ensure a clean start, no data in either TX or RX FIFO. */
        while ((LPC_UART1->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT));
        while ( LPC_UART1->LSR & LSR_RDR )
        {
            regVal = LPC_UART1->RBR;      /* Dump data from RX FIFO */
        }

        /* Enable the UART Interrupt */
        NVIC_EnableIRQ(UART1_IRQn);

#if TX_INTERRUPT
        LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;  /* En UART RX/TX intr */
#else
        LPC_UART1->IER = IER_RBR | IER_RLS;     /* Enable UART RX intr */
#endif
    }
    return;
}

/*
******************************************************************************
** Function:     UARTSend
**
** Description:  Sends a block of data to UART0 or UART1 port.
**               The callers buffer pointer and length in bytes are
**               passed to function.
**
** Input  Parms: uint32_t portNum  (0 or 1)
**               uint8_t  *BufferPtr (callers buffer ptr)
**               uin32_t  Length in bytes to send.
** 
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void UARTSend(uint32_t portNum, uint8_t *BufferPtr, uint32_t Length)
{
    while ( Length != 0 )
    {
        if ( portNum == 0 )
        {
            /* THRE status, contain valid data */
#if !TX_INTERRUPT
            while ( !(LPC_UART0->LSR & LSR_THRE) );
            LPC_UART0->THR = *BufferPtr;
#else
            /* Below flag is set inside the intr handler when THRE occurs. */
            while ( !(UARTTxEmpty0 & 0x01) );
            LPC_UART0->THR = *BufferPtr;
            UARTTxEmpty0 = 0; /* not empty in the THR until it shifts out */
#endif
        }
        else
        {
            /* THRE status, contain valid data */
#if !TX_INTERRUPT
            while ( !(LPC_UART1->LSR & LSR_THRE) );
            LPC_UART1->THR = *BufferPtr;
#else
            /* Below flag is set inside the intr handler when THRE occurs. */
            while ( !(UARTTxEmpty1 & 0x01) );
            LPC_UART1->THR = *BufferPtr;
            UARTTxEmpty1 = 0; /* not empty in the THR until it shifts out */
#endif 
        }
        BufferPtr++;
        Length--;
    }
    return;
}

/*
******************************************************************************
** Function:     putchar
**
** Description:  Outputs a char to UART1 Debug Comm port
**               Cannot be used on SIERRA
**
** Input  Parms: c : char to output
** Output Parms: None
**
** Return Value: 0
**
******************************************************************************
*/
int putchar (char c)
{
    /* THRE status, contain valid data */
    while ( !(LPC_UART1->LSR & LSR_THRE) );
    LPC_UART1->THR = c;
    return 0;
}

/*
******************************************************************************
** Function:     xmitFifoBufSend
**
** Description:  Sends the next message stored in the Xmit FIFO Buffers
**               to the  DiagGUI program running on the external computer.
**               Only one message is sent every 10ms scan.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void xmitFifoBufSend(void)
{
    if (XmitFifoBuf[XmitMsgNum].inuse)
    {
        UART0_Count     = 0;
        UART0_Buffer[0] = 0;
        UART0_Buffer[1] = 0;
        WaitForAck = TRUE;
        WaitForAckTimeout = 50;

        UARTSend(UART0,
                 XmitFifoBuf[XmitMsgNum].msg,
                 XmitFifoBuf[XmitMsgNum].length);
    }
}

/*
******************************************************************************
** Function:     waitForAck
**
** Description:  For every message sent to the DiagGUI the UAB code expects
**               an Acknowledge (ACK) or a Neg Acknowledge (NAK) to be
**               returned by the DiagGUI software.
**               If a NAK is returned, the message will be resent up to
**               4 times.  After 4 failed attempts, the message will be
**               discarded.  If no ACK or NAK is received after 5 different
**               messages are sent,  then communication with the DiagGUI on
**               the external computer is terminated.  No further messages  toSends the next message stored in the Xmit FIFO Buffers
**               to the DiagGUI will be sent.  The DiagGUI must reset the UAB
**               to re-establish DiagGUI communication.
**
** Input  Parms: None
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
int waitForAck(void)
{
    if (UART0_Count == 1)
    {
    	NoCommCnt = 0;
        if (UART0_Buffer[0] == ACK)
        {
        	AckCntr++;
#if defined(UART_DEBUG)
        	if ((AckCntr % 100) == 0)
        	  printf("A: %3d   %6d %6d %6d\n",
        		  	  XmitFifoBuf[XmitMsgNum].seq,
        			  AckCntr, NakCntr, OtherCntr);
#endif
        	NakRetryCnt = 0;
            WaitForAck = FALSE;
            XmitFifoBuf[XmitMsgNum].inuse = FALSE;
            XmitFifoBuf[XmitMsgNum].length = 0;
            XmitFifoBuf[XmitMsgNum].seq    = 0;
            if (++XmitMsgNum >= MAX_XMIT_BUFS)
              XmitMsgNum = 0;
            return(0);
        }
        else
        {
            if (UART0_Buffer[0] == NAK)
            {
                NakCntr++;
#if defined(UART_DEBUG)
            	printf("N: %3d   %6d %6d %6d   !!!!!!\n",
                       XmitFifoBuf[XmitMsgNum].seq,
                       AckCntr, NakCntr, OtherCntr);
            	printf("   %02X  %02X  %02X  %02X\n",
                       XmitFifoBuf[XmitMsgNum].msg[0],
                       XmitFifoBuf[XmitMsgNum].msg[1],
                       XmitFifoBuf[XmitMsgNum].msg[2],
                       XmitFifoBuf[XmitMsgNum].msg[3] );
#endif
            }
            else
            {
                OtherCntr++;
#if defined(UART_DEBUG)
            	printf("O: %3d   %6d %6d %6d   !!!!!!\n",
                       XmitFifoBuf[XmitMsgNum].seq,
                       AckCntr, NakCntr, OtherCntr);
#endif
            }
            if (++NakRetryCnt < 4)
            {
#if defined(UART_DEBUG)
            	printf("NAK Retry %1d:  Seq#: %3d\n",
                    	NakRetryCnt, XmitFifoBuf[XmitMsgNum].seq);
#endif
                xmitFifoBufSend();
            }
            else
            {
#if defined(UART_DEBUG)
            	printf("NAK Retry %1d:  Seq#: %3d .... Give Up\n",
                    	NakRetryCnt, XmitFifoBuf[XmitMsgNum].seq);
#endif
            	NakRetryCnt = 0;
                WaitForAck = FALSE;
                XmitFifoBuf[XmitMsgNum].inuse  = FALSE;
                XmitFifoBuf[XmitMsgNum].length = 0;
                XmitFifoBuf[XmitMsgNum].seq    = 0;
                if (++XmitMsgNum >= MAX_XMIT_BUFS)
                  XmitMsgNum = 0;
                return(1);
            }
        }
    }
    else
    {
    	if (NoCommCnt > 5)
    	{
#if defined(UART_DEBUG)
            printf("!!!  No ACK/NAK for last 5 Messages  !!!\n");
            printf("!!!  Disabling DiagGUI Communication  !!!\n");
#endif
            UabCommEnabled = FALSE;
            WaitForAck = FALSE;
            return(2);
    	}

        if (WaitForAckTimeout > 0)
           WaitForAckTimeout--;
        else
        {
            if (++XmitMsgRetryCnt < 4)
            {
#if defined(UART_DEBUG)
                printf("Wait for ACK Retry %1d:  Seq#: %3d\n",
                    	XmitMsgRetryCnt, XmitFifoBuf[XmitMsgNum].seq);
#endif
                xmitFifoBufSend();
            }
            else
            {
            	NoCommCnt++;
#if defined(UART_DEBUG)
            	printf("Wait for ACK Retry %1d:  Seq#: %3d .... Give Up\n",
                    	XmitMsgRetryCnt, XmitFifoBuf[XmitMsgNum].seq);
#endif
                XmitMsgRetryCnt = 0;
            	NakRetryCnt = 0;
                WaitForAck = FALSE;
                XmitFifoBuf[XmitMsgNum].inuse  = FALSE;
                XmitFifoBuf[XmitMsgNum].length = 0;
                XmitFifoBuf[XmitMsgNum].seq    = 0;
                if (++XmitMsgNum >= MAX_XMIT_BUFS)
                  XmitMsgNum = 0;
                return(1);
            }
        }
    }
    return(2);
}

/*
******************************************************************************
** Function:     xmitFifoBufLoad
**
** Description:  Loads a message to be sent to the DiagGUI on the external
**               computer into the next available Xmit FIFO buffer.
**
** Input  Parms: *msg   Ptr to maximum 40-byte DiagGUI message
**               len    Total Length in bytes of the message
**               seq    Sequence # of the message
**                         Each new message will have a new Seq #.
**                         Retried messages will have the same Seq # as
**                         the original message.
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void xmitFifoBufLoad(uint8_t *msg, uint8_t len, uint8_t seq)
{
    if (XmitFifoBuf[XmitFifoBufNum].inuse)
    {
#if defined(UART_DEBUG)
    	printf("!!!!  XMIT FIFO OVERFLOW  !!!!\n");
#endif
        XmitErrFifoOveflow++;
    }
    else
    {
        memset(XmitFifoBuf[XmitFifoBufNum].msg, 0, MAX_XMIT_MSG_SIZE);
        memcpy(XmitFifoBuf[XmitFifoBufNum].msg, msg, len);
        XmitFifoBuf[XmitFifoBufNum].length = len;
        XmitFifoBuf[XmitFifoBufNum].inuse = TRUE;
        XmitFifoBuf[XmitFifoBufNum].seq   = seq;
        if (++XmitFifoBufNum >= MAX_XMIT_BUFS)
          XmitFifoBufNum = 0;
    }
}

/*
******************************************************************************
** Function:     aftof
**
** Description:  Converts an ASCII Float Num string to Float
**
** Input  Parms: num : ASCII Float Num string to convert
** Output Parms: None
**
** Return Value: float value
**
******************************************************************************
*/
float aftof(char* num)
{
    float integerPart = 0;
    float fractionPart = 0;
    int   divisorForFraction = 1;
    uint8_t inFraction = FALSE;

     if (!num || !*num)
       return 0.0;

     while (*num != '\0')
     {
         if ( (*num >= '0') && (*num <= '9') )
         {
             if (inFraction)
             {
                 /*See how are we converting a character to integer*/
                 fractionPart = (fractionPart*10) + (*num - '0');
                 divisorForFraction *= 10;
             }
             else
             {
                 integerPart = (integerPart*10) + (*num - '0');
             }
         }
         else if (*num == '.')
         {
             if (inFraction)
                 return(integerPart + (fractionPart/divisorForFraction));
             else
                 inFraction = TRUE;
         }
         else
         {
             return(integerPart + (fractionPart/divisorForFraction));
         }
         ++num;
     }
     return(integerPart + (fractionPart/divisorForFraction));
 }

