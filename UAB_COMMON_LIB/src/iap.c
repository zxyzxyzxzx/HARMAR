/*
******************************************************************************
** Filename:  iap.c
**
** Description:
**       This file contains functions used to access the LPC122x Info Page
**       NVRAM space. Data stored in this InfoPage will persist over a reset.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#include "LPC122x.h"
#include "main.h"
#include "utils.h"
#include "uab.h"
#include "iap.h"
#include "ir.h"

#define IAP_LOCATION         0x1FFF1FF1
#define IAP_CMD_ERASE_INFO   60
#define IAP_CMD_COPY2FLASH   51

typedef void (*IAP)(uint32_t command[], uint32_t result[]);

static const IAP iap_entry = (IAP) IAP_LOCATION;

/*
******************************************************************************
** Function:     readInfoPage32
**
** Description:  Reads Info Page data (uint32 size words) into callers buffer.
**
** Input  Parms: uint32_t* dest   Callers buffer ptr.
**               uint32_t  size   Number of bytes to read.
**
** Output Parms: uint32_t* dest   Callers buffer filled with data.
**
** Return Value: None
** 
******************************************************************************
*/
void readInfoPage32(uint32_t* dest, uint32_t size)
{
  uint32_t i;
  uint32_t* infoPtr = (uint32_t*) INFO_LOCATION;
  
  /* Bounds check */
  if (size>1536)
    size = 1536;
  
  /* use word aligned access for optimal performance */
  for ( i=0; i<size; i+=sizeof(uint32_t) )
  {
    *dest++ = *infoPtr++;
  }
}

/*
******************************************************************************
** Function:     writeInfoPage32
**
** Description:  Writes Info Page data (uint32 size words) from callers buffer.
**
** Input  Parms: uint32_t* pageptr   Ptr to starting location in Info Page
**               uint32_t* buffPtr   Callers buffer to be loaded
**               uint32_t  size      Number of bytes to write.
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void writeInfoPage32(uint32_t* pagePtr, uint32_t* buffPtr, uint32_t size)
{
  uint32_t command[5];
  uint32_t result[4];

  /* Bounds check */
  if (pagePtr < (uint32_t*) INFO_LOCATION)
    pagePtr = (uint32_t*) INFO_LOCATION;
  
  if (size>1536)
    size = 1536;
  
  command[0] = IAP_CMD_COPY2FLASH;	//Copy RAM to FLASH
  command[1] = (uint32_t)pagePtr;       //FLASH addr
  command[2] = (uint32_t)buffPtr;       //RAM addr
  command[3] = size & (~(0x3));         //Byte count (must be multiple of 4)
  command[4] = SystemCoreClock/1000;    //Freq [KHz]

  /*
  ** IAP code is contained inside the LPC122x ROM space.
  ** This code is provided by NXP and is not changeable.
  */
  iap_entry(command, result);
  if (result[0] != 0)
  {
      /* while(1); Just return anyway for now */
  }
}

/*
******************************************************************************
** Function:     eraseInfoPage
**
** Description:  Erases Info Pages
**
** Input  Parms: uint32_t startPage (0-2)
**               uint32_t endPage   (0-2)
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void eraseInfoPage(uint32_t startPage, uint32_t endPage)
{
  uint32_t command[4];
  uint32_t result[4];

  /* Bounds check */
  if (startPage < 0)
    startPage = 0;

  if (endPage > 2)
    endPage = 2;  
  
  command[0] = IAP_CMD_ERASE_INFO;	//Erase Info Page
  command[1] = startPage;               //Start
  command[2] = endPage;                 //Stop
  command[3] = SystemCoreClock/1000;    //Freq [KHz]
  
  /*
  ** IAP code is contained inside the LPC122x ROM space.
  ** This code is provided by NXP and is not changeable.
  */
  iap_entry(command, result);
  if (result[0] != 0)
  {
      /* while(1);    Just return anyway for now */
  }
}

/*
******************************************************************************
** Function:     uabInfoPageInit
**
** Description:  Initialize Info Page with UAB specific data.
**               The Software Major and Minor numbers are stored.
**               The IR Remote control Unit #s are stored and default
**               to Unit# 3 for both IR remotes.
**
** Input  Parms: sys_id      Harmar System ID
**               sw_maj_ver  Software Major Version
**               sw_min_ver  Software Minor Version
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabInfoPageInit(uint32_t sys_id,
		             uint32_t sw_maj_ver,
		             uint32_t sw_min_ver)
{
    uint32_t buff[UAB_INFO_PAGE_UINT32_MAX];

    readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
    buff[UAB_MAJ_VER_OFFSET]   = sw_maj_ver;
    buff[UAB_MIN_VER_OFFSET]   = sw_min_ver;
    buff[UAB_SYSTEM_ID_OFFSET] = sys_id;
    if ( (buff[UAB_IR0_UNIT_OFFSET]  > (uint32_t)(IR_UNIT3)) &&
         (buff[UAB_IR0_UNIT_OFFSET] != (uint32_t)(IR_UNIT_NOT_SET)) )
      buff[UAB_IR0_UNIT_OFFSET] = IR_UNIT3;
    if ( (buff[UAB_IR1_UNIT_OFFSET]  > (uint32_t)(IR_UNIT3)) &&
         (buff[UAB_IR1_UNIT_OFFSET] != (uint32_t)(IR_UNIT_NOT_SET)) )
      buff[UAB_IR1_UNIT_OFFSET] = IR_UNIT3;
    UabIr0UnitNum = buff[UAB_IR0_UNIT_OFFSET];
    UabIr1UnitNum = buff[UAB_IR1_UNIT_OFFSET];
    __disable_irq();
    eraseInfoPage(0, 0);
    __enable_irq();
    delayMs(200, BUSY_WAIT_BASED, TRUE);
    __disable_irq();
    writeInfoPage32( (uint32_t*) INFO_LOCATION, buff,
                     UAB_INFO_PAGE_BYTES_MAX );
    __enable_irq();
    delayMs(200, BUSY_WAIT_BASED, TRUE);
}

/*
******************************************************************************
** Function:     uabInfoPageGet
**
** Description:  Read only the 1st eleven uint32 words of the Info Page.
**              
** Input  Parms: uint32_t*  Ptr to eight uint32 caller buffer.
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabInfoPageGet(uint32_t* buff)
{
    readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
}

/*
******************************************************************************
** Function:     uabLoadInfoPageIrUnits
**
** Description:  If new IR Remote Unit#s are learned during IR Learning,
**               this function is called to update the Info Page data.
**              
** Input  Parms: None
**
** Output Parms: None
**
** Return Value: None
** 
******************************************************************************
*/
void uabLoadInfoPageIrUnits(void)
{
    uint32_t buff[UAB_INFO_PAGE_UINT32_MAX];

    readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
    buff[UAB_IR0_UNIT_OFFSET] = UabIr0UnitNum;
    buff[UAB_IR1_UNIT_OFFSET] = UabIr1UnitNum;

    __disable_irq();
    eraseInfoPage(0, 0);
    __enable_irq();
    delayMs(200, BUSY_WAIT_BASED, TRUE);
    __disable_irq();
    writeInfoPage32( (uint32_t*) INFO_LOCATION, buff,
                     UAB_INFO_PAGE_BYTES_MAX );
    __enable_irq();
    delayMs(200, BUSY_WAIT_BASED, TRUE);
}

#if 0    // Enable only for development debug

#include "uart.h"
#include "small_printf.h"

/*
******************************************************************************
** Function:     uabLoadInfoPageIrUnits
**
** Description:  If new IR Remote Unit#s are learned during IR Learning,
**               this function is called to update the Info Page data.
**
** Input  Parms: None
**
** Output Parms: None
**
** Return Value: None
**
******************************************************************************
*/
void printInfoPage(void)
{
    uint32_t buff[UAB_INFO_PAGE_UINT32_MAX];
    int i;

    readInfoPage32(buff, UAB_INFO_PAGE_BYTES_MAX);
    for (i = 0; i < UAB_INFO_PAGE_UINT32_MAX; i++)
    {
        if ((i % 8) == 0)
            printf("\n%02X:  %08X ", i , buff[i]);
        else
            printf("%08X ", buff[i]);
    }
    printf("\n");
}
#endif
