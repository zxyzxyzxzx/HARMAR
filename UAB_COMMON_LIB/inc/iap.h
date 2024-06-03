/*
******************************************************************************
** Filename:  iap.h
**
** Description:
**     This file contains defines that are used to access and modify the
**     Info Page on the LPC1225.  The LPC1125 Info page is NVRAM and is
**     used to store info/data that must persist over a reset.
** 
** Copyright (C) 2014 Control Solutions LLC. All rights reserved.
******************************************************************************
*/

#ifndef IAP_H_
#define IAP_H_

#define INFO_LOCATION 0x00040200

#define UAB_INFO_PAGE_UINT32_MAX  11
#define UAB_INFO_PAGE_BYTES_MAX  (UAB_INFO_PAGE_UINT32_MAX * 4)

/* 32-bit Offsets into the Info Page for UAB NV Data */
#define UAB_MAJ_VER_OFFSET      0    /* Software Major Version */
#define UAB_MIN_VER_OFFSET      1    /* Software Minor Version */
#define UAB_SYSTEM_ID_OFFSET    2    /* System ID */
/* 2 Reserved */
#define UAB_HW_VER_OFFSET       4    /* HW Version (like 0x20 0x20 0x20 'A') */
#define UAB_IR0_UNIT_OFFSET     5    /* IR0 Unit# (0-3) */
#define UAB_IR1_UNIT_OFFSET     6    /* IR1 Unit# (0-3) */
/* 7 Reserved */
/* UAB Serial # 12-bytes 8-10 */
#define UAB_SERIAL_NUM_OFFSET   8  /* (like 0x20 20 20 20 */
                                   /*       'S' 31 32 33  */
                                   /*        34 35 36 37) */

/* Function Prototypes */
/* use word aligned access for optimal performance */
void readInfoPage32(uint32_t* dest, uint32_t size);
void writeInfoPage32(uint32_t* pagePtr, uint32_t* buffPtr, uint32_t size);
void eraseInfoPage(uint32_t startPage, uint32_t endPage);
void uabInfoPageInit(uint32_t sys_id,
		             uint32_t sw_maj_ver,
		             uint32_t sw_min_ver);
void uabInfoPageGet(uint32_t* buff);
void uabLoadInfoPageIrUnits(void);
void printInfoPage(void);

#endif  /* IAP_H_ */
