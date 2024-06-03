/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2024
 * (c) NXP Semiconductors 2013-2024
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v6.1.4 [Build 194] [2014-01-15]  on Mon Mar 25 18:03:14 EDT 2024)
*/

MEMORY
{
  /* Define each memory region */
  MFlash64 (rx) : ORIGIN = 0x0, LENGTH = 0x10000 /* 64K bytes */
  RamLoc8 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x2000 /* 8K bytes */


}
  /* Define a symbol for the top of each memory region */
  __top_MFlash64 = 0x0 + 0x10000;
  __top_RamLoc8 = 0x10000000 + 0x2000;



