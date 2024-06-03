//*****************************************************************************
//   +--+
//   | ++----+
//   +-++    |
//     |     |
//   +-+--+  |
//   | +--+--+
//   +----+    Copyright (c) 2011 Code Red Technologies Ltd.
//
// aeabi_romdiv_patch.s
//  - Provides "patch" versions of the aeabi integer divide functions to
//    replace the standard ones pulled in from the C library, which vector
//    integer divides onto the Cortex-M0 rom division functions contained
//    in the LPC12xx library rom.
//  - Note that this patching will only occur if "__USE_ROMDIVIDE" is
//    defined for the project build for both the compiler and assembler.
//
// Software License Agreement
//
// The software is owned by Code Red Technologies and/or its suppliers, and is
// protected under applicable copyright laws.  All rights are reserved.  Any
// use in violation of the foregoing restrictions may subject the user to criminal
// sanctions under applicable laws, as well as to civil liability for the breach
// of the terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// USE OF THIS SOFTWARE FOR COMMERCIAL DEVELOPMENT AND/OR EDUCATION IS SUBJECT
// TO A CURRENT END USER LICENSE AGREEMENT (COMMERCIAL OR EDUCATIONAL) WITH
// CODE RED TECHNOLOGIES LTD.
//
//*****************************************************************************

#if defined(__USE_ROMDIVIDE)

// Note that the romdivide "divmod" functions are not actually called from
// the below code, as these functions are actually just wrappers to the
// main romdivide "div" functions which push the quotient and remainder onto
// the stack, so as to be compatible with the way that C returns structures.
//
// This is not needed for the aeabi "divmod" functions, as the compiler
// automatically generates code that handles the return values being passed
// back in registers when it generates inline calls to __aeabi_idivmod and
// __aeabi_uidivmod routines.

	.syntax unified
	.text

// ========= __aeabi_idiv &  __aeabi_idivmod =========
	.align 2
	.section .text.__aeabi_idiv

	.global	__aeabi_idiv
	.set __aeabi_idivmod, __aeabi_idiv   // make __aeabi_uidivmod an alias
	.global __aeabi_idivmod
	.global pDivRom_idiv   // pointer to the romdivide 'idiv' functione
    .func
	.thumb_func
	.type	__aeabi_idiv, %function

__aeabi_idiv:
	push	{r4, lr}
	ldr	r3, =pDivRom_idiv
	ldr	r3, [r3, #0]     // Load address of function
	blx	r3               // Call divide function
	pop	{r4, pc}

	.endfunc

// ========  __aeabi_uidiv &  __aeabi_uidivmod ========
	.align 2

	.section .text.__aeabi_uidiv

	.global	__aeabi_uidiv
	.set __aeabi_uidivmod, __aeabi_uidiv  // make __aeabi_uidivmod an alias
	.global __aeabi_uidivmod
	.global pDivRom_uidiv	// pointer to the romdivide 'uidiv' function
    .func
	.thumb_func
	.type	__aeabi_uidiv, %function

__aeabi_uidiv:
	push	{r4, lr}
	ldr	r3, =pDivRom_uidiv
	ldr	r3, [r3, #0]      // Load address of function
	blx	r3               // Call divide function
	pop	{r4, pc}

	.endfunc

#endif // (__USE_ROMDIVIDE)
