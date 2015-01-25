/*    Copyright (C) 2013 Hofer-Automation
 *
  *	  Author: Franz Flasch, Hofer Andreas
 *
 *    This file is part of Universal CNC-Controller AF-Series.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef __STEPGEN_H__
#define __STEPGEN_H__

#define MAXGEN	4

/*

#define disable_int()								\
	do {									\
		asm volatile("di");						\
		asm volatile("ehb");						\
	} while (0)

#define enable_int()								\
	do {									\
		asm volatile("ei");						\
	} while (0)

*/


typedef struct {
	int32_t velocity[MAXGEN];
} stepgen_input_struct;



//void stepgen(void *arg, long period);
void stepgen(void);
void stepgen_reset(void);
void stepgen_get_position(void *buf);
void stepgen_update_input(const void *buf);

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

//#define SYS_FREQ		(80000000ul)    // Hz
//#define GetSystemClock()	(SYS_FREQ)
//#define	GetPeripheralClock()	(GetSystemClock())
//#define	GetInstructionClock()	(GetSystemClock())

#define SPICHAN			2

/*
#define LED0_TRIS		TRISCbits.TRISC13
#define LED0_IO			LATCbits.LATC13

#define REQ_TRIS		TRISGbits.TRISG2
#define REQ_IO_IN		PORTGbits.RG2

#define AUX_TRIS		TRISCbits.TRISC14
#define AUX_IO_IN		PORTCbits.RC14

#define RDY_TRIS		TRISGbits.TRISG3
#define RDY_IO			LATGbits.LATG3
#define RDY_IO_0		(LATGCLR = _LATG_LATG3_MASK)
#define RDY_IO_1		(LATGSET = _LATG_LATG3_MASK)
*/

// Used OutputPorts

//if (BitVal != Bit_RESET)
//{
//  GPIOx->BSRRL = GPIO_Pin;
//}
//else
//{
//  GPIOx->BSRRH = GPIO_Pin ;
//}

#define STEPHI_X		GPIOC->BSRRL = GPIO_Pin_8 // Für High Bit
#define STEPLO_X		GPIOC->BSRRH = GPIO_Pin_8 // Für Low Bit
#define DIR_HI_X		GPIOC->BSRRL = GPIO_Pin_9
#define DIR_LO_X		GPIOC->BSRRH = GPIO_Pin_9

#define STEPHI_Y		GPIOC->BSRRL = GPIO_Pin_10
#define STEPLO_Y		GPIOC->BSRRH = GPIO_Pin_10
#define DIR_HI_Y		GPIOC->BSRRL = GPIO_Pin_11
#define DIR_LO_Y		GPIOC->BSRRH = GPIO_Pin_11

#define STEPHI_Z		GPIOC->BSRRL = GPIO_Pin_12
#define STEPLO_Z		GPIOC->BSRRH = GPIO_Pin_12
#define DIR_HI_Z		GPIOC->BSRRL = GPIO_Pin_13
#define DIR_LO_Z		GPIOC->BSRRH = GPIO_Pin_13

#define STEPHI_A		GPIOC->BSRRL = GPIO_Pin_14
#define STEPLO_A		GPIOC->BSRRH = GPIO_Pin_14
#define DIR_HI_A		GPIOC->BSRRL = GPIO_Pin_15
#define DIR_LO_A		GPIOC->BSRRH = GPIO_Pin_15

#endif				/* __STEPGEN_H__ */
