/* 
 * File:   ll_defs.h
 * Author: root
 *
 * Created on July 20, 2015, 3:10 PM
 */

#ifndef LL_DEFS_H
#define	LL_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define LOW_VECTOR      0x18            // ISR low address
#define HIGH_VECTOR     0x8             // ISR high address
	
#define BAUD_FAST	2 // 375000 BPS is 41
#define BAUD_SLOW	0
	
#define LCD_SHORT	0xfffe
#define LCD_LONG	0xffb0
	
#define	TIMEROFFSET	26474           // timer0 16bit counter value for 1 second to overflow
#define SLAVE_ACTIVE	10		// Activity counter level

	/* DIO defines */
#define LOW		(unsigned char)0        // digital output state levels, sink
#define	HIGH            (unsigned char)1        // digital output state levels, source
#define	ON		LOW       		//
#define OFF		HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states, relay is on when output gate is high, uln2803,omron relays need the CPU at 5.5vdc to drive
#define R_OFF           LOW			// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON	0xff
#define NO		LOW
#define YES		HIGH

#define DLED0		LATCbits.LATC0
#define DLED1		LATCbits.LATC1
#define DLED2		LATCbits.LATC1
#define DLED3		LATCbits.LATC1
#define DLED4		LATCbits.LATC1
#define DLED5		LATCbits.LATC1
#define DLED6		LATCbits.LATC1
#define DLED7		LATCbits.LATC1

#define SLED        LATEbits.LATE0
#define RS          LATEbits.LATE1
#define CSB         LATEbits.LATE2
    
#ifdef	__cplusplus
}
#endif

#endif	/* LL_DEFS_H */

