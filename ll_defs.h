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

#define	CHECKMARK_CRC	0x5757			// 16 bit crc checkmark
#define	MAGIC			0x0300  		// data version checkmark
#define LOW_VECTOR      0x18            // ISR low address
#define HIGH_VECTOR     0x8             // ISR high address

#define BAUD_FAST		41 // 375000 BPS is 41, high speed testing is 2
#define BAUD_SLOW		0

#define LCD_SHORT		0xfffe
#define LCD_LONG		0xff80

#define	PDELAY	0xff
#define	TIMEROFFSET		26474	// timer0 16bit counter value for 1 second to overflow
#define SLAVE_ACTIVE	10		// Activity counter level

	/* DIO defines */
#define LOW				(unsigned char)0        // digital output state levels, sink
#define	HIGH			(unsigned char)1        // digital output state levels, source
#define IN				HIGH
#define OUT				LOW
#define	ON				LOW       		//
#define OFF				HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states, relay is on when output gate is high
#define R_OFF           LOW				// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON		0xff
#define NO				LOW
#define YES				HIGH

#define DLED0			LATDbits.LATD0
#define DLED1			LATDbits.LATD1
#define DLED2			LATDbits.LATD2
#define DLED3			LATDbits.LATD3
#define DLED4			LATDbits.LATD4
#define DLED5			LATDbits.LATD5
#define DLED6			LATDbits.LATD4
#define DLED7			LATDbits.LATD5

#define BLED0			LATAbits.LATA2
#define BLED1			LATAbits.LATA3

#define SLED			LATEbits.LATE0
#define RS			LATEbits.LATE1
#define CSB			LATEbits.LATE2

#define EADOGM_CMD_CLR 1
#define EADOGM_CMD_CURSOR_ON     0b00001111
#define EADOGM_CMD_CURSOR_OFF    0b00001100
#define EADOGM_CMD_DISPLAY_ON    0b00001100
#define EADOGM_CMD_DISPLAY_OFF   0b00001000
#define EADOGM_CMD_DDRAM_ADDR    0b10000000
#define EADOGM_CMD_CGRAM_ADDR    0b01000000
#define EADOGM_CMD_SELECT_R0     0b00011000
#define EADOGM_CMD_SELECT_R1     0b00010000
#define EADOGM_CMD_SET_TABLE2    0b00101010
#define EADOGM_COLSPAN 16

#ifdef	__cplusplus
}
#endif

#endif	/* LL_DEFS_H */

