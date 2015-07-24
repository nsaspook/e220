#include "ll_vector.h"

//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma	tmpdata	ISRHtmpdata
#pragma interrupt data_handler   nosave=section (".tmpdata")

void data_handler(void)
{
	static uint8_t channel = 0, upper, command, port_tmp, char_txtmp, char_rxtmp, char_status = 0,
		c1, c2;
	static union Timers timer;

	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		V.mbmcdata_count++; // total data sent counter
		V.c1tx_int++;
		tx_tmp++; // count for 1 second
		if (ll_flag.data_pos >= ll_flag.data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = LOW; // stop data xmit
			}
		} else {
			TXREG1 = *ll_flag.data_ptr; // send data and clear PIR1bits.TX1IF
			ll_flag.data_pos++; // move the data pointer
			ll_flag.data_ptr++; // move the buffer pointer position
		}
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for terminal link */
	if (PIE3bits.TX2IE && PIR3bits.TX2IF) {
		if (ll_dumpflag.data_pos >= ll_dumpflag.data_len) { // buffer has been sent
			if (TXSTA2bits.TRMT) { // last bit has been shifted out
				PIE3bits.TX2IE = LOW; // stop data xmit
			}
		} else {
			TXREG2 = *ll_dumpflag.data_ptr; // send data and clear PIR1bits.TX1IF
			V.c2_int++;
			ll_dumpflag.data_pos++; // move the data pointer
			ll_dumpflag.data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR1bits.RC1IF) { // is data from network 9n1 port
		V.c1rx_int++; // total count
		rx_tmp++; // count for 1 second
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = LOW; // clear overrun
			RCSTA1bits.CREN = HIGH; // re-enable
		}

		/* clear com1 interrupt flag */
		// a read clears the flag
		c1 = RCREG1; // read data from port1 and clear PIR1bits.RC1IF
	}

	if (PIR3bits.RC2IF) { // is data from user command/dump terminal port
		V.c2_int++;
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = LOW; //      clear overrun
			RCSTA2bits.CREN = HIGH; // re-enable
		}

		c2 = RCREG2; // read from host port2 and clear PIR3bits.RC2IF
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer int handler
		INTCONbits.TMR0IF = LOW; //clear interrupt flag
		//check for TMR0 overflow

		timer.lt = TIMEROFFSET; // Copy timer value into union
		TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
		TMR0L = timer.bt[LOW]; // Write low byte to Timer0
		DLED0 = LOW;
	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		PIR1bits.ADIF = LOW;
		adc_count++; // just keep count
		adc_buffer[channel] = ADRES;
		DLED1 = !DLED1;
	}
}
#pragma	tmpdata

#pragma	tmpdata	ISRLtmpdata
#pragma interruptlow work_handler   nosave=section (".tmpdata")

void work_handler(void) // This is the low priority ISR routine, the high ISR routine will be called during this code section
{
}
#pragma	tmpdata
