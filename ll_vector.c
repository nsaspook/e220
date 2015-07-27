#include "ll_vector.h"
#include "ringbufs.h"

//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma	tmpdata	ISRHtmpdata
#pragma interrupt data_handler   nosave=section (".tmpdata")

void data_handler(void)
{
	static uint16_t channel = 0, cr1, cr2, ct1, ct2;
	static union Timers timer;

	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		tx_tmp++; // count for 1 second
		if (ringBufS_empty(L.tx1b)) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = LOW; // stop data xmit
			}
		} else {
			ct1 = ringBufS_get(L.tx1b); // get the 9 bit data
			TXSTA1bits.TX9D = (ct1 & 0b100000000) ? 1 : 0;
			TXREG1 = ct1; // send data and clear FLAG
			V.c1t_int++;
		}
	}

	if (PIE3bits.TX2IE && PIR3bits.TX2IF) {
		if (ringBufS_empty(L.tx2b)) { // buffer has been sent
			if (TXSTA2bits.TRMT) { // last bit has been shifted out
				PIE3bits.TX2IE = LOW; // stop data xmit
			}
		} else {
			ct2 = ringBufS_get(L.tx2b); // get the 9 bit data
			TXSTA2bits.TX9D = (ct2 & 0b100000000) ? 1 : 0;
			TXREG2 = ct2; // send data and clear FLAG
			V.c2t_int++;
		}
	}

	if (PIR1bits.RC1IF) { // is data from network 9n1 port
		V.c1r_int++; // total count
		rx_tmp++; // count for 1 second
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = LOW; // clear overrun
			RCSTA1bits.CREN = HIGH; // re-enable
		}

		/* clear com1 interrupt flag */
		// a read clears the flag
		cr1 = RCREG1; // read data from port1 and clear PIR1bits.RC1IF
		if (RCSTA1bits.RX9D) {
			cr1 |= 0b100000000;
		}
		ringBufS_put(L.rx1b, cr1);
	}

	if (PIR3bits.RC2IF) { // is data from user command/dump terminal port
		V.c2r_int++;
		LATDbits.LATD0 = !LATDbits.LATD0; // DEBUG flasher
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = LOW; //      clear overrun
			RCSTA2bits.CREN = HIGH; // re-enable
		}

		cr2 = RCREG2; // read from host port2 and clear PIR3bits.RC2IF
		if (RCSTA2bits.RX9D) {
			cr2 |= 0b100000000;
		}
		ringBufS_put(L.rx2b, cr2);
		if (cr2 == 0) LATDbits.LATD0 = 1; // DEBUG flasher
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

/* 
 * start the tx usart running 
 * return TRUE is it's already enabled
 */
int8_t start_tx1(void)
{
	int8_t tx_running = 0;

	if (PIE1bits.TX1IE) tx_running = 1;
	PIE1bits.TX1IE = 1;
	PIR1bits.TX1IF = 1;
	return tx_running;
}

int8_t start_tx2(void)
{
	int8_t tx_running = 0;

	if (PIE3bits.TX2IE) tx_running = 1;
	PIE3bits.TX2IE = 1;
	PIR3bits.TX2IF = 1;
	return tx_running;
}
