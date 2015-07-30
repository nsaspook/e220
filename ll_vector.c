#include "ll_vector.h"
#include "ringbufs.h"

//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma	tmpdata	ISRHtmpdata
#pragma interrupt data_handler   nosave=section (".tmpdata")

void data_handler(void)
{
	static uint16_t channel = 0, cr1, cr2, ct1, ct2, ct_spi;
	static union Timers timer;

	if (PIE1bits.SSPIE && PIR1bits.SSPIF) { // send data to SPI bus
		PIR1bits.SSPIF = LOW;
		ct1 = SSPBUF; // read to clear the BF flag, don't care about the data with LCD
		if (spi_link.SPI_LCD) {
			if (ringBufS_empty(spi_link.tx1b)) { // buffer has been sent
				PIE1bits.SSPIE = LOW; // stop data xmit
				spi_link.TIMER = LOW;
				CSB = HIGH; // deselect the display
			} else {
				ct_spi = ringBufS_get(spi_link.tx1b); // get the 16 bit data
				spi_link.config = ct_spi >> 8;

				if (spi_link.config & 0b00000001) {
					spi_link.delay = LCD_LONG;
					RS = LOW; // send cmd
				} else {
					spi_link.delay = LCD_SHORT;
					RS = HIGH; // send data
				}
				CSB = LOW; // select the display
				spi_link.TIMER = HIGH;
				spi_link.DATA = HIGH;
				INTCONbits.TMR0IF = HIGH; //set interrupt flag
			}
		}
		if (spi_link.SPI_AUX) {

		}
	}

	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to USART
		if (ringBufS_empty(L.tx1b)) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = LOW; // stop data xmit
			}
		} else {
			ct1 = ringBufS_get(L.tx1b); // get the 9 bit data from 16 bit data buffer
			TXSTA1bits.TX9D = (ct1 & 0b100000000) ? HIGH : LOW;
			TXREG1 = ct1; // send data and clear FLAG
			V.c1t_int++;
		}
	}

	if (PIE3bits.TX2IE && PIR3bits.TX2IF) {
		if (ringBufS_empty(L.tx2b)) {
			if (TXSTA2bits.TRMT) {
				PIE3bits.TX2IE = LOW;
			}
		} else {
			ct2 = ringBufS_get(L.tx2b);
			TXSTA2bits.TX9D = (ct2 & 0b100000000) ? HIGH : LOW;
			TXREG2 = ct2;
			V.c2t_int++;
		}
	}

	if (PIR1bits.RC1IF) { // is data from network down-link 9n1 port
		V.c1r_int++; // total count
		DLED0 = !DLED0; // link flasher
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = LOW; // clear overrun
			RCSTA1bits.CREN = HIGH; // re-enable
			SLED = HIGH;
		}
		if (RCSTA1bits.FERR) SLED = HIGH;

		/* clear com1 interrupt flag */
		// a read clears the flag
		cr1 = RCREG1; // read from port1 and clear PIR1bits.RC1IF
		if (RCSTA1bits.RX9D) {
			cr1 |= 0b100000000; // OR bit 9 for data buffer
		}
		ringBufS_put(L.rx1b, cr1);
		if (cr1 == 0) DLED0 = S_OFF; // DEBUG flasher
	}

	if (PIR3bits.RC2IF) { // is data from network up-link 9n1 port
		V.c2r_int++;
		DLED1 = !DLED1; // link flasher
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = LOW; //      clear overrun
			RCSTA2bits.CREN = HIGH; // re-enable
			SLED = HIGH;
		}
		if (RCSTA2bits.FERR) SLED = HIGH;

		cr2 = RCREG2; // read from port2 and clear PIR3bits.RC2IF
		if (RCSTA2bits.RX9D) {
			cr2 |= 0b100000000;
		}
		ringBufS_put(L.rx2b, cr2);
		if (cr2 == 0) DLED1 = S_OFF; // DEBUG flasher
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer & SPI delay int handler
		if (spi_link.TIMER) {
			spi_link.TIMER = LOW;
			timer.lt = spi_link.delay; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
		} else {
			if (spi_link.DATA) {
				spi_link.DATA = LOW;
				SSPBUF = ct_spi; // send data
			}
			//check for TMR0 overflow
			timer.lt = TIMEROFFSET; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
		}
		INTCONbits.TMR0IF = LOW; //clear interrupt flag
		DLED6 = !DLED6;
	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		PIR1bits.ADIF = LOW;
		adc_count++; // just keep count
		adc_buffer[channel] = ADRES;
		DLED2 = !DLED2;
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
 * start the tx usart running from user context
 * return TRUE if it's already enabled
 */
int8_t start_tx1(void)
{
	int8_t tx_running = LOW;

	if (PIE1bits.TX1IE) tx_running = HIGH;
	PIE1bits.TX1IE = HIGH;
	PIR1bits.TX1IF = HIGH;
	return tx_running;
}

int8_t start_tx2(void)
{
	int8_t tx_running = LOW;

	if (PIE3bits.TX2IE) tx_running = HIGH;
	PIE3bits.TX2IE = HIGH;
	PIR3bits.TX2IF = HIGH;
	return tx_running;
}
