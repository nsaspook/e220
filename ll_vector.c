#include "ll_vector.h"

//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma	tmpdata	ISRHtmpdata
#pragma interrupt data_handler   nosave=section (".tmpdata")

void data_handler(void)
{
	static uint8_t channel = 0, upper, command, port_tmp, char_txtmp, char_rxtmp, char_status = 0, cmd_dummy = CMD_DUMMY,
		c1, c2;
	static union Timers timer;

	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		V.mbmcdata_count++; // total data sent counter
		V.c1tx_int++;
		tx_tmp++; // count for 1 second
		if (mbmcflag.data_pos >= mbmcflag.data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = LOW; // stop data xmit
			}
		} else {
			TXREG1 = *mbmcflag.data_ptr; // send data and clear PIR1bits.TX1IF
			mbmcflag.data_pos++; // move the data pointer
			mbmcflag.data_ptr++; // move the buffer pointer position
		}
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for terminal link */
	if (PIE3bits.TX2IE && PIR3bits.TX2IF) {
		if (mbmc_dumpflag.data_pos >= mbmc_dumpflag.data_len) { // buffer has been sent
			if (TXSTA2bits.TRMT) { // last bit has been shifted out
				PIE3bits.TX2IE = LOW; // stop data xmit
			}
		} else {
			TXREG2 = *mbmc_dumpflag.data_ptr; // send data and clear PIR1bits.TX1IF
			V.c2_int++;
			mbmc_dumpflag.data_pos++; // move the data pointer
			mbmc_dumpflag.data_ptr++; // move the buffer pointer position
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
		/* if we are just idle don't reset the PIC */
		if ((slave_int_count - last_slave_int_count) < SLAVE_ACTIVE) {
			_asm clrwdt _endasm // reset the WDT timer
			DLED1 = HIGH;
			DLED2 = HIGH;
			DLED3 = HIGH;
			DLED4 = HIGH;
			DLED5 = HIGH;
			DLED6 = HIGH;
			DLED7 = HIGH;
		}
		spi_comm.REMOTE_LINK = FALSE;
		DLED0 = LOW;
	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		PIR1bits.ADIF = LOW;
		adc_count++; // just keep count
		adc_buffer[channel] = ADRES;
		if (upper) {
			SSPBUF = (uint8_t) (adc_buffer[channel] >> 8); // stuff with upper 8 bits
		} else {
			SSPBUF = (uint8_t) adc_buffer[channel]; // stuff with lower 8 bits
		}
		spi_comm.ADC_DATA = TRUE;
		DLED1 = !DLED1;
	}

	/* we only get this when the master  wants data, the slave never generates one */
	if (PIR1bits.SSPIF) { // SPI port #2 SLAVE receiver
		PIR1bits.SSPIF = LOW;
		data_in2 = SSPBUF;

		DLED0 = HIGH; // rx data led off
		if (PIR3bits.RC2IF) { // we need to read the buffer in sync with the *_CHAR_* commands so it's polled
			char_rxtmp = RCREG2;
			cmd_dummy |= UART_DUMMY_MASK; // We have real USART data waiting
			spi_comm.CHAR_DATA = TRUE;
			DLED0 = LOW; // rx data led on
		}

		spi_stat.slave_int_count++;
		command = data_in2 & HI_NIBBLE;
		DLED7 = !DLED7;

		if (command == CMD_PORT_GO) {
			SSPBUF = PORTB; // read inputs into the buffer
			port_tmp = (data_in2 & LO_NIBBLE); // read lower 4 bits
			spi_stat.port_count++;
			spi_stat.last_slave_int_count = spi_stat.slave_int_count;
		}

		if (command == CMD_PORT_DATA) {
#ifndef	DLED_DEBUG
			PORTE = ((data_in2 & 0b00001111) << 4) | port_tmp; // PORTE pins [0..7]
#endif
			spi_comm.REMOTE_LINK = TRUE;
			/* reset link data timer if we are talking */
			timer.lt = TIMEROFFSET; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
			INTCONbits.TMR0IF = LOW; //clear possible interrupt flag
			SSPBUF = cmd_dummy; // send the input data
		}

		if (command == CMD_CHAR_GO) {
			char_txtmp = (data_in2 & LO_NIBBLE); // read lower 4 bits
			char_status &= (~UART_TX_MASK); // clear tx bit
			if (TXSTA2bits.TRMT) char_status |= UART_TX_MASK; // The USART send buffer is ready
			DLED1 = HIGH; // rx data read
			SSPBUF = char_rxtmp; // send current receive data to master
			spi_stat.char_count++;
		}

		if (command == CMD_CHAR_DATA) { // get upper 4 bits send bits and send the data
			if (char_status & UART_TX_MASK) {
				TXREG2 = ((data_in2 & LO_NIBBLE) << 4) | char_txtmp; // send data to RS-232 #2 output
				DLED6 = !DLED6; // tx data
			} else {
				DLED6 = LOW; // TX busy, overun
			}
			SSPBUF = cmd_dummy; // send rx status first, the next SPI transfer will contain it.
			cmd_dummy = CMD_DUMMY; // clear rx bit
			spi_comm.CHAR_DATA = FALSE;
			spi_comm.REMOTE_LINK = TRUE;
			/* reset link data timer if we are talking */
			timer.lt = TIMEROFFSET; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
			INTCONbits.TMR0IF = LOW; //clear possible interrupt flag
		}

		if ((command == CMD_ADC_GO) || (command == CMD_ADC_GO_H)) { // Found a ADC GO command
			if (data_in2 & ADC_SWAP_MASK) {
				upper = TRUE;
			} else {
				upper = FALSE;
			}
			channel = data_in2 & LO_NIBBLE;
#ifdef P25K22
			if (channel > 4) channel += 7; // skip missing channels
			if (channel > 19) channel = 0; // invalid to set to 0
#endif

			if (!ADCON0bits.GO) { // select the channel first
				ADCON0 = ((channel << 2) & 0b01111100) | (ADCON0 & 0b00000011);
				spi_comm.ADC_DATA = FALSE;
				ADCON0bits.GO = HIGH; // start a conversion
			} else {
				ADCON0bits.GO = LOW; // stop a conversion
				SSPBUF = cmd_dummy; // Tell master  we are here
				spi_comm.ADC_DATA = FALSE;
			}
		}


		if (command == CMD_ADC_DATA) {
			if (!ADCON0bits.GO) {
				if (upper) {
					SSPBUF = (uint8_t) adc_buffer[channel]; // stuff with lower 8 bits
				} else {
					SSPBUF = (uint8_t) (adc_buffer[channel] >> 8); // stuff with upper 8 bits
				}
				spi_comm.REMOTE_LINK = TRUE;
				/* reset link data timer if we are talking */
				timer.lt = TIMEROFFSET; // Copy timer value into union
				TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
				TMR0L = timer.bt[LOW]; // Write low byte to Timer0
				INTCONbits.TMR0IF = LOW; //clear possible interrupt flag
			} else {
				SSPBUF = cmd_dummy;
			}
		}
		if (command == CMD_DUMMY_CFG) {
			SSPBUF = cmd_dummy; // Tell master  we are here
		}

		if (command == CMD_CHAR_RX) {
			SSPBUF = char_rxtmp; // Send current RX buffer contents
			cmd_dummy = CMD_DUMMY; // clear rx bit
		}

	}

}
#pragma	tmpdata

#pragma	tmpdata	ISRLtmpdata
#pragma interruptlow work_handler   nosave=section (".tmpdata")

void work_handler(void) // This is the low priority ISR routine, the high ISR routine will be called during this code section
{
}
#pragma	tmpdata
