#include "ll_vector.h"
#include "ringbufs.h"

/* two point temperature diode calibration code from
 * http://www.ti.com/lit/an/sbaa073a/sbaa073a.pdf
 * https://www.ccsinfo.com/forum/viewtopic.php?p=173294
 */
#define ZERO_DEGC_IN_CENTIK 27315ul // centi-K = 0.01K units 

struct {
	// alpha numerator and denominator 
	uint16_t alphan; // ADC bits 
	int32_t alphad; // 0.01K units 
} calibration;

/* 
 * Initialize module 
 * 89.5F (31.9C) = alphan 293
 */
void inttemp_init(void)
{
	// arbitrary values 
	calibration.alphan = 293;
	calibration.alphad = 319 * 10 + ZERO_DEGC_IN_CENTIK;
}

/*
 *  Single point calibration to assumed ambient temperature 
 */
void inttemp_calibrate(int16_t temperature)
{
	// alpha = delta diode voltage / temperature 
	calibration.alphan = inttemp_deltav();
	calibration.alphad = ((int32_t) temperature * 10 + ZERO_DEGC_IN_CENTIK);
}

void b0_on(void)
{
	BLED0 = S_ON;
}

void b1_on(void)
{
	BLED1 = S_ON;
	L.omode = LL_E220;
}

void b0_off(void)
{
	BLED0 = S_OFF;
}

void b1_off(void)
{
	BLED1 = S_OFF;
	L.omode = LL_OPEN;
}

// High priority interrupt routine
#pragma	tmpdata	ISRHtmpdata
#pragma interrupt data_handler   nosave=section (".tmpdata")

void data_handler(void)
{
	static uint16_t cr1, cr2, ct1, ct2, ct_spi;
	static union Timers timer;
	static uint8_t b_data;

	DLED5 = LOW;
	if (PIE1bits.SSPIE && PIR1bits.SSPIF) { // send data to SPI bus
		spi_link.int_count++;
		PIR1bits.SSPIF = LOW;
		ct1 = SSPBUF; // read to clear the BF flag, don't care about the data with LCD but AUX might
		/*
		 * the SPI data is not sent here, it's scheduled by timer0
		 * with about 63us for the whole process with LCD data byte to next byte with timer0 LCD delay
		 * fixed times: ~8us per byte 1MHz clock, 27.3us LCD instruction execution time
		 * and ~18us overhead for this part
		 */
		if (spi_link.SPI_LCD) {
			if (spi_link.tx1b->count == 0) { // data buffer is empty but maybe the delay is not done, 3.6us overhead
				PIE1bits.SSPIE = LOW; // stop data transmitter
				spi_link.LCD_TIMER = LOW;
				if (spi_link.LCD_DATA) INTCONbits.TMR0IF = HIGH; //set interrupt flag
			} else {
				ct_spi = ringBufS_get(spi_link.tx1b); // get the 16 bit data
				spi_link.config = ct_spi >> 8;

				if (spi_link.config & LCD_CMD_MASK) { // is this command data
					/* 
					 * check for clear and home commands
					 */
					if ((ct_spi & 0xff) > LCD_CLEAR_HOME) { // check only the data bits for the last 2 bits
						spi_link.delay = LCD_LONG; // needs at least 1.08 ms LCD instruction execution time
					} else {
						spi_link.delay = LCD_SHORT; // needs at least 27.3us LCD instruction execution time
					}
					RS = LOW; // sending LCD command data
				} else {
					spi_link.delay = LCD_SHORT;
					RS = HIGH; // sending character data
				}
				CSB = LOW; // select the display SPI receiver
				/*
				 * setup timer0 for SPI delay and buffer write
				 */
				spi_link.LCD_TIMER = HIGH;
				spi_link.LCD_DATA = HIGH;
				INTCONbits.TMR0IF = HIGH; //set interrupt flag to update timer0 as we fall down the ISR
			}
		}
		if (spi_link.SPI_AUX) {
			CSA = LOW; // select the AUX SPI receiver
		}
	}

	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to USART
		if (L.tx1b->count == 0) { // buffer has been sent
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
		if (L.tx2b->count == 0) {
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
		DLED0 = !DLED0; // link flasher
		V.c1r_int++; // total count
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = LOW; // clear overrun
			RCSTA1bits.CREN = HIGH; // re-enable
			SLED = HIGH;
			ll_flag.overrun1_error++;
		}
		if (RCSTA1bits.FERR) {
			SLED = HIGH;
			ll_flag.frame1_error++;
		}

		/* clear com1 interrupt flag */
		// a read clears the flag
		cr1 = RCREG1; // read from port1 and clear PIR1bits.RC1IF
		if (RCSTA1bits.RX9D) {
			cr1 |= 0b100000000; // OR bit 9 for data buffer
		}

		switch (L.omode) {
		case LL_E220:
			TXSTA2bits.TX9D = RCSTA1bits.RX9D;
			TXREG2 = cr1;
			break;
		case LL_LOOP:
			TXSTA1bits.TX9D = RCSTA1bits.RX9D;
			TXREG1 = cr1;
			break;
		default:
			break;
		}

		ringBufS_put(L.rx1b, cr1);
		if (cr1 == 0) DLED0 = S_OFF; // DEBUG flasher
	}

	if (PIR3bits.RC2IF) { // is data from network up-link 9n1 port
		DLED1 = !DLED1; // link flasher
		V.c2r_int++;
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = LOW; //      clear overrun
			RCSTA2bits.CREN = HIGH; // re-enable
			SLED = HIGH;
			ll_flag.overrun2_error++;
		}
		if (RCSTA2bits.FERR) {
			SLED = HIGH;
			ll_flag.frame2_error++;
		}

		cr2 = RCREG2; // read from port2 and clear PIR3bits.RC2IF
		if (RCSTA2bits.RX9D) {
			cr2 |= 0b100000000;
		}

		switch (L.omode) {
		case LL_E220:
			TXSTA1bits.TX9D = RCSTA2bits.RX9D;
			TXREG1 = cr2;
			break;
		case LL_LOOP:
			TXSTA2bits.TX9D = RCSTA2bits.RX9D;
			TXREG2 = cr2;
			break;
		default:
			break;
		}

		ringBufS_put(L.rx2b, cr2);
		if (cr2 == 0) DLED1 = S_OFF; // DEBUG flasher
	}

	/*
	 * This is a little tricky, it normally runs at a very slow speed for a system heartbeat
	 * but it also times a delay for SPI data to a LCD display for data and commands
	 * ~36us for data and ~1.2ms for commands set in spi_link.delay
	 * with ~3us overhead for this part
	 */
	if (INTCONbits.TMR0IF) { // check timer0 1 second timer & SPI delay int handler
		DLED3 = S_ON;
		if (spi_link.LCD_TIMER) {
			spi_link.LCD_TIMER = LOW;
			/*
			 * send the SPI data then reset timer0 for fast speed data delay
			 * send time per byte 11us
			 */
			if (spi_link.LCD_DATA) {
				PIE1bits.SSPIE = LOW; // don't process the receive interrupt from the send yet
				SSPBUF = ct_spi; // send data
			}
			/*
			 * set the link delay
			 */
			timer.lt = spi_link.delay; // Copy timer value into union
			TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer.bt[LOW]; // Write low byte to Timer0
		} else {
			if (spi_link.LCD_DATA) { // we are in a delay from sending a byte
				spi_link.LCD_DATA = LOW; // clear the data sent flag
				PIE1bits.SSPIE = HIGH; // enable to receive byte when ISR exits, ISR will be called again in ~5us to get it
			}
			// set normal delay
			TMR0H = timer_long.bt[HIGH]; // Write high byte to Timer0
			TMR0L = timer_long.bt[LOW]; // Write low byte to Timer0
		}
		INTCONbits.TMR0IF = LOW; //clear interrupt flag
		DLED3 = S_OFF;
	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		DLED2 = S_ON;
		PIR1bits.ADIF = LOW;
		adc_count++; // just keep count
		if (L.ctmu_data) {
			L.ctmu_adc = ADRES;
			L.ctmu_data = LOW;
		} else {
			if (L.ctmu_data_temp) {
				L.pic_temp = ADRES;
				L.ctmu_data_temp = LOW;
			}
			adc_buffer[L.adc_chan] = ADRES;
		}
		DLED2 = S_OFF;
	}

	/*
	 * trigger the ADC here
	 */
	if (PIR3bits.CTMUIF) { // CTED (tx1) is high then CTED2 (rx2)went high
		PIR3bits.CTMUIF = LOW; // clear CTMU flag 
		if (L.ctmu_data) {
			ADCON0bits.GO = HIGH; // and begin A/D conv, will set adc int flag when done.
			DLED4 = !DLED4;
			CTMUCONHbits.CTMUEN = LOW;
			CTMUICON = 0x00; // current off
			PIE3bits.CTMUIE = LOW; // disable interrupt
			CTMUCONHbits.EDGEN = LOW;
			CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
			CTMUCONLbits.EDG2STAT = 0;
		}
	}

	if (INTCONbits.RBIF) {
		b_data = PORTB;
		INTCONbits.RBIF = LOW;
	}

	if (INTCONbits.INT0IF) {
		INTCONbits.INT0IF = LOW;
		V.buttonint_count++;
		hid0_ptr->bled_on = !hid0_ptr->bled_on;
	}

	if (INTCON3bits.INT1IF) {
		INTCON3bits.INT1IF = LOW;
		V.buttonint_count++;
		hid1_ptr->bled_on = !hid1_ptr->bled_on;
	}

	DLED5 = HIGH;
}
#pragma	tmpdata

// Low priority interrupt routine
#pragma	tmpdata	ISRLtmpdata
#pragma interruptlow work_handler   nosave=section (".tmpdata")

/*
 *  This is the low priority ISR routine, the high ISR routine will be called during this code section
 */
void work_handler(void)
{
	static int8_t task = 0;

	if (PIR1bits.TMR1IF) {

		PIR1bits.TMR1IF = LOW; // clear TMR1 interrupt flag
		WriteTimer1(PDELAY);
		/*
		 * task work stack
		 */
		switch (task) {
		case 0:
			if (hid0_ptr->bled_on) {
				hid0_ptr->t_on();
			} else {
				hid0_ptr->t_off();
			}
			break;
		case 1:
			if (hid1_ptr->bled_on) {
				hid1_ptr->t_on();
			} else {
				hid1_ptr->t_off();
			}
			break;
		default:
			task = -1; // null task
			break;
		}
		task++;
	}
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

volatile void s_crit(void) // Start critical section of code that needs protection for the ISR
{
	INTCONbits.GIEL = LOW;
	INTCONbits.GIEH = LOW;
}

volatile void e_crit(void) // End section of code that need protection from ISR
{
	INTCONbits.GIEH = HIGH;
	INTCONbits.GIEL = HIGH;
}

/* 
 * EEPROM data array: 0=MAGIC checksum, 1=length of array, 2=index into array data, 
 * 3 = array offset, writes must be protected from ISR
 */
void write_data_eeprom(uint8_t data, uint8_t count, uint16_t addr, uint16_t offset)
{
	if (addr == 0) { // only write header when on address 0
		s_crit();
		Busy_eep();
		Write_b_eep(0 + offset, MAGIC); // write data MAGIC at byte 0 of the offset
		Busy_eep();
		Write_b_eep(1 + offset, count); // length of data
		Busy_eep();
		e_crit();
	}
	s_crit();
	Busy_eep();
	Write_b_eep(addr + 2 + offset, data); //  data
	Busy_eep();
	e_crit();
}

uint8_t read_data_eeprom(uint16_t addr, uint16_t offset)
{
	Busy_eep();
	return Read_b_eep(addr + offset);
}

void wipe_data_eeprom(uint16_t max_eeprom)
{
	static uint16_t z;

	for (z = 0; z < max_eeprom; z++) {
		s_crit();
		Busy_eep();
		Write_b_eep(z, 0xff); // overwrite with all ones.
		Busy_eep();
		e_crit();
	}
}

/*
 * ADC results from build calibration
 * value with 6 inch jumper 54 as the 'zero' CTED1 to CTED2 = 23ns
 * value with 2M jumper 74, CTED1 to CTED2 = 33ns
 * ~5ns & 10 counts per meter, ~0.5ns timing resolution
 */
void start_ctmu(void)
{
	L.ctmu_data = HIGH;
	L.ctmu_data_temp = LOW;
	ADCON1bits.VCFG = 3; // Vref+ = 4.096
	ADCON0bits.CHS = CTMU_CHAN; // Select ADC channel
	CTMUCONHbits.CTMUEN = LOW;
	CTMUICON = 0x03; // 55uA
	CTMUCONHbits.CTMUEN = HIGH;
	CTMUCONHbits.IDISSEN = HIGH; // start drain
	wdtdelay(100); // time to drain the internal cap for measurements
	CTMUCONHbits.IDISSEN = LOW; // end drain
	PIE3bits.CTMUIE = HIGH; //enable interrupt on edges
	CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
	CTMUCONLbits.EDG2STAT = 0;
	CTMUCONHbits.EDGEN = HIGH; // start looking at external edges
}

/*
 * use the CTMU to measure the internal temp diode and return a voltage
 * It's only good for about +- 1.0C resolution at best but it will work
 * for time measurement calibration drift
 */
uint16_t measure_chip_temp(uint8_t mode)
{
	L.ctmu_data = LOW;
	L.ctmu_data_temp = HIGH;
	ADCON1bits.VCFG = 2; // Vref+ = 2.048
	ADCON0bits.CHS = TEMP_DIODE; // Select ADC channel
	CTMUCONHbits.CTMUEN = LOW;
	if (mode) {
		CTMUICON = 0b01111111; // 55uA CC max trim +62%
	} else {
		CTMUICON = 0b01111110; // 5.5uA CC max trim +62%
	}
	CTMUCONHbits.CTMUEN = HIGH;
	CTMUCONHbits.IDISSEN = LOW; // end drain
	PIE3bits.CTMUIE = LOW; // don't generate CTMU interrupts for edges
	CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
	CTMUCONLbits.EDG2STAT = 0;
	CTMUCONLbits.EDG1STAT = 1; // start current source
	wdtdelay(20); // CTMU setup time before sampling
	ADCON0bits.GO = 1; // Start conversion 
	wdtdelay(20); // wait for ISR to update buffer
	CTMUCONLbits.EDG1STAT = 0; // deactivate current source  
	CTMUICON = 0x00; // current off
	return L.pic_temp;
}

/* 
 * Measure delta voltage across internal diode at two currents. 
 */
uint16_t inttemp_deltav(void)
{
	uint16_t v1, v2;

	v1 = (uint16_t) lp_filter((float) measure_chip_temp(LOW), 1, HIGH);
	v2 = (uint16_t) lp_filter((float) measure_chip_temp(HIGH), 2, HIGH);
	return v2 - v1;
}

/* 
 * Read current internal temperature in 0.1C
 */
int16_t inttemp_read(void)
{
	// temperature = delta diode voltage * (1 / alpha) 
	return((int32_t) inttemp_deltav() * (int32_t) calibration.alphad / (int32_t) calibration.alphan - ZERO_DEGC_IN_CENTIK) / 10;
}

float lp_filter(float new, uint8_t bn, int8_t slow)
{
	static float smooth[4] = {0.0}, lp_speed = 0.0, lp_x = 0.0;

	if (bn >= 4) // end of the filter array so just return passed value
		return new;
	if (slow) {
		lp_speed = 0.01;
	} else {
		lp_speed = 0.125;
	}
	lp_x = ((smooth[bn]*100.0) + (((new * 100.0)-(smooth[bn]*100.0)) * lp_speed)) / 100.0;
	smooth[bn] = lp_x;
	if (slow == (-1)) { // reset and return zero
		lp_x = 0.0;
		smooth[bn] = 0.0;
	}
	return lp_x;
}