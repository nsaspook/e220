#define P45K80
/* 
 * LLLT
 *
 * spam@sma2.rain.com   Jul 2015
 * 
 * General I/O
 * button1 int 0 pin 33, led pin 4 using debounce chip MC14490
 * button2 int 1 pin 34, led pin 5
 * Diag led array 8 led D[0..5]
 * CTMU CTED1 pin 35, CTED2 pin 36
 * 
 * Diag pins right to left
 * 1 tx1
 * 2 rx1
 * 3 tx2
 * 4 rx2
 * 5 sdo
 * 6 sdi
 * 7 sck
 * 8 sled
 * 9 vcc
 * Diag pins left to right
 * 1 dled0
 * 2 dled1
 * 3 dled2
 * 4 dled3
 * 5 dled4
 * 6 dled5
 * 7 dled6
 * 8 dled7
 * 9 gnd
 * 
 * HID Diag pins
 * 1 rs
 * 2 csb
 * 3 bled0
 * 4 bled1
 * 5 sw0
 * 6 sw1
 * 7
 * 8
 */

#ifdef P45K80
// PIC18F45K80 Configuration Bit Settings
#include <p18f45k80.h>

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = ON       // Extended Instruction Set (Enabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 8192     // Watchdog Postscaler (1:8192)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-01FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 02000-03FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 04000-05FFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 06000-07FFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-01FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 02000-03FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 04000-05FFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 06000-07FFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

#endif

#include <p18cxxx.h>
#include <spi.h>
#include <timers.h>
#include <adc.h>
#include <usart.h>
#include <delays.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GenericTypeDefs.h>
#include "light_link.h"
#include "ll_vector.h"
#include "ringbufs.h"
#include "eadog.h"

#define eaDogM_Cls()             eaDogM_WriteCommand(EADOGM_CMD_CLR)
#define eaDogM_CursorOn()        eaDogM_WriteCommand(EADOGM_CMD_CURSOR_ON)
#define eaDogM_CursorOff()       eaDogM_WriteCommand(EADOGM_CMD_CURSOR_OFF)
#define eaDogM_DisplayOn()       eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_ON)
#define eaDogM_DisplayOff()      eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_OFF)

volatile struct spi_link_type spi_link;

const rom char *screen_data = " Light Link Box ";
const rom char *dspace = " ";
const rom int8_t *build_date = __DATE__, *build_time = __TIME__;

volatile uint8_t data_in2, adc_buffer_ptr = 0,
	adc_channel = 0;
volatile uint32_t adc_count = 0, adc_error_count = 0,
	slave_int_count = 0, last_slave_int_count = 0;
volatile uint16_t adc_buffer[4] = {0}, adc_data_in = 0;
volatile uint8_t WDT_TO = FALSE, EEP_ER = FALSE;
uint8_t CRITC = 0, BOOT_STATUS = 0;
volatile int16_t tx_tmp = 0, rx_tmp = 0;
#pragma udata gpr2
volatile struct V_data V;
volatile struct llflagtype ll_flag;
#pragma udata gpr3
volatile struct L_data L;
#pragma udata gpr4
volatile struct ringBufS_t ring_buf3,ring_buf4;
#pragma udata gpr5
volatile struct ringBufS_t ring_buf5;
#pragma udata gpr6
volatile struct ringBufS_t ring_buf1,ring_buf2;
#pragma udata gpr7
volatile struct L_data L_EEPROM;
volatile union Timers timer_long;
#pragma idata gpr8
volatile hidtype hid0 = {0, 0, 0, b0_on, b0_off},
hid1 = {0, 0, 0, b1_on, b1_off};
volatile hidtype *hid0_ptr = &hid0, *hid1_ptr = &hid1;
#pragma udata 

//High priority interrupt vector, placed at address HIGH_VECTOR
#pragma code data_interrupt = HIGH_VECTOR

void data_int(void)
{
	_asm goto data_handler //jump to interrupt routine
	_endasm
}
#pragma code

#pragma code work_interrupt = LOW_VECTOR

void work_int(void)
{
	_asm goto work_handler _endasm // low
}
#pragma code

void wdtdelay(uint32_t delay)
{
	static uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		Nop();
		ClrWdt(); // reset the WDT timer
	};
}

void config_pic(void)
{
	if (RCONbits.TO == (uint8_t) LOW) WDT_TO = TRUE;
	if (EECON1bits.WRERR && (EECON1bits.EEPGD == (uint8_t) LOW)) EEP_ER = TRUE;
	/*
	 * default operation mode
	 */
	L.rs232_mode = RS232_LL;
	L.omode = LL_OPEN;
	L.checksum = CHECKMARK_CRC;
	L.adc_chan = 0;
	timer_long.lt = TIMEROFFSET;

	/* setup the link buffers first */
	L.rx1b = &ring_buf1;
	L.tx1b = &ring_buf2;
	L.rx2b = &ring_buf3;
	L.tx2b = &ring_buf4;
	spi_link.tx1b = &ring_buf5;
	ringBufS_init(L.rx1b);
	ringBufS_init(L.tx1b);
	ringBufS_init(L.rx2b);
	ringBufS_init(L.tx2b);
	ringBufS_init(spi_link.tx1b);

	OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
	OSCTUNE = 0b01000000; // 4x pll
	SLRCON = 0x00; // all slew rates to max
	TRISA = 0x00; // all outputs
	TRISB = 0x00;
	TRISC = 0x00;
	TRISD = 0x00;
	TRISE = 0x00;
	LATA = 0xff;
	LATB = 0x00;
	LATC = 0x00;
	LATD = 0xff;
	LATE = 0xff;

	/* SPI pins setup */
	TRISCbits.TRISC3 = OUT; // SCK 
	TRISCbits.TRISC4 = IN; // SDI
	TRISCbits.TRISC5 = OUT; // SDO

	/* RS-232 #1 TX/RX setup */
	TRISCbits.TRISC6 = OUT; // digital output,TX
	TRISCbits.TRISC7 = IN; // digital input, RX

	/* RS-232 #2 TX/RX setup */
	TRISDbits.TRISD6 = OUT; // digital output,TX
	TRISDbits.TRISD7 = IN; // digital input, RX

	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_VDD_VSS); // open ADC channel
	ANCON0 = 0b00000011; // analog bit enables
	ANCON1 = 0; // analog bit enables

	// ADCON2
	ADCON2bits.ADFM = 1; // Results format 1= Right justified
	ADCON2bits.ACQT = 7; // Acquition time 7 = 20TAD 2 = 4TAD 1=2TAD
	ADCON2bits.ADCS = 6; // Clock conversion bits 6= FOSC/64 2=FOSC/32
	// ADCON1
	ADCON1bits.VCFG = 3; // Vref+ = 4.096
	ADCON1bits.VNCFG = 0; // Vref- = AVss
	ADCON1bits.CHSN = 0; // single ended
	ADCON0bits.CHS = CTMU_CHAN; // Select ADC
	ADCON0bits.ADON = 1; // Turn on ADC	

	SLED = HIGH; // run indicator
	RS = HIGH; // lcd
	CSB = HIGH; //lcd

	PIE1bits.ADIE = LOW; // the ADC interrupt enable bit
	IPR1bits.ADIP = HIGH; // ADC use high pri

	OpenSPI(SPI_FOSC_64, MODE_00, SMPEND); // 1MHz
	SSPCON1 |= SPI_FOSC_64; // set clock to low speed

	/*
	 * Open the USART configured as
	 * 9N1, 375000 baud, in send and receive INT mode
	 */
	BAUDCON1 |= 0x08; // 16 bit mode speed register
	BAUDCON2 |= 0x08; // 16 bit mode speed register

	Open1USART(USART_TX_INT_ON & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_NINE_BIT & USART_CONT_RX & USART_BRGH_HIGH, BAUD_FAST); // 64mhz osc INTPLL 38.4 kbaud, 16bit divider
	USART1_Status.TX_NINE = HIGH;
	TXSTA1bits.TX9D = HIGH; // same in uC
	RCSTA1bits.ADDEN = LOW; // receive all data
	TXSTA1bits.TX9 = HIGH;
	RCSTA1bits.RX9 = HIGH;
	SPBRGH1 = BAUD_SLOW; // set to 2 for slow speed testing
	SPBRG1 = BAUD_FAST;

	/*
	 * Open the USART configured as
	 * 9N1, 375000 baud, transmit/receive INT mode
	 */
	Open2USART(USART_TX_INT_ON & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_NINE_BIT & USART_CONT_RX & USART_BRGH_HIGH, BAUD_FAST); // 64mhz osc INTPLL 38.4 kbaud, 16bit divider
	USART2_Status.TX_NINE = HIGH;
	TXSTA2bits.TX9D = LOW; // same in uC
	RCSTA2bits.ADDEN = LOW; // receive all data
	TXSTA2bits.TX9 = HIGH;
	RCSTA2bits.RX9 = HIGH;
	SPBRGH2 = BAUD_SLOW;
	SPBRG2 = BAUD_FAST;

	if (L.rs232_mode == RS232_LL) {
		BAUDCON1 |= 0b00110000;
		BAUDCON2 |= 0b00110000;
	}

	while (DataRdy1USART()) { // dump 1 rx data`
		Read1USART();
	};
	while (DataRdy2USART()) { // dump 2 rx data
		Read2USART();
	};

	/* System activity timer */
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	WriteTimer0(TIMEROFFSET); //      start timer0 at ~1 second ticks

	/* event timer */
	OpenTimer1(T1_SOURCE_FOSC_4 & T1_16BIT_RW & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF, 0);
	IPR1bits.TMR1IP = 0; // set timer2 low pri interrupt
	WriteTimer1(PDELAY);

	/* clear SPI module possible flag */
	PIR1bits.SSPIF = LOW;

	/*
	 * CTMU
	 */

	//CTMUCONH/1 - CTMU Control registers
	CTMUCONH = 0x04; //make sure CTMU is disabled and ready for edge 1 before 2
	CTMUCONL = 0x90; // positive edges
	CTMUICON = 0x01; //.55uA, Nominal - No Adjustment default
	CTMUCONLbits.EDG1SEL = 3; // Set Edge CTED1
	CTMUCONLbits.EDG2SEL = 2; // CTED2
	CTMUCONHbits.CTMUEN = 1; //Enable the CTMU
	CTMUCONHbits.IDISSEN = 1; // drain the circuit

	/*
	 * Switch input/PORTB config
	 */
	INTCON2bits.RBPU = HIGH; // turn off weak pullups
	INTCONbits.RBIE = LOW; // disable PORTB interrupts
	IOCB = 0x00;
	TRISBbits.TRISB0 = HIGH;
	INTCONbits.INT0IE = HIGH;
	TRISBbits.TRISB1 = HIGH;
	INTCON3bits.INT1IP = HIGH;
	INTCON3bits.INT1IE = HIGH;
	INTCON2bits.INTEDG0 = LOW;
	INTCON2bits.INTEDG1 = LOW;
	INTCONbits.RBIF = LOW; // reset flags
	INTCONbits.INT0IF = LOW;
	INTCON3bits.INT1IF = LOW;

	/* Enable interrupt priority */
	RCONbits.IPEN = HIGH;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = HIGH;
	INTCONbits.GIEL = HIGH;

	/* clear any SSP error bits */
	SSPCON1bits.WCOL = SSPCON1bits.SSPOV = LOW;
	SLED = LOW;
	BLED0 = S_OFF;
	BLED1 = S_OFF;
}

/*
 * Light Link Loop Tester
 */
void main(void)
{
	int16_t i, j, k = 0;
	char bootstr2[32];

	config_pic(); // setup the uC for work

	if (STKPTRbits.STKFUL) BOOT_STATUS += 1;
	if (STKPTRbits.STKUNF) BOOT_STATUS += 2;
	if (WDT_TO) BOOT_STATUS += 4;
	if (EEP_ER) BOOT_STATUS += 8;
	if (BOOT_STATUS) {
		SLED = HIGH; // we had a previous error condition on boot
		L.omode = LL_OPEN;
	}

	init_display();

	//	eaDogM_Cls();
	strncpypgm2ram(bootstr2, screen_data, 16);
	eaDogM_WriteString(bootstr2);
	eaDogM_SetPos(1, 0);
	strncpypgm2ram(bootstr2, build_time, 16);
	eaDogM_WriteString(bootstr2);
	strncpypgm2ram(bootstr2, dspace, 16);
	eaDogM_WriteString(bootstr2);
	eaDogM_SetPos(2, 0);
	strncpypgm2ram(bootstr2, build_date, 16);
	eaDogM_WriteString(bootstr2);
	sprintf(bootstr2, " B%i", BOOT_STATUS);
	eaDogM_WriteString(bootstr2);
	strncpypgm2ram(bootstr2, screen_data, 16);

	while (1) { // just loop and output results on DIAG LCD

		if (SSPCON1bits.WCOL || SSPCON1bits.SSPOV) { // check for overruns/collisions
			SSPCON1bits.WCOL = SSPCON1bits.SSPOV = LOW;
			SLED = HIGH;
		}


		for (i = 0; i < 1; i++) {
			for (j = 0; j < 1; j++) {
			}
		}

		if (!BLED0) {
			ringBufS_put(L.tx2b, 0b000000000);
			ringBufS_put(L.tx2b, 0b111111111);
			ringBufS_put(L.tx2b, 0b011111111);
			ringBufS_put(L.tx2b, 0b111111111);
			ringBufS_put(L.tx2b, 0b011111111);
			ringBufS_put(L.tx2b, 0b111111111);
			ringBufS_put(L.tx2b, 0b011111111);
			ringBufS_put(L.tx2b, 0b111111111);
			ringBufS_put(L.tx2b, 0b000000000);

			ringBufS_put(L.tx1b, 0b000000000);
			ringBufS_put(L.tx1b, 0b111111111);
			ringBufS_put(L.tx1b, 0b011111111);
			ringBufS_put(L.tx1b, 0b111111111);
			ringBufS_put(L.tx1b, 0b011111111);
			ringBufS_put(L.tx1b, 0b111111111);
			ringBufS_put(L.tx1b, 0b011111111);
			ringBufS_put(L.tx1b, 0b111111111);
			ringBufS_put(L.tx1b, 0b000000000);

			start_tx1();
			start_tx2();
			while (!ringBufS_empty(L.tx1b));
		}

		ClrWdt(); // reset the WDT timer

		//		eaDogM_SetPos(0, 0);
		//		eaDogM_Cls();
		eaDogM_WriteStringAtPos(0, 0, bootstr2);
	}

}
