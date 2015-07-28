#define P45K80
/* 
 *
 *
 * spam@sma2.rain.com   Jul 2015
 */


#ifdef P25K22
#include <p18f25k22.h>
// PIC18F25K22 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = ON       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#endif

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

volatile struct spi_link_type spi_link;

const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
volatile uint8_t data_in2, adc_buffer_ptr = 0,
	adc_channel = 0;

volatile uint32_t adc_count = 0, adc_error_count = 0,
	slave_int_count = 0, last_slave_int_count = 0;
volatile uint16_t adc_buffer[64] = {0}, adc_data_in = 0;
#pragma udata gpr13
volatile struct V_data V;
volatile struct llflagtype ll_flag, ll_dumpflag;
volatile int16_t tx_tmp = 0, rx_tmp = 0;
#pragma udata gpr2
volatile struct L_data L;
#pragma udata gpr9

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

void wdtdelay(unsigned long delay)
{
	static uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		Nop();
		ClrWdt(); // reset the WDT timer
	};
}

void config_pic(void)
{
	/* setup the link buffers first */
	L.rx1b = &L.ring_buf1;
	L.tx1b = &L.ring_buf2;
	L.rx2b = &L.ring_buf3;
	L.tx2b = &L.ring_buf4;
	spi_link.tx1b = &spi_link.ring_buf1;
	ringBufS_init(L.rx1b);
	ringBufS_init(L.tx1b);
	ringBufS_init(L.rx2b);
	ringBufS_init(L.tx2b);
	ringBufS_init(spi_link.tx1b);

#ifdef P25K22
	OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
	OSCTUNE = 0xC0; // 4x pll
	TRISC = 0b11111100; // [0..1] outputs for DIAG leds [2..7] for analog
	LATC = 0x00; // all LEDS on
	TRISAbits.TRISA6 = 0; // CPU clock out

	TRISBbits.TRISB1 = 1; // SSP2 pins clk in SLAVE
	TRISBbits.TRISB2 = 1; // SDI
	TRISBbits.TRISB3 = 0; // SDO
	TRISBbits.TRISB0 = 1; // SS2

	/* ADC channels setup */
	TRISAbits.TRISA0 = HIGH; // an0
	TRISAbits.TRISA1 = HIGH; // an1
	TRISAbits.TRISA2 = HIGH; // an2
	TRISAbits.TRISA3 = HIGH; // an3
	TRISAbits.TRISA5 = HIGH; // an4
	TRISBbits.TRISB4 = HIGH; // an11
	TRISBbits.TRISB0 = HIGH; // an12 SS2, don't use for analog
	TRISBbits.TRISB5 = HIGH; // an13
	TRISCbits.TRISC2 = HIGH; // an14
	TRISCbits.TRISC3 = HIGH; // an15
	TRISCbits.TRISC4 = HIGH; // an16
	TRISCbits.TRISC5 = HIGH; // an17
	TRISCbits.TRISC6 = HIGH; // an17
	TRISCbits.TRISC7 = HIGH; // an18

	TRISBbits.TRISB4 = 1; // QEI encoder inputs
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB6 = 1;
	TRISBbits.TRISB7 = 1;

	ANSELA = 0b00101111; // analog bit enables
	ANSELB = 0b00110000; // analog bit enables
	ANSELC = 0b11111100; // analog bit enables
	VREFCON0 = 0b11100000; // ADC voltage ref 2.048 volts
	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_FVR_BUF & ADC_REF_VDD_VSS); // open ADC channel
#endif

#ifdef P45K80
	OSCCON = 0x70; // internal osc 16mhz, CONFIG OPTION 4XPLL for 64MHZ
	OSCTUNE = 0b01000000; // 4x pll
	SLRCON = 0x00; // all slew rates to max
	TRISA = 0x00;
	TRISB = 0x00;
	TRISC = 0x00;
	TRISD = 0x00;
	TRISE = 0x00;
	LATA = 0x00;
	LATB = 0x00;
	LATC = 0x00;
	LATD = 0x00;
	LATE = 0x00;

	INTCON2bits.RBPU = 0; // turn on weak pullups
	INTCONbits.RBIE = 0; // disable PORTB interrupts
	INTCONbits.INT0IE = 0; // disable interrupt
	INTCONbits.INT0IF = 0; // disable interrupt
	INTCONbits.RBIF = LOW; // reset B flag
	IOCB = 0x00;

	/* SPI pins setup */
	TRISCbits.TRISC3 = 1; // SCK pins clk in SLAVE
	TRISCbits.TRISC4 = 1; // SDI
	TRISCbits.TRISC5 = 0; // SDO
	TRISAbits.TRISA5 = 1; // SS2

	/* RS-232 #1 TX/RX setup */
	TRISCbits.TRISC6 = 0; // digital output,TX
	TRISCbits.TRISC7 = 1; // digital input, RX

	/* RS-232 #2 TX/RX setup */
	TRISDbits.TRISD6 = 0; // digital output,TX
	TRISDbits.TRISD7 = 1; // digital input, RX

	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON, ADC_REF_VDD_VSS); // open ADC channel
	ANCON0 = 0b11101111; // analog bit enables
	ANCON1 = 0b00000011; // analog bit enables
	ADCON1 = 0b11100000; // ADC voltage ref 2.048 volts, vref- and neg channels to Vss

#endif

	PIE1bits.ADIE = LOW; // the ADC interrupt enable bit
	IPR1bits.ADIP = HIGH; // ADC use high pri

	OpenSPI(SPI_FOSC_64, MODE_00, SMPEND);
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

	while (DataRdy1USART()) { // dump 1 rx data`
		Read1USART();
	};
	while (DataRdy2USART()) { // dump 2 rx data
		Read2USART();
	};

	/* System activity timer, can reset the processor */
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	WriteTimer0(TIMEROFFSET); //      start timer0 at 1 second ticks

	/* clear SPI module possible flag */
	PIR1bits.SSPIF = LOW;

	/* Enable interrupt priority */
	RCONbits.IPEN = HIGH;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = HIGH;
	INTCONbits.GIEL = HIGH;

	/* clear any SSP error bits */
	SSPCON1bits.WCOL = SSPCON1bits.SSPOV = LOW;
}

void main(void) /* SPI Master/Slave loopback */
{
	int16_t i, j, k = 0;

	config_pic(); // setup the slave for work
	putrs2USART("\r\r\r\r\r\r\n #### \x1b[7m SPI Slave Ready! \x1b[0m ####\r\n");
	putrs2USART(" #### \x1b[7m SPI Slave Ready! \x1b[0m ####\r\n");

	while (1) { // just loop and output results on DIAG LCD for 8722

		if (SSPCON1bits.WCOL || SSPCON1bits.SSPOV) { // check for overruns/collisions
			SSPCON1bits.WCOL = SSPCON1bits.SSPOV = 0;
			adc_error_count = adc_count - adc_error_count;
		}


		for (i = 0; i < 1; i++) {
			for (j = 0; j < 1; j++) {
			}
		}

		ringBufS_put(L.tx2b, 0b000000000);
		ringBufS_put(L.tx2b, 0b111111111);
		ringBufS_put(L.tx2b, 0b011111111);
		ringBufS_put(L.tx2b, 0b111111111);
		ringBufS_put(L.tx2b, 0b011111111);
		ringBufS_put(L.tx2b, 0b111111111);
		ringBufS_put(L.tx2b, 0b011111111);
		ringBufS_put(L.tx2b, 0b111111111);
		ringBufS_put(L.tx2b, 0b000000000);

		ringBufS_put(spi_link.tx1b, 0b000000000);
		ringBufS_put(spi_link.tx1b, 0b011111111);
		ringBufS_put(spi_link.tx1b, 0b101010101);
		ringBufS_put(spi_link.tx1b, 0b110101010);

		start_tx2();
		start_lcd();
		while (!ringBufS_empty(L.tx2b));
		while (!ringBufS_empty(spi_link.tx1b));
	}

}