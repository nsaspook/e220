/*
 *            file: EA-DOGM_MIO.c
 *         version: 2.03
 *     description: Multi I/O driver for EA DOGM displays
 *                : Uses 8Bit, SPI HW or SPI SW (bitbang)
 *     written by : Michael Bradley (mbradley@mculabs.com)
 *   contributions: Imaginos (CCS forum), Emil Nad (8Bit testing)
 *                  jgschmidt (CCS forum)
 *
 *   Changelog:
 *       04/22/2010 v2.03 Minor update, changed EADOGM_SPI_HW for HW SPI 1 and 2
 *                  Added setup_spi() for HW SPI use, and spi_clk_div option
 *                  thanks to jgschmidt (ccs forum) for noticing hw setup issues
 *       12/03/2009 v2.02 bug fix and printChr change, no user change needed for use
 *       11/25/2009 v2.01 Minor changes, replaced some functions with macros
 *       09/17/2009 v2.00 Major change, IO is now 8Bit or SPI (hw or sw)
 *                  rewrote some defines as related to pins
 *       09/15/2009 v1.13 added function eaDogM_WriteIntAtPos()
 *                  and eaDogM_WriteStringAtPos()
 *                  added some defines to clean source
 *       08/28/2009 v1.12 EADOGM_COLSPAN defined internally, allowing for
 *                  162 model to position correctly (noted by Imaginos, CCS forum)
 *                  Added support for 2nd SPI HW port (suggested by Imaginos, CCS forum)
 *                  defined a few more special chr's
 *       07/12/2009 v1.11 Fixed #define error surrounding BB SPI ini
 *                  added eaDogM_ClearRow(row);
 *       07/11/2009 Created/Consolidated this file from my testing sets
 *
 * Usage:
 *   In your source, you need to define which EA DOGM display you are using,
 *   there are 3 units supported EADOGM081 , EADOGM162 , EADOGM163 To use
 *   define with a 1.
 *
 * #define EADOGM081 1  // to use MODEL EADOG081
 * #define EADOGM162 1  // to use MODEL EADOG162
 * #define EADOGM163 1  // to use MODEL EADOG163
 * #define EADOGMVDD  5   // 5v LCD Vdd
 * //#define EADOGMVDD  3   // 3.3v LCD Vdd
 *
 *
 * // we need to define the IO mode we want, select only one of these, set to 1
 * #define EADOGM_SPI_HW  1   // hw spi, uses on chip spi
 * #define EADOGM_SPI_HW  2   // hw spi, 2nd on chip spi
 * #define EADOGM_SPI_DIV SPI_CLK_DIV_64 // used to slow hw spi clock (you can use other constants)
 * #define EADOGM_SPI_SW  1   // sw bit bang, can use any io pins
 * #define EADOGM_8BIT    1   // 8 bit data bus
 *
 * // for 8Bit mode only, we need to define the output port
 * // however, this can be our own function if need be.
 * // example shows output to port b
 * #define EADOGM_8BIT_PORT(d) output_b(d);   // we need to define how the byte goes out
 * //#define EADOGM_8BIT_PORT(d) your_func(d);   // we need to define how the byte goes out
 *
 *
 * // we need to define a few pins
 * #define EADOGM_PIN_RS  PIN_C1   // RS line, (pin 39 on the LCD)
 * #define EADOGM_PIN_CSB PIN_C2   // /CSB line, (pin 38 on the LCD) Req for SPI Opt for 8BIT
 *
 * // for 8 bit mode, we need to define a few more pins
 * #define EADOGM_NOCSB 1         // set to 1 if pin 38 (CSB) on lcd is tied to Vss
 * #define EADOGM_PIN_E   PIN_C2   // E (pin 36 on the LCD)
 * #define EADOGM_PIN_RW  PIN_C6   // RW (pin 37 on the LCD)
 *
 * // set these if you are using EADOGM_SPI_SW (bit bang)
 * #define EADOGM_SCLK_BB PIN_C3 // Connects to pin 29 on LCD
 * #define EADOGM_MOSI_BB PIN_C5 // Connects to pin 28 on LCD
 *
 *
 *
 * #include "EA-DOGM_MIO.c"
 *
 *    In your main code, do an ini call
 * eaDogM_Initialize();
 * eaDogM_DisplayOn();
 *
 *
 * Available Functions:
 * -------------------------------------------------------------------------------
 *    eaDogM_Cls();                 // clears the screen, homes cursor
 *    eaDogM_ClearRow(row);         // clears a row (v1.11)
 *    eaDogM_CursorOn();            // turns on the cursor
 *    eaDogM_CursorOff();           // turns of the cursor
 *    eaDogM_DisplayOn();           // turns on display with cursor off
 *    eaDogM_DisplayOff();          // turns off display
 *    eaDogM_SetPos(row, col);      // sets position row:0-2, col:0-15
 *    eaDogM_WriteChr(byte);        // writes a single chr to the display
 *    eaDogM_WriteString(char str); // writes a string to the display
 *                                  // note: add this line after device selection
 *                                  // to do this: eaDogM_WriteString("Constant")
 *                                  //   #device PASS_STRINGS=IN_RAM
 *
 *    // writes a 2 digit integer at row,col set flag = 1 to disable interrupts
 *    eaDogM_WriteIntAtPos(row,col,int[,flag])
 *
 *    // writes a string at row,col set flag = 1 to disable interrupts
 *    eaDogM_WriteStringAtPos(row,col,char str[,flag])
 *
 *    eaDogM_SetContrast(c);      // set contrast 0 to 15
 *    eaDogM_DoubleHeight(row);     // turn on double height, row = 0 or 1
 *    eaDogM_DoubleHeightOff();     // turn off double height
 * -------------------------------------------------------------------------------
 *
 */



#ifndef EADOGM_SPI
#define EADOGM_SPI 1

// some special symbol chrs defined
#define EADMSYM_DEG     0b11011111     // degree symbol
#define EADMSYM_DARWL   0b11111011     // double <<
#define EADMSYM_DARWR   0b11111100     // double >>
#define EADMSYM_LT      0b00111100     // less than <
#define EADMSYM_GT      0b00111110     // greater than >
#define EADMSYM_OHM     0b00011110     // ohm symbol

// some command defines
#define EADMCMD_CONTRAST 0b0111000      // contrast command (0b0111xxxx)


// we noticed some issues with GLOBAL on pic24 devices
#ifndef GLOBAL
#define GLOBAL INTR_GLOBAL
#endif


// 1x16
#ifdef EADOGM081
#define EADOGM_ROWS 1
#if EADOGMVDD == 5
#define EADOGM_INIT_BIAS_SET 0x1C
#define EADOGM_INIT_POWER_CONTROL 0x51
#define EADOGM_INIT_FOLLOWER_CONTROL 0x6A
#define EADOGM_INIT_CONTRAST_SET 0x74
#else
#define EADOGM_INIT_BIAS_SET 0x14
#define EADOGM_INIT_POWER_CONTROL 0x55
#define EADOGM_INIT_FOLLOWER_CONTROL 0x6D
#define EADOGM_INIT_CONTRAST_SET 0x7C
#endif

#define EADOGM_INIT_FS1 0x31
#define EADOGM_INIT_FS2 0x30
#define EADOGM_INIT_CLEAR_DISPLAY 0x01
#define EADOGM_INIT_ENTRY_MODE 0x06
#define EADOGM_COLSPAN 16
#endif


// 2x16
#ifdef EADOGM162
#define EADOGM_ROWS 2
#if EADOGMVDD == 5
#define EADOGM_INIT_BIAS_SET 0x1C
#define EADOGM_INIT_POWER_CONTROL 0x52
#define EADOGM_INIT_FOLLOWER_CONTROL 0x69
#define EADOGM_INIT_CONTRAST_SET 0x74
#else
#define EADOGM_INIT_BIAS_SET 0x14
#define EADOGM_INIT_POWER_CONTROL 0x55
#define EADOGM_INIT_FOLLOWER_CONTROL 0x6D
#define EADOGM_INIT_CONTRAST_SET 0x78
#endif

#define EADOGM_INIT_FS1 0x39
#define EADOGM_INIT_FS2 0x38
#define EADOGM_INIT_CLEAR_DISPLAY 0x01
#define EADOGM_INIT_ENTRY_MODE 0x06
#define EADOGM_COLSPAN 40  // suggested that this be 40 on model 162
#endif

// 3x16
#ifdef EADOGM163
#define EADOGM_ROWS 3
#if EADOGMVDD == 5
#define EADOGM_INIT_BIAS_SET 0x1D
#define EADOGM_INIT_POWER_CONTROL 0x50
#define EADOGM_INIT_FOLLOWER_CONTROL 0x6C
#define EADOGM_INIT_CONTRAST_SET 0x7C
#else
#define EADOGM_INIT_BIAS_SET 0x15
#define EADOGM_INIT_POWER_CONTROL 0x55
#define EADOGM_INIT_FOLLOWER_CONTROL 0x6E
#define EADOGM_INIT_CONTRAST_SET 0x72
#endif

#define EADOGM_INIT_FS1 0x39
#define EADOGM_INIT_FS2 0x38
#define EADOGM_INIT_CLEAR_DISPLAY 0x01
#define EADOGM_INIT_ENTRY_MODE 0x06
#define EADOGM_COLSPAN 16
#endif


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


// spi hw clock div, v2.03 fix
#ifndef EADOGM_SPI_DIV
#define EADOGM_SPI_DIV SPI_CLK_DIV_4
#endif



// sw spi emulation routine (bit bang)
#ifdef EADOGM_SPI_SW
#ifndef EADOGM_SCLK_BB
#define EADOGM_SCLK_BB PIN_C3
#endif
#ifndef EADOGM_MOSI_BB
#define EADOGM_MOSI_BB PIN_C5
#endif

void eaDogM_iniSPI_BB(void)
{
	output_drive(EADOGM_SCLK_BB);
	output_drive(EADOGM_MOSI_BB);
	output_low(EADOGM_SCLK_BB);
	output_low(EADOGM_MOSI_BB);
}

void eaDogM_spiWrite_BB(int8 regData)
{

	int1 bitOut;
	int8 SPICount; // Counter used to clock out the data
	int8 SPIData; // Define a data structure for the SPI data.

	output_low(EADOGM_SCLK_BB); // and CK low

	SPIData = regData;
	for (SPICount = 0; SPICount < 8; SPICount++) // Prepare to clock out the Address byte
	{
		bitOut = bit_test(SPIData, 7);
		output_bit(EADOGM_MOSI_BB, bitOut);
		output_high(EADOGM_SCLK_BB); // Toggle the clock line
		//delay_us(10);
		output_low(EADOGM_SCLK_BB);
		//delay_us(10);
		SPIData = SPIData << 1; // Rotate to get the next bit
	} // and loop back to send the next bit

	output_low(EADOGM_MOSI_BB);
}

// wrapper for sw spi calls in main program
// v2.01 moved to macro define
#define eaDogM_outSPI(c) eaDogM_spiWrite_BB(c)
#endif

// wrapper for hw spi calls in main program
// v2.01 moved to a macro define
#if EADOGM_SPI_HW == 1
#define eaDogM_outSPI(c) spi_write(c)
#endif

// wrapper for hw2 spi calls in main program
// v2.01 moved to a macro define, v2.03 using EADOGM_SPI_HW to test condition
#if EADOGM_SPI_HW == 2
#define eaDogM_outSPI(c) spi_write2(c)
#endif


#ifdef EADOGM_8BIT

void eaDogM_ini8Bit(void)
{
#ifndef EADOGM_NOCSB
	output_drive(EADOGM_PIN_CSB);
	output_high(EADOGM_PIN_CSB);
#endif
	output_drive(EADOGM_PIN_E);
	output_drive(EADOGM_PIN_RW);
	output_drive(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_E);
	output_low(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_RW);
}
#endif



#ifdef EADOGM_8BIT
// 8bit mode

void eaDogM_WriteChr(char value)
{
	output_high(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_RW);
	output_low(EADOGM_PIN_E);
#ifndef EADOGM_NOCSB
	output_low(EADOGM_PIN_CSB);
#endif
	output_high(EADOGM_PIN_E);
	delay_ms(1);
	EADOGM_8BIT_PORT(value);
	output_low(EADOGM_PIN_E);
	delay_ms(1);
#ifndef EADOGM_NOCSB
	output_low(EADOGM_PIN_CSB);
#endif
	delay_ms(1);
}

void eaDogM_WriteCommand(int8 cmd)
{
	output_low(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_RW);
	output_low(EADOGM_PIN_E);
#ifndef EADOGM_NOCSB
	output_low(EADOGM_PIN_CSB);
#endif
	output_high(EADOGM_PIN_E);
	delay_ms(1);
	EADOGM_8BIT_PORT(cmd);
	output_low(EADOGM_PIN_E);
	delay_ms(1);
#ifndef EADOGM_NOCSB
	output_low(EADOGM_PIN_CSB);
#endif
	delay_ms(1);
}
#else
// spi mode

void eaDogM_WriteChr(char value)
{
	output_high(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_CSB);
	eaDogM_outSPI(value);
	output_high(EADOGM_PIN_CSB);
	delay_ms(1);
}

void eaDogM_WriteCommand(int8 cmd)
{
	output_low(EADOGM_PIN_RS);
	output_low(EADOGM_PIN_CSB);
	eaDogM_outSPI(cmd);
	output_high(EADOGM_PIN_CSB);
	delay_ms(1);
}
#endif

void eaDogM_Initialize(void)
{

	// v2.03 fix
#if EADOGM_SPI_HW  == 1
	setup_spi(SPI_MASTER | SPI_H_TO_L | EADOGM_SPI_DIV);
#endif

	// v2.03 fix
#if EADOGM_SPI_HW  == 2
	setup_spi2(SPI_MASTER | SPI_H_TO_L | EADOGM_SPI_DIV);
#endif

#ifdef EADOGM_SPI_SW
	eaDogM_iniSPI_BB();
#endif

#ifdef EADOGM_8BIT
	eaDogM_ini8Bit();
#else
	output_drive(EADOGM_PIN_CSB);
	output_drive(EADOGM_PIN_RS);
	output_high(EADOGM_PIN_CSB);
	output_high(EADOGM_PIN_RS);
#endif

	delay_ms(200);

	eaDogM_WriteCommand(EADOGM_INIT_FS1);
	eaDogM_WriteCommand(EADOGM_INIT_BIAS_SET);
	eaDogM_WriteCommand(EADOGM_INIT_POWER_CONTROL);
	eaDogM_WriteCommand(EADOGM_INIT_FOLLOWER_CONTROL);
	eaDogM_WriteCommand(EADOGM_INIT_CONTRAST_SET);
	eaDogM_WriteCommand(EADOGM_INIT_FS2);
	eaDogM_WriteCommand(EADOGM_INIT_CLEAR_DISPLAY);
	eaDogM_WriteCommand(EADOGM_INIT_ENTRY_MODE);

}

// sets contrast, call with a value from 0 to 15
// we also mask off upper 4 bits from c
// v2.01 moved to a macro define
#define eaDogM_SetContrast(c) eaDogM_WriteCommand(EADMCMD_CONTRAST + (c & 0b00001111))

// only tested on 3 line display at the moment,
// thus no constants defined. when fully tested, I will define them

void eaDogM_DoubleHeight(int8 row) // row 0 or 1
{
	eaDogM_WriteCommand(EADOGM_CMD_SET_TABLE2); // set instruction table 2
	if (row == 0) {
		eaDogM_WriteCommand(EADOGM_CMD_SELECT_R0); // select row 0
	}
	if (row == 1) {
		eaDogM_WriteCommand(EADOGM_CMD_SELECT_R1); // select row 1
	}
	eaDogM_WriteCommand(0b00101100); // turns on double line mode
	// and set instruction table back to 0
}

// v2.01 moved functions to macros
#define eaDogM_DoubleHeightOff() eaDogM_WriteCommand(0b00101000)
#define eaDogM_Cls()             eaDogM_WriteCommand(EADOGM_CMD_CLR)
#define eaDogM_CursorOn()        eaDogM_WriteCommand(EADOGM_CMD_CURSOR_ON)
#define eaDogM_CursorOff()       eaDogM_WriteCommand(EADOGM_CMD_CURSOR_OFF)
#define eaDogM_DisplayOn()       eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_ON)
#define eaDogM_DisplayOff()      eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_OFF)

void eaDogM_SetPos(int8 r, int8 c)
{
	int8 cmdPos;
	cmdPos = EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c;
	eaDogM_WriteCommand(cmdPos);
}

void eaDogM_ClearRow(int8 r)
{
	int8 i;
	eaDogM_SetPos(r, 0);
	for (i = 0; i < EADOGM_COLSPAN; i++) {
		eaDogM_WriteChr(' ');
	}
}

void eaDogM_WriteString(char *strPtr)
{
	printf(eaDogM_WriteChr, "%s", strPtr);
}


// Optional DisGIE, set to 1 to disable interrupts
// v1.4 -- provided by Imaginos

void eaDogM_WriteStringAtPos(int8 r, int8 c, char *strPtr, int1 DisGIE = 0)
{
	if (DisGIE) {
		disable_interrupts(GLOBAL);
	}

	eaDogM_WriteCommand((EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c));
	printf(eaDogM_WriteChr, "%s", strPtr);

	if (DisGIE) {
		enable_interrupts(GLOBAL);
	}
}

// Optional DisGIE, set to 1 to disable interrupts
// v1.4 -- provided by Imaginos

void eaDogM_WriteIntAtPos(int8 r, int8 c, int8 i, int1 DisGIE = 0)
{
	if (DisGIE) {
		disable_interrupts(GLOBAL);
	}

	eaDogM_WriteCommand((EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c));

	eaDogM_WriteChr(i / 10 + '0');
	eaDogM_WriteChr(i % 10 + '0');

	if (DisGIE) {
		enable_interrupts(GLOBAL);
	}
}

// this writes a byte to the internal CGRAM (v2.02)
// format for ndx: 00CCCRRR = CCC = character 0 to 7, RRR = row 0 to 7

void eaDogM_WriteByteToCGRAM(char ndx, char data)
{
	unsigned int cmd;

	cmd = ndx & 0b00111111; // mask off upper to bits
	cmd = cmd | EADOGM_CMD_CGRAM_ADDR; // set bit cmd bits

	eaDogM_WriteCommand(cmd);
	eaDogM_WriteChr(data);

	// this is done to make sure we are back in data mode
	eaDogM_SetPos(0, 0);
}


#endif


