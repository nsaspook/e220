#include "eadog.h"
#include <stdio.h>

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

#define eaDogM_Cls()             eaDogM_WriteCommand(EADOGM_CMD_CLR)
#define eaDogM_CursorOn()        eaDogM_WriteCommand(EADOGM_CMD_CURSOR_ON)
#define eaDogM_CursorOff()       eaDogM_WriteCommand(EADOGM_CMD_CURSOR_OFF)
#define eaDogM_DisplayOn()       eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_ON)
#define eaDogM_DisplayOff()      eaDogM_WriteCommand(EADOGM_CMD_DISPLAY_OFF)

/*
 * Init the EA DOGM163 in 8bit serial mode
 */
void init_display(void)
{
	stdout = _H_USER;
	ringBufS_put(spi_link.tx1b, 0x139);
	ringBufS_put(spi_link.tx1b, 0x11d);
	ringBufS_put(spi_link.tx1b, 0x150);
	ringBufS_put(spi_link.tx1b, 0x16c);
	ringBufS_put(spi_link.tx1b, 0x176); // contrast last 4 bits
	ringBufS_put(spi_link.tx1b, 0x138);
	ringBufS_put(spi_link.tx1b, 0x10f);
	ringBufS_put(spi_link.tx1b, 0x101);
	ringBufS_put(spi_link.tx1b, 0x106);
	start_lcd();
	while (!ringBufS_empty(spi_link.tx1b));
}

/*
 * bit 9 is unset for short spi delay (default)
 */
void send_lcd_data(uint8_t data)
{
	ringBufS_put(spi_link.tx1b, (uint16_t) data);
}

/*
 * set bit 9 to add long spi delay
 */
void send_lcd_cmd(uint8_t cmd)
{
	uint16_t symbol = 0;

	symbol = (uint16_t) cmd | 0b100000000;
	ringBufS_put(spi_link.tx1b, symbol);
}

/*
 * Trigger the SPI interrupt
 */
void start_lcd(void)
{
	spi_link.SPI_LCD = HIGH;
	PIR1bits.SSPIF = HIGH;
	PIE1bits.SSPIE = HIGH;
}

/*
 * 
 *            file: EA-DOGM_MIO.c
 *         version: 2.03
 *     description: Multi I/O driver for EA DOGM displays
 *                : Uses 8Bit, SPI HW or SPI SW (bitbang)
 *     written by : Michael Bradley (mbradley@mculabs.com)
 *   contributions: Imaginos (CCS forum), Emil Nad (8Bit testing)
 *                  jgschmidt (CCS forum)
 */

void eaDogM_WriteChr(char value)
{
	send_lcd_data(value);
	start_lcd();
}

int _user_putc(char c)
{
	eaDogM_WriteChr(c);
}

void eaDogM_WriteCommand(uint8_t cmd)
{
	send_lcd_cmd(cmd);
	start_lcd();
}

void eaDogM_SetPos(uint8_t r, uint8_t c)
{
	uint8_t cmdPos;
	cmdPos = EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c;
	eaDogM_WriteCommand(cmdPos);
}

void eaDogM_ClearRow(uint8_t r)
{
	uint8_t i;
	eaDogM_SetPos(r, 0);
	for (i = 0; i < EADOGM_COLSPAN; i++) {
		eaDogM_WriteChr(' ');
	}
}

void eaDogM_WriteString(char *strPtr)
{
	printf("%s", strPtr);
}

void eaDogM_WriteStringAtPos(uint8_t r, uint8_t c, char *strPtr)
{
	eaDogM_WriteCommand((EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c));
	printf("%s", strPtr);
}

void eaDogM_WriteIntAtPos(uint8_t r, uint8_t c, uint8_t i)
{
	eaDogM_WriteCommand((EADOGM_CMD_DDRAM_ADDR + (r * EADOGM_COLSPAN) + c));

	eaDogM_WriteChr(i / 10 + '0');
	eaDogM_WriteChr(i % 10 + '0');

}

// this writes a byte to the internal CGRAM (v2.02)
// format for ndx: 00CCCRRR = CCC = character 0 to 7, RRR = row 0 to 7

void eaDogM_WriteByteToCGRAM(uint8_t ndx, uint8_t data)
{
	uint8_t cmd;

	cmd = ndx & 0b00111111; // mask off upper to bits
	cmd = cmd | EADOGM_CMD_CGRAM_ADDR; // set bit cmd bits

	eaDogM_WriteCommand(cmd);
	eaDogM_WriteChr(data);

	// this is done to make sure we are back in data mode
	eaDogM_SetPos(0, 0);
}