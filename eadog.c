#include "eadog.h"

void init_display(void)
{
	RS = LOW; // send cmd
	CSB = LOW; //
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

void send__lcd_data(uint8_t data, uint8_t config)
{
	uint16_t symbol;

	symbol = (uint16_t) data + ((uint16_t) config << 8);
	ringBufS_put(spi_link.tx1b, symbol);
}

void send_lcd_cmd(uint8_t cmd, uint8_t config)
{
	uint16_t symbol;

	symbol = (uint16_t) cmd + ((uint16_t) config << 8);
	ringBufS_put(spi_link.tx1b, symbol);
}

void start_lcd(void)
{
	PIR1bits.SSPIF = 1;
	PIE1bits.SSPIE = 1;
}