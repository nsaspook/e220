#include "eadog.h"

/*
 * Init the EA DOGM163 in 8bit serial mode
 */
void init_display(void)
{
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