#include "eadog.h"

void init_display(void)
{
	RS = LOW; // send command
	CSB = LOW; //
	ringBufS_put(spi_link.tx1b, 0x039);
	ringBufS_put(spi_link.tx1b, 0x01d);
	ringBufS_put(spi_link.tx1b, 0x050);
	ringBufS_put(spi_link.tx1b, 0x06c);
	ringBufS_put(spi_link.tx1b, 0x07c);
	ringBufS_put(spi_link.tx1b, 0x038);
	ringBufS_put(spi_link.tx1b, 0x00f);
	ringBufS_put(spi_link.tx1b, 0x001);
	ringBufS_put(spi_link.tx1b, 0x006);
	start_lcd();
	while (!ringBufS_empty(spi_link.tx1b));

	ringBufS_put(spi_link.tx1b, 'A');
	ringBufS_put(spi_link.tx1b, 'O');
	ringBufS_put(spi_link.tx1b, 'K');
	RS = HIGH; // send data
	CSB = LOW; //
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