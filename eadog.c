#include "eadog.h"

void init_display(void)
{

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