/* 
 * File:   eadog.h
 * Author: root
 *
 * Created on July 27, 2015, 2:05 PM
 */

#ifndef EADOG_H
#define	EADOG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <spi.h>
#include "light_link.h"

	extern volatile struct spi_link_type spi_link;
	extern void wdtdelay(uint32_t);

	void init_display(void);
	void send_lcd_data(uint8_t);
	void send_lcd_cmd(uint8_t);
	void start_lcd(void);
	void wait_lcd(void);

	void eaDogM_WriteChr(char);
	void eaDogM_WriteCommand(uint8_t);
	void eaDogM_SetPos(uint8_t, uint8_t);
	void eaDogM_ClearRow(uint8_t);
	void eaDogM_WriteString(char *);
	void eaDogM_WriteStringAtPos(uint8_t, uint8_t, char *);
	void eaDogM_WriteIntAtPos(uint8_t, uint8_t, uint8_t);
	void eaDogM_WriteByteToCGRAM(uint8_t, uint8_t);

#ifdef	__cplusplus
}
#endif

#endif	/* EADOG_H */

