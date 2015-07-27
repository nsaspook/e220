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

    void init_display(void);
    void send__lcd_data(uint8_t);
    void send_lcd_cmd(uint8_t);
    void send_lcd_go(void);

#ifdef	__cplusplus
}
#endif

#endif	/* EADOG_H */

