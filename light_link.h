
/* 
 * File:   light_link.h
 * Author: root
 *
 * Created on July 20, 2015, 12:21 PM
 */

#ifndef LIGHT_LINK_H
#define	LIGHT_LINK_H

#include <GenericTypeDefs.h>

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
/*unsigned types*/
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
/*signed types*/
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;
#endif

#include "ll_defs.h"
#include  "ringbufs.h"

typedef enum {
	LL_OPEN, LL_LOOP, LL_E220, LL_VISION, LL_GSD, LL_VISTA
} mode_t;

typedef enum {
	RS232_HH, RS232_LL
} level_t;

struct link_buffer_type {
	uint8_t stream[LINK_B_SIZE];
};

struct spi_link_type { // internal state table
	uint8_t SPI_LCD : 1;
	uint8_t SPI_AUX : 1;
	uint8_t LCD_TIMER : 1;
	uint8_t LCD_DATA : 1;
	uint16_t delay;
	uint8_t config;
	struct ringBufS_t *tx1b, *tx1a;
	int32_t int_count;
};

typedef struct V_data { // OS Counters
	uint32_t highint_count, lowint_count, eeprom_count, timerint_count, adc_count, c1t_int, c2t_int, c1r_int, c2r_int, buttonint_count,
	clock50, pwm4int_count;
} V_data;

typedef struct L_data { // light link state data
	uint8_t ctmu_data : 1;
	uint8_t ctmu_data_temp : 1;
	uint8_t boot_code : 1;
	uint8_t adc_chan;
	uint8_t tx1_dac, tx2_dac;
	struct ringBufS_t *rx1b, *tx1b, *rx2b, *tx2b;
	mode_t omode;
	level_t rs232_mode;
	uint16_t ctmu_adc, ctmu_adc_zero, pic_temp, checksum;
} L_data;

typedef struct llflagtype {
	uint32_t crc1_error, crc2_error, frame1_error, frame2_error, overrun1_error, overrun2_error;
} llflagtype;

typedef struct hidtype {
	uint8_t bled_on : 1;
	uint8_t bled_flash : 1;
	uint8_t bled_flash_fast : 1;
	void (*t_on)(void);
	void (*t_off)(void);
} hidtype;

#ifdef	__cplusplus
}
#endif

#endif	/* LIGHT_LINK_H */

