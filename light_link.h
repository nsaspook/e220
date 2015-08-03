
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

struct spi_link_type { // internal state table
	uint8_t SPI_LCD : 1;
	uint8_t SPI_AUX : 1;
	uint8_t TIMER : 1;
	uint8_t DATA : 1;
	uint16_t delay;
	uint8_t config;
	struct ringBufS_t *tx1b, ring_buf1;
	int32_t int_count;
};

typedef struct V_data { // OS Counters
	uint32_t highint_count, lowint_count, eeprom_count, timerint_count, adc_count, c1t_int, c2t_int, c1r_int, c2r_int, buttonint_count,
	clock50, pwm4int_count;
} V_data;

typedef struct L_data { // light link state data
	uint8_t tx1_dac, tx2_dac;
	struct ringBufS_t *rx1b, *tx1b, *rx2b, *tx2b, ring_buf1, ring_buf2, ring_buf3, ring_buf4;
	mode_t omode;
	level_t rs232_mode;
	uint16_t checksum;
} L_data;

typedef struct llflagtype {
	uint32_t crc1_error, crc2_error, frame1_error, frame2_error, overrun1_error, overrun2_error;
} llflagtype;

#ifdef	__cplusplus
}
#endif

#endif	/* LIGHT_LINK_H */

