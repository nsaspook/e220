/* 
 * File:   light_link.h
 * Author: root
 *
 * Created on July 20, 2015, 12:21 PM
 */

#ifndef LIGHT_LINK_H
#define	LIGHT_LINK_H

#include <GenericTypeDefs.h>
#include "ll_defs.h"

#ifdef	__cplusplus
extern "C" {
#endif


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

	struct spi_link_type { // internal state table
		uint8_t SPI_DATA : 1;
		uint8_t ADC_DATA : 1;
		uint8_t PORT_DATA : 1;
		uint8_t CHAR_DATA : 1;
		uint8_t REMOTE_LINK : 1;
		uint8_t REMOTE_DATA_DONE : 1;
		uint8_t LOW_BITS : 1;
	};

	struct spi_stat_type {
		volatile uint32_t adc_count, adc_error_count,
		port_count, port_error_count,
		char_count, char_error_count,
		slave_int_count, last_slave_int_count;
	};

	typedef struct V_data { // OS Counters
		uint32_t highint_count, lowint_count, eeprom_count, timerint_count, adc_count, mbmcdata_count, c1rx_int, c1tx_int, c2_int, buttonint_count,
		clock50, pwm4int_count;
	} V_data;

	typedef struct L_data { // light link state data
		uint32_t dac;
	} L_data;

	typedef struct llflagtype {
		uint16_t mbmc_cmd, mbmc_data, mbmc_ack;
		uint16_t host_cmd, host_data, host_ack;
		uint32_t cmd_timeout, host_timeout, data_timeout, data_len, data_pos;
		uint8_t rx_9bit, tx_9bit, mbmc_done, host_done, *data_ptr;
	} volatile llflagtype;

#ifdef	__cplusplus
}
#endif

#endif	/* LIGHT_LINK_H */

