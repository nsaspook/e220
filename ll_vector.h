/* 
 * File:   ll_vector.h
 * Author: root
 *
 * Created on July 20, 2015, 2:28 PM
 */

#ifndef LL_VECTOR_H
#define	LL_VECTOR_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p18cxxx.h>
#include <usart.h>
#include <timers.h>
#include "light_link.h"
#include <EEP.h>
#include "eadog.h"

	volatile void s_crit(void);
	volatile void e_crit(void);
	void write_data_eeprom(uint8_t, uint8_t, uint16_t, uint16_t);
	uint8_t read_data_eeprom(uint16_t, uint16_t);
	void wipe_data_eeprom(uint16_t);

	void data_handler(void);
	void work_handler(void);
	int8_t start_tx1(void);
	int8_t start_tx2(void);
	void start_ctmu(void);
	uint16_t measure_chip_temp(uint8_t);
	void inttemp_init(void);
	uint16_t inttemp_deltav(void);
	void inttemp_calibrate(int16_t);
	int16_t inttemp_read(void);
	float lp_filter(float, uint8_t, int8_t);

	void b0_on(void);
	void b0_off(void);
	void b1_on(void);
	void b1_off(void);

	extern volatile union Timers timer_long;
	extern void wdtdelay(uint32_t);
	extern volatile struct V_data V;
	extern volatile struct L_data L;
	extern volatile struct llflagtype ll_flag;
	extern volatile int16_t tx_tmp, rx_tmp;
	extern volatile uint32_t adc_count, adc_error_count,
		slave_int_count, last_slave_int_count;
	extern volatile struct spi_link_type spi_link;
	extern volatile uint8_t data_in2, adc_buffer_ptr,
		adc_channel;
	extern volatile uint16_t adc_buffer[4], adc_data_in;
	extern volatile hidtype *hid0_ptr, *hid1_ptr;
#ifdef	__cplusplus
}
#endif

#endif	/* LL_VECTOR_H */

