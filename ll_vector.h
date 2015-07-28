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

    void data_handler(void);
    void work_handler(void);
    int8_t start_tx1(void);
    int8_t start_tx2(void);

    extern volatile struct V_data V;
    extern volatile struct L_data L;
    extern volatile struct llflagtype ll_flag, ll_dumpflag;
    extern volatile int16_t tx_tmp, rx_tmp;
    extern volatile uint32_t adc_count, adc_error_count,
            slave_int_count, last_slave_int_count;
    extern volatile struct spi_link_type spi_link;
    extern volatile struct spi_stat_type spi_stat;
    extern volatile uint8_t data_in2, adc_buffer_ptr,
            adc_channel;
    extern volatile uint16_t adc_buffer[64], adc_data_in;
#ifdef	__cplusplus
}
#endif

#endif	/* LL_VECTOR_H */

