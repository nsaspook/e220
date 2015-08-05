#ifndef CRC_H_INCLUDED
#define CRC_H_INCLUDED
#define poly 0x1021
#include "light_link.h"

uint8_t crc8_calculate(uint8_t*, uint16_t);
uint16_t crc16(uint8_t*, uint16_t, uint16_t); // XMODEM crc-16
uint16_t crc16_ccitt(uint8_t*, uint16_t); // CCITT

#endif /* CRC_H_INCLUDED */