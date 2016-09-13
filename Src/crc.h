/*
 * crc.h
 *
 *  Created on: 8Sep.,2016
 *      Author: paul
 */

#include <stdint.h>

#ifndef CRC_H_
#define CRC_H_

uint16_t crc16_ccitt( const void* buf, int len );

#endif /* CRC_H_ */
