/*
 * crc.h
 *
 *	Created on: 22 aug. 2016
 *	Author: Sterna
 *	The contents of this file is for internal use at CELLINK only.
 *	You are not allowed to distribute the contents of this file without prior written approval from CELLINK AB.
 *	If you have received this file in error, you are kindly asked to
 *	notify us of the reception and delete the file.
 *	Copyright (c) CELLINK AB
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>
#include <stdlib.h>
//The first data for the crc
static const uint16_t CRC_INIT_BYTE16=0xCAFE;

uint16_t crcCalculate16(uint16_t initData, const void *c_ptr, size_t len);

#endif /* CRC_H_ */
