/*
 * crc16.c
 *
 *  Created on: Apr 27, 2024
 *      Author: reza
 */


#include "crc16.h"

uint16_t CRC16(uint8_t *puchMsg, unsigned short usDataLen ) /* quantity of bytes in message */
{
	uint8_t uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	uint8_t uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	unsigned uIndex ; /* will index into CRC lookup table */
	while (usDataLen--){
		uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}
