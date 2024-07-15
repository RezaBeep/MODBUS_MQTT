/*
 * oled.h
 *
 *  Created on: May 3, 2024
 *      Author: reza
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_



#endif /* INC_OLED_H_ */



#include "main.h"


#define MAX_LINE 5		// 0 - 5


typedef struct {
	I2C_HandleTypeDef *pHi2c;
	uint8_t current_line;
}oled_t;



void oled_print(oled_t *pOled, const char* str);
void oled_printl(oled_t *pOled, const char* str);
void oled_init(oled_t *pOled, I2C_HandleTypeDef *pHi2c);
void oled_clear(oled_t *pOled);
