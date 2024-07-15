/*
 * oled.c
 *
 *  Created on: May 3, 2024
 *      Author: reza
 */


#include "oled.h"
#include "ssd1306.h"



void oled_print(oled_t *pOled, const char* str){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(str, Font_7x10, White);
	ssd1306_UpdateScreen(pOled->pHi2c);
}



void oled_printl(oled_t *pOled, const char* str){
	if(pOled->current_line < 6){
		uint8_t y = pOled->current_line * 10;
		ssd1306_SetCursor(0, y);
		ssd1306_WriteString(str, Font_7x10, White);
		ssd1306_UpdateScreen(pOled->pHi2c);
		pOled->current_line++;
	}
	else{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(str, Font_7x10, White);
		ssd1306_UpdateScreen(pOled->pHi2c);
		pOled->current_line = 1;
	}
}


void oled_init(oled_t *pOled, I2C_HandleTypeDef *pHi2c){
	 ssd1306_Init(pHi2c);
	 ssd1306_Fill(Black);
	 ssd1306_SetCursor(0, 0);
	 ssd1306_WriteString("Loading...", Font_7x10, White);
	 ssd1306_UpdateScreen(pOled->pHi2c);
	 pOled->pHi2c = pHi2c;
}


void oled_clear(oled_t *pOled){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_UpdateScreen(pOled->pHi2c);
}
