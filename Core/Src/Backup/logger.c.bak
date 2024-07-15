/*
 * logger.c
 *
 *  Created on: Jul 3, 2024
 *      Author: reza
 */


#include <logger.h>
#include "string.h"


void logger_init(logger_t* pLogger, UART_HandleTypeDef* pHuart){
	pLogger->pHuart = pHuart;
}




void logger_log(logger_t* pLogger, char* TAG, char* text){
	#ifdef _DEBUG
		if(_DEBUG){
			uint16_t len = strlen(TAG) + strlen(text);
			char logtext[len+3];
			strcat(logtext, TAG);
			strcat(logtext, ": ");
			strcat(logtext, text);
			strcat(logtext, "\n");
			HAL_UART_Transmit(pLogger->pHuart, logtext, len, 2000);
		}
	#endif
}
