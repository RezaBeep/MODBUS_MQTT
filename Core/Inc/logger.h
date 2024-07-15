/*
 * log.h
 *
 *  Created on: Jul 3, 2024
 *      Author: reza
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "main.h"


typedef struct{
	UART_HandleTypeDef* pHuart;
} logger_t;


void logger_init(logger_t* pLogger, UART_HandleTypeDef* pHuart);
void logger_log(logger_t* pLogger, char* TAG, char* text);


#endif /* INC_LOGGER_H_ */
