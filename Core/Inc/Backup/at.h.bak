/*
 * at.h
 *
 *  Created on: Jun 24, 2024
 *      Author: reza
 */

#ifndef INC_AT_H_
#define INC_AT_H_



#include "main.h"
#include <stdbool.h>
#include <stdarg.h>


#define AT_RX_BUFF_SIZE 50
#define AT_TX_BUFF_SIZE 50


#define AT_ERROR "ERROR"
#define AT_OK "OK"


typedef enum{
	AT_STATE_ERROR = 0,
	AT_STATE_OK = 1,
	AT_STATE_ENTER_INPUT
}at_state_t;

at_state_t _at_response_error_check(char* rx_buff);


at_state_t at_read(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout);
at_state_t at_write(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout, uint8_t argc, ...);
at_state_t at_execute(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout);

at_state_t at_read_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout);
at_state_t at_write_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout, uint8_t argc, ...);
at_state_t at_execute_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout);


void flush_buff(char* buffer);
bool find_substr(char* str, char* substr);


#endif /* INC_AT_H_ */
