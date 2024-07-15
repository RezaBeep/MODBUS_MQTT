/*
 * at.c
 *
 *  Created on: Jun 24, 2024
 *      Author: reza
 */


#include <at.h>
#include <string.h>


char tx_buff[AT_TX_BUFF_SIZE] = "";
uint16_t RxLen;



at_state_t _at_response_error_check(char* rx_buff){
	if(find_substr(rx_buff, "ERROR")){
		return AT_STATE_ERROR;
	}
	return AT_STATE_OK;
}


bool find_substr(char* str, char* substr){
	char* pch = strstr(str, substr);
		if(pch != NULL){
			return true;
		}
		return false;
}


void flush_buff(char* buffer){
	memset(buffer,0,strlen(buffer));
	strcpy(buffer, "");
}



at_state_t at_read(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout){
	flush_buff(rx_buff);
	strcpy(tx_buff, "AT+");
	strcat(tx_buff,cmd);
	strcat(tx_buff, "?\r\n");
	uint8_t tx_size = strlen(tx_buff) + 1;
	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
	//	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
	HAL_UARTEx_ReceiveToIdle(pHuart, rx_buff, AT_RX_BUFF_SIZE, &RxLen, timeout);
	if(_at_response_error_check(rx_buff)){
//		char at_read_res[10] = "+";
//		strcat(at_read_res, cmd);
//		if(find_substr(rx_buff, at_read_res)){
			return AT_STATE_OK;
//		}
	}
	else{

	}
	return AT_STATE_ERROR;
}



at_state_t at_write(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout, uint8_t argc, ...){
	flush_buff(rx_buff);
	strcpy(tx_buff, "AT+");
	strcat(tx_buff,cmd);
	strcat(tx_buff, "=");
	va_list args;
	va_start(args,argc);
	for(int i=0;i<argc;i++){
		strcat(tx_buff, (va_arg(args, char*)));
	}
	strcat(tx_buff, "\r\n");
	va_end(args);

	uint8_t tx_size = strlen(tx_buff) + 1;

	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
//	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
	HAL_UARTEx_ReceiveToIdle(pHuart, rx_buff, AT_RX_BUFF_SIZE, &RxLen, timeout);
	if(_at_response_error_check(rx_buff)){
		if(find_substr(rx_buff, AT_OK)){
			return AT_STATE_OK;
		}
//		else if(find_substr(rx_buff, ">")){
//			return AT_STATE_ENTER_INPUT;
//		}
	}
	else{

	}

	return AT_STATE_ERROR;
}



at_state_t at_execute(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout){
	flush_buff(rx_buff);
	if(strlen(cmd) > 0){
		strcpy(tx_buff, "AT+");
	}
	else{
		strcpy(tx_buff, "AT");
	}
	strcat(tx_buff,cmd);
	strcat(tx_buff, "\r\n");
	uint8_t tx_size = strlen(tx_buff) + 1;
	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
	//	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
	HAL_UARTEx_ReceiveToIdle(pHuart, rx_buff, AT_RX_BUFF_SIZE, &RxLen, timeout);
	if(_at_response_error_check(rx_buff)){
		if(find_substr(rx_buff, AT_OK)){
			return AT_STATE_OK;
		}
	}
	return AT_STATE_ERROR;
}





at_state_t at_read_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout){
	flush_buff(rx_buff);
	strcpy(tx_buff, "AT+");
	strcat(tx_buff,cmd);
	strcat(tx_buff, "?\r\n");
	uint8_t tx_size = strlen(tx_buff) + 1;
	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
//	HAL_UARTEx_ReceiveToIdle(pHuart, rx_buff, AT_RX_BUFF_SIZE, &RxLen, timeout);
	if(_at_response_error_check(rx_buff)){
//		char at_read_res[10] = "+";
//		strcat(at_read_res, cmd);
//		if(find_substr(rx_buff, at_read_res)){
			return AT_STATE_OK;
//		}
	}
	else{

	}
	return AT_STATE_ERROR;

}





at_state_t at_write_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout, uint8_t argc, ...){
	flush_buff(rx_buff);
	strcpy(tx_buff, "AT+");
	strcat(tx_buff,cmd);
	strcat(tx_buff, "=");
	va_list args;
	va_start(args,argc);
	for(int i=0;i<argc;i++){
		strcat(tx_buff, (va_arg(args, char*)));
	}
	strcat(tx_buff, "\r\n");
	va_end(args);

	uint8_t tx_size = strlen(tx_buff) + 1;

	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
	if(_at_response_error_check(rx_buff)){
		if(find_substr(rx_buff, AT_OK)){
			return AT_STATE_OK;
		}
		if(find_substr(rx_buff, ">")){
			return AT_STATE_ENTER_INPUT;
		}
	}
	else{

	}

	return AT_STATE_ERROR;

}






at_state_t at_execute_blocking(UART_HandleTypeDef* pHuart, char* rx_buff, char* cmd, uint16_t timeout){
	flush_buff(rx_buff);
	if(strlen(cmd) > 0){
		strcpy(tx_buff, "AT+");
	}
	else{
		strcpy(tx_buff, "AT");
	}
	strcat(tx_buff,cmd);
	strcat(tx_buff, "\r\n");
	uint8_t tx_size = strlen(tx_buff) + 1;
	HAL_UART_Transmit_IT(pHuart, tx_buff, tx_size);
	HAL_UART_Receive(pHuart, rx_buff, AT_RX_BUFF_SIZE, timeout);
	if(_at_response_error_check(rx_buff)){
		if(find_substr(rx_buff, AT_OK)){
			return AT_STATE_OK;
		}
	}
	return AT_STATE_ERROR;

}
