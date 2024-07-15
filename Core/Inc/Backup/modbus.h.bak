/*
 * modbus.h
 *
 *  Created on: Apr 30, 2024
 *      Author: reza
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#endif /* INC_MODBUS_H_ */

#include "main.h"



#define TX_SIZE 8


typedef enum{
	MODBUS_RES_OK,
	MODBUS_RES_EXCEPTION,
	MODBUS_RES_UNKNOWN
}modbus_res_type;


typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t *pchTxBuffer;
	uint8_t *pchRxBuffer;
} MODBUS_MASTER_InitTypeDef;



typedef struct {
	uint8_t slave_addr;
	uint8_t function_code;
	uint16_t register_addr;
	uint16_t number_of_points;
} MODBUS_MASTER_req;


typedef struct {
	uint8_t slave_addr;
	uint8_t function_code;
	uint8_t byte_count;
	uint8_t *register_data;
	uint16_t crc;
} MODBUS_MASTER_res;



typedef struct {
	uint8_t slave_addr;
	uint8_t function_code;
	uint8_t exception_code;
	uint16_t crc;
} MODBUS_MASTER_exception;



void MODBUS_MASTER_init(
		MODBUS_MASTER_InitTypeDef *master,
		UART_HandleTypeDef *huart,
		uint8_t *pchTxBuffer,
		uint8_t *pchRxBuffer);
void MODBUS_MASTER_request(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint8_t function_code,
		uint16_t register_addr,
		uint16_t number_of_points);

modbus_res_type MODBUS_MASTER_response_handler(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		MODBUS_MASTER_res *pNormalRes,
		MODBUS_MASTER_exception *pException);

void MODBUS_MASTER_read_coils(MODBUS_MASTER_InitTypeDef *master, uint8_t slave_addr, uint16_t register_addr, uint16_t number_of_points);
























