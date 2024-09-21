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
#include "stdbool.h"



#define TX_SIZE 8
#define MODBUS_FC_RD_DO		1
#define MODBUS_FC_RD_DI		2
#define MODBUS_FC_RD_HR		3
#define MODBUS_FC_RD_IR		4
#define MODBUS_FC_WR_DO		5
#define MODBUS_FC_WR_HR		6

#define MODBUS_TX_SIZE	32
#define MODBUS_RX_SIZE	128

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
	uint16_t register_addr;
	uint16_t value;
} MODBUS_MASTER_wr_req;


typedef struct {
	uint8_t slave_addr;
	uint8_t function_code;
	uint16_t register_addr;
	uint16_t value;
	uint16_t crc;
} MODBUS_MASTER_wr_res;


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


modbus_res_type MODBUS_MASTER_response_check(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr);


void MODBUS_MASTER_read_coils(
		MODBUS_MASTER_InitTypeDef *master,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points);


void MODBUS_MASTER_read_input_reg(
		MODBUS_MASTER_InitTypeDef *master,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points);


void MODBUS_MASTER_read_holding_reg(
		MODBUS_MASTER_InitTypeDef *master,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points);


void MODBUS_MASTER_read_discrete_input(
		MODBUS_MASTER_InitTypeDef* pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points);


void MODBUS_MASTER_write_single_coil(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		bool output);


modbus_res_type MODBUS_MASTER_write_single_coil_blocking(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		bool output,
		uint16_t timeout);



void MODBUS_MASTER_write_single_holding_reg(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t val);



void modbus_transmit_485(MODBUS_MASTER_InitTypeDef* pMaster);






















