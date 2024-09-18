/*
 * modbus.c
 *
 *  Created on: Apr 30, 2024
 *      Author: reza
 */


#include "modbus.h"
#include "crc16.h"
#include "string.h"



void _null(void** p){
	*p = NULL;
}



void MODBUS_MASTER_init(
		MODBUS_MASTER_InitTypeDef *master,
		UART_HandleTypeDef *huart,
		uint8_t *pchTxBuffer,
		uint8_t *pchRxBuffer)
{
	master->huart = huart;
	master->pchRxBuffer = pchRxBuffer;
	master->pchTxBuffer = pchTxBuffer;
}


void MODBUS_MASTER_request(MODBUS_MASTER_InitTypeDef *pMaster, uint8_t slave_addr, uint8_t function_code, uint16_t register_addr, uint16_t number_of_points){
	uint16_t rx_size = 5;


	pMaster->pchTxBuffer[0] = slave_addr;
	pMaster->pchTxBuffer[1] = function_code;
	pMaster->pchTxBuffer[2] = register_addr>>8;
	pMaster->pchTxBuffer[3] = register_addr;
	pMaster->pchTxBuffer[4] = number_of_points>>8;
	pMaster->pchTxBuffer[5] = number_of_points;
	uint16_t crc = CRC16(pMaster->pchTxBuffer, 6);
	pMaster->pchTxBuffer[6] = crc&0xff;
	pMaster->pchTxBuffer[7] = (crc>>8)&0xff;


	HAL_UART_Transmit_IT(pMaster->huart, pMaster->pchTxBuffer, TX_SIZE);
	HAL_UART_Receive_DMA(pMaster->huart, pMaster->pchRxBuffer, rx_size);
}



modbus_res_type MODBUS_MASTER_response_handler(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		MODBUS_MASTER_res *pNormalRes,
		MODBUS_MASTER_exception *pException
		)
{
	uint8_t rx_size = strlen(pMaster->pchRxBuffer);
	if(pMaster->pchRxBuffer[0] == slave_addr){

		if(pMaster->pchRxBuffer[1] >= 0x80){
			// Exception
			pNormalRes = NULL;
			pException->slave_addr = pMaster->pchRxBuffer[0];
			pException->function_code = pMaster->pchRxBuffer[1];
			pException->exception_code = pMaster->pchRxBuffer[2];
			pException->crc = (pMaster->pchRxBuffer[3] >> 8) | (pMaster->pchRxBuffer[4]);

			return MODBUS_RES_EXCEPTION;
		}

		else{
			pNormalRes->slave_addr = pMaster->pchRxBuffer[0];
			pNormalRes->function_code = pMaster->pchRxBuffer[1];
			pNormalRes->byte_count = pMaster->pchRxBuffer[2];

			uint8_t byte_cnt = pNormalRes->byte_count;
			pNormalRes->crc = pMaster->pchRxBuffer + 3 + byte_cnt;
			memset(pMaster->pchRxBuffer+3+byte_cnt, 0, strlen(pNormalRes->crc));	// remove crc from rxBuff

			pNormalRes->register_data = pMaster->pchRxBuffer+3;


			//clear crc from response


			return MODBUS_RES_OK;
		}
	}
	else{
		return MODBUS_RES_UNKNOWN;
	}
}



modbus_res_type MODBUS_MASTER_response_check(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr)
{
	uint8_t rx_size = strlen(pMaster->pchRxBuffer);
	if(pMaster->pchRxBuffer[0] == slave_addr){
		uint8_t fc = pMaster->pchRxBuffer[1];
		if(pMaster->pchRxBuffer[1] >= 0x80){
			// Exception
			return MODBUS_RES_EXCEPTION;
		}

		else{
//			*data = pMaster->pchRxBuffer[3]<<8 | pMaster->pchRxBuffer[4];
			return MODBUS_RES_OK;
		}
	}
	else{
		return MODBUS_RES_UNKNOWN;
	}


}



void MODBUS_MASTER_read_coils(MODBUS_MASTER_InitTypeDef *pMaster, uint8_t slave_addr, uint16_t register_addr, uint16_t number_of_points){
	MODBUS_MASTER_request(pMaster, slave_addr, MODBUS_FC_RD_DO, register_addr, number_of_points);
}





void MODBUS_MASTER_read_input_reg(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points)
{
	MODBUS_MASTER_request(pMaster, slave_addr, MODBUS_FC_RD_IR, register_addr, number_of_points);
}



void MODBUS_MASTER_read_holding_reg(
		MODBUS_MASTER_InitTypeDef* pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points)
{
	MODBUS_MASTER_request(pMaster, slave_addr, MODBUS_FC_RD_HR, register_addr, number_of_points);
}




void MODBUS_MASTER_read_discrete_input(
		MODBUS_MASTER_InitTypeDef* pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t number_of_points)
{
	MODBUS_MASTER_request(pMaster, slave_addr, MODBUS_FC_RD_DI, register_addr, number_of_points);
}



void MODBUS_MASTER_write_single_coil(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		bool output)
{
	uint16_t rx_size = 5;
	uint8_t function_code = MODBUS_FC_WR_DO;

	uint16_t value = 0;
	if(output){
		value = 0xFF00;
	}

	pMaster->pchTxBuffer[0] = slave_addr;
	pMaster->pchTxBuffer[1] = function_code;
	pMaster->pchTxBuffer[2] = register_addr>>8;
	pMaster->pchTxBuffer[3] = register_addr;
	pMaster->pchTxBuffer[4] = value>>8;
	pMaster->pchTxBuffer[5] = value;
	uint16_t crc = CRC16(pMaster->pchTxBuffer, 6);
	pMaster->pchTxBuffer[6] = crc&0xff;
	pMaster->pchTxBuffer[7] = (crc>>8)&0xff;


	HAL_UART_Transmit_IT(pMaster->huart, pMaster->pchTxBuffer, TX_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(pMaster->huart, pMaster->pchRxBuffer, rx_size);
}



modbus_res_type MODBUS_MASTER_write_single_coil_blocking(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		bool output,
		uint16_t timeout)
{
	memset(pMaster->pchRxBuffer, '\0', MODBUS_RX_SIZE);
	uint16_t rx_size = 5;
	uint8_t function_code = MODBUS_FC_WR_DO;

	uint16_t value = 0;
	if(output){
		value = 0xFF00;
	}

	pMaster->pchTxBuffer[0] = slave_addr;
	pMaster->pchTxBuffer[1] = function_code;
	pMaster->pchTxBuffer[2] = register_addr>>8;
	pMaster->pchTxBuffer[3] = register_addr;
	pMaster->pchTxBuffer[4] = value>>8;
	pMaster->pchTxBuffer[5] = value;
	uint16_t crc = CRC16(pMaster->pchTxBuffer, 6);
	pMaster->pchTxBuffer[6] = crc&0xff;
	pMaster->pchTxBuffer[7] = (crc>>8)&0xff;


	HAL_UART_Transmit(pMaster->huart, pMaster->pchTxBuffer, TX_SIZE, 10);
	HAL_UART_Receive(pMaster->huart, pMaster->pchRxBuffer, rx_size, timeout);
	return MODBUS_MASTER_response_check(pMaster, slave_addr);
}



void MODBUS_MASTER_write_single_holding_reg(
		MODBUS_MASTER_InitTypeDef *pMaster,
		uint8_t slave_addr,
		uint16_t register_addr,
		uint16_t val)
{
	uint16_t rx_size = 5;
	uint8_t function_code = MODBUS_FC_WR_HR;

	pMaster->pchTxBuffer[0] = slave_addr;
	pMaster->pchTxBuffer[1] = function_code;
	pMaster->pchTxBuffer[2] = register_addr>>8;
	pMaster->pchTxBuffer[3] = register_addr;
	pMaster->pchTxBuffer[4] = val>>8;
	pMaster->pchTxBuffer[5] = val;
	uint16_t crc = CRC16(pMaster->pchTxBuffer, 6);
	pMaster->pchTxBuffer[6] = crc&0xff;
	pMaster->pchTxBuffer[7] = (crc>>8)&0xff;


	HAL_UART_Transmit_IT(pMaster->huart, pMaster->pchTxBuffer, TX_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(pMaster->huart, pMaster->pchRxBuffer, rx_size);
}




