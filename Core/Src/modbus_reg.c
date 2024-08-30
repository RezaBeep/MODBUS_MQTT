/*
 * modbus_reg.c
 *
 *  Created on: Aug 29, 2024
 *      Author: j
 */

#include "modbus_reg.h"



uint16_t modbus_reg_read_din_addr(uint8_t num, uint16_t* addr){
	if(num > MODBUS_REG_DIN_COUNT-1){
		return -1;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_DIN_ADDR_OFFSET + num;
	return EE_ReadVariable(ee_virtual_addr, addr);
}

uint16_t modbus_reg_write_din_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_DIN_COUNT-1){
		return -1;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_DIN_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
	HAL_Delay(10);
}

uint16_t modbus_reg_read_dout_addr(uint8_t num, uint16_t* addr){
	if(num > MODBUS_REG_DOUT_COUNT-1){
		return -1;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_DOUT_ADDR_OFFSET + num;
	return EE_ReadVariable(ee_virtual_addr, addr);
}

uint16_t modbus_reg_write_dout_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_DOUT_COUNT-1){
		return -1;
	}
	int8_t ee_virtual_addr = MODBUS_REG_DOUT_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
	HAL_Delay(10);
}

uint16_t modbus_reg_read_wdata_addr(uint8_t num, uint16_t* addr){
	if(num > MODBUS_REG_WDATA_COUNT-1){
		return -1;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_WDATA_ADDR_OFFSET + num;
	return EE_ReadVariable(ee_virtual_addr, addr);
}

uint16_t modbus_reg_write_wdata_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_WDATA_COUNT-1){
		return -1;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_WDATA_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
	HAL_Delay(10);
}

