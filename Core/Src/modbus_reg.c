/*
 * modbus_reg.c
 *
 *  Created on: Aug 29, 2024
 *      Author: j
 */

#include "modbus_reg.h"



modbus_reg_state_t modbus_reg_read_din_addr(uint8_t num, uint16_t* addr){
	uint8_t ee_virtual_addr = MODBUS_REG_DIN_ADDR_OFFSET + num;
	uint16_t res = EE_ReadVariable(ee_virtual_addr, addr);
	switch(res){
		case 0:
			return MODBUS_REG_STATE_OK;
		case 1:
			return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
		case NO_VALID_PAGE:
			return MODBUS_REG_STATE_NO_VALID_PAGE;
	}
	return MODBUS_REG_STATE_OK;
}

modbus_reg_state_t modbus_reg_write_din_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_DIN_COUNT-1){
		return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_DIN_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
}

modbus_reg_state_t modbus_reg_read_dout_addr(uint8_t num, uint16_t* addr){
	uint8_t ee_virtual_addr = MODBUS_REG_DOUT_ADDR_OFFSET + num;
	uint16_t res = EE_ReadVariable(ee_virtual_addr, addr);
	switch(res){
		case 0:
			return MODBUS_REG_STATE_OK;
		case 1:
			return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
		case NO_VALID_PAGE:
			return MODBUS_REG_STATE_NO_VALID_PAGE;
	}
	return MODBUS_REG_STATE_OK;
}

modbus_reg_state_t modbus_reg_write_dout_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_DOUT_COUNT-1){
		return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
	}
	int8_t ee_virtual_addr = MODBUS_REG_DOUT_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
}

modbus_reg_state_t modbus_reg_read_wdata_addr(uint8_t num, uint16_t* addr){
	uint8_t ee_virtual_addr = MODBUS_REG_WDATA_ADDR_OFFSET + num;
	uint16_t res = EE_ReadVariable(ee_virtual_addr, addr);
	switch(res){
		case 0:
			return MODBUS_REG_STATE_OK;
		case 1:
			return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
		case NO_VALID_PAGE:
			return MODBUS_REG_STATE_NO_VALID_PAGE;
	}
	return MODBUS_REG_STATE_OK;
}

modbus_reg_state_t modbus_reg_write_wdata_addr(uint8_t num, uint16_t addr){
	if(num > MODBUS_REG_WDATA_COUNT-1){
		return MODBUS_REG_STATE_INVALID_VIRT_ADDR;
	}
	uint8_t ee_virtual_addr = MODBUS_REG_WDATA_ADDR_OFFSET + num;
	return EE_WriteVariable(ee_virtual_addr, addr);
}

