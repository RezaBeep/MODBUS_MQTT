/*
 * modbus_reg.h
 *
 *  Created on: Aug 29, 2024
 *      Author: j
 */

#ifndef INC_MODBUS_REG_H_
#define INC_MODBUS_REG_H_

#include "main.h"
#include "eeprom.h"


typedef enum{
	MODBUS_REG_STATE_OK = 0,
	MODBUS_REG_STATE_INVALID_VIRT_ADDR,
	MODBUS_REG_STATE_NO_VALID_PAGE

}modbus_reg_state_t;


#define MODBUS_REG_DIN_COUNT			64
#define MODBUS_REG_DIN_ADDR_OFFSET		0
#define MODBUS_REG_DIN_ADDR_BEGIN		0		// Virtual address
#define MODBUS_REG_DIN_ADDR_END			(MODBUS_REG_DIN_ADDR_BEGIN + MODBUS_REG_DIN_COUNT -1)

#define MODBUS_REG_DOUT_COUNT			64
#define MODBUS_REG_DOUT_ADDR_BEGIN		MODBUS_REG_DIN_ADDR_END + 1
#define MODBUS_REG_DOUT_ADDR_OFFSET		MODBUS_REG_DOUT_ADDR_BEGIN
#define MODBUS_REG_DOUT_ADDR_END		(MODBUS_REG_DOUT_ADDR_BEGIN + MODBUS_REG_DOUT_COUNT -1)


// mwmory WORDs(ie.HoldingRegisters)
#define MODBUS_REG_WDATA_COUNT			63		// first WORD of the page is reserved for page status
#define MODBUS_REG_WDATA_ADDR_BEGIN		MODBUS_REG_DOUT_ADDR_END + 1
#define MODBUS_REG_WDATA_ADDR_OFFSET	MODBUS_REG_WDATA_ADDR_BEGIN
#define MODBUS_REG_WDATA_ADDR_END		(MODBUS_REG_WDATA_ADDR_BEGIN + MODBUS_REG_WDATA_COUNT -1)


/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_read_din_addr(uint8_t num, uint16_t* addr);
/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_write_din_addr(uint8_t num, uint16_t addr);
/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_read_dout_addr(uint8_t num, uint16_t* addr);
/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_write_dout_addr(uint8_t num, uint16_t addr);
/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_read_wdata_addr(uint8_t num, uint16_t* addr);
/*
 * @param num:  0 < num < reg_count
 */
modbus_reg_state_t modbus_reg_write_wdata_addr(uint8_t num, uint16_t addr);



#endif /* INC_MODBUS_REG_H_ */
