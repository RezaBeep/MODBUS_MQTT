/*
 * sim_event.c
 *
 *  Created on: Aug 31, 2024
 *      Author: j
 */

#include "sim_event.h"
#include "modbus_reg.h"
#include "sim.h"
#include "modbus.h"
#include "mqtt.h"


void sim_event_init(
		sim_event_listener_t* psim_evt,
		UART_HandleTypeDef* phuart,
		uint8_t* pRxBuff,
		sim_t* psim)
{
	psim_evt->phuart = phuart;
	psim_evt->pRxBuff = pRxBuff;
	psim_evt->psim = psim;

}



void sim_event_listen(sim_event_listener_t* psim_evt){
	HAL_UARTEx_ReceiveToIdle_DMA(psim_evt->phuart, psim_evt->pRxBuff, SIM_EVENT_RX_SIZE);
}



