/*
 * sim_event.h
 *
 *  Created on: Aug 31, 2024
 *      Author: j
 */

#ifndef INC_SIM_EVENT_H_
#define INC_SIM_EVENT_H_


#include "main.h"
#include "at.h"
#include "sim.h"
#include "stdbool.h"


#define SIM_EVENT_RX_SIZE	50


typedef struct{
	UART_HandleTypeDef* phuart;
	uint8_t* pRxBuff;
	sim_t* psim;
}sim_event_listener_t;



void sim_event_init(
		sim_event_listener_t* psim_evt,
		UART_HandleTypeDef* phuart,
		uint8_t* pRxBuff,
		sim_t* psim);


void sim_event_listen(sim_event_listener_t* psim_evt);



#endif /* INC_SIM_EVENT_H_ */
