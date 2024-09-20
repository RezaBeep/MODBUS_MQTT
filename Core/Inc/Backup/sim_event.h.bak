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
#include "stdlib.h"
#include "string.h"


#define SIM_EVENT_RX_SIZE	200


typedef enum{
	SIM_EVENT_TYPE_SUB_SET_REG_ADDR,
	SIM_EVENT_TYPE_SUB_SET_REG_VALUE,
	SIM_EVENT_TYPE_SUB_UNKNOWN
}sim_event_type_sub_t;


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
void sim_event_smsub_decode(sim_event_listener_t* psim_evt, char* topic, char* payload);
sim_event_type_sub_t sim_event_type_sub(char* topic);
void sim_event_reg_decode_val_addr(
		char* topic,
		char* payload,
		uint16_t* pAddr,
		uint16_t* pVal,
		bool hex_val);
bool sim_event_mqtt_conn_status(sim_event_listener_t* psim_evt, sim_t* psim);


#endif /* INC_SIM_EVENT_H_ */
