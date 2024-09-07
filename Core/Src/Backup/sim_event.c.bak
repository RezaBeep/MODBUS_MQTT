/*
 * sim_event.c
 *
 *  Created on: Aug 31, 2024
 *      Author: j
 */

#include "sim_event.h"


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


void sim_event_smsub_decode(sim_event_listener_t* psim_evt, char* topic_buff, char* payload_buff){
    uint8_t len = strlen(psim_evt->pRxBuff);
    char* colon = memchr(psim_evt->pRxBuff, ':', len);
    char* comma = memchr(psim_evt->pRxBuff, ',', len);
    uint8_t topic_len = comma - colon - 2;
    char topic[topic_len];
    strncpy(topic, colon+2, sizeof(topic));
    uint8_t payload_len = strlen(comma) - 4;
    char payload[payload_len];
    strncpy(payload, comma+2, sizeof(payload));

    strcpy(topic_buff, topic);
    strcpy(payload_buff, payload);
}



sim_event_type_sub_t sim_event_type_sub(char* topic){
	if(find_substr(topic, "SET_ADDR")){
		return SIM_EVENT_TYPE_SUB_SET_REG_ADDR;
	}
	else if(find_substr(topic, "SET_VALUE")){
		return SIM_EVENT_TYPE_SUB_SET_REG_VALUE;
	}

	return SIM_EVENT_TYPE_SUB_UNKNOWN;

}


void sim_event_reg_decode_val_addr(char* topic, char* payload, uint16_t* pAddr, uint16_t* pVal){
    char* reg_pos = strstr(topic, "REG");
    int reg_pos_len = strlen(reg_pos) - 5;
    char reg_pos2[reg_pos_len];
    strncpy(reg_pos2, reg_pos+4, reg_pos_len);
    uint16_t pos = strtol(reg_pos2, (char**) NULL, 10);
    uint16_t val = strtol(payload, (char**) NULL, 10);

    (*pAddr) = pos;
    (*pVal) = val;
}






