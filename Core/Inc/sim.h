/*
 * sim.h
 *
 *  Created on: Jun 24, 2024
 *      Author: reza
 */

#ifndef INC_SIM_H_
#define INC_SIM_H_



#include "main.h"
#include <stdbool.h>
#include "at.h"



#define SIM_APN_MTN		"\"mtnirancell\""
#define SIM_APN_MCI		"\"mcinet\""


#define SIM_AT_MIN_TIMEOUT	2000



typedef enum{
	SIM_STATE_OFF = -1,
	SIM_STATE_AT_OK = 0,
	SIM_STATE_REPORT_ERROR_ENABLED,
	SIM_STATE_PIN_READY,
	SIM_STATE_FULL_FUNC,
	SIM_STATE_CREG_OK,
	SIM_STATE_CGREG_OK,
	SIM_STATE_GPRS_OK
}sim_state_t;



typedef struct {
	UART_HandleTypeDef *huart;
	char* apn;
	char* username;
	char* password;
	sim_state_t state;
	bool app_network;
}sim_t;


// Commands



void sim_init(sim_t* psim, UART_HandleTypeDef *huart, char* apn, char* username, char* password);


bool sim_reboot(sim_t* psim);

bool sim_test_at(sim_t* psim);
bool sim_report_error_enable(sim_t* psim);
bool sim_is_ready(sim_t* psim);
bool sim_registered(sim_t* psim);

bool sim_gprs_registered(sim_t* psim);
bool sim_gprs_connect(sim_t* psim);
bool sim_gprs_disconnect(sim_t* psim);
bool sim_app_network_is_connected(sim_t* psim);

bool sim_setup(sim_t* psim);


void sim_event_handler(sim_t* psim, char* event_buff);
void sim_event_poll_once(sim_t* psim, uint32_t timeout);

#endif /* INC_SIM_H_ */
