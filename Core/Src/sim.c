/*
 * sim.c
 *
 *  Created on: Jun 24, 2024
 *      Author: reza
 */

#include "sim.h"
#include "i2c.h"
#include "oled.h"


oled_t debug_oled;




#define SIM_RX_BUFF_SIZE	50
#define SIM_EVENT_BUFF_SIZE	50


char* cfun = "CFUN";
char* cpin = "CPIN";
char* cmee = "CMEE";

char* creg = "CREG";
char* cops = "COPS";

char* cgreg = "CGREG";
char* cgatt = "CGATT";
char* cgact = "CGACT";
char* cgpaddr = "CGPADDR";

char* sapbr = "SAPBR";
char* cnact = "CNACT";

char* cipstatus = "CIPSTATUS";



char sim_rx_buff[SIM_RX_BUFF_SIZE];
char sim_event_buff[SIM_EVENT_BUFF_SIZE];
uint16_t sim_event_rx_len;



bool _sim_finish_operation(){
	flush_buff(sim_rx_buff);
	return true;
}


void sim_init(sim_t* psim, UART_HandleTypeDef* huart, char* apn, char* username, char* password){
	psim->huart = huart;
	psim->apn = apn;
	psim->username = username;
	psim->password = password;
	psim->app_network = false;


}




void sim_event_listen_once(sim_t* psim){
	HAL_UARTEx_ReceiveToIdle_IT(psim->huart, sim_event_buff, SIM_EVENT_BUFF_SIZE);
}




void sim_event_poll_once(sim_t* psim, uint32_t timeout){
//	HAL_UARTEx_ReceiveToIdle(psim->huart, sim_event_buff, SIM_EVENT_BUFF_SIZE, &sim_event_rx_len, timeout);
	HAL_UART_Receive(psim->huart, sim_event_buff, SIM_EVENT_BUFF_SIZE, timeout);

}




void sim_event_handler(sim_t* psim, char* event_buff){

	if(strlen(event_buff) > 0){
		if(find_substr(event_buff, "+APP")){
			if(find_substr(event_buff, "ACTIVE")){
				psim->app_network = true;
			}
			else if(find_substr(event_buff, "DEACTIVE")){
				psim->app_network = false;
			}
		}
	}
}





bool sim_test_at(sim_t* psim){
	if(at_execute(psim->huart, sim_rx_buff, "", SIM_AT_MIN_TIMEOUT)){
			psim->state = SIM_STATE_AT_OK;
			return _sim_finish_operation();
	}
	else{
		psim->state = SIM_STATE_OFF;
	}
	return false;
}



bool sim_reboot(sim_t* psim){
	sim_init(psim, psim->huart, psim->apn, psim->username, psim->password);
	if(at_write_blocking(psim->huart, sim_rx_buff, cfun, SIM_AT_MIN_TIMEOUT, 2, "1,", "1")){
		return true;
	}
	return false;
}



bool sim_report_error_enable(sim_t* psim){
	if(at_write(psim->huart, sim_rx_buff, cmee, 5000, 1, "2")){
		psim->state = SIM_STATE_REPORT_ERROR_ENABLED;
		return _sim_finish_operation();
	}
	return false;
}




bool sim_is_ready(sim_t* psim){
	if(psim->state >= SIM_STATE_AT_OK){
		if(at_read_blocking(psim->huart, sim_rx_buff, cpin, SIM_AT_MIN_TIMEOUT)){
			if(find_substr(sim_rx_buff, "READY")){
				psim->state = SIM_STATE_PIN_READY;
			}
			else{

			}
		}
		if(psim->state == SIM_STATE_PIN_READY){
			if(at_read_blocking(psim->huart, sim_rx_buff, cfun, SIM_AT_MIN_TIMEOUT)){
				if(find_substr(sim_rx_buff, "+CFUN: 1")){
					psim->state = SIM_STATE_FULL_FUNC;
					return _sim_finish_operation();
				}
				else{

				}
			}
		}
		return false;
	}
	return false;
}


bool sim_registered(sim_t* psim){
	if(psim->state == SIM_STATE_FULL_FUNC){
		if(at_read(psim->huart, sim_rx_buff, creg, 5000)){
			if(find_substr(sim_rx_buff, "+CREG: 0,1")){
				psim->state = SIM_STATE_CREG_OK;
				return _sim_finish_operation();
			}
			else{

			}
		}
	}
	return false;
}



bool sim_gprs_registered(sim_t* psim){
	if(psim->state == SIM_STATE_CREG_OK){
		if(at_read(psim->huart, sim_rx_buff, cgreg, 5000)){
			if(find_substr(sim_rx_buff, "+CGREG: 0,1")){
				psim->state = SIM_STATE_CGREG_OK;
				return _sim_finish_operation();
			}
			else{

			}
		}
	}

	return false;

}



bool sim_gprs_connect(sim_t* psim){
	if(psim->state == SIM_STATE_CGREG_OK){
		if(at_write_blocking(psim->huart, sim_rx_buff, cnact, 100, 2, "1,",SIM_APN_MTN) > 0){
//			sim_event_handler(psim);
			sim_event_poll_once(psim, 10000);
			sim_event_handler(psim, sim_event_buff);
			return _sim_finish_operation();
		}

	}
	flush_buff(sim_rx_buff);
	return false;

}



bool sim_gprs_disconnect(sim_t* psim){
	if(psim->state == SIM_STATE_CGREG_OK){
		if(at_write_blocking(psim->huart, sim_rx_buff, cnact, 5000, 1, "0") > 0){
			psim->app_network = false;
//			sim_event_handler(psim);
//			sim_event_poll_once(psim, 60000);
//			sim_event_handler(psim);
			return _sim_finish_operation();
		}

	}
	flush_buff(sim_rx_buff);
	return false;

}




bool sim_app_network_is_connected(sim_t* psim){
	if(at_read(psim->huart, sim_rx_buff, cnact, 5000)){
		if(find_substr(sim_rx_buff, "CNACT: 1")){
			psim->app_network = true;
			return _sim_finish_operation();
		}
	}
	flush_buff(sim_rx_buff);
	return false;
}


bool sim_setup(sim_t* psim){
	if(sim_test_at(psim)){
		if(sim_is_ready(psim)){
			if(sim_registered(psim)){
				if(sim_gprs_registered(psim)){
					if(sim_gprs_connect(psim)){
						return true;
					}
				}
			}
		}
	}
	return false;
}




