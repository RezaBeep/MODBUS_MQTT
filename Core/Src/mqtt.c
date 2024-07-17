/*
 * mqtt.c
 *
 *  Created on: Jun 30, 2024
 *      Author: reza
 */

#include "mqtt.h"
#include "oled.h"
#include "i2c.h"



char* smconf = "SMCONF";
char* smconn = "SMCONN";
char* smpub = "SMPUB";
char* smstate= "SMSTATE";
char* smdisc= "SMDISC";




char mqtt_rx_buff[MQTT_RX_BUFF_SIZE];

oled_t mqtt_debug_oled;





void mqtt_init(
	mqtt_conn_t* pMqttConn,
	sim_t* pSim,
	char* client_id,
	char* url,
	char* port,
	char* username,
	char* password,
	char* keep_time)
{
	pMqttConn->sim = pSim;
	pMqttConn->client_id = client_id;
	pMqttConn->url = url;
	pMqttConn->port = port;
	pMqttConn->username = username;
	pMqttConn->password = password;
	pMqttConn->keep_time = keep_time;
	pMqttConn->connected = false;
	flush_buff(mqtt_rx_buff);


}



bool mqtt_connect(mqtt_conn_t* pMqttConn){
	if(pMqttConn->sim->app_network){
		if(at_write(pMqttConn->sim->huart, mqtt_rx_buff, smconf, MQTT_AT_MIN_TIMEOUT, 5, "\"URL\",\"", pMqttConn->url, "\",\"", pMqttConn->port, "\"")){
			if(at_write(pMqttConn->sim->huart, mqtt_rx_buff, smconf, MQTT_AT_MIN_TIMEOUT, 2, "\"KEEPTIME\",", pMqttConn->keep_time)){

				HAL_Delay(2000);
				if(at_execute_blocking(pMqttConn->sim->huart, mqtt_rx_buff, smconn, 20000)>0){
					pMqttConn->connected = true;
					return true;
				}
				else{
				}
			}
		}
	}
	else{
		pMqttConn->connected = false;
	}
	return false;
}




bool mqtt_disconnect(mqtt_conn_t* pMqttConn){
	if(at_execute(pMqttConn->sim->huart, mqtt_rx_buff, smdisc, 5000)){
		pMqttConn->connected = false;
		return true;
	}
	return false;
}





bool mqtt_publish_string(mqtt_conn_t* pMqttConn, char* qos, char* retain, char* topic, char* payload){
	if(pMqttConn->connected){
		uint8_t content_length = strlen(payload);
		char content_len[5];
		sprintf(content_len, "%d", content_length);
		if(pMqttConn->connected){
			if(at_write_blocking(
					pMqttConn->sim->huart,
					mqtt_rx_buff,
					smpub,
					MQTT_AT_MIN_TIMEOUT,
					8,
					"\"",
					topic,
					"\",\"",
					content_len,
					"\",",
					qos,
					",",
					retain) == AT_STATE_ENTER_INPUT)
			{
				char msg[content_length+5];
//				sprintf(msg, "%s\x1A\r\n", payload);
				sprintf(msg, "%s", payload);
				HAL_UART_Transmit(pMqttConn->sim->huart, msg , strlen(msg), MQTT_AT_MIN_TIMEOUT);
				sprintf(msg, "%c", (char) 26);
				HAL_UART_Transmit_IT(pMqttConn->sim->huart, msg , strlen(msg));
				return true;
			}
		}
	}
	return false;
}



bool mqtt_conn_status(mqtt_conn_t* pMqttConn){
	if(pMqttConn->sim->app_network){
		if(at_read(pMqttConn->sim->huart, mqtt_rx_buff, smstate, 5000)){
			if(find_substr(mqtt_rx_buff, "1")){
				pMqttConn->connected = true;
				return true;
			}
			else{
				pMqttConn->connected = false;
			}
		}
	}
	pMqttConn->connected = false;
	return false;
}




void mqtt_event_handler(mqtt_conn_t* pMqttConn, char* event_buff){
	if(strlen(event_buff) > 0){
		if(find_substr(event_buff, "+SMSUB")){

		}
		if(find_substr(event_buff, "+SMSTATE")){
			if(find_substr(event_buff, "0")){
				pMqttConn->connected = false;
			}
		}
	}
}
