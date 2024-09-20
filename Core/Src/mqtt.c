/*
 * mqtt.c
 *
 *  Created on: Jun 30, 2024
 *      Author: reza
 */

#include "mqtt.h"
#include "oled.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "string.h"
#include "stdio.h"


char* smconf = "SMCONF";
char* smconn = "SMCONN";
char* smpub = "SMPUB";
char* smsub = "SMSUB";
char* smstate= "SMSTATE";
char* smdisc= "SMDISC";
char* smpubhex = "SMPUBHEX";




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
	memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);


}



bool mqtt_connect(mqtt_conn_t* pMqttConn){
	if(pMqttConn->sim->app_network){
		if(at_write(pMqttConn->sim->huart, mqtt_rx_buff, smconf, MQTT_AT_MIN_TIMEOUT, 5, "\"URL\",\"", pMqttConn->url, "\",\"", pMqttConn->port, "\"")){
			if(at_write(pMqttConn->sim->huart, mqtt_rx_buff, smconf, MQTT_AT_MIN_TIMEOUT, 2, "\"KEEPTIME\",", pMqttConn->keep_time)){

				HAL_Delay(2000);
				if(at_execute_blocking(pMqttConn->sim->huart, mqtt_rx_buff, smconn, 15000)>0){
					pMqttConn->connected = true;
					memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
					return true;
				}
				else{
				}
			}
		}
	}
	memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
	pMqttConn->connected = false;
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
	if(pMqttConn->sim->app_network){
		if(pMqttConn->connected){
			uint8_t content_length = strlen(payload);
			char content_len[5];
			snprintf(content_len, 5, "%d", content_length);
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
					snprintf(msg, content_length+5, "%s", payload);
					HAL_UART_Transmit(pMqttConn->sim->huart, msg , strlen(msg), MQTT_AT_MIN_TIMEOUT);
					snprintf(msg, content_length+5, "%c", (char) 26);
					HAL_UART_Transmit_IT(pMqttConn->sim->huart, msg , strlen(msg));
					memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
					return true;
			}
		}
	}
	memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
	return false;
}




bool mqtt_publish_hex(mqtt_conn_t* pMqttConn, char* qos, char* retain, char* topic, char* payload){
	if(pMqttConn->sim->app_network){
		if(pMqttConn->connected){
			if(at_write_blocking(pMqttConn->sim->huart, mqtt_rx_buff, smpubhex, 200, 1, "1")){
				uint8_t content_length = strlen(payload);
				char true_payload[content_length+1];
				char byte_size[4];
				if(content_length % 2 != 0){
					snprintf(true_payload, content_length+1, "%s0", payload);
					snprintf(byte_size, 4, "%d", (content_length+1)/2);
				}
				else{
					strcpy(true_payload, payload);
					snprintf(byte_size, 4, "%d", content_length/2);
				}

				if(pMqttConn->connected){
					if(at_write_blocking(
							pMqttConn->sim->huart,
							mqtt_rx_buff,
							smpub,
							500,
							8,
							"\"",
							topic,
							"\",\"",
							byte_size,
							"\",",
							qos,
							",",
							retain) == AT_STATE_ENTER_INPUT)
					{


						HAL_UART_Transmit(pMqttConn->sim->huart, true_payload , strlen(true_payload), 10);
						HAL_UART_Transmit(pMqttConn->sim->huart, "\n" , 1, 200);
						memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
						return true;
					}
				}
			}
		}
	}
	memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
	return false;
}




bool mqtt_sub(mqtt_conn_t* pMqttConn, char* qos, char* topic){
	if(pMqttConn->sim->app_network){
		if(pMqttConn->connected){
			if(at_write_blocking(
				pMqttConn->sim->huart,
				mqtt_rx_buff,
				smsub,
				MQTT_AT_MIN_TIMEOUT,
				4,
				"\"",
				topic,
				"\",",
				qos)){
					memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
					return true;
			}
		}
	}
	memset(mqtt_rx_buff, '\0', MQTT_RX_BUFF_SIZE);
	return false;
}






bool mqtt_conn_status(mqtt_conn_t* pMqttConn, uint8_t* rx){

	if(pMqttConn->sim->app_network){
		HAL_UART_Transmit(pMqttConn->sim->huart, "at+smstate?\r\n", 13, 10);
		while(HAL_UART_Receive(pMqttConn->sim->huart, rx, 50, 100) != HAL_TIMEOUT);
		if(find_substr(rx, "1")){
			return true;
		}
	}
	return false;
}




