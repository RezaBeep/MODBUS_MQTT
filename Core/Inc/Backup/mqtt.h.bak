/*
 * mqtt.h
 *
 *  Created on: Jun 30, 2024
 *      Author: reza
 */

#ifndef INC_MQTT_H_
#define INC_MQTT_H_






#include "main.h"
#include "stdbool.h"
#include "sim.h"
#include "at.h"

#define MQTT_RX_BUFF_SIZE 		100
#define MQTT_AT_MIN_TIMEOUT		2000

typedef struct{
	sim_t* sim;
	char* client_id;
	char* url;
	char* port;
	char* username;
	char* password;
	char* keep_time;
	bool connected;
} mqtt_conn_t;




void mqtt_init(
	mqtt_conn_t* pMqttConn,
	sim_t* pSim,
	char* client_id,
	char* url,
	char* port,
	char* username,
	char* password,
	char* keep_time);


bool mqtt_connect(mqtt_conn_t* pMqttConn);
bool mqtt_disconnect(mqtt_conn_t* pMqttConn);
bool mqtt_publish_string(mqtt_conn_t* pMqttConn, char* qos, char* retain, char* topic, char* payload);
bool mqtt_publish_hex(mqtt_conn_t* pMqttConn, char* qos, char* retain, char* topic, char* payload);
bool mqtt_conn_status(mqtt_conn_t* pMqttConn);
bool mqtt_sub(mqtt_conn_t* pMqttConn, char* qos, char* topic);

#endif /* INC_MQTT_H_ */
