/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mqtt.h"
#include "sim.h"
#include "oled.h"
#include "modbus.h"
#include "sim_event.h"
#include "eeprom.h"
#include "modbus_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MODBUS_SLAVE_ADDR	1
#define MODBUS_COIL_NB_POINTS	16
#define MODBUS_REG_NB_POINTS	1

#define MODBUS_REG_TYPE_DI		MODBUS_FC_RD_DI
#define MODBUS_REG_TYPE_COIL		MODBUS_FC_RD_DO
#define MODBUS_REG_TYPE_WDATA		MODBUS_FC_RD_HR


#define MQTT_QOS	"0"
#define MQTT_RETAIN		"0"

#define MQTT_KEEPTIME	"120"
#define MQTT_PAYLOAD_BUFF_SIZE	20
#define MQTT_TOPIC_BUFF_SIZE	50
#define MQTT_BROKER_ADDR	"5.198.179.50"
#define MQTT_BROKER_PORT	"1883"
#define MQTT_CLIENT_ID		"STM32_GATEWAY"


#define OLED_BUFF_SIZE	15

BaseType_t xHigherPriorityTaskWoken;

const UART_HandleTypeDef* PHUART_SIM = &huart1;
const UART_HandleTypeDef* PHUART_MODBUS = &huart2;
const UART_HandleTypeDef* PHUART_EVENT = &huart3;


const char* topic_coil = "MCU/COIL/REG/";
const char* topic_holding_reg = "MCU/WDATA/REG/";
const char* topic_discrete_input = "MCU/DI/REG/";
const char* topic_input_reg = "MCU/IR/REG/";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t event_semphr;
SemaphoreHandle_t modbus_res_semphr;


QueueHandle_t qmodbus_di_val;
QueueHandle_t qmodbus_coil_val;
QueueHandle_t qmodbus_hr_val;
QueueHandle_t qmodbus_ir_val;



sim_t sim;
oled_t oled;
mqtt_conn_t mqtt_conn;
MODBUS_MASTER_InitTypeDef master;
sim_event_listener_t sim_evt;

char modbus_tx_buff[MODBUS_TX_SIZE];
char modbus_rx_buff[MODBUS_RX_SIZE];
char mqtt_payload_buff[MQTT_PAYLOAD_BUFF_SIZE];
char mqtt_topic_buff[MQTT_TOPIC_BUFF_SIZE];
char oled_buff[OLED_BUFF_SIZE];
volatile char event_rx_buff[SIM_EVENT_RX_SIZE];

uint16_t din_addr[MODBUS_REG_DIN_COUNT];
uint16_t dout_addr[MODBUS_REG_DOUT_COUNT];
uint16_t wdata_addr[MODBUS_REG_WDATA_COUNT];


/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);


bool setup();
void modbus_res_handler_task(void* pvArgs);
void modbus_read_task(void* pvArgs);
void event_handler_task(void* pvArgs);
void task_mqtt_conn_check(void* args);


bool publish_reg_data(uint8_t reg_type, uint16_t reg_virt_addr, uint16_t reg_val);
bool _sim_restart();
void vSystemRst();


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12){

	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == PHUART_MODBUS->Instance){
//		oled_printl(&oled, "modbus req sent");

	}

}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == PHUART_MODBUS->Instance){
		if(huart->Instance == PHUART_MODBUS->Instance){
			xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(modbus_res_semphr, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}





void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == PHUART_EVENT->Instance){
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(event_semphr, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  oled_init(&oled, &hi2c1);
  sim_init(&sim, PHUART_SIM, "mtnirancell", "", "");
  mqtt_init(&mqtt_conn, &sim, "STM32_GATEWAY", "5.198.179.50", "1883", "", "", MQTT_KEEPTIME);
  MODBUS_MASTER_init(&master, PHUART_MODBUS, modbus_tx_buff, modbus_rx_buff);
  sim_event_init(&sim_evt, PHUART_EVENT, event_rx_buff, &sim);

  HAL_FLASH_Unlock();
  EE_Init();
  modbus_reg_retrieve_addresses(din_addr, dout_addr, wdata_addr);

setup:
  if(setup()){
		if(mqtt_sub(&mqtt_conn, "0", "SERVER/#")){
			oled_printl(&oled, "SUB DONE !");
		}
		else{
			oled_printl(&oled, "SUB FAILED !");
		}

		mqtt_publish_string(&mqtt_conn, "0", "0", "stm32", "connected");

		sim_event_listen(&sim_evt);


//	  repeative_task();
//	  rtc_set_alarm_seconds_it(&hrtc, REPEAT_DELAY);
  }
  else{
	  oled_printl(&oled, "sim reboot");
	  sim_reboot(&sim);
	  goto setup;
  }


  /* USER CODE END 2 */

  /* Init scheduler */
  event_semphr = xSemaphoreCreateBinary();
  modbus_res_semphr = xSemaphoreCreateBinary();

  qmodbus_di_val = xQueueCreate(1, sizeof(uint16_t));
  qmodbus_coil_val = xQueueCreate(1, sizeof(uint16_t));
  qmodbus_hr_val = xQueueCreate(1, sizeof(uint16_t));
  qmodbus_ir_val = xQueueCreate(1, sizeof(uint16_t));


	if(xTaskCreate(task_mqtt_conn_check, "conn_task", 128, NULL, 4, NULL) != pdPASS){
		oled_printl(&oled, "conn task not created");
		return pdFALSE;
	}
	if(xTaskCreate(event_handler_task, "event_task", 128, NULL, 3, NULL) != pdPASS){
		 oled_printl(&oled, "event task not created");
		 return pdFALSE;
	}
	if(xTaskCreate(modbus_read_task, "read_task", 128, NULL, 2, NULL) != pdPASS){
		oled_printl(&oled, "read task not created");
		return pdFALSE;
	}
	if(xTaskCreate(modbus_res_handler_task, "modbus_res_task", 128, NULL, 1, NULL) != pdPASS){
		oled_printl(&oled, "res handler not created");
		return pdFALSE;
	}


	/* We should never get here as control is now taken by the scheduler */
	vTaskStartScheduler();


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */






bool setup(){
	  oled_printl(&oled, "sending AT..");
	  if(sim_test_at(&sim)){
		  oled_printl(&oled, "AT OK!");
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  while(sim.state < SIM_STATE_AT_OK){
		  sim_test_at(&sim);
	  }
	  oled_printl(&oled, "+CMEE=2");
	  HAL_IWDG_Refresh(&hiwdg);
	  while(sim.state < SIM_STATE_REPORT_ERROR_ENABLED){
		  sim_report_error_enable(&sim);
	  }
	  while(sim.state < SIM_STATE_PIN_READY){
		  sim_is_ready(&sim);
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  oled_printl(&oled, "ready");
	  while(sim.state < SIM_STATE_CREG_OK){
		  sim_registered(&sim);
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  oled_printl(&oled, "registered");
	  while(sim.state < SIM_STATE_CGREG_OK){
		  sim_gprs_registered(&sim);
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  oled_printl(&oled, "gprs registered");
	  //mqtt disconnect
	  if(!mqtt_disconnect(&mqtt_conn)){
	  	  oled_printl(&oled, "broker disconnect error!");
	  }


	  // gprs disconnect
	  if(!sim_gprs_disconnect(&sim)){
		  oled_printl(&oled, "gprs disconnected already!");
	  }
	  HAL_IWDG_Refresh(&hiwdg);
//	  HAL_Delay(5000);
	  if(sim_gprs_connect(&sim)){
		  HAL_IWDG_Refresh(&hiwdg);
		  uint8_t i = 0;
		  oled_printl(&oled, "activatin app network");
		  while(!(sim.app_network)){
			  i++;
			  HAL_Delay(2000);
			  oled_printl(&oled, "retrying app net");
			  sim_gprs_connect(&sim);
			  HAL_IWDG_Refresh(&hiwdg);
			  if(i>2){
				  return false;
			  }
		  }
	//	  HAL_UART_AbortReceive_IT(sim.huart);

		  oled_printl(&oled, "Connecting to broker");
		  if(mqtt_connect(&mqtt_conn)){
			  HAL_IWDG_Refresh(&hiwdg);
			  oled_printl(&oled, "Connected to broker");
			  return true;
		  }
	  }
	  else{
	  	  oled_printl(&oled, "app network FAILED");
	  	  return false;
	  }
	  return false;
}




bool publish_reg_data(uint8_t reg_type, uint16_t reg_virt_addr, uint16_t reg_val){
	switch (reg_type) {
		case MODBUS_REG_TYPE_DI:
			snprintf(
					mqtt_topic_buff,
					MQTT_TOPIC_BUFF_SIZE,
					"%s%u",
					topic_discrete_input, reg_virt_addr);
			snprintf(
					mqtt_payload_buff,
					MQTT_PAYLOAD_BUFF_SIZE,
					"%04X",
					reg_val);
			if(mqtt_publish_hex(
					&mqtt_conn,
					MQTT_QOS,
					MQTT_RETAIN,
					mqtt_topic_buff,
					mqtt_payload_buff))
			{
				return true;
			}

			break;
		case MODBUS_REG_TYPE_COIL:
			snprintf(
					mqtt_topic_buff,
					MQTT_TOPIC_BUFF_SIZE,
					"%s%u",
					topic_coil, reg_virt_addr);
			snprintf(
					mqtt_payload_buff,
					MQTT_PAYLOAD_BUFF_SIZE,
					"%04X",
					reg_val);
			if(mqtt_publish_hex(
					&mqtt_conn,
					MQTT_QOS,
					MQTT_RETAIN,
					mqtt_topic_buff,
					mqtt_payload_buff))
			{
				return true;
			}
			break;
		case MODBUS_REG_TYPE_WDATA:
			snprintf(
					mqtt_topic_buff,
					MQTT_TOPIC_BUFF_SIZE,
					"%s%u",
					topic_holding_reg, reg_virt_addr);
			snprintf(
					mqtt_payload_buff,
					MQTT_PAYLOAD_BUFF_SIZE,
					"%04X",
					reg_val);
			if(mqtt_publish_hex(
					&mqtt_conn,
					MQTT_QOS,
					MQTT_RETAIN,
					mqtt_topic_buff,
					mqtt_payload_buff))
			{
				return true;
			}
			break;

		default:
			return false;
	}
	return false;
}





void event_handler_task(void* pvArgs){

	volatile uint8_t rx_buff[50];
	char topic_buff[50];
	char payload_buff[10];

	for(;;){
		xSemaphoreTake(event_semphr, portMAX_DELAY);

		strcpy(rx_buff, sim_evt.pRxBuff);
		if(find_substr(rx_buff, "SMSUB")){
			oled_printl(&oled, "SMSUB!");
			sim_event_smsub_decode(&sim_evt, topic_buff, payload_buff);
			sim_event_type_sub_t evt_type = sim_event_type_sub(topic_buff);
			uint16_t reg_virt_addr;
			uint16_t reg_val;
			switch(evt_type){
				case SIM_EVENT_TYPE_SUB_SET_REG_ADDR:

					sim_event_reg_decode_val_addr(
							topic_buff,
							payload_buff,
							&reg_virt_addr,
							&reg_val,
							false);

					if(find_substr(topic_buff, "DIN")){
						modbus_reg_write_din_addr(reg_virt_addr, reg_val);
						din_addr[reg_virt_addr] = reg_val;
					}
					if(find_substr(topic_buff, "DOUT")){
						modbus_reg_write_dout_addr(reg_virt_addr, reg_val);
						dout_addr[reg_virt_addr] = reg_val;
					}
					if(find_substr(topic_buff, "WDATA")){
						modbus_reg_write_wdata_addr(reg_virt_addr, reg_val);
						wdata_addr[reg_virt_addr] = reg_val;
					}
					break;
				case SIM_EVENT_TYPE_SUB_SET_REG_VALUE:

					sim_event_reg_decode_val_addr(
							topic_buff,
							payload_buff,
							&reg_virt_addr,
							&reg_val,
							true);

					if(find_substr(topic_buff, "COIL")){
						MODBUS_MASTER_write_single_coil(
								&master,
								MODBUS_SLAVE_ADDR,
								dout_addr[reg_virt_addr],
								reg_val);

					}
					if(find_substr(topic_buff, "WDATA")){
						MODBUS_MASTER_write_single_holding_reg(
								&master,
								MODBUS_SLAVE_ADDR,
								wdata_addr[reg_virt_addr],
								reg_val);
					}
					break;
				default:
					break;
			}
		}

		else if(find_substr(rx_buff, "DEACTIVE")){
			// NETWORK DEACTIVATED
			sim.app_network = false;
			mqtt_conn.connected = false;
			vSystemRst();

		}


		strcpy(sim_evt.pRxBuff, "");
		strcpy(topic_buff, "");
		strcpy(payload_buff, "");
		sim_event_listen(&sim_evt);
	}
}




void modbus_read_task(void* pvArgs){

	for(;;){
		uint16_t reg_addr = 0;
		uint16_t reg_val = 0;
		bool publish_res = false;
		for (uint8_t i = 0; i < MODBUS_REG_DIN_COUNT; ++i) {
			publish_res = false;
			reg_addr = din_addr[i];
			if(reg_addr > 0){
//				vTaskDelay(pdMS_TO_TICKS(100));
				MODBUS_MASTER_read_discrete_input(
						&master,
						MODBUS_SLAVE_ADDR,
						reg_addr,
						MODBUS_COIL_NB_POINTS);
//				oled_printl(&oled, "READ DI");

				if(xQueueReceive(qmodbus_di_val, (uint16_t*)(&reg_val), pdMS_TO_TICKS(500)) == pdPASS){
//					oled_printl(&oled, "DI RES");
					publish_res = publish_reg_data(MODBUS_REG_TYPE_DI, i, reg_val);
					if(!publish_res){
						oled_printl(&oled, "failed to publish");
					}
				}

			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}

		for (uint8_t i = 0; i < MODBUS_REG_DOUT_COUNT; ++i) {
			publish_res = false;
			reg_addr = dout_addr[i];
			if(reg_addr > 0){

				MODBUS_MASTER_read_coils(
						&master,
						MODBUS_SLAVE_ADDR,
						reg_addr,
						MODBUS_COIL_NB_POINTS);
//				oled_printl(&oled, "READ COIL");

				if(xQueueReceive(qmodbus_coil_val, &reg_val, pdMS_TO_TICKS(500)) == pdPASS){
//					oled_printl(&oled, "DOUT RES");
					publish_res = publish_reg_data(MODBUS_REG_TYPE_COIL, i, reg_val);
					if(!publish_res){
						oled_printl(&oled, "failed to publish");
					}
				}

			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}

		for (uint8_t i = 0; i < MODBUS_REG_WDATA_COUNT; ++i) {
			publish_res = false;
			reg_addr = wdata_addr[i];
			if(reg_addr > 0){

				MODBUS_MASTER_read_holding_reg(
						&master,
						MODBUS_SLAVE_ADDR,
						reg_addr,
						MODBUS_REG_NB_POINTS);
//				oled_printl(&oled, "READ WDATA");

				if(xQueueReceive(qmodbus_hr_val, &reg_val, pdMS_TO_TICKS(500)) == pdPASS){
//					oled_printl(&oled, "WDATA RES");
					publish_res = publish_reg_data(MODBUS_REG_TYPE_WDATA, i, reg_val);
					if(!publish_res){
						oled_printl(&oled, "failed to publish");
					}
				}

			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}





void modbus_res_handler_task(void* pvArgs){

	volatile uint8_t fc = 0;
	volatile modbus_res_type res_type = MODBUS_RES_UNKNOWN;
	volatile uint16_t data = 0;

	for(;;){

		xSemaphoreTake(modbus_res_semphr, portMAX_DELAY);

		res_type = MODBUS_MASTER_response_check(&master, MODBUS_SLAVE_ADDR);
		fc = master.pchRxBuffer[1];
		if(res_type == MODBUS_RES_OK){
			data = master.pchRxBuffer[3]<<8 | master.pchRxBuffer[4];
//			oled_printl(&oled, "MODBUS_RES_OK");
			if(fc == MODBUS_FC_RD_DI){
				xQueueSend(qmodbus_di_val, (void*)(&data), pdMS_TO_TICKS(100));
			}

			else if(fc == MODBUS_FC_RD_DO){
				xQueueSend(qmodbus_coil_val, (void*)(&data), pdMS_TO_TICKS(100));
			}
			else if(fc == MODBUS_FC_RD_HR){
				xQueueSend(qmodbus_hr_val, (void*)(&data), pdMS_TO_TICKS(100));
			}
			else if(fc == MODBUS_FC_RD_IR){
				xQueueSend(qmodbus_ir_val, (void*)(&data), pdMS_TO_TICKS(100));
			}
			else if(fc == MODBUS_FC_WR_DO){

			}
			else if(fc == MODBUS_FC_WR_HR){

			}
			else{

			}

//				oled_printl(&oled, "MODBUS_RES_OK");
		}
		else if(res_type == MODBUS_RES_EXCEPTION){
//			oled_printl(&oled, "MODBUS_RES_EXCEPTION");
//			sprintf(oled_buff, "exception 0x%X", (uint16_t) exception.exception_code);
		}
		else{
			oled_printl(&oled, "UNKNOWN RESPONSE!");
		}

//		memset(&(master.pchRxBuffer[0]), 0, MODBUS_RX_SIZE);
//		memset(&(master.pchTxBuffer[0]), 0, MODBUS_TX_SIZE);

	}
}


bool _sim_restart(){
	  sim_reboot(&sim);
	  HAL_Delay(5000);
	  HAL_NVIC_SystemReset();
	  if(setup()){
		  return true;
	//	  repeative_task();
	//	  rtc_set_alarm_seconds_it(&hrtc, REPEAT_DELAY);
	  }
	  return false;
}


void task_mqtt_conn_check(void* args){
	volatile uint8_t fault_count = 0;
	volatile uint8_t rx[60] = {0};
	for(;;){
		HAL_IWDG_Refresh(&hiwdg);
		vTaskDelay(pdMS_TO_TICKS(10000));
		HAL_UART_AbortReceive(mqtt_conn.sim->huart);
		memset(rx, '\0', 60);
		if(!mqtt_conn_status(&mqtt_conn, rx)){

			fault_count++;
			if(fault_count == 5){
				oled_printl(&oled, "NOT CONNECTED!");
				vSystemRst();
			}

		}
		else{
			fault_count = 0;
		}

	}
}




void vSystemRst(){
	oled_printl(&oled, "SYSTEM RESTART!");
	bool setup = _sim_restart();
	while(!setup){
		setup = _sim_restart();
	}

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
