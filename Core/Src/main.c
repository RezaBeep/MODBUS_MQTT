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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mqtt.h"
#include "sim.h"
#include "oled.h"
#include "modbus.h"
#include "rtc_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_TX_SIZE	8
#define MODBUS_RX_SIZE	64
#define MODBUS_SLAVE_ADDR	57
#define MODBUS_COIL_ADDR_STEP	16
#define MODBUS_REG_ADDR_STEP	1
#define MODBUS_ADDR_COIL_MAX	30
#define MODBUS_ADDR_HOLDING_REG_MAX		30
#define MODBUS_ADDR_INPUT_REG_MAX	30
#define MODBUS_ADDR_DISCRETE_INPUT_MAX	30


#define MQTT_KEEPTIME	"60"
#define MQTT_PAYLOAD_BUFF_SIZE	20
#define MQTT_TOPIC_BUFF_SIZE	50
#define MQTT_BROKER_ADDR	"5.198.179.50"
#define MQTT_BROKER_PORT	"1883"
#define MQTT_CLIENT_ID		"STM32_GATEWAY"

#define REPEAT_DELAY	10

#define OLED_BUFF_SIZE	15

#define _DEBUG	true


const UART_HandleTypeDef* PHUART_SIM = &huart1;
const UART_HandleTypeDef* PHUART_MODBUS = &huart2;


const char* topic_coil = "MCU/COIL/";
const char* topic_holding_reg = "MCU/HOLDING_REG/";
const char* topic_discrete_input = "MCU/DISCRETE_INPUT/";
const char* topic_input_reg = "MCU/INPUT_REG/";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId_t repeativeTaskHandle;
const osThreadAttr_t repeativeTask_attributes = {
  .name = "repeativeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



osSemaphoreId_t mySemHandle;



sim_t sim;
oled_t oled;
mqtt_conn_t mqtt_conn;
MODBUS_MASTER_InitTypeDef master;

char modbus_tx_buff[MODBUS_TX_SIZE];
char modbus_rx_buff[MODBUS_RX_SIZE];
char mqtt_payload_buff[MQTT_PAYLOAD_BUFF_SIZE];
char mqtt_topic_buff[MQTT_TOPIC_BUFF_SIZE];
char oled_buff[OLED_BUFF_SIZE];

bool coil_ready_to_send = false;
bool discrete_in_ready_to_send = false;
bool input_reg_ready_to_send = false;
bool holding_reg_ready_to_send = false;
MODBUS_MASTER_res* modbus_res;
uint16_t modbus_coil_addr = 0;
uint16_t modbus_holding_reg_addr = 4097;
uint16_t modbus_discrete_input_addr = 1025;
uint16_t modbus_input_reg_addr = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

bool setup();
void repeative_task(void *args);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	rtc_set_alarm_seconds_it(hrtc, REPEAT_DELAY);
//	repeative_task();
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12){
		if(sim_reboot(&sim)){
			oled_printl(&oled, "sim reboot");
		}
	}

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == PHUART_MODBUS->Instance){
		oled_printl(&oled, "modbus req sent");
	}

}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == PHUART_MODBUS->Instance){

	}
}





void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	oled_printl(&oled, "rx event");
	if(huart->Instance == PHUART_MODBUS->Instance){
		oled_printl(&oled, "modbus response");
		MODBUS_MASTER_res normal_res = {0};
		MODBUS_MASTER_exception exception = {0};

		if(MODBUS_MASTER_response_handler(&master, MODBUS_SLAVE_ADDR, &normal_res, &exception) == MODBUS_RES_OK){
			uint8_t* register_data = normal_res.register_data;
			if(!register_data){
				oled_printl(&oled, "register NULL");
			}
			else{
				if(normal_res.function_code == 1){
					coil_ready_to_send = true;
				}
				else if(normal_res.function_code == 2){
					discrete_in_ready_to_send = true;
				}
				else if(normal_res.function_code == 3){
					holding_reg_ready_to_send = true;
				}
				else if(normal_res.function_code == 4){
					input_reg_ready_to_send = true;
				}

				modbus_res = &normal_res;
				sprintf(mqtt_payload_buff, "%04X", (uint16_t)((register_data[0]<<8) | register_data[1]));
				oled_printl(&oled, "MODBUS_RES_OK");
			}
		}
		else if(MODBUS_MASTER_response_handler(&master, MODBUS_SLAVE_ADDR, &normal_res, &exception) == MODBUS_RES_EXCEPTION){
			oled_printl(&oled, "MODBUS_RES_EXCEPTION");
			sprintf(oled_buff, "exception 0x%X", (uint16_t) exception.exception_code);
		}
		else{
			oled_printl(&oled, "UNKNOWN RESPONSE!");
		}


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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  oled_init(&oled, &hi2c1);
  sim_init(&sim, PHUART_SIM, "mtnirancell", "", "");
  mqtt_init(&mqtt_conn, &sim, "STM32_GATEWAY", "5.198.179.50", "1883", "", "", MQTT_KEEPTIME);
  MODBUS_MASTER_init(&master, PHUART_MODBUS, modbus_tx_buff, modbus_rx_buff);


setup:
  if(setup()){
	  mqtt_publish_string(&mqtt_conn, "0", "0", "stm32", "connected");
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
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  mySemHandle = osSemaphoreNew(1, 1, NULL);
  repeativeTaskHandle = osThreadNew(repeative_task, NULL, &repeativeTask_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void repeative_task(void *args){

	while(1){
		MODBUS_MASTER_read_discrete_input(&master, MODBUS_SLAVE_ADDR, modbus_discrete_input_addr, MODBUS_COIL_ADDR_STEP);
		for(uint8_t i=0;i<2;i++){
			if(discrete_in_ready_to_send){
				sprintf(mqtt_topic_buff, "%s%d", topic_discrete_input, modbus_discrete_input_addr);
				if(!mqtt_publish_hex(&mqtt_conn, "0", "0", mqtt_topic_buff, mqtt_payload_buff)){
					oled_printl(&oled, "Failed to publish");
				}
				else{
					oled_printl(&oled, "published");
					modbus_discrete_input_addr += MODBUS_COIL_ADDR_STEP;
					MODBUS_MASTER_read_discrete_input(&master, MODBUS_SLAVE_ADDR, modbus_discrete_input_addr, MODBUS_COIL_ADDR_STEP);
				}
				discrete_in_ready_to_send = false;
			}
			osDelay(pdMS_TO_TICKS(1000));
		}
		modbus_discrete_input_addr = 1025;

		MODBUS_MASTER_read_holding_reg(&master, MODBUS_SLAVE_ADDR, modbus_holding_reg_addr, MODBUS_REG_ADDR_STEP);
		for(uint8_t i=0;i<3;i++){
			if(holding_reg_ready_to_send){
				sprintf(mqtt_topic_buff, "%s%d", topic_holding_reg, modbus_holding_reg_addr);
				if(!mqtt_publish_hex(&mqtt_conn, "0", "0", mqtt_topic_buff, mqtt_payload_buff)){
					oled_printl(&oled, "Failed to publish");
				}
				else{
					oled_printl(&oled, "published");
					modbus_holding_reg_addr += MODBUS_REG_ADDR_STEP;
					MODBUS_MASTER_read_holding_reg(&master, MODBUS_SLAVE_ADDR, modbus_holding_reg_addr, MODBUS_REG_ADDR_STEP);
				}
				holding_reg_ready_to_send = false;
			}
			osDelay(pdMS_TO_TICKS(1000));
		}
		modbus_holding_reg_addr = 4097;
	}
}




bool setup(){
	  oled_printl(&oled, "Please wait");
//	  HAL_Delay(15000);
	  oled_printl(&oled, "sending AT..");
	  if(sim_test_at(&sim)){
		  oled_printl(&oled, "AT OK!");
	  }
	  while(sim.state < SIM_STATE_AT_OK){
		  sim_test_at(&sim);
	  }
	  if(sim_report_error_enable(&sim)){
		  oled_printl(&oled, "+CMEE=2");
	  }
	  while(sim.state < SIM_STATE_REPORT_ERROR_ENABLED){}
	  if(sim_is_ready(&sim)){
		  oled_printl(&oled, "ready");
	  }
	  while(sim.state < SIM_STATE_PIN_READY){}
	  if(sim_registered(&sim)){
		  oled_printl(&oled, "registered");
	  }
	  while(sim.state < SIM_STATE_CREG_OK){}
	  if(sim_gprs_registered(&sim)){
		  oled_printl(&oled, "gprs registered");
	  }
	  while(sim.state < SIM_STATE_CGREG_OK){}

	  //mqtt disconnect
	  if(!mqtt_disconnect(&mqtt_conn)){
	  	  oled_printl(&oled, "broker disconnect error!");
	   }


	  // gprs disconnect
	  if(!sim_gprs_disconnect(&sim)){
		  oled_printl(&oled, "gprs disconnected already!");
	  }

//	  HAL_Delay(5000);
	  if(sim_gprs_connect(&sim)){
	//	  sim_event_listen_once(&sim);
		  uint8_t i = 0;
		  oled_printl(&oled, "activatin app network");
		  while(!(sim.app_network)){
			  i++;
			  HAL_Delay(5000);
			  oled_printl(&oled, "retrying app net");
			  sim_gprs_connect(&sim);

			  if(i>3){
				  return false;
			  }
		  }
	//	  HAL_UART_AbortReceive_IT(sim.huart);

		  oled_printl(&oled, "Connecting to broker");
		  if(mqtt_connect(&mqtt_conn)){
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



void publish(char* topic, char* payload){


}
/* USER CODE END 4 */

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
