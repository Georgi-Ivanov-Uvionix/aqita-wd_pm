/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define UVX_COMM_M2JMB_V2_0

#include "uvx_batt.h"
#include "uvx_gpio.h"
#include "uvx_uart.h"
#include "uvx_spi.h"
#include "uvx_ws2812.h"
#include "uvx_comm_m2m.h"
#include "uvx_comm_m2jmb.h"
#include "uvx_comm_m2jmb_v2_0.h"
#include "uvx_comm_bq.h"
#include "uvx_timer.h"
#include "uvx_led_strip.h"
#include "uvx_ws2812.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//=================== APP SETUP======================================================
#define PROJECT_AQITA_PM
#define APP_TIMEOUT_JMB_HEARTBEAT		    600000 //ms
#define APP_TIMEOUT_JMB_POWER_ON		    120000 //ms
#define APP_TIMEOUT_JMB_POWER_OFF		    6000 //ms
#define APP_TIMEOUT_JMB_CHECK		        1000 //ms
#define APP_TIMEOUT_IDLE    		        500 //ms
#define APP_TIMEOUT_BUTTON              1000 //ms
#define APP_TIMEOUT_BUTTON_SHORT        250 //ms
#define APP_TIMEOUT_BUTTON_LONG         500 //ms
#define APP_TIMEOUT_BEFORE_SLEEP        500 //ms
#define APP_TIMEOUT_LED                 10 //ms
#define APP_TIMEOUT_LED_BATT_LEVEL      10 //ms
#define APP_TIMEOUT_JMB_BEFORE_TURN_ON  1000 //ms
#define APP_TIMEOUT_LAND                5000 //ms
#define APP_TIMEOUT_BATT_LOW_VOLTAGE    10000 //ms
#ifdef APP_JETSON_PWR_FC
#define APP_TIMEOUT_JMB_MAX_TURN_OFF  30000 //ms
#else
#define APP_TIMEOUT_JMB_MAX_TURN_OFF  10000 //ms
#endif
#define UVX_APP_TIMEOUT_M2M				    50 //ms
#define APP_TIMEOUT_PACK_V_STABLE_HIGH     3000 //ms
#define APP_TIMEOUT_PACK_V_STABLE_LOW      1000 //ms

#define ADC_MAX        4095.0f
#define VDDA           3.3f
#define RESISTOR_R124  45.3f
#define RESISTOR_R8    2.80f
#define DIV_RATIO      (RESISTOR_R8 / (RESISTOR_R124 + RESISTOR_R8))
#define PACK_V_GAIN    ((VDDA / (ADC_MAX * DIV_RATIO)) * 1000)

//------------------- APP SETUP UARTs --------------------------------------------
#define UVX_APP_SETUP_UART_1		UVX_SETUP_UART_1
#define UVX_APP_SETUP_UART_2    UVX_SETUP_UART_2

//------------------- APP SETUP I2C ---------------------------------------------
#define UVX_APP_SETUP_I2C_BQ		UVX_SETUP_I2C_1

//------------------- APP SETUP TIMER --------------------------------------------

//=================== APP SETUP END =================================================
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM3_CH1_IC_CUBE_LED1_Pin GPIO_PIN_3
#define TIM3_CH1_IC_CUBE_LED1_GPIO_Port GPIOE
#define TIM3_CH2_IC_CUBE_LED2_Pin GPIO_PIN_4
#define TIM3_CH2_IC_CUBE_LED2_GPIO_Port GPIOE
#define TIM3_CH3_IC_CUBE_LED3_Pin GPIO_PIN_5
#define TIM3_CH3_IC_CUBE_LED3_GPIO_Port GPIOE

#define ADC1_IN1_BATTERY_VOLTAGE_Pin 				GPIO_PIN_0
#define ADC1_IN1_BATTERY_VOLTAGE_GPIO_Port 	GPIOC

#define ADC1_IN2_CUBE_CURRENT_Pin 					GPIO_PIN_1
#define ADC1_IN2_CUBE_CURRENT_GPIO_Port 		GPIOC

#define ADC1_IN3_FET_TEMP_Pin 							GPIO_PIN_2
#define ADC1_IN3_FET_TEMP_GPIO_Port 				GPIOC

#define ADC_IN4_CELL_1S_ADC_Pin 						GPIO_PIN_3
#define ADC_IN4_CELL_1S_ADC_GPIO_Port 			GPIOC

#define ADC_IN5_CELL_2S_ADC_Pin 						GPIO_PIN_0
#define ADC_IN5_CELL_2S_ADC_GPIO_Port 			GPIOA

#define ADC1_IN6_CELL_3S_ADC_Pin 						GPIO_PIN_1
#define ADC1_IN6_CELL_3S_ADC_GPIO_Port 			GPIOA

#define USART2_TX_JETSON_RX_Pin GPIO_PIN_2
#define USART2_TX_JETSON_RX_GPIO_Port GPIOA
#define USART2_RX_JETSON_TX_Pin GPIO_PIN_3
#define USART2_RX_JETSON_TX_GPIO_Port GPIOA
#define ADC1_IN9_CELL_4S_ADC_Pin GPIO_PIN_4
#define ADC1_IN9_CELL_4S_ADC_GPIO_Port GPIOA
#define ADC1_IN10_CELL_5S_ADC_Pin GPIO_PIN_5
#define ADC1_IN10_CELL_5S_ADC_GPIO_Port GPIOA
#define ADC1_IN11_CELL_6S_ADC_Pin GPIO_PIN_6
#define ADC1_IN11_CELL_6S_ADC_GPIO_Port GPIOA
#define ADC1_IN12_CELL_7S_ADC_Pin GPIO_PIN_7
#define ADC1_IN12_CELL_7S_ADC_GPIO_Port GPIOA
#define ADC1_IN15_CELL_8S_ADC_Pin GPIO_PIN_0
#define ADC1_IN15_CELL_8S_ADC_GPIO_Port GPIOB
#define GPIO_INPUT_PG_Pin GPIO_PIN_1
#define GPIO_INPUT_PG_GPIO_Port GPIOB
#define GPIO_INPUT_STAT1_Pin GPIO_PIN_2
#define GPIO_INPUT_STAT1_GPIO_Port GPIOB
#define GPIO_INPUT_STAT2_Pin GPIO_PIN_7
#define GPIO_INPUT_STAT2_GPIO_Port GPIOE
#define GPIO_INPUT_INT_Pin GPIO_PIN_8
#define GPIO_INPUT_INT_GPIO_Port GPIOE
#define GPIO_OUTPUT_CE_Pin GPIO_PIN_9
#define GPIO_OUTPUT_CE_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_1S_DISCH_EN_Pin GPIO_PIN_10
#define GPIO_OUTPUT_CELL_1S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_2S_DISCH_EN_Pin GPIO_PIN_11
#define GPIO_OUTPUT_CELL_2S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_3S_DISCH_EN_Pin GPIO_PIN_12
#define GPIO_OUTPUT_CELL_3S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_4S_DISCH_EN_Pin GPIO_PIN_13
#define GPIO_OUTPUT_CELL_4S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_5S_DISCH_EN_Pin GPIO_PIN_14
#define GPIO_OUTPUT_CELL_5S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_6S_DISCH_EN_Pin GPIO_PIN_15
#define GPIO_OUTPUT_CELL_6S_DISCH_EN_GPIO_Port GPIOE
#define GPIO_OUTPUT_CELL_7S_DISCH_EN_Pin GPIO_PIN_10
#define GPIO_OUTPUT_CELL_7S_DISCH_EN_GPIO_Port GPIOB
#define GPIO_OUTPUT_CELL_8S_DISCH_EN_Pin GPIO_PIN_11
#define GPIO_OUTPUT_CELL_8S_DISCH_EN_GPIO_Port GPIOB
#define GPIO_OUTPUT_RED_LED_Pin GPIO_PIN_13
#define GPIO_OUTPUT_RED_LED_GPIO_Port GPIOB
#define GPIO_OUTPUT_GREEN_LED_Pin GPIO_PIN_14
#define GPIO_OUTPUT_GREEN_LED_GPIO_Port GPIOB
#define GPIO_OUTPUT_BLUE_LED_Pin GPIO_PIN_15
#define GPIO_OUTPUT_BLUE_LED_GPIO_Port GPIOB
#define GPIO_OUTPUT_ILIM_HIZ_Pin GPIO_PIN_8
#define GPIO_OUTPUT_ILIM_HIZ_GPIO_Port GPIOD
#define GPIO_OUTPUT_EN_JETSON_PS_Pin GPIO_PIN_10
#define GPIO_OUTPUT_EN_JETSON_PS_GPIO_Port GPIOC
#define GPIO_OUTPUT_PERIPHERIAL_EN_Pin GPIO_PIN_13
#define GPIO_OUTPUT_PERIPHERIAL_EN_GPIO_Port GPIOD
#define GPIO_OUTPUT_NANO_FORCE_REC_Pin GPIO_PIN_14
#define GPIO_OUTPUT_NANO_FORCE_REC_GPIO_Port GPIOD
#define GPIO_OUTPUT_FC_EN_Pin GPIO_PIN_15
#define GPIO_OUTPUT_FC_EN_GPIO_Port GPIOD
#define GPIO_OUTPUT_DRONE_START_FET_EN_Pin GPIO_PIN_6
#define GPIO_OUTPUT_DRONE_START_FET_EN_GPIO_Port GPIOC
#define GPIO_OUTPUT_BATT_MEASURE_EN_Pin GPIO_PIN_7
#define GPIO_OUTPUT_BATT_MEASURE_EN_GPIO_Port GPIOC
#define GPIO_INPUT_PG_DRONE_START_Pin GPIO_PIN_8
#define GPIO_INPUT_PG_DRONE_START_GPIO_Port GPIOC
#define GPIO_EXTI8_DRONE_START_BUT_Pin GPIO_PIN_8
#define GPIO_EXTI8_DRONE_START_BUT_GPIO_Port GPIOA
#define USART1_TX_FC_RX_Pin GPIO_PIN_9
#define USART1_TX_FC_RX_GPIO_Port GPIOA
#define USART1_RX_FC_TX_Pin GPIO_PIN_10
#define USART1_RX_FC_TX_GPIO_Port GPIOA
#define GPIO_OUTPUT_AP_ON_Pin GPIO_PIN_11
#define GPIO_OUTPUT_AP_ON_GPIO_Port GPIOA
#define GPIO_OUTPUT_JETSON_EN_Pin GPIO_PIN_10
#define GPIO_OUTPUT_JETSON_EN_GPIO_Port GPIOC
#define GPIO_OUTPUT_FAN_CONTROL_Pin GPIO_PIN_0
#define GPIO_OUTPUT_FAN_CONTROL_GPIO_Port GPIOD

#define GPIO_EXTI9_WIFi_AP_CLIENT_Pin 				GPIO_PIN_9
#define GPIO_EXTI9_WIFi_AP_CLIENT_GPIO_Port 		GPIOC
#define GPIO_EXTI9_WIFi_AP_CLIENT_EXTI_IRQn 		EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

typedef enum
{
  DRONE_IDLE = 0,
  DRONE_CHECK_BUTTON_PRESS_ONCE,
  DRONE_CHECK_BUTTON_PRESS_TWICE,
  DRONE_CHECK_BUTTON_TIMEOUT,
  DRONE_JETSON_POWER_OFF,
  DRONE_JETSON_POWERED_OFF,
  DRONE_JETSON_POWER_ON,
  DRONE_JETSON_POWERED_ON,
  DRONE_SLEEP,  
  DRONE_TIMEOUT
} DRONE_STATE;

typedef enum
{
  NO_POWER = 0,
  LOW_POWER,
  MIDDLE_POWER,
  HIGH_POWER
} DRONE_CHARGE_STATE;

typedef struct 
{
  DRONE_STATE state_previous; // Previous state of the M2JMB communication
  DRONE_STATE state_current;  // Current state of the M2JMB communication
  DRONE_STATE state_next; // Next state of the M2JMB communication
} UVX_DRONE_STATE_MACHINE;

typedef struct
{
  uint8_t cell_count   : 1;
} UVX_UNIT_TEST;

typedef struct
{
  uint8_t btn_cnt;
  uint8_t btn_state   : 1;
  uint8_t pwr_fet     : 1;
  uint8_t pwr_jtson   : 1;
  uint8_t pwr_fc      : 1;
  uint8_t hall_land_2 : 1;
  uint8_t esc_comm    : 1;
  uint8_t esc_init    : 1;
  uint8_t esc_arm     : 1;
  uint8_t esc_land_complete : 1;
  uint8_t esc_flying  : 1;
  uint8_t esc_psys_arm : 1;
  uint8_t esc_psys_arm_old : 1;
  uint8_t charge_overvoltage : 1;
  uint8_t charge_cell_count_error : 1;

  DRONE_CHARGE_STATE charge_state;
} UVX_DRONE_STATUS;

extern UVX_DRONE_STATE_MACHINE drone_state;
extern UVX_TIMER timer_app_batt_pwr_high;
extern UVX_TIMER timer_app_batt_pwr_low;
extern UVX_TIMER timer_app_batt_low_voltage;
extern UVX_TIMER timer_app_comm_jmb;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern UVX_DRONE_STATUS drone_status;
extern UVX_UNIT_TEST unit_test;
extern uint8_t enable;

void UVX_APP_PWR_FET(uint8_t state);
void UVX_APP_Shutdown_JMB(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
