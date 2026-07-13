/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE									1
#define FALSE									0

#define	ENALBE_BATT_MEASURE						HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_BATT_MEASURE_EN_Pin, GPIO_PIN_SET)
#define	DISABLE_BATT_MEASURE					HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_BATT_MEASURE_EN_Pin, GPIO_PIN_RESET)
#define	PWR_DRONE								HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_DRONE_START_FET_EN_Pin, GPIO_PIN_SET)
#define	SHUTDWON_DRONE							HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_DRONE_START_FET_EN_Pin, GPIO_PIN_RESET)
#define PWR_JETSON								HAL_GPIO_WritePin(GPIO_OUTPUT_EN_JETSON_PS_GPIO_Port, GPIO_OUTPUT_EN_JETSON_PS_Pin, GPIO_PIN_SET)
#define SHUTDOWN_JETSON							HAL_GPIO_WritePin(GPIO_OUTPUT_EN_JETSON_PS_GPIO_Port, GPIO_OUTPUT_EN_JETSON_PS_Pin, GPIO_PIN_RESET)
#define PWR_PER								    HAL_GPIO_WritePin(GPIOD, GPIO_OUTPUT_PERIPHERIAL_EN_Pin, GPIO_PIN_SET)
#define SHUDONW_PER				     			HAL_GPIO_WritePin(GPIOD, GPIO_OUTPUT_PERIPHERIAL_EN_Pin, GPIO_PIN_RESET)
#define	DRONE_PWR_GOOD_SET						(HAL_GPIO_ReadPin(GPIOC,GPIO_INPUT_PG_DRONE_START_Pin) == GPIO_PIN_RESET)
#define	DRONE_PWR_GOOD_RESET					HAL_GPIO_ReadPin(GPIOC,GPIO_INPUT_PG_DRONE_START_Pin)	== GPIO_PIN_SET
#define BUTTON_WAS_PRESSED						HAL_GPIO_ReadPin(GPIOA, GPIO_EXTI8_DRONE_START_BUT_Pin) == GPIO_PIN_RESET
#define BUTTON_WAS_RELEASED						HAL_GPIO_ReadPin(GPIOA, GPIO_EXTI8_DRONE_START_BUT_Pin) == GPIO_PIN_SET
#define PWR_BUTTON_INTERRUPT					GPIO_Pin == GPIO_PIN_8

#define __Change_Interrupt_To_Rising_Edge  		Change_To_Rising_Edge_Interrupt()
#define __Change_Interrupt_To_Falling_Edge  	Change_To_Falling_Edge_Interrupt()

#define __No_Button_Was_Pressed			    	(!g_Pressed_Once && !g_Released_Once && !g_Pressed_Twice && !g_Released_Twice)

#define __Button_Was_Pressed_Once				(g_Pressed_Once && !g_Released_Once && !g_Pressed_Twice && !g_Released_Twice)
#define __Button_Was_Released_Once				(g_Pressed_Once && g_Released_Once && !g_Pressed_Twice && !g_Released_Twice)

#define __Button_Was_Released_Twice				(g_Pressed_Once && g_Released_Once && g_Pressed_Twice && g_Released_Twice)
#define __Button_Was_Pressed_Twice				(g_Pressed_Once && g_Released_Once && g_Pressed_Twice && !g_Released_Twice)

#define TIME_FOR_FIRST_PRESS					100 	//50
#define TIME_FOR_SECOND_PRESS					2000	//1000
#define	TIME_BEFOR_GO_TO_SLEEP					8000	//2500
// #define TIME_FOR_DEBOUNCE					50		//25
#define TIME_FOR_OK_LED_TOGGLE					300		//150
#define TIME_FOR_ERROR_LED_TOGGLE				500		//250

//#define APP_JETSON_PWR_FC
//#define APP_NO_BATTERY_MODE
#define APP_HALL_POWER_ENABLE
/* USER CODE END PD */

#define HW_STRING 										"PM HW v2.1" //
#define SW_STRING 										"PM SW v1.0.1" //

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

uint8_t g_Start_Drone = 0, g_Stop_Drone = 0;
uint8_t g_Timer_Twice_Started = 0;
uint8_t g_Drone_Not_Started = 0;
uint8_t g_Drone_Started = 0;
uint8_t g_Drone_Has_Started = 0;
uint8_t g_Drone_Has_ShutDown = 0;
uint8_t g_Rising_Edge = 0, g_Falling_Edge = 1;
uint8_t g_Timer_Once_Started = 0;
uint8_t g_Button_Pressed_Once_Correct = 0;
uint8_t g_OK_LED = 0;
uint8_t g_Enter_Sleep = 0;
uint8_t state = TRUE;
uint8_t g_Pressed_Once = 0;
uint8_t g_Released_Once = 0;
uint8_t g_Pressed_Twice = 0;
uint8_t g_Released_Twice = 0;
uint8_t g_Time_For_No_Pressed_Buttton = 0;
uint8_t g_Error_LED = 0;
volatile uint8_t g_Sleep = 0;
uint8_t g_Toggle_Count = 0;
uint8_t g_Test = 0;
uint8_t g_esc_data_started = 0;
uint8_t g_HALL_LAND_2 = 0;

uint32_t sys_CLK;
uint32_t g_Battery_Voltage_Raw[3];

uint8_t u2_rx_buffer[100];

uint8_t *p_u2_rx;
uint32_t u2_cnt_rx = 0;
uint32_t u1_cnt_rx = 0;
uint16_t batt_reg_cnt = 0;
uint16_t for_test = 0;

float 	g_Battery_Voltage[3];

UVX_UART uart_1;
UVX_UART uart_2; // M2M - Mother board to Mother board
UVX_I2C i2c_bq; //BQ - Battery management IC
UVX_SPI  spi_1;
UVX_TIMER timer_app;
UVX_TIMER timer_app_comm_jmb;
UVX_TIMER timer_app_batt_pwr_high;
UVX_TIMER timer_app_batt_pwr_low;
UVX_TIMER timer_app_batt_low_voltage;
UVX_TIMER timer_app_drone;
UVX_TIMER timer_app_led;
UVX_TIMER timer_app_land;

UVX_DRONE_STATE_MACHINE drone_state;
UVX_DRONE_STATUS drone_status;
SRAM1 WS2812_Driver ws2812_strip;  // WS2812 LED strip driver

uint8_t btn_percent = 0;

uint8_t jmb_data_tx[BUFF_SIZE_TX_JMB] = {0};
uint8_t esc_data_tx[BUFF_SIZE_TX_ESC] = {0};

uint8_t simu_esc_data_tx[BUFF_SIZE_TX_ESC] = {
    0x28, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x22,
    0x00, 0x00, 0x00, 0x07, 0x7B, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xDC, 0xED, 0x00, 0x00, 0xFF, 0xF0, 0x00, 0x0E,
    0x00, 0x00, 0x42, 0x32, 0x00, 0x00, 0x00, 0x4A,
    0xFF, 0xF7, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x30,
    0x00, 0x00, 0x89, 0x00, 0x0D, 0xFF, 0x11, 0x00,
    0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD9,
    0x00, 0xFF, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xA0
};



uint16_t jmb_payload_size = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void UVX_APP_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void Change_To_Rising_Edge_Interrupt(void);
void Change_To_Falling_Edge_Interrupt(void);
void Dron_Button_Reset_State(void);
void OK_LED(void);
void Error_LED(void);
void Start_Button_Once_Pressed_Timer(void);
void Set_Timer_For_No_Pressed_Button(void);
void Stop_Button_Pressed_Once_Timer(void);
void Stop_Button_Pressed_Twice_Timer(void);
void Dron_Button_Once_Pressed_State(void);
void enter_LPSleep(void);
void exit_Sleep(void);
void Set_Timers_For_Sleep(void);
void Config_SysClk_MSI_131(void);
void Reset_Button_Timer(void);
void Config_SysClk_HSE(void);
void Set_Timers_After_Sleep(void);
void Start_Button_Pressed_Twice_Timer(void);
void UVX_APP(void);
void UVX_APP_Shutdown_JMB(void);
void UVX_APP_Comm_m2m(void);
void UVX_APP_Comm_m2jmb(void);
void UVX_APP_Batt(void);
void UVX_APP_LED_Strip (void);
void UVX_APP_HALL_LAND(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
 uint32_t test = 0;
 uint8_t data_to_send = 'A'; // Byte to send
 uint8_t enable = 0; // Flag to control sending
 uint8_t i2c_addr = 0x00;
 uint8_t i2c_data_rx[50] = {0};
 uint8_t i2c_data_tx[] = { 0x00, 0x54, 0x00 };

 uint8_t i2c_devices[10] = {0};
 uint8_t i2c_cnt_dev = 0;
 uint8_t test_batt_level = 0;
 uint8_t test_batt_marker = 0;
 uint16_t led_effect_repead = 0;

 uint32_t pclk1_freq = 0;

uint16_t adc_value;

int main(void)
{
 	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_ADC1_Init();
	MX_ADC2_Init();

	UVX_APP_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	MX_TIM4_Init();
	MX_I2C1_Init();

	Set_Timer_For_No_Pressed_Button();

	sys_CLK = HAL_RCC_GetSysClockFreq(); // should be 80MHz

	uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_SET);	
	uvx_gpio_set_pin(GPIO_OUTPUT_GREEN_LED, GPIO_PIN_SET);	
	uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
	uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);
	uvx_gpio_set_pin(GPIO_OUTPUT_BQH_I2C_EN, GPIO_PIN_SET); // bqh turn off
	uvx_gpio_set_pin(GPIO_OUTPUT_BQH_I2C_EN, GPIO_PIN_RESET);
	UVX_APP_PWR_FET(0); // power off
	uvx_gpio_set_pin(GPIO_OUT_LED_STRIP_ENABLE, GPIO_PIN_SET);

	pclk1_freq = HAL_RCC_GetPCLK1Freq();

	timer_app_batt_pwr_low.Timeout = APP_TIMEOUT_PACK_V_STABLE_LOW; // Reset timeout for power off
	timer_app_batt_pwr_low.Enable = true;
	
	while (1)
	{ 					
		UVX_APP();
	}
}

void UVX_APP(void)
{
	switch (drone_state.state_current)
	{
		case DRONE_IDLE:
			Change_To_Falling_Edge_Interrupt();
			drone_state.state_current = DRONE_CHECK_BUTTON_PRESS_ONCE;
			led_strip_state.state_next = LED_STRIP_MODE_IDLE;
		break;

		case DRONE_CHECK_BUTTON_PRESS_ONCE:
			if(drone_status.btn_state)
			{
				drone_status.btn_state = false;
				timer_app_drone.Timeout = APP_TIMEOUT_BUTTON_SHORT; // Reset timeout for power on
				timer_app_drone.Enable = true; 
				drone_state.state_current = DRONE_CHECK_BUTTON_TIMEOUT;
				drone_state.state_next = DRONE_CHECK_BUTTON_PRESS_TWICE;
				uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
				led_strip_state.state_next = LED_STRIP_MODE_BTN_PRESS;
				btn_percent = 0;		
			}
			else
			{
				timer_app_drone.Timeout = APP_TIMEOUT_BEFORE_SLEEP;
				timer_app_drone.Enable = true; 				
				drone_status.btn_cnt = 0;

				if((batt_data.init) && (!batt_data.adc_pack_v_stable_high))
				{
					drone_state.state_current = DRONE_SLEEP;
				}

				#ifdef APP_NO_BATTERY_MODE	
				if(comm_m2jmb_state.state_current == M2JMB_MODE_IDLE)			
				{
					drone_state.state_current = DRONE_SLEEP;
				}				
				#endif						
				
				if((drone_state.state_previous == DRONE_JETSON_POWER_ON) || (drone_state.state_previous == DRONE_JETSON_POWERED_ON))
				{
					drone_state.state_current = drone_state.state_previous; // Return to previous state if button is released after power on
				}

				drone_state.state_next = DRONE_IDLE;						
			}
		break;

		case DRONE_CHECK_BUTTON_PRESS_TWICE:
			if(timer_app_drone.Timeout == 0) // If timeout occurs
			{
				uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
				led_strip_state.state_next = LED_STRIP_MODE_BTN_PRESS;
				btn_percent = 0;					
				if(drone_status.btn_state)
				{
					drone_status.btn_state = false;
					timer_app_drone.Timeout = APP_TIMEOUT_BUTTON_LONG; // Reset timeout for power on
					timer_app_drone.Enable = true; 
					drone_state.state_current = DRONE_CHECK_BUTTON_TIMEOUT;
					if(comm_m2jmb_state.state_current == M2JMB_MODE_IDLE)
					{
						drone_state.state_next = DRONE_JETSON_POWER_ON;
						if(!batt_data.init)
						{
							batt_state.state_current = BATT_MODE_INIT;
						}
						else
						{
							batt_state.state_current = BATT_MODE_READ_BQ_L;
						}
						comm_m2jmb.Heartbeat = 0;
						memset(buff_rx_m2jmb, 0, sizeof(buff_rx_m2jmb));
						comm_m2m_state.state_current = M2M_MODE_IDLE;
					}
					else
					{
						if(!drone_status.esc_arm && !drone_status.esc_psys_arm)
						{
							drone_state.state_next = DRONE_JETSON_POWER_OFF;
						}
					}					
				}
				else
				{
					timer_app_drone.Timeout = APP_TIMEOUT_BEFORE_SLEEP;
					timer_app_drone.Enable = true; 				
					drone_status.btn_cnt = 0;

					if((batt_data.init) && (!batt_data.adc_pack_v_stable_high))
					{
						drone_state.state_current = DRONE_SLEEP;
					}

					drone_state.state_current = DRONE_IDLE;			
				}
			}
		break;		

		case DRONE_CHECK_BUTTON_TIMEOUT:
			if(timer_app_drone.Timeout == 0) 
			{
				uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);				
				led_strip_state.state_next = LED_STRIP_MODE_OFF;
				if(uvx_gpio_read_pin(GPIO_INPUT_EXTI8_DRONE_START))
				{
					timer_app_drone.Timeout = APP_TIMEOUT_BEFORE_SLEEP; // Reset timeout for power on
					timer_app_drone.Enable = true; 				
					drone_status.btn_cnt = 0;
					if((drone_state.state_previous == DRONE_JETSON_POWER_ON) || (drone_state.state_previous == DRONE_JETSON_POWERED_ON))
					{
						drone_state.state_current = drone_state.state_previous; // Return to previous state if button is released after power on
					}
					else
					{
						drone_state.state_current = DRONE_SLEEP;
					}
					
				}
				else
				{
					drone_status.btn_cnt++;
					timer_app_drone.Timeout = APP_TIMEOUT_BUTTON; // Reset timeout for power on
					timer_app_drone.Enable = true; 
					drone_status.btn_state = false;
					drone_state.state_current = drone_state.state_next;								
					if(drone_state.state_current == DRONE_JETSON_POWER_ON)
					{
						timer_app_comm_jmb.Timeout = APP_TIMEOUT_JMB_BEFORE_TURN_ON; // Reset timeout for power on
						timer_app_comm_jmb.Enable = true;
					}
				}
			}
		break;		

		case DRONE_JETSON_POWER_OFF:
			if(timer_app_drone.Timeout == 0) 
			{
				led_strip_state.state_next = LED_STRIP_MODE_WAITING;
				UVX_APP_Shutdown_JMB();				
				drone_state.state_previous = DRONE_JETSON_POWER_OFF;
			}
		break;

		case DRONE_JETSON_POWERED_OFF:
			UVX_APP_Comm_m2jmb();
		break;

		case DRONE_JETSON_POWER_ON:	
			UVX_APP_Comm_m2jmb();
			drone_state.state_previous = DRONE_JETSON_POWER_ON;
		break;

		case DRONE_JETSON_POWERED_ON:			
			UVX_APP_Comm_m2m();			
			UVX_APP_Comm_m2jmb();
			drone_state.state_previous = DRONE_JETSON_POWERED_ON;
		break;

		case DRONE_SLEEP:
			if(timer_app_drone.Timeout == 0) // If timeout occurs
			{
				#ifdef APP_NO_BATTERY_MODE
				batt_data.adc_pack_v_stable_high = 0;
				batt_data.adc_pack_v_stable_low = 1;	
				#endif							

				if((!batt_data.adc_pack_v_stable_high) &&
				   (batt_data.adc_pack_v_stable_low) &&
				   (comm_m2jmb_state.state_current == M2JMB_MODE_IDLE) &&
				   (!comm_bq_h.Force_balance || !comm_bq_l.Force_balance)) 
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
					batt_state.state_current = BATT_MODE_STOP;
					enter_LPSleep();
				}	
				else
				{
					drone_state.state_current = drone_state.state_next;
				}			
			}			
		break;			

		case DRONE_TIMEOUT:
			if(timer_app_drone.Timeout == 0) // If timeout occurs
			{
				drone_state.state_current = DRONE_CHECK_BUTTON_TIMEOUT;
			}			
		break;		
	}

#ifndef APP_NO_BATTERY_MODE	
	UVX_APP_Batt();
#endif

	UVX_APP_LED_Strip();
	UVX_APP_HALL_LAND();
}

void UVX_APP_HALL_LAND(void)
{
#ifdef APP_HALL_POWER_ENABLE	
	if(uvx_gpio_read_pin(GPIO_EXTI1_HALL_LAND_2) == GPIO_PIN_RESET)
	{
		if(timer_app_land.Timeout == 0) // If timeout occurs
		{
			if(drone_status.hall_land_2 == false)
			{
				drone_status.hall_land_2 = true;
				uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_RESET);
				if(batt_data.init)
				{
					batt_state.state_current = BATT_MODE_READ_BQ_L;
				}				
			}

			if((batt_data.temperature_cell_l <= MAX_CELL_TEMPERATURE) && (batt_data.temperature_cell_h <= MAX_CELL_TEMPERATURE))
			{
				if(batt_data.payload.relative_state_of_charge >= SOC_START_LOW_POWER)
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_LOW_CP, GPIO_PIN_SET);
					uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_HIGH_CP, GPIO_PIN_SET);
					drone_status.charge_state = LOW_POWER;
				}
				else
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_LOW_CP, GPIO_PIN_RESET);
					uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_HIGH_CP, GPIO_PIN_SET);
					drone_status.charge_state = HIGH_POWER;
				}

			}
			else
			{
				uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_LOW_CP, GPIO_PIN_SET);
				uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_HIGH_CP, GPIO_PIN_RESET);
				drone_status.charge_state = MIDDLE_POWER;
			}
		}
	}
	else
	{
		timer_app_land.Timeout = APP_TIMEOUT_LAND; // Reset timeout for power on
		timer_app_land.Enable = true;
		drone_status.hall_land_2 = false;
		uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
		uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_LOW_CP, GPIO_PIN_RESET);
		uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_HIGH_CP, GPIO_PIN_RESET);
	}
#else
	uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_LOW_CP, GPIO_PIN_RESET);
	uvx_gpio_set_pin(GPIO_OUTPUT_DOCK_HIGH_CP, GPIO_PIN_SET);
#endif	
}

void UVX_APP_LED_Strip (void)
{

	switch (led_strip_state.state_current)
	{
		case LED_STRIP_MODE_IDLE:
			if(timer_app_led.Timeout == 0)
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 
				uvx_led_strip_effect_idle(&ws2812_strip);
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;

		case LED_STRIP_MODE_WAITING:
			if(timer_app_led.Timeout == 0)
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 
				uvx_led_strip_effect_waiting(&ws2812_strip);
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}

			}
		break;

		case LED_STRIP_MODE_BTN_PRESS:
			if(timer_app_led.Timeout == 0)
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 
				uvx_led_strip_effect_charge_button(&ws2812_strip, btn_percent);

				if(btn_percent < 100)
				{
					btn_percent += 5;
				}
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;

		case LED_STRIP_MODE_BATTERY_LEVEL:
			if((timer_app_led.Timeout == 0) && (spi_1.TX_Ready)) // If timeout occurs
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED_BATT_LEVEL;// - test_batt_level; // Reset timeout for power on
				timer_app_led.Enable = true; 
				uvx_led_strip_effect_battery_level(&ws2812_strip, test_batt_level, test_batt_marker);
				
				test_batt_marker++;
				if(test_batt_marker >= test_batt_level)
				{
					test_batt_level++;
					test_batt_marker = 0;

					if(test_batt_level > 100)
					{
						test_batt_level = 0;
					}
				}

				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;	

		case LED_STRIP_MODE_HEARTBEAT:
			if(timer_app_led.Timeout == 0) // If timeout occurs
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 				
				uvx_led_strip_effect_heartbeat(&ws2812_strip);
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;		
		
		case LED_STRIP_MODE_COMM_FC:
			if(timer_app_led.Timeout == 0) // If timeout occurs
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 				
				uvx_led_strip_effect_active_comm(&ws2812_strip);
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;			

		case LED_STRIP_MODE_OFF:
			if(timer_app_led.Timeout == 0) // If timeout occurs
			{
				timer_app_led.Timeout = APP_TIMEOUT_LED; // Reset timeout for power on
				timer_app_led.Enable = true; 				
				uvx_led_strip_effect_off(&ws2812_strip);
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;				

		case LED_STRIP_MODE_STOP:
			if(timer_app_led.Timeout == 0) // If timeout occurs
			{
				timer_app_led.Timeout = 1; // Reset timeout for power on
				timer_app_led.Enable = true; 				
				
				led_effect_repead++;
				if(led_effect_repead > LED_EFFECT_MIN_TIMES)
				{
					led_effect_repead = 0;
					led_strip_state.state_current = led_strip_state.state_next;
				}
			}
		break;		
		
		default:
			led_effect_repead = 0;
			led_strip_state.state_current = led_strip_state.state_next;
		break;
	}
}

void         UVX_APP_Batt(void)
{
	switch(batt_state.state_current)
	{
		case BATT_MODE_INIT:
			uvx_comm_bq_change_list(&comm_bq_l, bq_l_register_list_read_once);
			uvx_comm_bq_change_list(&comm_bq_h, bq_h_register_list_read_once);
			batt_reg_cnt = 0;
			batt_state.state_current = BATT_MODE_READ_ONCE_BQ_L;
		break;

		case BATT_MODE_READ_ONCE_BQ_L:
			if(uvx_comm_bq_read_list(&comm_bq_l, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_ONCE_BQ_H;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_ONCE_BQ_L;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_READ_ONCE_BQ_H:
			if(uvx_comm_bq_read_list(&comm_bq_h, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_data.design_capacity = (bq_data_h.design_capacity + bq_data_l.design_capacity)/2;
				batt_data.design_voltage = (bq_data_h.design_voltage + bq_data_l.design_voltage);
				uvx_comm_bq_change_list(&comm_bq_l, bq_l_register_list_read);
				uvx_comm_bq_change_list(&comm_bq_h, bq_h_register_list_read);
				batt_state.state_current = BATT_MODE_INIT_BALANCE_L;
				uvx_batt_parse_data();				
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_ONCE_BQ_H;
				HAL_Delay(1);
			}
		break;		

		case BATT_MODE_INIT_BALANCE_L:
			uvx_comm_bq_force_balance(&comm_bq_l, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_INIT_BALANCE_H;
		break;

		case BATT_MODE_INIT_BALANCE_H:
			uvx_comm_bq_force_balance(&comm_bq_h, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_BQ_L;			
			batt_reg_cnt = 0;
		break;		

		case BATT_MODE_READ_BQ_L:
			if(uvx_comm_bq_read_list(&comm_bq_l, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_BQ_H;
				uvx_batt_parse_data();

				if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);
				}
				uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_BQ_L;
				batt_state.state_previous = BATT_MODE_WAIT_RESPONSE;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_READ_BQ_H:
			if(uvx_comm_bq_read_list(&comm_bq_h, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_CHECK_STATUS;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_BQ_H;
				batt_state.state_previous = BATT_MODE_WAIT_RESPONSE;
				HAL_Delay(1);
			}
		break;	

		case BATT_MODE_CHECK_STATUS:
			if(batt_data.CHG_fet_en)
			{
				uvx_comm_bq_write_mba_register(&comm_bq_h, BQ_MA_FET_CONTROL, NULL, 0);
			}

			if(batt_data.tc)
			{
				uvx_comm_bq_charge_fet(&comm_bq_h, 0);
				//UVX_APP_PWR_FET(0); // PWR off
			}
			else
			{
				if((batt_data.adc_pack_v_stable_high) && (drone_status.pwr_fet))
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_RESET);
					if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
					{
						uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
					}

					uvx_comm_bq_charge_fet(&comm_bq_h, 1);
				}
				else
				{
					uvx_comm_bq_charge_fet(&comm_bq_h, 0);
				}
			}

			if(!batt_data.cell_ball_h && !batt_data.cell_ball_l)
			{
				if((batt_data.payload.voltage_diff_pack > BATT_CELL_VOLTAGE_DIFF) && (batt_data.CHG_fet_stat))
				{
					if(bq_data_h.voltage_per_cell < bq_data_l.voltage_per_cell)
					{
						uvx_comm_bq_force_balance(&comm_bq_l, 1);
						batt_state.state_next = BATT_MODE_OFF_BALANCE_H;
					}
					else if (bq_data_h.voltage_per_cell > bq_data_l.voltage_per_cell)
					{
						uvx_comm_bq_force_balance(&comm_bq_h, 1);
						batt_state.state_next = BATT_MODE_OFF_BALANCE_L;
					}
				}
				else
				{
					if(comm_bq_h.Force_balance)
					{
						batt_state.state_next = BATT_MODE_OFF_BALANCE_H;
					}
					else if(comm_bq_l.Force_balance)
					{
						batt_state.state_next = BATT_MODE_OFF_BALANCE_L;
					}
					else
					{
						batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
					}					
				}	
			}
			else
			{
				if(comm_bq_h.Force_balance)
				{
					batt_state.state_next = BATT_MODE_OFF_BALANCE_H;
				}
				else if(comm_bq_l.Force_balance)
				{
					batt_state.state_next = BATT_MODE_OFF_BALANCE_L;
				}
				else
				{
					batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
				}		
			}	
		
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;			
		break;

		case BATT_MODE_OFF_BALANCE_L:
			uvx_comm_bq_force_balance(&comm_bq_l, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
		break;

		case BATT_MODE_OFF_BALANCE_H:
			uvx_comm_bq_force_balance(&comm_bq_h, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;			
			batt_reg_cnt = 0;
		break;				
		
		case BATT_MODE_READ_CHECK_PACK_V:
			batt_data.init = true;
			batt_state.state_current = BATT_MODE_READ_BQ_L;
			if((led_strip_state.state_current < LED_STRIP_MODE_BATTERY_LEVEL))
			{
				led_strip_state.state_next = LED_STRIP_MODE_BATTERY_LEVEL;				
			}

			if(enable) //manual learn new battery
			{
				uvx_batt_learn();	
			}			

			HAL_Delay(1);
		break;

		case BATT_MODE_WAIT_RESPONSE:
			switch(batt_state.state_next)
			{
				case BATT_MODE_READ_CHECK_PACK_V:
					if((comm_bq_l.RX_Ready == 1) || (comm_bq_h.RX_Ready == 1) || (batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE))
					{
						batt_data.cnt_no_response = 0;
						batt_state.state_current = batt_state.state_next;
						batt_reg_cnt = 0;
						HAL_Delay(1);
					}
					else
					{
						batt_state.state_current = batt_state.state_previous;
						batt_data.cnt_no_response++;
					}
				break;

				case BATT_MODE_INIT_BALANCE_H:
				case BATT_MODE_OFF_BALANCE_L:
				case BATT_MODE_READ_ONCE_BQ_L:
				case BATT_MODE_READ_BQ_L:
					if((comm_bq_l.RX_Ready == 1)|| (batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE))
					{
						if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
						{
							bq_data_l.No_response = true;
							batt_reg_cnt = 0;
							comm_bq_l.RX_Ready = 1; // Force ready to avoid blocking
						}						
						else
						{
							bq_data_l.No_response = false;
						}

						batt_data.cnt_no_response = 0;						
						batt_state.state_current = batt_state.state_next;
						
						if(bq_l_register_list_read[batt_reg_cnt].reg_addr == CBSTATUS)
						{
							for_test = 0;
						}

						batt_reg_cnt++;

						
						HAL_Delay(1);
					}
					else
					{
						batt_state.state_current = batt_state.state_previous;
						batt_data.cnt_no_response++;
					}
				break;
				
				case BATT_MODE_OFF_BALANCE_H:
				case BATT_MODE_READ_ONCE_BQ_H:
				case BATT_MODE_READ_BQ_H:
					if((comm_bq_h.RX_Ready == 1) || (batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE))
					{
						if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
						{
							bq_data_h.No_response = true;
							batt_reg_cnt = 0;
							comm_bq_h.RX_Ready = 1; // Force ready to avoid blocking
						}						
						else
						{
							bq_data_h.No_response = false;
						}				

						batt_data.cnt_no_response = 0;
						batt_state.state_current = batt_state.state_next;
						batt_reg_cnt++;
						HAL_Delay(1);
					}
					else
					{
						batt_state.state_current = batt_state.state_previous;
						batt_data.cnt_no_response++;
						HAL_Delay(1);
					}
				break;
			}	
		break;

		case BATT_MODE_STOP:
			//batt_data.init = false;
			if((batt_data.tc) && (batt_data.adc_pack_v_stable_high))
			{
				batt_state.state_current = BATT_MODE_READ_BQ_L;
			}
			HAL_Delay(10);
		break;		
	}

	uvx_batt_read_pack_v();


}

void UVX_APP_PWR_FET(uint8_t state)
{
#ifdef APP_NO_BATTERY_MODE	
	state = 1; // Force power on if no battery mode is enabled
#endif

	if(state)
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_PM_INH_CHG, GPIO_PIN_RESET);
		drone_status.pwr_fet = true;		
	}
	else
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_PM_INH_CHG, GPIO_PIN_SET);
		drone_status.pwr_fet = false;
	}

}

void UVX_APP_Shutdown_JMB(void)
{	
	if(drone_status.esc_arm || drone_status.esc_psys_arm)
	{
		return; // Do not shutdown if ESC is armed
	}

	timer_app_comm_jmb.Timeout = APP_TIMEOUT_JMB_MAX_TURN_OFF; // Reset timeout for power on
	timer_app_comm_jmb.Enable = true; 

	uvx_comm_m2jmb_send(CMD_TURN_OFF, NULL, 0); // Send shutdown command to JMB	

	while(uvx_gpio_read_pin(GPIO_INPUT_EXTI1_JETSON)) // Wait until the JMB is powered off
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_SET); // Turn off red LED to indicate waiting
		uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);
		uvx_led_strip_effect_solid_color(&ws2812_strip, 0, 0, 0);
		HAL_Delay(210);
		uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_RESET); // Turn on red LED to indicate waiting
		uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
		uvx_led_strip_effect_solid_color(&ws2812_strip, 120, 255, 255);
		HAL_Delay(90);

		if(timer_app_comm_jmb.Timeout == 0) // If timeout occurs
		{
			break; // Exit the loop to avoid infinite waiting
		}
	}

	SHUTDOWN_JETSON;	
	uvx_gpio_set_pin(GPIO_OUTPUT_DRONE_START_FET_EN, GPIO_PIN_RESET); // power off FC
	uvx_gpio_set_pin(GPIO_OUTPUT_ESC_EN, GPIO_PIN_RESET); // power off FC
	uvx_gpio_set_pin(GPIO_OUTPUT_5V_EN, GPIO_PIN_RESET); // power off FC
	uvx_gpio_set_pin(GPIO_OUTPUT_CUBE_EN, GPIO_PIN_RESET); // power off FC
	drone_status.esc_comm = false;
	drone_status.pwr_fc = false;
	comm_m2jmb.Heartbeat = 0; 
	timer_app_comm_jmb.Timeout = 0;
	drone_state.state_current = DRONE_IDLE;
	comm_m2jmb_state.state_current = M2JMB_MODE_IDLE; // Reset state machine
	uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_SET); // Turn off red LED	
	uvx_gpio_set_pin(GPIO_OUTPUT_GREEN_LED, GPIO_PIN_SET);
	uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
}

void UVX_APP_Comm_m2jmb(void)
{
	switch (comm_m2jmb_state.state_current)
	{
		case M2JMB_MODE_IDLE:
			comm_m2jmb_state.state_current = M2JMB_MODE_TURN_ON;							
		break;

		case M2JMB_MODE_TURN_ON:
			if(timer_app_comm_jmb.Timeout == 0) // If timeout occurs
			{
				PWR_JETSON; // Power on JMB peripheral
				uvx_gpio_set_pin(GPIO_OUTPUT_DRONE_START_FET_EN, GPIO_PIN_SET);

				#ifdef APP_JETSON_PWR_FC
				comm_m2jmb_state.state_next = M2JMB_MODE_TURN_ON; // Set next state to wait for response
				comm_m2jmb_state.state_current = M2JMB_MODE_WAIT_RESPONSE; // Wait for response
				#else
				comm_m2jmb_state.state_next = M2JMB_MODE_TURN_ON;
				comm_m2jmb_state.state_current = M2JMB_MODE_TURN_ON; 
				uvx_gpio_set_pin(GPIO_OUTPUT_DRONE_START_FET_EN, GPIO_PIN_SET);
				uvx_gpio_set_pin(GPIO_OUTPUT_ESC_EN, GPIO_PIN_SET);
				uvx_gpio_set_pin(GPIO_OUTPUT_5V_EN, GPIO_PIN_SET);
				uvx_gpio_set_pin(GPIO_OUTPUT_CUBE_EN, GPIO_PIN_SET);
				drone_state.state_current = DRONE_JETSON_POWERED_ON;
				comm_m2jmb.Heartbeat = 1; // Reset heartbeat flag
				#endif

				timer_app_comm_jmb.Timeout = comm_m2jmb.timeout_power_on; // Reset timeout for power on
				timer_app_comm_jmb.Enable = true; 
				HAL_UART_Receive_IT(&uart_2.hal_uart.huart, &uart_2.byte_rx, 1); // Start receiving data
				uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_RESET);
				uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
				led_strip_state.state_next = LED_STRIP_MODE_WAITING;
			}
		break;

		case M2JMB_MODE_TURN_OFF:
			if(timer_app_comm_jmb.Timeout == 0) // If timeout occurs
			{
				// #ifdef APP_JETSON_PWR_FC
				// SHUTDOWN_JETSON; // Power off JMB peripheral
				// comm_m2jmb.Heartbeat = 0; // Reset heartbeat flag
				// #else
				// comm_m2jmb.Heartbeat = 1; // Reset heartbeat flag
				// #endif
				
				// comm_m2jmb_state.state_current = M2JMB_MODE_TURN_ON; // Wait for response
				// timer_app_comm_jmb.Timeout = comm_m2jmb.timeout_power_off; // Reset timeout for power off
				// timer_app_comm_jmb.Enable = true; 
				// uvx_gpio_set_pin(GPIO_OUTPUT_RED_LED, GPIO_PIN_SET);
				// led_strip_state.state_next = LED_STRIP_MODE_STOP;
			}
		break;

		case M2JMB_MODE_SEND_FC_ON_ACK:
			if(uvx_comm_m2jmb_send(CMD_PWR_ON_FC_ACK, NULL, NULL) == UVX_M2JMB_OK)
			{
				drone_state.state_current = DRONE_JETSON_POWERED_ON;
				drone_status.pwr_fc = true;
				comm_m2jmb_state.state_current = M2JMB_MODE_WAIT_RESPONSE; // Wait for response
				uvx_gpio_set_pin(GPIO_OUTPUT_GREEN_LED, GPIO_PIN_RESET);		
				uvx_gpio_set_pin(GPIO_OUTPUT_DRONE_START_FET_EN, GPIO_PIN_SET);				
				uvx_gpio_set_pin(GPIO_OUTPUT_ESC_EN, GPIO_PIN_SET);				
				uvx_gpio_set_pin(GPIO_OUTPUT_5V_EN, GPIO_PIN_SET);		
				uvx_gpio_set_pin(GPIO_OUTPUT_CUBE_EN, GPIO_PIN_SET);		
			}
						
			led_strip_state.state_next = LED_STRIP_MODE_COMM_FC;
		break;

		case M2JMB_MODE_SEND_FC_OFF_ACK:
			if(uvx_comm_m2jmb_send(CMD_PWR_OFF_FC_ACK, NULL, NULL) == UVX_M2JMB_OK)
			{
				drone_state.state_current = DRONE_JETSON_POWERED_OFF;
				drone_status.pwr_fc = false;
				drone_status.esc_comm = false;
				comm_m2jmb_state.state_current = M2JMB_MODE_WAIT_RESPONSE; // Wait for response
				uvx_gpio_set_pin(GPIO_OUTPUT_GREEN_LED, GPIO_PIN_SET);		
				//uvx_gpio_set_pin(GPIO_OUTPUT_DRONE_START_FET_EN, GPIO_PIN_RESET);					
				uvx_gpio_set_pin(GPIO_OUTPUT_ESC_EN, GPIO_PIN_RESET);					
				uvx_gpio_set_pin(GPIO_OUTPUT_5V_EN, GPIO_PIN_RESET);	
				uvx_gpio_set_pin(GPIO_OUTPUT_CUBE_EN, GPIO_PIN_RESET); // power off FC
			}
						
			led_strip_state.state_next = LED_STRIP_MODE_WAITING;
		break;

		case M2JMB_MODE_WAIT_RESPONSE:
			// Check if the RX buffer is ready
			if (comm_m2jmb.RX_Ready == 1)
			{				
				if(buff_rx_m2jmb[M2JMB_BYTE_CMD] == CMD_HEARTBEAT)
				{
					comm_m2jmb.Heartbeat = 1; // Set heartbeat flag
					timer_app_comm_jmb.Timeout = comm_m2jmb.timeout_heartbeat; // Reset heartbeat timeout
					HAL_UART_Receive_IT(&uart_2.hal_uart.huart, &uart_2.byte_rx, 1); // Start receiving data
					uvx_gpio_toggle_pin(GPIO_OUTPUT_RED_LED);
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);

					if(comm_m2m_state.state_current == M2M_MODE_IDLE)
					{
						led_strip_state.state_next = LED_STRIP_MODE_HEARTBEAT;
					}
					
					if(!drone_status.pwr_fc)
					{
						// memcpy(esc_data_tx, &buff_rx_m2m[M2M_BYTE_DATA], M2M_SENT_DATA_PACKET_SIZE); // Copy ESC data from RX buffer						

						// drone_status.esc_init = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_INIT) != 0U); // Update drone init status from ESC data
						// drone_status.esc_arm = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_ARM_BIT) != 0U); // Update drone arm status from ESC data
						// drone_status.esc_land_complete = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_LAND_COMPLETE) != 0U); // Update drone land complete status from ESC data
						// drone_status.esc_flying = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_FLYING) != 0U); // Update drone flying status from ESC data
						// drone_status.esc_psys_arm = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_PSYS_ARM) != 0U); // Update powered system arm status from ESC data

						comm_m2m.size_payload = M2M_SENT_DATA_PACKET_SIZE; // Set payload size for ESC data
						memset(esc_data_tx, 0, M2M_SENT_DATA_PACKET_SIZE); // Clear ESC data buffer

						if(batt_data.size_payload > 0)
						{
							memcpy(&esc_data_tx[comm_m2m.size_payload], &batt_data.payload.SOH, batt_data.size_payload); // Copy BATT data to ESC data buffer
							comm_m2m.size_payload += batt_data.size_payload; // Update payload size to include BATT data							
						}

						if(comm_m2jmb.Heartbeat)
						{
							comm_m2jmb.size_payload = comm_m2m.size_payload + jmb_payload_size; // Set payload size for JMB data
							//comm_m2jmb.size_payload = comm_m2m.size_payload; // Set payload size for JMB data
							uvx_comm_m2jmb_send(CMD_SEND_DATA_ESC, esc_data_tx, comm_m2jmb.size_payload);
							if(led_strip_state.state_current == LED_STRIP_MODE_COMM_FC)
							{
								led_strip_state.state_next = LED_STRIP_MODE_STOP;
							}
							else
							{
								if(batt_data.adc_pack_v_stable_high)
								{
									led_strip_state.state_next = LED_STRIP_MODE_BATTERY_LEVEL;
								}
								else
								{
									led_strip_state.state_next = LED_STRIP_MODE_COMM_FC;
								}								
							}
							
						}
					}
					
				}
				else if(buff_rx_m2jmb[M2JMB_BYTE_CMD] == CMD_PWR_ON_FC)
				{							
					comm_m2jmb_state.state_current = M2JMB_MODE_SEND_FC_ON_ACK; // Set next state to send ACK
				}
				else if(buff_rx_m2jmb[M2JMB_BYTE_CMD] == CMD_PWR_OFF_FC)
				{					
					comm_m2jmb_state.state_current = M2JMB_MODE_SEND_FC_OFF_ACK; // Set next state to send ACK
				}
				else
				{
					comm_m2jmb.Error = 1; // Set error flag if the command is not recognized
				}

				comm_m2jmb.RX_Ready = 0; // Reset RX ready flag
				comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count
			}
			else
			{
				if(timer_app_comm_jmb.Timeout == 0) // If timeout occurs
				{
					#ifdef APP_JETSON_PWR_FC					
					comm_m2jmb.Heartbeat = 0; // Reset heartbeat flag
					#else
					comm_m2jmb.Heartbeat = 1; // Set heartbeat flag
					#endif

					comm_m2jmb_state.state_current = comm_m2jmb_state.state_next;
					led_strip_state.state_next = LED_STRIP_MODE_STOP;
				}
				else
				{
					HAL_UART_Receive_IT(&uart_2.hal_uart.huart, &uart_2.byte_rx, 1); // Start receiving data
				}
			}
		break;
	}

}

void UVX_APP_Comm_m2m(void)
{
	switch (comm_m2m_state.state_current)
	{
		case M2M_MODE_IDLE:
			timer_app.Timeout = 1000;
			timer_app.Enable = true;
			comm_m2m_state.state_current = M2M_MODE_READ_DATA_ESC;

			HAL_UART_Receive_IT(&uart_1.hal_uart.huart, &uart_1.byte_rx, 1);
			__HAL_UART_ENABLE_IT(&uart_1.hal_uart.huart, UART_IT_RXNE); // Enable RXNE interrupt for USART1				
			HAL_UART_Receive_IT(&uart_2.hal_uart.huart, &uart_2.byte_rx, 1); // Start receiving data
			__HAL_UART_ENABLE_IT(&uart_2.hal_uart.huart, UART_IT_RXNE); // Enable RXNE interrupt for USART2							
			break;

		case M2M_MODE_READ_DATA_ESC:
			if(timer_app.Timeout == 0)
			{				
				uvx_comm_m2m_read(CMD_READ_DATA_ESC); // Read data from ESC
				HAL_UART_Receive_IT(&uart_1.hal_uart.huart, &uart_1.byte_rx, 1);
				
				comm_m2m_state.state_previous = M2M_MODE_READ_DATA_ESC; // Set next state to read data from JMB
				comm_m2m_state.state_current = M2M_MODE_WAIT_RESPONSE; // Wait for response
				comm_m2m_state.state_next = M2M_MODE_READ_DATA_ESC; // Set next state to read data from JMB
				timer_app.Timeout = UVX_APP_TIMEOUT_M2M;
				timer_app.Enable = true;				
			}
			break;

		case M2M_MODE_READ_DATA_JMB:
			if(timer_app.Timeout == 0)
			{
				uvx_comm_m2m_read(CMD_READ_DATA_JMB); // Read data from JMB
				HAL_UART_Receive_IT(&uart_1.hal_uart.huart, &uart_1.byte_rx, 1);
				comm_m2m_state.state_previous = M2M_MODE_READ_DATA_JMB; // Set next state to read data from JMB
				comm_m2m_state.state_current = M2M_MODE_WAIT_RESPONSE; // Wait for response
				comm_m2m_state.state_next = M2M_MODE_READ_DATA_ESC; // Set next state to read data from ESC	
				timer_app.Timeout = UVX_APP_TIMEOUT_M2M; // Reset timeout for M2M communication
				timer_app.Enable = true;				
			}
			break;

		case M2M_MODE_WAIT_RESPONSE:
			// Check if the RX buffer is ready
			if (comm_m2m.RX_Ready == 1)
			{						
				comm_m2m_state.state_current = comm_m2m_state.state_next; // Set current state to read data from ESC
				if(buff_rx_m2m[M2M_BYTE_CTRL] == CTRL_SEND)
				{
					comm_m2m.Error = 0; // Reset error flag
					if(buff_rx_m2m[M2M_BYTE_CMD] == CMD_READ_DATA_ESC) // If the command is to read data from ESC
					{						
						drone_status.esc_comm = true;

						// Process ESC data here
						memcpy(esc_data_tx, &buff_rx_m2m[M2M_BYTE_DATA], comm_m2m.size_payload); // Copy ESC data from RX buffer						

						drone_status.esc_init = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_INIT) != 0U); // Update drone init status from ESC data
						drone_status.esc_arm = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_ARM_BIT) != 0U); // Update drone arm status from ESC data
						drone_status.esc_land_complete = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_LAND_COMPLETE) != 0U); // Update drone land complete status from ESC data
						drone_status.esc_flying = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_FLYING) != 0U); // Update drone flying status from ESC data
						drone_status.esc_psys_arm = ((esc_data_tx[M2M_BYTE_DATA_FLAGS] & M2M_DATA_FLAGS_PSYS_ARM) != 0U); // Update powered system arm status from ESC data

						if(batt_data.size_payload > 0)
						{
							memcpy(&esc_data_tx[comm_m2m.size_payload], &batt_data.payload.SOH, batt_data.size_payload); // Copy BATT data to ESC data buffer
							comm_m2m.size_payload += batt_data.size_payload; // Update payload size to include BATT data							
						}

						if(comm_m2jmb.Heartbeat)
						{
							comm_m2jmb.size_payload = comm_m2m.size_payload + jmb_payload_size; // Set payload size for JMB data
							//comm_m2jmb.size_payload = comm_m2m.size_payload; // Set payload size for JMB data
							uvx_comm_m2jmb_send(CMD_SEND_DATA_ESC, esc_data_tx, comm_m2jmb.size_payload);
							uvx_gpio_toggle_pin(GPIO_OUTPUT_GREEN_LED);
							uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
							if(led_strip_state.state_current == LED_STRIP_MODE_COMM_FC)
							{
								led_strip_state.state_next = LED_STRIP_MODE_STOP;
							}
							else
							{
								if(batt_data.adc_pack_v_stable_high)
								{
									led_strip_state.state_next = LED_STRIP_MODE_BATTERY_LEVEL;
								}
								else
								{
									led_strip_state.state_next = LED_STRIP_MODE_COMM_FC;
								}								
							}
							
						}

						comm_m2m_state.state_current = comm_m2m_state.state_next;
					}
					else if(buff_rx_m2m[M2M_BYTE_CMD] == CMD_READ_DATA_JMB) // If the command is to read data from JMB
					{
						// Process JMB data here
						memcpy(jmb_data_tx, &buff_rx_m2m[M2M_BYTE_DATA], comm_m2m.size_payload); // Copy JMB data from RX buffer

						jmb_payload_size= comm_m2m.size_payload;

						comm_m2m_state.state_current = comm_m2m_state.state_next;
					}
				}
				else if(buff_rx_m2m[1] == CTRL_SEND_ERROR)
				{
					// Handle error here
					comm_m2m_state.state_current = comm_m2m_state.state_previous; // Reset to previous state
				}
				else
				{
					comm_m2m.Error = 1; // Set error flag if the command is not recognized
				}

				comm_m2m.RX_Ready = 0; // Reset RX ready flag
				comm_m2m.buff_rx_cnt = 0; // Reset RX buffer count
			}
			else
			{
				if(timer_app.Timeout == 0) // If timeout occurs
				{
					comm_m2m_state.state_current = comm_m2m_state.state_previous;
				}
			}
		break;
	
	default:
		break;
	}	
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
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 1;
   RCC_OscInitStruct.PLL.PLLN = 10;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 
   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
   {
	 Error_Handler();
   }
   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
							   |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
							   |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_ADC;
   PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
   PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
   PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
   PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
   PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
   PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
   {
	 Error_Handler();
   }
   /** Configure the main internal regulator output voltage
   */
   if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
   {
	 Error_Handler();
   }
 }

 static void UVX_APP_Init(void)
{
	HAL_Delay(100); // Delay to allow peripherals to stabilize after power on
	uvx_gpio_init_all(); // Initialize all GPIOs used by the UVX application

	timer_app = UVX_SETUP_TIMER_GENERAL;
	if(uvx_timer_add(&timer_app) != UVX_TIMER_OK) // Add the button timer
	{
		Error_Handler();
	}

	timer_app_comm_jmb = UVX_SETUP_TIMER_APP_COMM_JMB; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_comm_jmb) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}

	timer_app_batt_pwr_high = UVX_SETUP_timer_app_batt_pwr_high; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_batt_pwr_high) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}

	timer_app_batt_pwr_low = UVX_SETUP_timer_app_batt_pwr_low; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_batt_pwr_low) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}

	timer_app_batt_low_voltage = UVX_SETUP_timer_app_batt_low_voltage; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_batt_low_voltage) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}	

	timer_app_drone = UVX_SETUP_TIMER_APP_DRONE; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_drone) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}	

	timer_app_led = UVX_SETUP_TIMER_APP_LED; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_led) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}	

	timer_app_land = UVX_SETUP_TIMER_APP_LAND; // Initialize the JMB communication timer
	if(uvx_timer_add(&timer_app_land) != UVX_TIMER_OK) // Add the JMB communication timer
	{
		Error_Handler();
	}
	timer_app_land.Timeout = APP_TIMEOUT_LAND; // Set the timeout for landing
	timer_app_land.Enable = enable;

	comm_m2m_state.state_current = M2M_MODE_IDLE; // Set the initial state of the M2M communication
	uart_1 = UVX_APP_SETUP_UART_1; // Initialize the UART HAL structure
	uvx_comm_m2m_init(&uart_1); // Initialize the M2M communication

	i2c_bq = UVX_APP_SETUP_I2C_BQ; // Initialize the I2C HAL structure for BQ communication	
	uvx_comm_bq_init(&comm_bq_l, &i2c_bq, BQ_L_I2C_ADDRESS, bq_l_register_list_read); // Initialize the BQ communication

	comm_bq_h.addr_i2c = BQ_H_I2C_ADDRESS; // Set the I2C address for BQ
	uvx_comm_bq_init(&comm_bq_h, &i2c_bq, BQ_H_I2C_ADDRESS, bq_h_register_list_read); // Initialize the BQ communication

	uart_2 = UVX_APP_SETUP_UART_2; // Initialize the UART HAL structure for M2JMB communication
	uvx_comm_m2jmb_init(&uart_2); // Initialize the M2JMB communication

	spi_1  = UVX_SETUP_SPI_1;     // Initialize the SPI HAL structure for led strip operation
	ws2812_init(&ws2812_strip, &spi_1);  // Initialize with 1 LED for simple test

	//batt_data.batt_data_size = &batt_data.start_batt_data - &batt_data.end_of_batt_data;
	batt_data.size_payload = sizeof(batt_data.payload); // Set the size of the battery data payload

	drone_status.esc_comm = false;
	drone_status.pwr_fc = false;
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909EEA;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 40000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

static void MX_TIM4_Init(void)
{	
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
		
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = TIMERS_PRESCALER_FOR_RUN_MODE;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = BATTERIES_COMMUNICATION_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* USER CODE BEGIN TIM4_Init 2 */
	// Clear the update interrupt flag to suppress the interrupt upon timer start
	htim4.Instance->SR &= ~TIM_SR_UIF;
	/* USER CODE END TIM4_Init 2 */
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_TIM5_Init(void)
 {
 
   /* USER CODE BEGIN TIM5_Init 0 */
 
   /* USER CODE END TIM5_Init 0 */
 
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
 
   /* USER CODE BEGIN TIM5_Init 1 */
 
   /* USER CODE END TIM5_Init 1 */
   htim5.Instance = TIM5;
   htim5.Init.Prescaler = 40000-1;
   htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim5.Init.Period = 100;
   htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
   {
	 Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
   {
	 Error_Handler();
   }
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
   {
	 Error_Handler();
   }
   /* USER CODE BEGIN TIM5_Init 2 */
 
   /* USER CODE END TIM5_Init 2 */
 
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

void Change_To_Rising_Edge_Interrupt(void)
{
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT8;
	EXTI->RTSR1 |= EXTI_RTSR1_RT8;
	g_Rising_Edge = 1;
	g_Falling_Edge = 0;
}

void Change_To_Falling_Edge_Interrupt(void)
{
	EXTI->RTSR1 &= ~EXTI_RTSR1_RT8;
	EXTI->FTSR1 |= EXTI_FTSR1_FT8;
	g_Rising_Edge = 0;
	g_Falling_Edge = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);
	if(drone_state.state_current == DRONE_SLEEP)
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
	}

	uvx_timer_callback(htim);

	if(drone_state.state_current == DRONE_SLEEP)
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);
	}

	/* NOTE: This function should not be modified, when the callback is needed,
			 the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	*/	
	if ( htim == &htim4 )
	{		
		if ( g_Enter_Sleep || g_Stop_Drone )
		{
			HAL_TIM_Base_Stop_IT(&htim4);
//			batt->Battery_Com_Started = 0;
			return;
		}
		
	}
	
	if ( htim == &htim5 )
	{
		g_Pressed_Once = 1;
		// if (BUTTON_WAS_PRESSED)
		// {
		// 	state = TRUE;
		// 	HAL_TIM_Base_Stop_IT(&htim5);
		// 	if ( __No_Button_Was_Pressed )
		// 	{
		// 		if ( g_Drone_Started ) 
		// 		{
		// 			// if ( batt->Num_Batt_Packs_Connected > 0 )
		// 			// 	batt->New_LED_Mode = NOTIFY_LED_MODE_MSR_VOLTAGE_LEVEL;
		// 		}
		// 		else if ( g_Drone_Not_Started )
		// 		{
		// 			// battery.New_LED_Mode = NOTIFY_LED_MODE_MSR_VOLTAGE_LEVEL; // <--
		// 			// HAL_TIM_Base_Start_IT(&htim4); // <--
		// 		}
		// 		g_Pressed_Once = 1;
		// 		Start_Button_Once_Pressed_Timer();
		// 		__Change_Interrupt_To_Rising_Edge; 
		// 	}
		// 	else if ( __Button_Was_Released_Once )
		// 	{
		// 		if ( g_Drone_Started ) 
		// 		{
		// 			// if ( batt->LED_Mode == NOTIFY_LED_MODE_SHOW_VOLTAGE_LEVEL )
		// 			// 	batt->New_LED_Mode = NOTIFY_LED_MODE_ALL_OFF;
		// 		}
        		
		// 		g_Button_Pressed_Once_Correct = 0;
		// 		g_Pressed_Twice = 1;
		// 		Start_Button_Pressed_Twice_Timer();
		// 		__Change_Interrupt_To_Rising_Edge; 
		// 	}	
		// }
		// else if ( BUTTON_WAS_RELEASED )
		// {
		// 	state = TRUE;
		// 	HAL_TIM_Base_Stop_IT(&htim5);
			
		// 	if ( __Button_Was_Pressed_Once )
		// 	{
		// 		if ( g_Drone_Started ) 
		// 		{

		// 		}
		// 		else if ( g_Drone_Not_Started )
		// 		{

		// 		}
		// 		g_Released_Once = 1;
		// 		if ( g_Button_Pressed_Once_Correct )
		// 		{
		// 			__Change_Interrupt_To_Falling_Edge;	
		// 		}
		// 	}
		// 	else if ( __Button_Was_Pressed_Twice )
		// 	{
		// 		if ( g_Drone_Has_Started ) 
		// 		{
		// 			g_Released_Twice = 1;
		// 		}
		// 		else if ( g_Drone_Has_ShutDown ) 
		// 		{
		// 			g_Released_Twice = 1;
		// 		}
		// 		else if ( g_Drone_Started ) 
		// 		{
		// 			g_Released_Twice = 1;
		// 		}
		// 		else if ( g_Drone_Not_Started ) 
		// 		{
		// 			g_Released_Twice = 1;
		// 		}
				
		// 		__Change_Interrupt_To_Falling_Edge;
		// 	}	
		// }
	}
	
	if ( htim == &htim2 )
	{
		if (g_Drone_Not_Started && g_Timer_Once_Started && __Button_Was_Pressed_Once)
		{
			Stop_Button_Pressed_Once_Timer();
			g_Button_Pressed_Once_Correct = 1;
			Set_Timer_For_No_Pressed_Button();
			// TODO: --> Move code marked with "<--" here to fix the slight breath in just before powering off the LEDs (test it)
		}
		else if (g_Drone_Started && g_Timer_Once_Started && __Button_Was_Pressed_Once)
		{
			Stop_Button_Pressed_Once_Timer();
			g_Button_Pressed_Once_Correct = 1;
			Set_Timer_For_No_Pressed_Button();
		}
		else if (g_Time_For_No_Pressed_Buttton)
		{
			if (g_Drone_Started)
			{
				HAL_TIM_Base_Stop_IT(&htim2);
				Dron_Button_Reset_State();
				__Change_Interrupt_To_Falling_Edge;
			}
			else if (g_Drone_Not_Started)
			{
				HAL_TIM_Base_Stop_IT(&htim2);
				Dron_Button_Reset_State();
				g_Enter_Sleep = 1;
				__Change_Interrupt_To_Falling_Edge;

			}
			g_Button_Pressed_Once_Correct = 0;
			g_Time_For_No_Pressed_Buttton = 0;
		}
		else if (g_OK_LED)
		{
			g_Toggle_Count++;
			if(g_Toggle_Count > 5)
			{
				g_Toggle_Count = 0;
				Reset_Button_Timer();
				g_OK_LED = 0;
			}
		}
		else if (g_Error_LED)
		{
			g_Toggle_Count++;
			if(g_Toggle_Count > 10)
			{
				g_Toggle_Count=0;
				g_Error_LED = 0;
				Reset_Button_Timer();
				g_Enter_Sleep = 1;
			} 
		}
		else if (g_Drone_Not_Started && g_Timer_Twice_Started &&  __Button_Was_Pressed_Twice) // && batt->Num_Batt_Packs_Connected == 4
		{
			 //g_Timer_Twice_Started = 0;
			 Stop_Button_Pressed_Twice_Timer();
			 g_Start_Drone = 1;
			//  Dron_Button_Reset_State();
			// __Change_Interrupt_To_Falling_Edge;
			 
			
			 // Check state of pin GPIO_EXTI9_WIFi_AP_CLIENT_Pin.
			 // If HIGH enable FRC pin of the Jetson Nano, set RED color for the waving LED indication and disable transition to NOTIFY_LED_MODE_FROM_ESC
			
			 if ( HAL_GPIO_ReadPin(GPIO_EXTI9_WIFi_AP_CLIENT_GPIO_Port, GPIO_EXTI9_WIFi_AP_CLIENT_Pin) == GPIO_PIN_SET )
			 {
				// batt->jnano.force_rec_mode = 1;
				 HAL_GPIO_WritePin(GPIO_OUTPUT_NANO_FORCE_REC_GPIO_Port, GPIO_OUTPUT_NANO_FORCE_REC_Pin, GPIO_PIN_SET);
			 }
			 else
			 {
				 //batt->jnano.force_rec_mode = 0;
				 HAL_GPIO_WritePin(GPIO_OUTPUT_NANO_FORCE_REC_GPIO_Port, GPIO_OUTPUT_NANO_FORCE_REC_Pin, GPIO_PIN_RESET);
			 }
			
			 //batt->New_LED_Mode = NOTIFY_LED_MODE_WAVE;
			 //HAL_TIM_Base_Start_IT(&htim7);
		}
		else if (g_Drone_Started && g_Timer_Twice_Started &&  __Button_Was_Pressed_Twice)
		{	
			g_Stop_Drone = 1;
			//HAL_TIM_Base_Stop_IT(&htim4);
			//HAL_TIM_Base_Stop_IT(&htim7);
			Stop_Button_Pressed_Twice_Timer();
		}
	}

	if ( htim == &uvx_timer_hal.htim )
	{
		if(drone_state.state_current == DRONE_SLEEP)
		{
			timer_app_batt_pwr_high.Timeout = 0;
			exit_Sleep();		
		}		
	}
}

void Start_Button_Once_Pressed_Timer(void)
{
	if(g_Time_For_No_Pressed_Buttton)
	{
		g_Time_For_No_Pressed_Buttton = 0;
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_FIRST_PRESS);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	HAL_TIM_Base_Start_IT(&htim2);
	g_Timer_Once_Started = 1;
}

void Start_Button_Pressed_Twice_Timer(void)
{
	if(g_Time_For_No_Pressed_Buttton)
	{
		g_Time_For_No_Pressed_Buttton = 0;
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_SECOND_PRESS);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	HAL_TIM_Base_Start_IT(&htim2);
	g_Timer_Twice_Started = 1;
}

void Stop_Button_Pressed_Twice_Timer(void)
{
	g_Timer_Twice_Started = 0;
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_SECOND_PRESS);
	__HAL_TIM_SET_COUNTER(&htim2,0);
}
void Stop_Button_Pressed_Once_Timer(void)
{
	g_Timer_Once_Started = 0;
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_FIRST_PRESS);
	__HAL_TIM_SET_COUNTER(&htim2,0);
}
void Dron_Button_Reset_State(void)
{
	g_Pressed_Once = 0;
	g_Released_Once = 0;
	g_Pressed_Twice = 0;
	g_Released_Twice = 0;
}

void Dron_Button_Once_Pressed_State(void)
{
	g_Pressed_Once = 1;
	g_Released_Once = 1;
	g_Pressed_Twice = 1;
	g_Released_Twice = 0;
}
void Error_LED(void)
{
	if(g_Time_For_No_Pressed_Buttton)
	{
		g_Time_For_No_Pressed_Buttton = 0;
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_ERROR_LED_TOGGLE);
	TIM2->CR1 |= TIM_CR1_ARPE;
	HAL_TIM_Base_Start_IT(&htim2);
	g_Error_LED = 1;
}

void OK_LED(void)
{
	if(g_Time_For_No_Pressed_Buttton)
	{
		g_Time_For_No_Pressed_Buttton = 0;
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_OK_LED_TOGGLE);
	TIM2->CR1 |= TIM_CR1_ARPE;
	HAL_TIM_Base_Start_IT(&htim2);
	g_OK_LED =1;
}

void Set_Timer_For_No_Pressed_Button(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SetAutoreload(&htim2,TIME_BEFOR_GO_TO_SLEEP);
	TIM2->CR1 &= ~TIM_CR1_ARPE;
	HAL_TIM_Base_Start_IT(&htim2);
	g_Time_For_No_Pressed_Buttton = 1;
}

void Set_Timers_After_Sleep(void)
{
	__HAL_TIM_PRESCALER(&htim2,TIMERS_PRESCALER_FOR_RUN_MODE);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	
	__HAL_TIM_PRESCALER(&htim5,TIMERS_PRESCALER_FOR_RUN_MODE);
	__HAL_TIM_SET_COUNTER(&htim5,0);
	
	__HAL_TIM_PRESCALER(&htim4,TIMERS_PRESCALER_FOR_RUN_MODE);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	// __HAL_TIM_PRESCALER(&htim7,TIM7_PRESCALER_FOR_RUN_MODE);
	// __HAL_TIM_SET_COUNTER(&htim7,0);
}

void enter_LPSleep( void )
{
	if (!g_Sleep)//G? pin config before enter sleep
	{
		uvx_gpio_set_pin(GPIO_OUT_LED_STRIP_ENABLE, GPIO_PIN_RESET);
		__HAL_UART_DISABLE_IT(&uart_1.hal_uart.huart, UART_IT_RXNE); // Enable RXNE interrupt for USART1
		__HAL_UART_DISABLE_IT(&uart_2.hal_uart.huart, UART_IT_RXNE); // Enable RXNE interrupt for USART2
		HAL_ADC_Stop_DMA(&hadc1);
		Set_Timers_For_Sleep();
		g_Enter_Sleep = 0;
		g_Sleep = 1;
		HAL_GPIO_WritePin(GPIO_OUTPUT_NANO_FORCE_REC_GPIO_Port, GPIO_OUTPUT_NANO_FORCE_REC_Pin, GPIO_PIN_RESET);
		__Change_Interrupt_To_Falling_Edge;
		
		ADC1->CR |= ADC_CR_DEEPPWD;
		__HAL_RCC_DMA1_CLK_DISABLE();
		__HAL_RCC_USART2_CLK_DISABLE();
		__HAL_RCC_USART3_CLK_DISABLE();
		__HAL_RCC_UART4_CLK_DISABLE();
		__HAL_RCC_UART5_CLK_DISABLE();
		__HAL_RCC_TIM3_CLK_DISABLE();
		__HAL_RCC_ADC_CLK_DISABLE();
		
		FLASH->ACR |= FLASH_ACR_SLEEP_PD;
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
		HAL_StatusTypeDef HAL_FLASHEx_EnableRunPowerDown();
		
		Config_SysClk_MSI_131();
		sys_CLK = HAL_RCC_GetSysClockFreq() ;
		HAL_SuspendTick();
		HAL_PWREx_EnableLowPowerRunMode();

		while (g_Sleep)
		{
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,PWR_SLEEPENTRY_WFI);
		}
	}
}

void exit_Sleep(void)
{
	if (g_Sleep)//G? pin config at exit sleep
	{
		HAL_PWREx_DisableLowPowerRunMode();
		Config_SysClk_HSE();
		HAL_RstTick();
		HAL_ResumeTick();
		HAL_InitTick(uwTickPrio);
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_USART3_CLK_ENABLE();
		__HAL_RCC_UART4_CLK_ENABLE();
		__HAL_RCC_UART5_CLK_ENABLE();
		__HAL_RCC_TIM3_CLK_ENABLE();
		__HAL_RCC_ADC_CLK_ENABLE();
		ADC1->CR &= ~ADC_CR_DEEPPWD;
		ADC1->CR |= ADC_CR_ADVREGEN;
		Set_Timers_After_Sleep();
		g_Sleep = 0;
		comm_m2m_state.state_current = M2M_MODE_IDLE;
		uvx_gpio_set_pin(GPIO_OUTPUT_JMB_PERIPHERIAL_EN, GPIO_PIN_RESET);
	}
}

void Config_SysClk_HSE(void)
{	
		
	//RCC->CFGR |=  RCC_CFGR_HPRE;
	RCC->CR |= RCC_CR_HSEON | RCC_CR_PLLON;
	while ( !( RCC->CR & RCC_CR_HSERDY && RCC->CR & RCC_CR_PLLRDY  ) ); // wait until HSE and PLL is ready
	
	/* Switch System Clock */
	// PLL oscillator used as system clock
	RCC->CFGR = ( RCC->CFGR & ~RCC_CFGR_SW ) | RCC_CFGR_SW_PLL;
	while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL ); // wait unit switched
	
	
	/* Disable other clocks (excluding LSE and LSI) */
	RCC->CR &= ~( RCC_CR_MSION);// | RCC_CR_HSION );
	
	SystemCoreClockUpdate();
}

void Config_SysClk_MSI_131(void)
{
	/* Enable an Configure Clock */
	RCC->CR &= ~RCC_CR_MSION;
	RCC->CR = ( RCC->CR & ~RCC_CR_MSIRANGE ) | RCC_CR_MSIRANGE_1;
	RCC->CR |= RCC_CR_MSIRGSEL;
	RCC->CR |= RCC_CR_MSION;
	while ( !( RCC->CR & RCC_CR_MSIRDY ) ); // wait until MSI is ready
	
	/* Switch System Clock */
	// MSI oscillator used as system clock
	RCC->CFGR = ( RCC->CFGR & ~RCC_CFGR_SW ) | RCC_CFGR_SW_MSI;
	while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_MSI ); // wait unit switched
	
	/* Disable other clocks (excluding LSE and LSI) */
	RCC->CR &= ~( RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_PLLON | RCC_CR_PLLSAI1ON );
	
	
	SystemCoreClockUpdate();
	HAL_InitTick(uwTickPrio);
}

void Set_Timers_For_Sleep(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_PRESCALER(&htim2,TIMERS_PRESCALER_FOR_SLEEP_MODE);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	
		HAL_TIM_Base_Stop_IT(&htim5);
	__HAL_TIM_PRESCALER(&htim5,TIMERS_PRESCALER_FOR_SLEEP_MODE);
	__HAL_TIM_SET_COUNTER(&htim5,0);
	
	HAL_TIM_Base_Stop_IT(&htim4);
	__HAL_TIM_PRESCALER(&htim4,TIMERS_PRESCALER_FOR_SLEEP_MODE);
	__HAL_TIM_SET_COUNTER(&htim4,0);	
}

void USART1_IRQHandler(void)
{
	uint32_t isrflags = READ_REG(uart_1.hal_uart.huart.Instance->ISR);

	HAL_UART_IRQHandler(&uart_1.hal_uart.huart);

    if (isrflags & USART_ISR_TC) 
	{
        __HAL_UART_CLEAR_FLAG(&uart_1.hal_uart.huart, UART_CLEAR_TCF);
    }

	if (isrflags & USART_ISR_WUF) 
	{
        __HAL_UART_CLEAR_FLAG(&uart_1.hal_uart.huart, UART_CLEAR_WUF);
    }

}

void USART2_IRQHandler(void)
{
	uint32_t isrflags = READ_REG(uart_2.hal_uart.huart.Instance->ISR);
	HAL_UART_IRQHandler(&uart_2.hal_uart.huart);

	if (isrflags & USART_ISR_TC) 
	{
        __HAL_UART_CLEAR_FLAG(&uart_2.hal_uart.huart, UART_CLEAR_TCF);
    }

	if (isrflags & USART_ISR_WUF) 
	{
        __HAL_UART_CLEAR_FLAG(&uart_2.hal_uart.huart, UART_CLEAR_WUF);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &uart_1.hal_uart.huart)
    {
		uvx_comm_m2m_process_rx(uart_1.byte_rx);
		
		if(comm_m2m.RX_Ready == false)
		{
			HAL_UART_Receive_IT(&uart_1.hal_uart.huart, &uart_1.byte_rx, 1);
		}
		
    }

    if (huart == &uart_2.hal_uart.huart)
    {
		uvx_comm_m2jmb_process_rx(uart_2.byte_rx);

		if(comm_m2jmb.RX_Ready == false)
		{
			HAL_UART_Receive_IT(&uart_2.hal_uart.huart, &uart_2.byte_rx, 1);
		}
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(spi_1.hal_spi.hspi.hdmatx);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(uart_1.hal_uart.huart.hdmatx);
	uart_1.hal_uart.tx_ready = 1;
}

void DMA1_Channel7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(uart_2.hal_uart.huart.hdmatx);
	//comm_m2jmb.p_hal_uart->tx_ready = 1;
	uart_2.hal_uart.tx_ready = 1;
}

// HAL UART TX complete callback (optional)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == uart_1.hal_uart.huart.Instance)
	{		
		uart_1.TX_Ready = 1;
		comm_m2m.TX_Ready = 1;
		uart_1.p_buff_tx = uart_1.p_buff_tx_start;
	}

	if( huart->Instance == uart_2.hal_uart.huart.Instance)
	{
		uart_2.TX_Ready = 1;
		comm_m2jmb.TX_Ready = 1;		
		uart_2.p_buff_tx = uart_2.p_buff_tx_start;
	}
}

void I2C1_EV_IRQHandler(void)
{
	uint32_t i2c_state = 0;
	HAL_I2C_EV_IRQHandler(&i2c_bq.hal_i2c.hi2c);
	// if( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_TC )
	// {
	// 	if(comm_bq.TX_Ready == 0)
	// 	{
	// 		comm_bq.TX_Ready = 1;
	// 	}		
	// }
	// else if (i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_STOPF)
	// {
	// 	/* Clear STOP */
	// 	i2c_bq.hal_i2c.hi2c.Instance->ICR = I2C_ICR_STOPCF;

	// 	// /* Force HAL back to READY if needed */
	// 	// i2c_bq.hal_i2c.hi2c.State = HAL_I2C_STATE_READY;

	// 	if(comm_bq.TX_Ready == 0)
	// 	{
	// 		comm_bq.TX_Ready = 1;
	// 	}	
	// }
	// else
	// {
	// 	if(comm_bq.TX_Ready == 0)
	// 	{
	// 		comm_bq.TX_Ready = 0;
	// 	}	
	// }

	if( ( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_TXE ) && 
	  (!( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_RXNE )) && 
	  (!( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_NACKF )) )
	{		
		if( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_STOPF )
		{
			i2c_bq.hal_i2c.hi2c.Instance->ICR = I2C_ICR_STOPCF;
		}
		
		if(!( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_BUSY ))
		{
			if(comm_bq_l.RX_Ready == 0)
			{
				comm_bq_l.RX_Ready = 1;
			}

			if(comm_bq_h.RX_Ready == 0)
			{
				comm_bq_h.RX_Ready = 1;
			}

			if(i2c_bq.RX_Ready == 0)
			{
				i2c_bq.RX_Ready = 1;
			}

			if(comm_bq_l.TX_Ready == 0)
			{
				comm_bq_l.TX_Ready = 1;
			}	

			if(comm_bq_h.TX_Ready == 0)
			{
				comm_bq_h.TX_Ready = 1;
			}			
		}

	}
	else if( ( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_TXE ) && 
	  		 (!( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_STOPF )) )
	{
		if(comm_bq_l.TX_Ready == 0)
		{
			comm_bq_l.TX_Ready = 1;
		}	

		if(comm_bq_h.TX_Ready == 0)
		{
			comm_bq_h.TX_Ready = 1;
		}		
	}
	else
	{
		if(comm_bq_l.TX_Ready == 0)
		{
			comm_bq_l.TX_Ready = 0;
		}	

		if(comm_bq_h.TX_Ready == 0)
		{
			comm_bq_h.TX_Ready = 1;
		}
	}

	if( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_NACKF )
	{
		i2c_bq.hal_i2c.hi2c.Instance->ICR = I2C_ICR_NACKCF;
	}

	if( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_STOPF )
	{
		i2c_bq.hal_i2c.hi2c.Instance->ICR = I2C_ICR_STOPCF;
	}	

	if( i2c_bq.hal_i2c.hi2c.Instance->ISR & I2C_ISR_BUSY )
	{
		i2c_bq.cnt_error_busy++;
		if(i2c_bq.cnt_error_busy >= 1000000)
		{
			i2c_state = i2c_bq.hal_i2c.hi2c.Instance->CR1;
			i2c_bq.hal_i2c.hi2c.Instance->CR1 &= ~I2C_CR1_PE;
			i2c_bq.cnt_error_busy = 0;			
			
			if(comm_bq_l.RX_Ready == 0)
			{
				comm_bq_l.RX_Ready = 1;
			}

			if(comm_bq_h.RX_Ready == 0)
			{
				comm_bq_h.RX_Ready = 1;
			}			

			i2c_bq.hal_i2c.hi2c.Instance->CR1 = i2c_state;
		}
	}
	else
	{
		i2c_bq.cnt_error_busy = 0;
	}

}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&hi2c1);	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (PWR_BUTTON_INTERRUPT)
	{
		uvx_gpio_set_pin(GPIO_OUT_LED_STRIP_ENABLE, GPIO_PIN_SET);
		drone_status.btn_state = true;
		exit_Sleep();
		if((drone_state.state_current != DRONE_CHECK_BUTTON_PRESS_ONCE) 
		&& (drone_state.state_current != DRONE_CHECK_BUTTON_PRESS_TWICE)
		&& (drone_state.state_current != DRONE_CHECK_BUTTON_TIMEOUT))
		{
			drone_state.state_current = DRONE_CHECK_BUTTON_PRESS_ONCE;
		}

		if(batt_state.state_current == BATT_MODE_STOP)
		{
			batt_state.state_current = BATT_MODE_READ_BQ_L;
		}
	}
	else
	{
		__NOP();
	}
}

// ---- SPI Interrupt Handlers and Callbacks ---- 
void DMA1_Channel3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(spi_1.hal_spi.hspi.hdmatx);
}

void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma){
	HAL_DMA_IRQHandler(hdma);
}

void SPI_DMAError(DMA_HandleTypeDef *hdma){
	HAL_DMA_IRQHandler(hdma);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == spi_1.hal_spi.hspi.Instance)
	{
		spi_1.TX_Ready = 1;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		uvx_spi_error_callback(&spi_1);
	}
}


void Reset_Button_Timer(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SetAutoreload(&htim2,TIME_FOR_FIRST_PRESS);
	TIM2->CR1 &= ~TIM_CR1_ARPE;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
