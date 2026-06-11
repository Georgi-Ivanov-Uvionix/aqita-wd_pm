
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UVX_TIMER_H
#define UVX_TIMER_H

#include "stm32l4xx_hal.h"
#include "uvx_universal_types.h"
#include "uvx_gpio.h"

/**
  * @brief  HAL Status structures definition
  */
 typedef enum
 {
   UVX_TIMER_OK       = 0x00,
   UVX_TIMER_ERROR    = 0x01,
   UVX_TIMER_BUSY     = 0x02,
   UVX_TIMER_TIMEOUT  = 0x03
 } UVX_TIMER_STATE;

typedef struct UVX_TIMER_HAL
{
	TIM_HandleTypeDef 		htim;
	TIM_ClockConfigTypeDef 	sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
    uint8_t is_Initilized 	: 1; // Flag to indicate if HAL timer is initialized
    uint8_t Reserved 		: 7; // Reserved for future use
}UVX_TIMER_HAL;

typedef enum UVX_TIMER_UNIT
{
    UNIT_MSEC = 0, // Timer uses milliseconds
    UNIT_USEC = 1  // Timer uses microseconds
} UVX_TIMER_UNIT;

typedef struct UVX_TIMER
{
	uint8_t 		ID;		    // Timer ID	
	uint32_t		Timeout;    // Timeout value in milliseconds or microseconds
	UVX_TIMER_UNIT 	Time_Unit;	// Time unit (msec or usec)

    uint8_t Enable 		: 1;   
    uint8_t Error 		: 1;  
    uint8_t Up_count 	: 1;
    uint8_t Down_count 	: 1;
    uint8_t Reserve 	: 4; // Reserved for future use	
}UVX_TIMER;

#define UVX_SETUP_TIMER_HAL  ((UVX_TIMER_HAL) \
{ \
    .htim = \
	{ \
        .Instance = TIM7, \
        .Init = \
		{ \
            .Prescaler = 40000, \
            .CounterMode = TIM_COUNTERMODE_UP, \
            .Period = 1, \
            .ClockDivision = TIM_CLOCKDIVISION_DIV1, \
            .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE \
        } \
    }, \
	\
    .sClockSourceConfig = \
	{ \
        .ClockSource = TIM_CLOCKSOURCE_INTERNAL \
    }, \
	\
    .sMasterConfig = \
	{ \
        .MasterOutputTrigger = TIM_TRGO_RESET, \
        .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE \
    } \
})

typedef enum
{
  TIMER_GENERAL       = 0x00,
  TIMER_JMB,
  TIMER_BATT_PWR_HIGH,
  TIMER_BATT_PWR_LOW,
  TIMER_BATT_LOW,
  TIMER_DRONE,
  TIMER_LED,
  TIMER_LAND,
  TIMER_NUMBER // Total number of timers
} UVX_TIMERS;

#define UVX_SETUP_TIMER_GENERAL  ((UVX_TIMER) \
{ \
    .ID = TIMER_GENERAL, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_TIMER_APP_COMM_JMB  ((UVX_TIMER) \
{ \
    .ID = TIMER_JMB, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_timer_app_batt_pwr_high  ((UVX_TIMER) \
{ \
    .ID = TIMER_BATT_PWR_HIGH, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_timer_app_batt_pwr_low  ((UVX_TIMER) \
{ \
    .ID = TIMER_BATT_PWR_LOW, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_timer_app_batt_low_voltage  ((UVX_TIMER) \
{ \
    .ID = TIMER_BATT_LOW, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_TIMER_APP_DRONE  ((UVX_TIMER) \
{ \
    .ID = TIMER_DRONE, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_TIMER_APP_LED  ((UVX_TIMER) \
{ \
    .ID = TIMER_LED, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

#define UVX_SETUP_TIMER_APP_LAND  ((UVX_TIMER) \
{ \
    .ID = TIMER_LAND, \
    .Timeout = 0, \
    .Time_Unit = UNIT_MSEC, \
    .Enable = 0, \
    .Error = 0, \
    .Up_count = 0, \
    .Down_count = 1, \
    .Reserve = 0 \
})

static UVX_TIMER_STATE uvx_timer_hal_init(UVX_TIMER_HAL* hal_timer);
UVX_TIMER_STATE uvx_timer_add(UVX_TIMER* timer);
UVX_TIMER_STATE uvx_timer_start(uint8_t timer_id, uint32_t timeout);
UVX_TIMER_STATE uvx_timer_callback(TIM_HandleTypeDef *htim);

extern UVX_TIMER_HAL uvx_timer_hal;;
extern UVX_TIMER* uvx_software_timers[TIMER_NUMBER]; // Pointer to the array of software timers
#endif
