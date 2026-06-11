/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_LED_STRIP
#define __UVX_LED_STRIP

#include "main.h"

#define SRAM1 __attribute__((section(".sram1")))
#define SRAM2 __attribute__((section(".sram2")))

#define LED_EFFECT_MIN_TIMES    10

typedef struct
{

    uint32_t cnt_no_response;
        
}UVX_LED_STRIP_DATA;

 typedef enum
 {
   UVX_LED_STRIP_OK = 0x00,
   UVX_LED_STRIP_ERROR,
   UVX_LED_STRIP_ERROR_INIT,
   UVX_LED_STRIP_REG_END,
   UVX_LED_STRIP_TIMEOUT
 } UVX_LED_STRIP_STATE;

typedef enum 
{
    LED_STRIP_MODE_INIT = 0x00,
    LED_STRIP_MODE_IDLE,
    LED_STRIP_MODE_BATTERY_LEVEL,
    LED_STRIP_MODE_BTN_PRESS,
    LED_STRIP_MODE_ALL_OFF,
    LED_STRIP_MODE_ALL_ON,
    LED_STRIP_MODE_HEARTBEAT,
    LED_STRIP_MODE_COMM_FC,
    LED_STRIP_MODE_ALL_BREATHING,
    LED_STRIP_MODE_ALL_IN_FLIGHT_STROBE,
    LED_STRIP_MODE_WAVE,
    LED_STRIP_MODE_WAITING,
    LED_STRIP_MODE_VOLTAGE_LEVEL,    
    LED_STRIP_MODE_OFF,
    LED_STRIP_MODE_STOP,
    LED_STRIP_MODE_WAIT_RESPONSE
} UVX_LED_STRIP_MODE;

typedef struct 
{
  UVX_LED_STRIP_MODE state_previous; // Previous state of the M2JMB communication
  UVX_LED_STRIP_MODE state_current;  // Current state of the M2JMB communication
  UVX_LED_STRIP_MODE state_next; // Next state of the M2JMB communication
} UVX_LED_STRIP_STATE_MACHINE;

extern UVX_LED_STRIP_STATE_MACHINE led_strip_state;
extern SRAM1 UVX_LED_STRIP_DATA led_strip_data;

#endif
