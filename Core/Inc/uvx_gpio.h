#ifndef UVX_GPIO_H
#define UVX_GPIO_H

#include <stdint.h>
#include "stm32l4xx_hal.h" // Include the HAL library for GPIO functions
#include <stdbool.h> // Include stdbool for boolean types

/**
 * @brief Button state enumeration definition
 */
typedef enum
{
    UVX_GPIO_OK = 0x00,       // Operation successful
    UVX_GPIO_ERROR = 0x01,    // General error
    UVX_GPIO_BUSY = 0x02,     // Button is busy
    UVX_GPIO_TIMEOUT = 0x03   // Operation timed out
} UVX_GPIO_STATE;

/**
 * @brief GPIO configuration structure definition
 */
typedef struct
{
    uint8_t Port;             // GPIO port (e.g., GPIOA, GPIOB, etc.)
    uint8_t Pin;              // GPIO pin number    
    uint32_t Mode;             // GPIO mode (e.g., input, output, alternate function)
    uint8_t Pull;             // GPIO pull-up/pull-down configuration
    uint8_t Speed;            // GPIO speed (e.g., low, medium, high)
    uint8_t Alternate;        // Alternate function (if applicable)
    uint8_t Interrupt;        // Interrupt configuration (if applicable)
    uint32_t Interrupt_line;   // Interrupt line number (if applicable)
    uint32_t Priority;        // Interrupt priority (if applicable)
    uint32_t Subpriority;     // Subpriority (if applicable)
} UVX_GPIO;


#define GPIO_EXTI \
    .Pull = GPIO_NOPULL, \
    .Speed = GPIO_SPEED_FREQ_LOW, \
    .Alternate = 0, \
    .Interrupt = true, \
    .Interrupt_line = EXTI9_5_IRQn, \
    .Priority = 1, \
    .Subpriority = 0

#define GPIO_OUTPUT_PP \
    .Mode = GPIO_MODE_OUTPUT_PP, \
    .Pull = GPIO_NOPULL, \
    .Speed = GPIO_SPEED_FREQ_LOW, \
    .Alternate = 0

#define GPIO_OUTPUT_PP_PU \
    .Mode = GPIO_MODE_OUTPUT_PP, \
    .Pull = GPIO_PULLUP, \
    .Speed = GPIO_SPEED_FREQ_LOW, \
    .Alternate = 0    

#define GPIO_OUTPUT_PP_PD \
    .Mode = GPIO_MODE_OUTPUT_PP, \
    .Pull = GPIO_PULLDOWN, \
    .Speed = GPIO_SPEED_FREQ_LOW, \
    .Alternate = 0        

#define GPIO_INPUT_NO_PUPD \
    .Mode = GPIO_MODE_INPUT, \
    .Pull = GPIO_NOPULL

#define GPIO_INPUT_PU \
.Mode = GPIO_MODE_INPUT, \
.Pull = GPIO_PULLUP

#define GPIO_INPUT_PD \
.Mode = GPIO_MODE_INPUT, \
.Pull = GPIO_PULLDOWN

#define GPIO_INPUT_ANALOG \
.Mode = GPIO_MODE_ANALOG_ADC_CONTROL, \
.Pull = GPIO_NOPULL

extern UVX_GPIO GPIO_EXTI1_HALL_LAND_2;
extern UVX_GPIO GPIO_INPUT_EXTI1_JETSON;
extern UVX_GPIO GPIO_OUT_LED_STRIP_ENABLE;
extern UVX_GPIO GPIO_INPUT_EXTI8_DRONE_START;
extern UVX_GPIO GPIO_OUTPUT_RED_LED;
extern UVX_GPIO GPIO_OUTPUT_GREEN_LED;
extern UVX_GPIO GPIO_OUTPUT_PWR_LED;
extern UVX_GPIO GPIO_OUTPUT_DRONE_START_FET_EN;
extern UVX_GPIO GPIO_OUTPUT_CELL_1S_DISCH_EN;
extern UVX_GPIO GPIO_OUTPUT_BLUE_LED;
extern UVX_GPIO GPIO_OUTPUT_JMB_PERIPHERIAL_EN;
extern UVX_GPIO GPIO_OUTPUT_BQH_I2C_EN;
extern UVX_GPIO GPIO_OUTPUT_PM_INH_CHG;
extern UVX_GPIO GPIO_OUTPUT_DOCK_LOW_CP;
extern UVX_GPIO GPIO_OUTPUT_DOCK_HIGH_CP;

UVX_GPIO_STATE uvx_gpio_init(UVX_GPIO* gpio_config);
UVX_GPIO_STATE uvx_gpio_init_all(void);
uint8_t uvx_gpio_read_pin(UVX_GPIO GPIO);
UVX_GPIO_STATE uvx_gpio_set_pin(UVX_GPIO GPIO, uint8_t state);
UVX_GPIO_STATE uvx_gpio_toggle_pin(UVX_GPIO GPIO);

#endif // UVX_GPIO_H
