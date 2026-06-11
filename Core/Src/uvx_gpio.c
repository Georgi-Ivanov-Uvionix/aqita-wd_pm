#include <stdlib.h> // For malloc, realloc, and free
#include <stdbool.h> // For true and false
#include "main.h"
#include "uvx_gpio.h"

typedef struct
{
    uint32_t Input;  // Count of input pins
    uint32_t Output; // Count of output pins
} UVX_GPIO_PortCount;

typedef struct
{
    UVX_GPIO_PortCount PortA;
    UVX_GPIO_PortCount PortB;
    UVX_GPIO_PortCount PortC;
    UVX_GPIO_PortCount PortD;
    UVX_GPIO_PortCount PortE;
    UVX_GPIO_PortCount PortF;
    UVX_GPIO_PortCount PortG;
    uint8_t total_count; // Total count of all GPIO pins
} UVX_GPIO_PortCounts;

UVX_GPIO_PortCounts gpio_port_counts = {0}; // Initialize all counts to 0

UVX_GPIO UVX_GPIO_START = {.Port = '0', .Pin = 0}; // Start marker for GPIO definitions
// --------------------- APP SETUP GPIO PORT A --------------------------------------------
UVX_GPIO GPIO_EXTI1_HALL_LAND_2         = {.Port = 'A', .Pin = 1, GPIO_INPUT_PU};
UVX_GPIO GPIO_IN_ANALOG_PACK_V          = {.Port = 'A', .Pin = 4, GPIO_INPUT_ANALOG};
UVX_GPIO GPIO_OUT_LED_STRIP_ENABLE      = {.Port = 'A', .Pin = 5, GPIO_OUTPUT_PP_PD};
UVX_GPIO GPIO_OUTPUT_AP_ON              = {.Port = 'A', .Pin = 11, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_INPUT_EXTI8_DRONE_START   = {.Port = 'A', .Pin =  8, GPIO_MODE_IT_FALLING,
                                            .Interrupt = true, .Interrupt_line = EXTI9_5_IRQn,
                                            .Priority = 1, .Subpriority = 0};

// --------------------- APP SETUP GPIO PORT B --------------------------------------------
UVX_GPIO GPIO_OUTPUT_CELL_7S_DISCH_EN   = {.Port = 'B', .Pin = 10, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_8S_DISCH_EN   = {.Port = 'B', .Pin = 11, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_PIN_12             = {.Port = 'B', .Pin = 12, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_RED_LED            = {.Port = 'B', .Pin = 13, GPIO_OUTPUT_PP_PU};
UVX_GPIO GPIO_OUTPUT_GREEN_LED          = {.Port = 'B', .Pin = 14, GPIO_OUTPUT_PP_PU};
UVX_GPIO GPIO_OUTPUT_BLUE_LED           = {.Port = 'B', .Pin = 15, GPIO_OUTPUT_PP_PU};
UVX_GPIO GPIO_OUTPUT_PWR_LED            = {.Port = 'B', .Pin =  0, GPIO_OUTPUT_PP_PD};
UVX_GPIO GPIO_INPUT_STAT1               = {.Port = 'B', .Pin =  2, GPIO_INPUT_NO_PUPD};
UVX_GPIO GPIO_OUTPUT_BQH_I2C_EN         = {.Port = 'B', .Pin =  5, GPIO_OUTPUT_PP_PU};

// --------------------- APP SETUP GPIO PORT C --------------------------------------------
UVX_GPIO GPIO_OUTPUT_DOCK_HIGH_CP        = {.Port = 'C', .Pin = 4, GPIO_OUTPUT_PP_PD};  
UVX_GPIO GPIO_OUTPUT_DOCK_LOW_CP       = {.Port = 'C', .Pin = 5, GPIO_OUTPUT_PP_PD};  
UVX_GPIO GPIO_OUTPUT_DRONE_START_FET_EN = {.Port = 'C', .Pin =  6, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_BATT_MEASURE_EN    = {.Port = 'C', .Pin =  7, GPIO_OUTPUT_PP};   
UVX_GPIO GPIO_OUTPUT_JETSON_EN          = {.Port = 'C', .Pin = 10, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_PM_INH_CHG         = {.Port = 'C', .Pin = 11, GPIO_OUTPUT_PP_PD};    
UVX_GPIO GPIO_INPUT_PG_DRONE_START      = {.Port = 'C', .Pin =  8, GPIO_INPUT_NO_PUPD};

// --------------------- APP SETUP GPIO PORT D --------------------------------------------
UVX_GPIO GPIO_OUTPUT_ILIM_HIZ           = {.Port = 'D', .Pin =  8, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_EN_JETSON_PS       = {.Port = 'D', .Pin = 12, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_JMB_PERIPHERIAL_EN = {.Port = 'D', .Pin = 13, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_NANO_FORCE_REC     = {.Port = 'D', .Pin = 14, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_FC_EN              = {.Port = 'D', .Pin = 15, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_FAN_CONTROL        = {.Port = 'D', .Pin =  0, GPIO_OUTPUT_PP};

// --------------------- APP SETUP GPIO PORT E --------------------------------------------
UVX_GPIO GPIO_INPUT_EXTI1_JETSON        = {.Port = 'E', .Pin =  1, GPIO_INPUT_PD};
UVX_GPIO GPIO_OUTPUT_CE                 = {.Port = 'E', .Pin =  9, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_1S_DISCH_EN   = {.Port = 'E', .Pin = 10, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_2S_DISCH_EN   = {.Port = 'E', .Pin = 11, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_3S_DISCH_EN   = {.Port = 'E', .Pin = 12, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_4S_DISCH_EN   = {.Port = 'E', .Pin = 13, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_5S_DISCH_EN   = {.Port = 'E', .Pin = 14, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_OUTPUT_CELL_6S_DISCH_EN   = {.Port = 'E', .Pin = 15, GPIO_OUTPUT_PP};
UVX_GPIO GPIO_INPUT_STAT2               = {.Port = 'E', .Pin =  7, GPIO_INPUT_NO_PUPD};
UVX_GPIO GPIO_INPUT_INT                 = {.Port = 'E', .Pin =  8, GPIO_INPUT_NO_PUPD};
  
UVX_GPIO UVX_GPIO_END = {.Port = '0', .Pin = 0}; // End marker for GPIO definitions

/**
 * @brief  Initializes all GPIO pins defined between UVX_GPIO_START and UVX_GPIO_END.
 * @retval UVX_GPIO_OK if all pins were successfully initialized.
 *         UVX_GPIO_ERROR if any pin initialization failed.
 * @note   This function iterates through all GPIO configurations defined between
 *         the UVX_GPIO_START and UVX_GPIO_END markers. It initializes each pin
 *         using the uvx_gpio_init function and updates the global gpio_port_counts
 *         structure to track the number of input and output pins for each port,
 *         as well as the total number of initialized pins.
 * 
 *         The function performs the following steps:
 *         - Resets the gpio_port_counts structure to zero.
 *         - Iterates through all GPIO configurations.
 *         - Initializes each GPIO pin using uvx_gpio_init.
 *         - Updates the input and output counts for each port.
 *         - Increments the total pin count.
 */
UVX_GPIO_STATE uvx_gpio_init_all(void)
{
    UVX_GPIO* current_gpio = &UVX_GPIO_START; // Start from GPIO_START

    // Reset counts
    gpio_port_counts = (UVX_GPIO_PortCounts){0}; // Reset all counts to 0
    gpio_port_counts.total_count = 0; // Initialize total count to 0

    while (current_gpio != &UVX_GPIO_END) // Iterate until GPIO_END
    {
        // Move to the next GPIO structure
        current_gpio++;

        // Skip if it's the end marker
        if (current_gpio == &UVX_GPIO_END)
        {
            break;
        }

        // Initialize the current GPIO pin
        UVX_GPIO_STATE result = uvx_gpio_init(current_gpio);
        if (result != UVX_GPIO_OK)
        {
            return UVX_GPIO_ERROR; // Return error if initialization fails
        }

        // Update counts based on the port and mode
        switch (current_gpio->Port)
        {
            case 'A':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortA.Input++;
                else
                    gpio_port_counts.PortA.Output++;
                break;
            case 'B':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortB.Input++;
                else
                    gpio_port_counts.PortB.Output++;
                break;
            case 'C':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortC.Input++;
                else
                    gpio_port_counts.PortC.Output++;
                break;
            case 'D':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortD.Input++;
                else
                    gpio_port_counts.PortD.Output++;
                break;
            case 'E':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortE.Input++;
                else
                    gpio_port_counts.PortE.Output++;
                break;
            case 'F':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortF.Input++;
                else
                    gpio_port_counts.PortF.Output++;
                break;
            case 'G':
                if (current_gpio->Mode == GPIO_MODE_INPUT)
                    gpio_port_counts.PortG.Input++;
                else
                    gpio_port_counts.PortG.Output++;
                break;
            default:
                break; // Ignore other ports
        }

        gpio_port_counts.total_count++; // Increment total count
    }

    return UVX_GPIO_OK; // Return success if all pins were initialized
}

UVX_GPIO_STATE uvx_gpio_init(UVX_GPIO* gpio_config)
{
    // Initialize the GPIO pin with the specified configuration
    #if defined(STM32L4xx_HAL_H)
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_TypeDef* GPIO_Port = NULL; // Pointer to the GPIO port
    #else
        #error "Unsupported STM32 series"
    #endif

    uint16_t GPIO_Pin = 0; // Variable to store the pin number
    
    // Check if the GPIO configuration is valid
    
    if (gpio_config == NULL)
    {
        return UVX_GPIO_ERROR; // Invalid configuration
    }
        
    #if defined(STM32L4xx_HAL_H)
        if(gpio_config->Port == 'A')
        {
            __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable the clock for GPIOA
            GPIO_Port = GPIOA; // Set the GPIO port to GPIOA
        }
        else if(gpio_config->Port == 'B')
        {
            __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable the clock for GPIOB
            GPIO_Port = GPIOB; // Set the GPIO port to GPIOB
        }
        else if(gpio_config->Port == 'C')
        {
            __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the clock for GPIOC
            GPIO_Port = GPIOC; // Set the GPIO port to GPIOC
        }
        else if(gpio_config->Port == 'D')
        {
            __HAL_RCC_GPIOD_CLK_ENABLE(); // Enable the clock for GPIOD
            GPIO_Port = GPIOD; // Set the GPIO port to GPIOD
        }
        else if(gpio_config->Port == 'E')
        {
            __HAL_RCC_GPIOE_CLK_ENABLE(); // Enable the clock for GPIOE
            GPIO_Port = GPIOE; // Set the GPIO port to GPIOE        
        }
        else if(gpio_config->Port == 'F')
        {
            __HAL_RCC_GPIOF_CLK_ENABLE(); // Enable the clock for GPIOF
            GPIO_Port = GPIOF; // Set the GPIO port to GPIOF
        }
        else if(gpio_config->Port == 'G')
        {
            __HAL_RCC_GPIOG_CLK_ENABLE(); // Enable the clock for GPIOG
            GPIO_Port = GPIOG; // Set the GPIO port to GPIOG
        }
        else if(gpio_config->Port == 'H')
        {
            __HAL_RCC_GPIOH_CLK_ENABLE(); // Enable the clock for GPIOH
            GPIO_Port = GPIOH; // Set the GPIO port to GPIOH
        }
        else
        {
            return UVX_GPIO_ERROR; // Invalid port        
        }
    #else
        #error "Unsupported STM32 series"
    #endif
    
    switch(gpio_config->Pin)
    {        
        #if defined (STM32L4xx_HAL_H)
            case 0:
                GPIO_Pin = GPIO_PIN_0;
                break;

            case 1:
                GPIO_Pin = GPIO_PIN_1;
                break;

            case 2:
                GPIO_Pin = GPIO_PIN_2;
                break;

            case 3:
                GPIO_Pin = GPIO_PIN_3;
                break;

            case 4:
                GPIO_Pin = GPIO_PIN_4;
                break;

            case 5:
                GPIO_Pin = GPIO_PIN_5;
                break;

            case 6:
                GPIO_Pin = GPIO_PIN_6;
                break;
            case 7:
                GPIO_Pin = GPIO_PIN_7;
                break;

            case 8:
                GPIO_Pin = GPIO_PIN_8;
                break;
                
            case 9:
                GPIO_Pin = GPIO_PIN_9;
                break;

            case 10:
                GPIO_Pin = GPIO_PIN_10;
                break;

            case 11:
                GPIO_Pin = GPIO_PIN_11;
                break;        

            case 12:
                GPIO_Pin = GPIO_PIN_12;
                break;

            case 13:
                GPIO_Pin = GPIO_PIN_13;
                break;

            case 14:
                GPIO_Pin = GPIO_PIN_14;
                break;

            case 15:
                GPIO_Pin = GPIO_PIN_15;
                break;          

            default:
                return UVX_GPIO_ERROR; // Invalid pin number
        #else
            #error "Unsupported STM32 series"
        #endif                        
    }

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = gpio_config->Mode;
    GPIO_InitStruct.Pull = gpio_config->Pull;
    GPIO_InitStruct.Speed = gpio_config->Speed;
    GPIO_InitStruct.Alternate = gpio_config->Alternate;

    #if defined (STM32L4xx_HAL_H)
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET); // Set the pin to low initially
        HAL_GPIO_Init(GPIO_Port, &GPIO_InitStruct); // Initialize the GPIO pin
    #else
        #error "Unsupported STM32 series"
    #endif

    if (gpio_config->Interrupt == true)
    {
        #if defined (STM32L4xx_HAL_H)
            // Configure the interrupt line
            HAL_NVIC_SetPriority((IRQn_Type)(gpio_config->Interrupt_line), gpio_config->Priority, gpio_config->Subpriority);
            HAL_NVIC_EnableIRQ((IRQn_Type)(gpio_config->Interrupt_line)); // Enable the interrupt line
        #else
            #error "Unsupported STM32 series"
        #endif
    }

    return UVX_GPIO_OK; // Successfully initialized the GPIO pin
}

/**
 * @brief  Reads the state of a GPIO pin.
 * @param  GPIO: The UVX_GPIO structure containing the pin configuration.
 * @retval 1 if the pin is high, 0 if the pin is low.
 */
uint8_t uvx_gpio_read_pin(UVX_GPIO GPIO)
{
    GPIO_TypeDef* port;

    // Determine the GPIO port based on the Port field
    switch (GPIO.Port)
    {
        case 'A':
            port = GPIOA;
            break;
        case 'B':
            port = GPIOB;
            break;
        case 'C':
            port = GPIOC;
            break;
        case 'D':
            port = GPIOD;
            break;
        case 'E':
            port = GPIOE;
            break;
        case 'F':
            port = GPIOF;
            break;
        case 'G':
            port = GPIOG;
            break;
        default:
            return UVX_GPIO_ERROR; // Invalid port, return low as default
    }

    #if defined (STM32L4xx_HAL_H)
    // Read the pin state
        return HAL_GPIO_ReadPin(port, (1 << GPIO.Pin));
    #else
        #error "Unsupported STM32 series"
    #endif
}

/** * @brief  Sets the state of a GPIO pin.
 * @param  GPIO: The UVX_GPIO structure containing the pin configuration.
 * @param  state: The desired state of the pin (1 for high, 0 for low).
 * @retval UVX_GPIO_OK if the operation was successful, UVX_GPIO_ERROR if the port is invalid.
 */
UVX_GPIO_STATE uvx_gpio_set_pin(UVX_GPIO GPIO, uint8_t state)
{
    GPIO_TypeDef* port; 

    // Determine the GPIO port based on the Port field
    switch (GPIO.Port)
    {
        case 'A':
            port = GPIOA;
            break;
        case 'B':
            port = GPIOB;
            break;
        case 'C':
            port = GPIOC;
            break;
        case 'D':
            port = GPIOD;
            break;
        case 'E':
            port = GPIOE;
            break;
        case 'F':
            port = GPIOF;
            break;
        case 'G':
            port = GPIOG;
            break;
        default:
            return UVX_GPIO_ERROR; // Invalid port, return low as default
    }
    #if defined (STM32L4xx_HAL_H)
    // Set or reset the pin based on the state
    if (state)  
    {
        HAL_GPIO_WritePin(port, (1 << GPIO.Pin), GPIO_PIN_SET); // Set the pin high
    }
    else
    {
        HAL_GPIO_WritePin(port, (1 << GPIO.Pin), GPIO_PIN_RESET); // Set the pin low
    }
    #else
        #error "Unsupported STM32 series"
    #endif
    return UVX_GPIO_OK; // Successfully set the pin state
}

/**
 * @brief  Toggles the state of a GPIO pin.
 * @param  GPIO: The UVX_GPIO structure containing the pin configuration.
 * @retval UVX_GPIO_OK if the operation was successful, UVX_GPIO_ERROR if the port is invalid.
 */
UVX_GPIO_STATE uvx_gpio_toggle_pin(UVX_GPIO GPIO)
{
    GPIO_TypeDef* port; 

    // Determine the GPIO port based on the Port field
    switch (GPIO.Port)
    {
        case 'A':
            port = GPIOA;
            break;
        case 'B':
            port = GPIOB;
            break;
        case 'C':
            port = GPIOC;
            break;
        case 'D':
            port = GPIOD;
            break;
        case 'E':
            port = GPIOE;
            break;
        case 'F':
            port = GPIOF;
            break;
        case 'G':
            port = GPIOG;
            break;
        default:
            return UVX_GPIO_ERROR; // Invalid port, return low as default
    }

    #if defined (STM32L4xx_HAL_H)    
        HAL_GPIO_TogglePin(port, (1 << GPIO.Pin)); // Toggle the pin state
    #else
        #error "Unsupported STM32 series"
    #endif
    return UVX_GPIO_OK; // Successfully set the pin state
}
