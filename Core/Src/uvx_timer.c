
#include <stdlib.h> // For malloc, realloc, and free
#include "uvx_timer.h"

#define TIMER_RESOLUTION_mS 	1 // Timer resolution in milliseconds

UVX_TIMER* 	uvx_software_timers[TIMER_NUMBER]; // Array of software timers
uint32_t 	uvx_num_timers = 0; 			// Current number of timers
uint32_t 	uvx_timeout_drift = 0; 			// Drift value for the timer
uint32_t 	uvx_timer_cnt = 0; 				// Timer counter for the callback function

UVX_TIMER_HAL uvx_timer_hal;
/**
 * @brief  Initializes a hardware timer using the HAL library.
 * @param  hal_timer: Pointer to a UVX_TIMER_HAL structure containing the timer configuration.
 * @retval UVX_TIMER_OK if the timer was successfully initialized.
 *         UVX_TIMER_ERROR if an error occurred during initialization.
 * @note   This function performs the following steps:
 *         - Initializes the base timer using HAL_TIM_Base_Init.
 *         - Configures the clock source using HAL_TIM_ConfigClockSource.
 *         - Sets up master synchronization using HAL_TIMEx_MasterConfigSynchronization.
 *         - Sets the auto-reload value and resets the timer counter.
 *         - Starts the timer in interrupt mode.
 */
static UVX_TIMER_STATE uvx_timer_hal_init(UVX_TIMER_HAL* hal_timer)
{  
	UVX_TIMER_STATE res = UVX_TIMER_ERROR;	

	if(hal_timer)
	{
		if(hal_timer->is_Initilized == 0)
		{
			#if defined(STM32L4xx_HAL_H)
				if (HAL_TIM_Base_Init(&hal_timer->htim) != HAL_OK)
				{
					return UVX_TIMER_ERROR;
				}
			
				if (HAL_TIM_ConfigClockSource(&hal_timer->htim, &hal_timer->sClockSourceConfig) != HAL_OK)
				{
					return UVX_TIMER_ERROR;
				}
			
				if (HAL_TIMEx_MasterConfigSynchronization(&hal_timer->htim, &hal_timer->sMasterConfig) != HAL_OK)
				{
					return UVX_TIMER_ERROR;
				}
		
				//__HAL_TIM_SetAutoreload(&hal_timer->htim, 2000);
				__HAL_TIM_SET_COUNTER(&hal_timer->htim,0);
				HAL_TIM_Base_Start_IT(&hal_timer->htim);
			#else
				#error "Unsupported STM32 series"
			#endif
	
			hal_timer->is_Initilized = 1; // Set the initialization flag to indicate that the timer is initialized
			res = UVX_TIMER_OK;
		}
		else
		{
			res = UVX_TIMER_OK; // Timer is already initialized
		}
	}
	else
	{
		res = UVX_TIMER_ERROR;
	}

	return res;
}

/**
 * @brief  Dynamically adds a new software timer to the array.
 * @param  timer_id: The unique identifier for the new timer.
 * @param  time_unit: The time unit for the timer (milliseconds or microseconds).
 * @retval UVX_TIMER_OK if the timer was successfully added and initialized.
 *         UVX_TIMER_ERROR if memory allocation failed.
 * @note   This function uses realloc to expand the software_timers array.
 *         The newly added timer is initialized with default values:
 *         - State: Disabled (Enable = 0, Disable = 1)
 *         - Timeout = 0
 *         - Time_Unit = time_unit
 */
UVX_TIMER_STATE uvx_timer_add(UVX_TIMER* timer)
{
    uvx_software_timers[uvx_num_timers] = timer; // Update the pointer to the new array
    uvx_num_timers++; // Increment the number of timers

	if(uvx_num_timers > TIMER_NUMBER) // Check if the number of timers exceeds the limit
	{
		return UVX_TIMER_ERROR; // Memory allocation failed
	}

	if(uvx_timer_hal.is_Initilized == 0) // Check if the hardware timer is initialized
	{
		uvx_timer_hal = UVX_SETUP_TIMER_HAL; // Initialize the hardware timer structure

		if(uvx_timer_hal_init(&uvx_timer_hal) != UVX_TIMER_OK) // Initialize the hardware timer
		{
			return UVX_TIMER_ERROR;
		}
	}

    return UVX_TIMER_OK; // Successfully added and initialized the timer
}

/**
 * @brief  Handles the software timers during the hardware timer interrupt callback.
 * @param  htim: Pointer to the hardware timer handle that triggered the interrupt.
 * @retval UVX_TIMER_OK if the callback executed successfully.
 *         UVX_TIMER_ERROR if an error occurred during execution.
 * @note   This function performs the following steps:
 *         - Verifies if the interrupt is from the correct hardware timer.
 *         - Iterates through all software timers to check their state.
 *         - If a timer is enabled:
 *             - Increments the `Timeout` value if the timer is in up-count mode.
 *             - Decrements the `Timeout` value if the timer is in down-count mode.
 *         - Ensures that `Timeout` does not exceed its maximum value or go below zero.
 *         - Handles timer expiration by disabling the timer and setting an error flag.
 *         - Calculates and stores the drift value for timing adjustments.
 */
UVX_TIMER_STATE uvx_timer_callback(TIM_HandleTypeDef *htim)
{
	if (htim == &uvx_timer_hal.htim)
	{				
		uvx_timer_cnt++;
		if(uvx_timer_cnt >= TIMER_RESOLUTION_mS - uvx_timeout_drift)
		{
			uvx_timeout_drift = uwTick; // Store the current tick value for drift calculation
			uvx_timer_cnt = 0;

			for (uint8_t i = 0; i < uvx_num_timers; i++) // Iterate through all software timers
			{
				if (uvx_software_timers[i]->Enable) // Check if the timer is enabled
				{
					if (uvx_software_timers[i]->Up_count) // If the timer is in up-count mode
					{
						if (uvx_software_timers[i]->Timeout < 0xFFFFFFFF) // Ensure timeout doesn't exceed max value
						{
							uvx_software_timers[i]->Timeout =+ TIMER_RESOLUTION_mS; // Increment the timeout
						}
						else
						{
							uvx_software_timers[i]->Error = 1; // Set an error or expired flag
						}						
					}
					else if (uvx_software_timers[i]->Down_count) // If the timer is in down-count mode
					{
						if (uvx_software_timers[i]->Timeout > 0) // Ensure timeout doesn't go below zero
						{
							if((uvx_software_timers[i]->Timeout > 0) && (uvx_software_timers[i]->Timeout >= TIMER_RESOLUTION_mS)) // Check if timeout is greater than resolution
							{
								uvx_software_timers[i]->Timeout -= TIMER_RESOLUTION_mS; // Decrement the timeout
							}                
							else
							{
								uvx_software_timers[i]->Timeout = 0;
								uvx_software_timers[i]->Error = 1; // Set an error or expired flag
							}        
						}
						else
						{
							// Handle timer expiration (optional)
							uvx_software_timers[i]->Enable = 0; // Disable the timer
							uvx_software_timers[i]->Error = 1; // Set an error or expired flag
						}
					}
				}
			}

			if(uwTick > uvx_timeout_drift)
			{
				uvx_timeout_drift = uwTick - uvx_timeout_drift; // Calculate the drift value

				if(uvx_timeout_drift > TIMER_RESOLUTION_mS)
				{
					uvx_timeout_drift = TIMER_RESOLUTION_mS; // Limit the drift value to the timeout
				}
			}
			else
			{
				uvx_timeout_drift = 0;
			}			
			
		}
	}

	return UVX_TIMER_OK; // Successfully handled the timer callback
}
