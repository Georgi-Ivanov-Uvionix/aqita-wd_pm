
/* Define to prevent recursive inclusion -------------------------------------*/
#include "uvx_uart.h"
#include <stdlib.h>

static UVX_UART_STATE uvx_uart_hal_init(UVX_UART_HAL* hal_uart)
{
    UVX_UART_STATE res = UVX_UART_ERROR;
    if(hal_uart)
    {                    
        hal_uart->error_dma_cnt = 0;
        
        #if defined(STM32L4xx_HAL_H)
        if(hal_uart->huart.Instance == USART1)
        {
            __HAL_RCC_USART1_CLK_ENABLE();
        }
        else if(hal_uart->huart.Instance == USART2)
        {
            __HAL_RCC_USART2_CLK_ENABLE();
        }
        else if(hal_uart->huart.Instance == USART3)
        {
            __HAL_RCC_USART3_CLK_ENABLE();
        }
        else if(hal_uart->huart.Instance == UART4)
        {
            __HAL_RCC_UART4_CLK_ENABLE();
        }
        else if(hal_uart->huart.Instance == UART5)
        {
            __HAL_RCC_UART5_CLK_ENABLE();
        }
        #else
                #error "Unsupported STM32 series"
        #endif

        if (HAL_UART_Init(&hal_uart->huart) == HAL_OK)
        {
            if(hal_uart->uart_interrupt_rx)
            {
                #if defined(STM32L4xx_HAL_H)
                    /* Enable the USART1 global Interrupt */
                    HAL_NVIC_SetPriority((IRQn_Type)hal_uart->uart_interrupt_line, hal_uart->uart_interrupt_rx, hal_uart->uart_interrupt_rx);
                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_uart->uart_interrupt_line);
                    __HAL_UART_ENABLE_IT(&hal_uart->huart, UART_IT_RXNE); // Enable RXNE interrupt                
                #else
                    #error "Unsupported STM32 series"
                #endif

            }

            if(hal_uart->uart_interrupt_tx)
            {
                #if defined(STM32L4xx_HAL_H)
                    /* Enable the USART1 global Interrupt */
                    HAL_NVIC_SetPriority((IRQn_Type)hal_uart->uart_interrupt_line, hal_uart->uart_interrupt_tx, hal_uart->uart_interrupt_tx);
                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_uart->uart_interrupt_line);
                    __HAL_UART_ENABLE_IT(&hal_uart->huart, UART_IT_TXE); // Enable TXE interrupt
                #else
                    #error "Unsupported STM32 series"
                #endif
            }

            // Enable the DMA clock for TX and RX
            if((hal_uart->dma_tx_enabled) || (hal_uart->dma_rx_enabled))
            {
                if ((hal_uart->hdma_tx.Instance >= DMA1_Channel1) && (hal_uart->hdma_tx.Instance <= DMA1_Channel7))
                {
                    __HAL_RCC_DMA1_CLK_ENABLE();
                }
                else if ((hal_uart->hdma_tx.Instance >= DMA2_Channel1) && (hal_uart->hdma_tx.Instance <= DMA2_Channel7))
                {
                    __HAL_RCC_DMA2_CLK_ENABLE();
                }
                else
                {
                    return UVX_UART_INIT_ERROR_DMA;
                }

                if(hal_uart->dma_tx_enabled)
                {
                    HAL_DMA_Init(&hal_uart->hdma_tx);
                    __HAL_LINKDMA(&hal_uart->huart, hdmatx, hal_uart->hdma_tx);

                    if(hal_uart->dma_interrupt_tx)
                    {
                        HAL_NVIC_SetPriority((IRQn_Type)hal_uart->dma_interrupt_line_tx, hal_uart->dma_priority_tx, hal_uart->dma_subpriority_tx);
                        HAL_NVIC_EnableIRQ((IRQn_Type)hal_uart->dma_interrupt_line_tx);
                    }
                }

                if(hal_uart->dma_rx_enabled)
                {
                    HAL_DMA_Init(&hal_uart->hdma_rx);
                    __HAL_LINKDMA(&hal_uart->huart, hdmarx, hal_uart->hdma_rx);

                    if(hal_uart->dma_interrupt_rx)
                    {
                        HAL_NVIC_SetPriority((IRQn_Type)hal_uart->dma_interrupt_line_rx, hal_uart->dma_priority_rx, hal_uart->dma_subpriority_rx);
                        HAL_NVIC_EnableIRQ((IRQn_Type)hal_uart->dma_interrupt_line_rx);
                    }
                }
            }
            
            res = UVX_UART_OK;
        }
        else
        {
            res = UVX_UART_INIT_ERROR;
        }
    }
    else
    {
        res = UVX_UART_ERROR;
    }

    return res;
}

/**
 * @brief Initializes the UART peripheral and its associated GPIO pins.
 * @param uart Pointer to the UVX_UART structure containing UART configuration.
 * @retval UVX_UART_STATE Status of the initialization process.
 */
UVX_UART_STATE uvx_uart_init(UVX_UART* uart, uint8_t* p_buff_tx, uint8_t* p_buff_rx)
{
    UVX_UART_STATE res = UVX_UART_ERROR;
    if(uart)
    {
        if(uart->is_Initilized == 0)
        {
            if(uvx_gpio_init(&uart->gpio_tx) != UVX_GPIO_OK)
            {
                return UVX_UART_INIT_ERROR_GPIO;
            }

            if(uvx_gpio_init(&uart->gpio_rx) != UVX_GPIO_OK)
            {
                return UVX_UART_INIT_ERROR_GPIO;
            }

            if(uvx_uart_hal_init(&uart->hal_uart) == UVX_UART_OK)
            {                
                if(uart->buff_size_tx > 0)
                {
                    uart->p_buff_tx_start = p_buff_tx;
                    if(uart->p_buff_tx_start == NULL) //
                    {
                        return UVX_UART_INIT_ERROR_TX_MEMORY;
                    }
                    uart->p_buff_tx = uart->p_buff_tx_start;                    
                }
                else
                {
                    return UVX_UART_INIT_ERROR_TX_MEMORY;
                }

                if(uart->buff_size_rx > 0)
                {
                    uart->p_buff_rx_start = p_buff_rx;// Allocate memory for RX buffer
                    if(uart->p_buff_rx_start == NULL)
                    {
                        return UVX_UART_INIT_ERROR_RX_MEMORY;
                    }
                    uart->p_buff_rx = uart->p_buff_rx_start;
                    HAL_UART_Receive_IT(&uart->hal_uart.huart, &uart->byte_rx, 1);
                }
                else
                {
                    return UVX_UART_INIT_ERROR_RX_MEMORY;
                }

                uart->hal_uart.tx_ready = 1;
                uart->is_Initilized = 1;
                res = UVX_UART_OK;
            }
            else
            {
                res = UVX_UART_INIT_ERROR;
            }
        }
        else
        {
            res = UVX_UART_ALREADY_INITIALIZED;
        }
    }
    else
    {
        res = UVX_UART_ERROR;
    }

    return res;
}

UVX_UART_STATE uvx_uart_send(UVX_UART_HAL* p_uart, uint8_t *data, uint16_t size) 
{
	#if defined(STM32L4xx_HAL_H)
		// Check if the UART handle is initialized
		if (p_uart != NULL && p_uart->huart.Instance != NULL)
		{
            if(p_uart->tx_ready)
            {
                p_uart->tx_ready = 0; // Clear the TX ready flag before sending
                //HAL_UART_Transmit(&comm_m2m.p_hal_uart->huart, data, size, HAL_MAX_DELAY);
                if(HAL_UART_Transmit_DMA(&p_uart->huart, data, size) != HAL_OK)
                {
                    p_uart->error_dma_cnt++;                
                    if(p_uart->error_dma_cnt > UVX_UART_MAX_DMA_ERRORS) // If there are too many consecutive DMA errors, reset the UART
                    {
                        HAL_UART_AbortTransmit(&p_uart->huart);
                        p_uart->error_dma_cnt = 0; // Reset error counter after aborting transmission
                    }
                    return UVX_UART_ERROR; // Return error if transmission fails
                }
            }
            else
            {
                p_uart->error_dma_cnt++;                
                if(p_uart->error_dma_cnt > UVX_UART_MAX_DMA_ERRORS) // If there are too many consecutive DMA errors, reset the UART
                {
                    p_uart->tx_ready = true; // Set TX ready flag to true before aborting transmission
                    HAL_UART_AbortTransmit(&p_uart->huart);
                    p_uart->error_dma_cnt = 0; // Reset error counter after aborting transmission
                    p_uart->error_cnt++; // Increment total error counter
                }
                return UVX_UART_BUSY; // Return busy if the UART is not ready for transmission
            }
		}
		else
		{
			return UVX_UART_INIT_ERROR; // Return error if the UART handle is not initialized
		}

	#else
		#error "Unsupported STM32 series"
	#endif

	return UVX_UART_OK; // Return success
}
