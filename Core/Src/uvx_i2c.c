/* Define to prevent recursive inclusion -------------------------------------*/
#include "uvx_i2c.h"
#include <stdlib.h>
#include <string.h>

static UVX_I2C_STATE uvx_i2c_hal_init(UVX_I2C_HAL* hal_i2c)
{
    UVX_I2C_STATE res = UVX_I2C_ERROR;
    if(hal_i2c)
    {                    
        hal_i2c->error_dma_cnt = 0;
        hal_i2c->lock = 0U;
        hal_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;
        hal_i2c->transfer_error = HAL_I2C_ERROR_NONE;
        
        #if defined(STM32L4xx_HAL_H)
        if(hal_i2c->hi2c.Instance == I2C1)
        {
            __HAL_RCC_I2C1_CLK_ENABLE();
        }
        else if(hal_i2c->hi2c.Instance == I2C2)
        {
            __HAL_RCC_I2C2_CLK_ENABLE();
        }
        else if(hal_i2c->hi2c.Instance == I2C3)
        {
            __HAL_RCC_I2C3_CLK_ENABLE();
        }

        #else
                #error "Unsupported STM32 series"
        #endif

        if (HAL_I2C_Init(&hal_i2c->hi2c) == HAL_OK)
        {
            if(hal_i2c->i2c_interrupt_ev != (IRQn_Type)0)
            {
                #if defined(STM32L4xx_HAL_H)
                    /* Enable the I2C1 global Interrupt */
                    HAL_NVIC_SetPriority((IRQn_Type)hal_i2c->i2c_interrupt_ev,
                     hal_i2c->i2c_interrupt_ev_priority,
                     hal_i2c->i2c_interrupt_ev_subpriority);

                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_i2c->i2c_interrupt_ev);               
                #else
                    #error "Unsupported STM32 series"
                #endif

                if(hal_i2c->i2c_interrupt_rx)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_RXI); // Enable RXNE interrupt                
                    #else
                        #error "Unsupported STM32 series"
                    #endif

                }

                if(hal_i2c->i2c_interrupt_tx)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_TXI); // Enable TXE interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }

                if(hal_i2c->i2c_interrupt_tc)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_TCI); // Enable TC interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }                
            }

            if(hal_i2c->i2c_interrupt_err != (IRQn_Type)0)
            {
                #if defined(STM32L4xx_HAL_H)
                    /* Enable the I2C1 Error Interrupt */
                    HAL_NVIC_SetPriority((IRQn_Type)hal_i2c->i2c_interrupt_err,
                     hal_i2c->i2c_interrupt_err_priority,
                     hal_i2c->i2c_interrupt_err_subpriority);

                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_i2c->i2c_interrupt_err);
                #else
                    #error "Unsupported STM32 series"
                #endif

                if(hal_i2c->i2c_interrupt_berr)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable BERR interrupt                
                    #else
                        #error "Unsupported STM32 series"
                    #endif

                }

                if(hal_i2c->i2c_interrupt_arlo)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable ARLO interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }

                if(hal_i2c->i2c_interrupt_ovr)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable OVR interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }

                if(hal_i2c->i2c_interrupt_pecerr)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable PECERR interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }

                if(hal_i2c->i2c_interrupt_timeout)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable TIMEOUT interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }

                if(hal_i2c->i2c_interrupt_alert)
                {
                    #if defined(STM32L4xx_HAL_H)
                        __HAL_I2C_ENABLE_IT(&hal_i2c->hi2c, I2C_IT_ERRI); // Enable ALERT interrupt
                    #else
                        #error "Unsupported STM32 series"
                    #endif
                }
            }

            // Enable the DMA clock for TX and RX
            if((hal_i2c->dma_tx_enabled) || (hal_i2c->dma_rx_enabled))
            {
                if ((hal_i2c->hdma_tx.Instance >= DMA1_Channel1) && (hal_i2c->hdma_tx.Instance <= DMA1_Channel7))
                {
                    __HAL_RCC_DMA1_CLK_ENABLE();
                }
                else if ((hal_i2c->hdma_tx.Instance >= DMA2_Channel1) && (hal_i2c->hdma_tx.Instance <= DMA2_Channel7))
                {
                    __HAL_RCC_DMA2_CLK_ENABLE();
                }
                else
                {
                    return UVX_I2C_INIT_ERROR_DMA;
                }

                if(hal_i2c->dma_tx_enabled)
                {
                    HAL_DMA_Init(&hal_i2c->hdma_tx);
                    __HAL_LINKDMA(&hal_i2c->hi2c, hdmatx, hal_i2c->hdma_tx);

                    if(hal_i2c->dma_interrupt_tx)
                    {
                        HAL_NVIC_SetPriority((IRQn_Type)hal_i2c->dma_interrupt_line_tx, hal_i2c->dma_priority_tx, hal_i2c->dma_subpriority_tx);
                        HAL_NVIC_EnableIRQ((IRQn_Type)hal_i2c->dma_interrupt_line_tx);
                    }
                }

                if(hal_i2c->dma_rx_enabled)
                {
                    HAL_DMA_Init(&hal_i2c->hdma_rx);
                    __HAL_LINKDMA(&hal_i2c->hi2c, hdmarx, hal_i2c->hdma_rx);

                    if(hal_i2c->dma_interrupt_rx)
                    {
                        HAL_NVIC_SetPriority((IRQn_Type)hal_i2c->dma_interrupt_line_rx, hal_i2c->dma_priority_rx, hal_i2c->dma_subpriority_rx);
                        HAL_NVIC_EnableIRQ((IRQn_Type)hal_i2c->dma_interrupt_line_rx);
                    }
                }
            }
            
            res = UVX_I2C_OK;
        }
        else
        {
            res = UVX_I2C_INIT_ERROR;
        }
    }
    else
    {
        res = UVX_I2C_ERROR;
    }

    return res;
}

/**
 * @brief Initializes the I2C peripheral and its associated GPIO pins.
 * @param i2c Pointer to the UVX_I2C structure containing I2C configuration.
 * @retval UVX_I2C_STATE Status of the initialization process.
 */
UVX_I2C_STATE uvx_i2c_init(UVX_I2C* i2c/*, uint8_t* p_buff_tx, uint8_t* p_buff_rx*/)
{
    UVX_I2C_STATE res = UVX_I2C_ERROR;
    if(i2c)
    {
        if(i2c->is_Initilized == 0)
        {
            if(uvx_gpio_init(&i2c->gpio_sda) != UVX_GPIO_OK)
            {
                return UVX_I2C_INIT_ERROR_GPIO;
            }

            if(uvx_gpio_init(&i2c->gpio_scl) != UVX_GPIO_OK)
            {
                return UVX_I2C_INIT_ERROR_GPIO;
            }

            if(uvx_i2c_hal_init(&i2c->hal_i2c) == UVX_I2C_OK)
            {                
                // if(i2c->buff_size_tx > 0)
                // {
                //     i2c->p_buff_tx_start = p_buff_tx;
                //     if(i2c->p_buff_tx_start == NULL) //
                //     {
                //         return UVX_I2C_INIT_ERROR_TX_MEMORY;
                //     }
                //     i2c->p_buff_tx = i2c->p_buff_tx_start;                    
                // }
                // else
                // {
                //     return UVX_I2C_INIT_ERROR_TX_MEMORY;
                // }

                // if(i2c->buff_size_rx > 0)
                // {
                //     i2c->p_buff_rx_start = p_buff_rx;// Allocate memory for RX buffer
                //     if(i2c->p_buff_rx_start == NULL)
                //     {
                //         return UVX_I2C_INIT_ERROR_RX_MEMORY;
                //     }
                //     i2c->p_buff_rx = i2c->p_buff_rx_start;
                //     // HAL_I2C_Receive_IT(&i2c->hal_i2c.hi2c, &i2c->byte_rx, 1);
                // }
                // else
                // {
                //     return UVX_I2C_INIT_ERROR_RX_MEMORY;
                // }

                i2c->is_Initilized = 1;
                i2c->hal_i2c.RX_Ready = 1;
                i2c->hal_i2c.TX_Ready = 1;
                res = UVX_I2C_OK;
            }
            else
            {
                res = UVX_I2C_INIT_ERROR;
            }
        }
        else
        {
            res = UVX_I2C_ALREADY_INITIALIZED;
        }
    }
    else
    {
        res = UVX_I2C_ERROR;
    }

    return res;
}

UVX_I2C_STATE uvx_i2c_send_mem(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint16_t reg_addr, uint16_t reg_size, uint8_t* data, uint16_t size)  
{
    HAL_StatusTypeDef res = HAL_ERROR;
	#if defined(STM32L4xx_HAL_H)
		// Check if the I2C handle is initialized
		if (p_i2c != NULL && p_i2c->hi2c.Instance != NULL)
		{
            if((p_i2c->lock == 0U) ||
               (p_i2c->transfer_state != UVX_I2C_TRANSFER_IDLE))
            {
                return UVX_I2C_BUSY;
            }
            
            if( !( p_i2c->hi2c.Instance->ISR & I2C_ISR_STOPF ) && 
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_NACKF ) &&
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_BUSY ) )
            {
                if( ( ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) ) ||                 
                    ( ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) &&
                      ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXIS ) ) )
                {
                    if(p_i2c->TX_Ready)
                    {
                        p_i2c->TX_Ready = 0;
                        dev_addr <<= 1; // Shift address            
                        p_i2c->transfer_state = UVX_I2C_TRANSFER_PENDING;
                        res = HAL_I2C_Mem_Write_IT(&p_i2c->hi2c, dev_addr, reg_addr, reg_size, data, size);
                        //res = HAL_I2C_Master_Transmit(&p_i2c->hi2c, dev_addr, data, size, HAL_MAX_DELAY);
                        if(res != HAL_OK)
                        {
                            p_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;
                            p_i2c->hi2c.State = HAL_I2C_STATE_READY;                      
                                        
                            return UVX_I2C_ERROR; // Return error if transmission fails
                        }  
                    }
                    else
                    {
                        return UVX_I2C_BUSY;
                    }
    
                }
                else
                {
                    p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
                    return UVX_I2C_BUSY; // Return error if transmission fails
                }                          
            }
            else
            {
                p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

                if( p_i2c->hi2c.Instance->ISR & I2C_ISR_BUSY )
                {
                    return UVX_I2C_BUSY;
                }

                return UVX_I2C_ERROR; // Return error if transmission fails
            }   
		}
		else
		{
			return UVX_I2C_INIT_ERROR; // Return error if the I2C handle is not initialized
		}

	#else
		#error "Unsupported STM32 series"
	#endif

	return UVX_I2C_OK; // Return success
}

UVX_I2C_STATE uvx_i2c_send(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint8_t* data, uint16_t size)  
{
    HAL_StatusTypeDef res = HAL_ERROR;
	#if defined(STM32L4xx_HAL_H)
		// Check if the I2C handle is initialized
		if (p_i2c != NULL && p_i2c->hi2c.Instance != NULL && data != NULL)
		{
            if((p_i2c->lock == 0U) ||
               (p_i2c->transfer_state != UVX_I2C_TRANSFER_IDLE))
            {
                return UVX_I2C_BUSY;
            }
            if((size == 0U) || (size > UVX_I2C_TX_IT_BUFFER_SIZE))
            {
                return UVX_I2C_ERROR;
            }
            
            if( !( p_i2c->hi2c.Instance->ISR & I2C_ISR_STOPF ) && 
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_NACKF ) &&
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_BUSY ) )
            {
                if( ( ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) ) ||                 
                    ( ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) &&
                      ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXIS ) ) )
                {
                    if(p_i2c->TX_Ready)
                    {
                        p_i2c->TX_Ready = 0;

                        dev_addr <<= 1; // Shift address            
                        //res = HAL_I2C_Mem_Write_IT(&p_i2c->hi2c, dev_addr, reg_addr, reg_size, data, size); 
                        //res = HAL_I2C_Master_Transmit(&p_i2c->hi2c, dev_addr, data, size, HAL_MAX_DELAY);
                        memcpy(p_i2c->tx_it_buffer, data, size);
                        p_i2c->transfer_state = UVX_I2C_TRANSFER_PENDING;
                        res = HAL_I2C_Master_Transmit_IT(&p_i2c->hi2c, dev_addr, p_i2c->tx_it_buffer, size);
                        if(res != HAL_OK)
                        {
                            p_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;
                            p_i2c->hi2c.State = HAL_I2C_STATE_READY;                      
                                        
                            return UVX_I2C_ERROR; // Return error if transmission fails
                        }
                    }
                    else
                    {
                        return UVX_I2C_BUSY;
                    }                      
                }
                else
                {
                    p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
                    return UVX_I2C_BUSY; // Return error if transmission fails
                }                          
            }
            else
            {
                p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

                if( p_i2c->hi2c.Instance->ISR & I2C_ISR_BUSY )
                {
                    return UVX_I2C_BUSY;
                }

                return UVX_I2C_ERROR; // Return error if transmission fails
            }   
		}
		else
		{
			return UVX_I2C_INIT_ERROR; // Return error if the I2C handle is not initialized
		}

	#else
		#error "Unsupported STM32 series"
	#endif

	return UVX_I2C_OK; // Return success
}

UVX_I2C_STATE uvx_i2c_read_mem(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint16_t reg_addr, uint16_t reg_size, uint8_t* data, uint16_t size) 
{
	#if defined(STM32L4xx_HAL_H)
		// Check if the I2C handle is initialized
		if (p_i2c != NULL && p_i2c->hi2c.Instance != NULL)
		{
            if((p_i2c->lock == 0U) ||
               (p_i2c->transfer_state != UVX_I2C_TRANSFER_IDLE))
            {
                return UVX_I2C_BUSY;
            }
            if(!( p_i2c->hi2c.Instance->ISR & I2C_ISR_STOPF ) && 
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_NACKF ) &&
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
            {
                if(p_i2c->RX_Ready)
                {
                    p_i2c->RX_Ready = 0;
                    dev_addr <<= 1; // Shift address
                    p_i2c->transfer_state = UVX_I2C_TRANSFER_PENDING;
                    if(HAL_I2C_Mem_Read_IT(&p_i2c->hi2c, dev_addr, reg_addr, reg_size, data, size) != HAL_OK)
                    {
                        p_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;
                        if( !(p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
                        {
                            p_i2c->hi2c.State = HAL_I2C_STATE_READY;
                        }  

                        return UVX_I2C_ERROR; // Return error if transmission fails
                    }                    
                }
                else
                {
                    return UVX_I2C_BUSY;
                }
            }
            else
            {
                if( p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE )
                {
                    uint32_t rx_data = 0;
                    rx_data = p_i2c->hi2c.Instance->RXDR;
                    (void)rx_data;

                    if( !(p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
                    {
                         p_i2c->hi2c.State = HAL_I2C_STATE_READY;
                    }                                          
                }

                if((p_i2c->hi2c.Instance->ISR & I2C_ISR_BERR ) )
                {
                        p_i2c->hi2c.Instance->ICR = I2C_ICR_BERRCF;
                }      
                
                if((p_i2c->hi2c.Instance->ISR & I2C_ISR_ARLO ) )
                {
                        p_i2c->hi2c.Instance->ICR = I2C_ICR_ARLOCF;
                }              

                p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

                HAL_I2C_Master_Abort_IT(&p_i2c->hi2c, dev_addr);
                return UVX_I2C_BUSY; // Return error if transmission fails
            }

		}
		else
		{
			return UVX_I2C_INIT_ERROR; // Return error if the I2C handle is not initialized
		}

	#else
		#error "Unsupported STM32 series"
	#endif

	return UVX_I2C_OK; // Return success
}

UVX_I2C_STATE uvx_i2c_read(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint8_t* data, uint16_t size) 
{
	#if defined(STM32L4xx_HAL_H)
		// Check if the I2C handle is initialized
		if (p_i2c != NULL && p_i2c->hi2c.Instance != NULL)
		{
            if((p_i2c->lock == 0U) ||
               (p_i2c->transfer_state != UVX_I2C_TRANSFER_IDLE))
            {
                return UVX_I2C_BUSY;
            }
            if(!( p_i2c->hi2c.Instance->ISR & I2C_ISR_STOPF ) && 
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_NACKF ) &&
                !( p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
            {
                if(p_i2c->RX_Ready)
                {
                    p_i2c->RX_Ready = 0;
                    dev_addr <<= 1; // Shift address
                    p_i2c->transfer_state = UVX_I2C_TRANSFER_PENDING;
                    if(HAL_I2C_Master_Receive_IT(&p_i2c->hi2c, dev_addr, data, size) != HAL_OK)
                    {
                        p_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;
                        if( !(p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
                        {
                            p_i2c->hi2c.State = HAL_I2C_STATE_READY;
                        }  

                        return UVX_I2C_ERROR; // Return error if transmission fails
                    }                    
                }
                else
                {
                    return UVX_I2C_BUSY;
                }
            }
            else
            {
                if( p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE )
                {
                    uint32_t rx_data = 0;
                    rx_data = p_i2c->hi2c.Instance->RXDR;
                    (void)rx_data;

                    if( !(p_i2c->hi2c.Instance->ISR & I2C_ISR_RXNE ) )
                    {
                         p_i2c->hi2c.State = HAL_I2C_STATE_READY;
                    }                   
                }

                p_i2c->hi2c.Instance->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
                return UVX_I2C_BUSY; // Return error if transmission fails
            }

		}
		else
		{
			return UVX_I2C_INIT_ERROR; // Return error if the I2C handle is not initialized
		}

	#else
		#error "Unsupported STM32 series"
	#endif

	return UVX_I2C_OK; // Return success
}


UVX_I2C_STATE uvx_i2c_check_state(UVX_I2C_HAL* p_i2c)
{
    #if defined(STM32L4xx_HAL_H)
        // Check if the I2C handle is initialized
        if (p_i2c != NULL && p_i2c->hi2c.Instance != NULL)
        {
            if( !( p_i2c->hi2c.Instance->ISR & I2C_ISR_BUSY ) )
            {
                if( ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) && 
                    ( p_i2c->hi2c.Instance->ISR & I2C_ISR_STOPF ) || 
                    ( p_i2c->hi2c.Instance->ISR & I2C_ISR_TXE ) )
                {
                    if( !(p_i2c->hi2c.Instance->ISR & I2C_ISR_NACKF ) )
                    {
                        p_i2c->hi2c.Instance->ICR = I2C_ICR_NACKCF;
                    }
                    
                    return UVX_I2C_TX_READY;
                }
                else
                {
                    return UVX_I2C_BUSY;
                }
            }

        }

    #else
        #error "Unsupported STM32 series"
    #endif

    return UVX_I2C_ERROR; // Return error
}

UVX_I2C_STATE uvx_i2c_lock(UVX_I2C_HAL* p_i2c)
{
    uint32_t primask;

    if(p_i2c == NULL)
    {
        return UVX_I2C_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if((p_i2c->lock != 0U) ||
       (p_i2c->transfer_state != UVX_I2C_TRANSFER_IDLE))
    {
        if(primask == 0U)
        {
            __enable_irq();
        }
        return UVX_I2C_BUSY;
    }

    p_i2c->lock = 1U;
    p_i2c->transfer_error = HAL_I2C_ERROR_NONE;
    /*
     * transfer_state is the authoritative asynchronous state.  When an idle
     * peripheral is successfully locked there cannot be an outstanding RX or
     * TX operation, so repair any stale legacy readiness flags here.
     */
    p_i2c->RX_Ready = 1U;
    p_i2c->TX_Ready = 1U;

    if(primask == 0U)
    {
        __enable_irq();
    }

    return UVX_I2C_OK;
}

UVX_I2C_STATE uvx_i2c_unlock(UVX_I2C_HAL* p_i2c)
{
    uint32_t primask;

    if((p_i2c == NULL) || (p_i2c->lock == 0U))
    {
        return UVX_I2C_ERROR;
    }

    if(p_i2c->transfer_state == UVX_I2C_TRANSFER_PENDING)
    {
        return UVX_I2C_BUSY;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    p_i2c->lock = 0U;
    p_i2c->transfer_state = UVX_I2C_TRANSFER_IDLE;

    if(primask == 0U)
    {
        __enable_irq();
    }

    return UVX_I2C_OK;
}

UVX_I2C_TRANSFER_STATE uvx_i2c_get_transfer_state(UVX_I2C_HAL* p_i2c)
{
    if((p_i2c == NULL) || (p_i2c->lock == 0U))
    {
        return UVX_I2C_TRANSFER_IDLE;
    }

    return p_i2c->transfer_state;
}

void uvx_i2c_transfer_complete_callback(UVX_I2C_HAL* p_i2c)
{
    if((p_i2c != NULL) &&
       (p_i2c->lock != 0U) &&
       (p_i2c->transfer_state == UVX_I2C_TRANSFER_PENDING))
    {
        p_i2c->transfer_error = HAL_I2C_ERROR_NONE;
        p_i2c->transfer_state = UVX_I2C_TRANSFER_COMPLETE;
        p_i2c->TX_Ready = 1U;
        p_i2c->RX_Ready = 1U;
    }
}

void uvx_i2c_transfer_error_callback(UVX_I2C_HAL* p_i2c)
{
    if((p_i2c != NULL) &&
       (p_i2c->lock != 0U) &&
       (p_i2c->transfer_state == UVX_I2C_TRANSFER_PENDING))
    {
        p_i2c->transfer_error = p_i2c->hi2c.ErrorCode;
        p_i2c->transfer_state = UVX_I2C_TRANSFER_ERROR;
        p_i2c->TX_Ready = 1U;
        p_i2c->RX_Ready = 1U;
    }
}
