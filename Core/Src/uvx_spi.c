/* Define to prevent recursive inclusion -------------------------------------*/
#include "uvx_spi.h"
#include <stdlib.h>

/**
 * @brief Static function to initialize the HAL SPI peripheral
 * @param hal_spi Pointer to the UVX_SPI_HAL structure
 * @retval UVX_SPI_STATE Status of the HAL initialization
 */
static UVX_SPI_STATE uvx_spi_hal_init(UVX_SPI_HAL* hal_spi)
{
    UVX_SPI_STATE res = UVX_SPI_ERROR;
    
    if(hal_spi)
    {   
        hal_spi->error_dma_cnt = 0;
        //----  HAL DMA SPI INIT -----
        // Enable the DMA clock for TX and RX
        if((hal_spi->dma_tx_enabled) || (hal_spi->dma_rx_enabled))
        {
            if ((hal_spi->hdma_tx.Instance >= DMA1_Channel1) && (hal_spi->hdma_tx.Instance <= DMA1_Channel7))
            {
                __HAL_RCC_DMA1_CLK_ENABLE();
            }
            else if ((hal_spi->hdma_tx.Instance >= DMA2_Channel1) && (hal_spi->hdma_tx.Instance <= DMA2_Channel7))
            {
                __HAL_RCC_DMA2_CLK_ENABLE();
            }
            else
            {
                return UVX_SPI_INIT_ERROR_DMA;
            }

            if(hal_spi->dma_tx_enabled)
            {
                HAL_DMA_Init(&hal_spi->hdma_tx);
                __HAL_LINKDMA(&hal_spi->hspi, hdmatx, hal_spi->hdma_tx);

                if(hal_spi->dma_interrupt_tx)
                {
                    HAL_NVIC_SetPriority((IRQn_Type)hal_spi->dma_interrupt_line_tx, hal_spi->dma_priority_tx, hal_spi->dma_subpriority_tx);
                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_spi->dma_interrupt_line_tx);
                }
            }

            if(hal_spi->dma_rx_enabled)
            {
                HAL_DMA_Init(&hal_spi->hdma_rx);
                __HAL_LINKDMA(&hal_spi->hspi, hdmarx, hal_spi->hdma_rx);

                if(hal_spi->dma_interrupt_rx)
                {
                    HAL_NVIC_SetPriority((IRQn_Type)hal_spi->dma_interrupt_line_rx, hal_spi->dma_priority_rx, hal_spi->dma_subpriority_rx);
                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_spi->dma_interrupt_line_rx);
                }
            }
        }
        
        //---- HAL SPI INIT ------

        #if defined(STM32L4xx_HAL_H)
            // Enable clock for SPI peripheral
            if(hal_spi->hspi.Instance == SPI1)
            {
                __HAL_RCC_SPI1_CLK_ENABLE();
            }
            else if(hal_spi->hspi.Instance == SPI2)
            {
                __HAL_RCC_SPI2_CLK_ENABLE();
            }
            else if(hal_spi->hspi.Instance == SPI3)
            {
                __HAL_RCC_SPI3_CLK_ENABLE();
            }
            else
            {
                return UVX_SPI_INIT_ERROR;
            }
        #else
                #error "Unsupported STM32 series"
        #endif

        if (HAL_SPI_Init(&hal_spi->hspi) == HAL_OK)
        {
            if(hal_spi->spi_interrupt_rx)
            {
                #if defined(STM32L4xx_HAL_H)
                    /* Enable the SPI global Interrupt */
                    HAL_NVIC_SetPriority((IRQn_Type)hal_spi->spi_interrupt_line, hal_spi->spi_priority_rx, hal_spi->spi_subpriority_rx);
                    HAL_NVIC_EnableIRQ((IRQn_Type)hal_spi->spi_interrupt_line);         
                #else
                    #error "Unsupported STM32 series"
                #endif

            }

            // Configure interrupt if enabled
            if(hal_spi->spi_interrupt_tx)
            {   
                 #if defined(STM32L4xx_HAL_H)
                // Set interrupt priority and enable
                HAL_NVIC_SetPriority((IRQn_Type)hal_spi->spi_interrupt_line, 
                                     hal_spi->spi_priority_tx, 
                                     hal_spi->spi_subpriority_tx);
                HAL_NVIC_EnableIRQ((IRQn_Type)hal_spi->spi_interrupt_line);
                #else
                    #error "Unsupported STM32 series"
                #endif
            }
            res = UVX_SPI_OK;
        }
        else
        {
            res = UVX_SPI_INIT_ERROR;
        }
    }
    else
    {
        res = UVX_SPI_ERROR;
    }
    
    return res;
}

/**
 * @brief Initializes the SPI peripheral and its associated GPIO pins
 * @param spi Pointer to the UVX_SPI structure containing SPI configuration
 * @retval UVX_SPI_STATE Status of the initialization process
 */
UVX_SPI_STATE uvx_spi_init(UVX_SPI* spi, uint8_t* p_buff_tx, uint8_t* p_buff_rx)
{
    UVX_SPI_STATE res = UVX_SPI_ERROR;
    
    if(spi)
    {
        // Check if already initialized
        if(spi->is_Initialized == 0)
        {
            // Initialize SCK GPIO pin
            if(uvx_gpio_init(&spi->gpio_sck) != UVX_GPIO_OK)
            {
                return UVX_SPI_INIT_ERROR_GPIO;
            }
            
            // Initialize MOSI GPIO pin
            if(uvx_gpio_init(&spi->gpio_mosi) != UVX_GPIO_OK)
            {
                return UVX_SPI_INIT_ERROR_GPIO;
            }
            
            // Initialize HAL SPI peripheral
            if(uvx_spi_hal_init(&spi->hal_spi) == UVX_SPI_OK)
            {
                // Set initialization flag
                spi->TX_Ready       = 1; // Ready to transmit
                spi->Error          = 0; // No error
                
                if(spi->buff_size_tx > 0)
                {
                    spi->p_buff_tx_start = p_buff_tx;
                    if(spi->p_buff_tx_start == NULL) //
                    {
                        return UVX_SPI_INIT_ERROR_TX_MEMORY;
                    }
                    spi->p_buff_tx = spi->p_buff_tx_start;                    
                }
                else
                {
                    return UVX_SPI_INIT_ERROR_TX_MEMORY;
                }

                /*
                if(spi->buff_size_rx > 0)
                {
                    spi->p_buff_rx_start = p_buff_rx;// Allocate memory for RX buffer
                    if(spi->p_buff_rx_start == NULL)
                    {
                        return UVX_SPI_INIT_ERROR_RX_MEMORY;
                    }
                    spi->p_buff_rx = spi->p_buff_rx_start;
                    HAL_SPI_Receive_IT(&spi->hal_spi.hspi, &spi->byte_rx, 1);
                }
                else
                {
                    return UVX_SPI_INIT_ERROR_RX_MEMORY;
                }*/

                spi->is_Initialized = 1;
                res = UVX_SPI_OK;
            }
            else
            {
                res = UVX_SPI_INIT_ERROR;
            }

            
        }
        else
        {
            res = UVX_SPI_ALREADY_INITIALIZED;
        }
    }
    else
    {
        res = UVX_SPI_ERROR;
    }
    
    return res;
}

/**
 * @brief Transmits data via SPI using interrupt-driven mode
 * @param spi Pointer to the UVX_SPI structure
 * @param data Pointer to data buffer to transmit
 * @param size Number of bytes to transmit
 * @retval UVX_SPI_STATE Status of the transmission
 */
UVX_SPI_STATE uvx_spi_transmit(UVX_SPI* spi, uint8_t* data, uint16_t size)
{
    #if defined(STM32L4xx_HAL_H)
        
        // Check if SPI handle is initialized and data is valid
        if (spi != NULL && spi->is_Initialized != 0 && data != NULL && size > 0)
        {
            // Transmit data using polling mode (works reliably for WS2812)
            if (spi->TX_Ready)
            {   
                spi->TX_Ready = 0;
                __HAL_SPI_ENABLE(&spi->hal_spi.hspi);
                SET_BIT(spi->hal_spi.hspi.Instance->CR1, SPI_CR1_SSI);
                spi->TX_Ready = 0; // Mark as busy
                //HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&spi->hal_spi.hspi, data, size); // (If use IT enable Interrupts for SPI)
                HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&spi->hal_spi.hspi, data, size);
                if(status != HAL_OK)
                {
                    spi->hal_spi.error_dma_cnt++;
                    // Abort transmission if it failed
                    HAL_SPI_Abort(&spi->hal_spi.hspi);
                    return UVX_SPI_ERROR;
                }

                return UVX_SPI_OK;
            }
            else
            {
                return UVX_SPI_BUSY;
            }
        }
        
    #else
        #error "Unsupported STM32 series"
    #endif
    
    return UVX_SPI_INIT_ERROR; 
}

/**
 * @brief Callback function called when TX transmission is complete
 * Should be called from HAL_SPI_TxCpltCallback()
 * @param spi Pointer to the UVX_SPI structure
 */
void uvx_spi_tx_complete_callback(UVX_SPI* spi)
{
    if(spi != NULL)
    {
        // Set complete flag and clear busy flag
        spi->TX_Ready = 1;
        spi->Error = 0;
    }
}

/**
 * @brief Callback function called when SPI error occurs
 * Should be called from HAL_SPI_ErrorCallback()
 * @param spi Pointer to the UVX_SPI structure
 */
void uvx_spi_error_callback(UVX_SPI* spi)
{
    if(spi != NULL)
    {
        // Set error flag and clear busy flag
        spi->Error = 1;
    }
}
