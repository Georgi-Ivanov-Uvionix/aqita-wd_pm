/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_SPI_H
#define __UVX_SPI_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "uvx_gpio.h"

/**
 * @brief SPI state enumeration
 */
typedef enum
{
    UVX_SPI_OK = 0,
    UVX_SPI_ERROR, 
    UVX_SPI_BUSY,
    UVX_SPI_TIMEOUT,
    UVX_SPI_INIT_ERROR,
    UVX_SPI_INIT_ERROR_TX_MEMORY,
    UVX_SPI_INIT_ERROR_RX_MEMORY,
    UVX_SPI_INIT_ERROR_TX_BUFF_SIZE,
    UVX_SPI_INIT_ERROR_RX_BUFF_SIZE,
    UVX_SPI_INIT_ERROR_GPIO,
    UVX_SPI_INIT_ERROR_DMA,
    UVX_SPI_ALREADY_INITIALIZED
} UVX_SPI_STATE;

/**
 * @brief HAL layer structure for SPI peripheral configuration
 */
typedef struct UVX_SPI_HAL
{
    SPI_HandleTypeDef  hspi;
    DMA_HandleTypeDef   hdma_tx;
    DMA_HandleTypeDef   hdma_rx;

    uint8_t     spi_priority_rx         ; // Interrupt priority
    uint8_t     spi_subpriority_rx      ; // Interrupt subpriority
    uint8_t     spi_priority_tx         ; // Interrupt priority
    uint8_t     spi_subpriority_tx      ; // Interrupt subpriority
    IRQn_Type   spi_interrupt_line      ; // Interrupt line
    uint8_t     dma_priority_rx         ; // DMA RX interrupt priority
    uint8_t     dma_subpriority_rx      ; // DMA RX interrupt subpriority
    uint8_t     dma_priority_tx         ; // DMA TX interrupt priority
    uint8_t     dma_subpriority_tx      ; // DMA TX interrupt subpriority
    IRQn_Type   dma_interrupt_line_tx   ; // DMA interrupt line
    IRQn_Type   dma_interrupt_line_rx   ; // DMA interrupt line

    uint32_t    error_dma_cnt           ; // DMA error counter

    uint8_t     spi_interrupt_rx 	: 1; // Flag to indicate if RX interrupt is enabled
    uint8_t     spi_interrupt_tx 	: 1; // Flag to indicate if TX interrupt is enabled
    uint8_t     dma_rx_enabled 	    : 1; // Flag to indicate if DMA is enabled
    uint8_t     dma_tx_enabled 	    : 1; // Flag to indicate if DMA is enabled
    uint8_t     dma_interrupt_rx 	: 1; // Flag to indicate if RX DMA interrupt is enabled
    uint8_t     dma_interrupt_tx 	: 1; // Flag to indicate if TX DMA interrupt is enabled
    uint8_t     Reserved 		    : 2; // Reserved for future use

} UVX_SPI_HAL;

/**
 * @brief Main SPI driver structure
 */
typedef struct UVX_SPI
{
    uint8_t             ID;          // SPI ID
    
    UVX_SPI_HAL         hal_spi;     // HAL SPI structure
    UVX_GPIO            gpio_sck;    // SCK GPIO pin
    UVX_GPIO            gpio_mosi;   // MOSI GPIO pin
    
    uint8_t Enable 		        : 1; // Flag to indicate if UART is enabled
    uint8_t is_Initialized 	    : 1; // Flag to indicate if HAL timer is initialied
    volatile uint8_t Error 	    : 1; // Error flag
    volatile uint8_t RX_Ready   : 1; // RX byte ready
    volatile uint8_t TX_Ready   : 1; // TX byte ready
    uint8_t TX_Ready_Buffer     : 1; // TX buffer ready
    uint8_t RX_Ready_Buffer     : 1; // RX buffer ready        
    uint8_t Reserve 	        : 1; // Reserved for future use

    uint16_t                buff_size_rx; // RX buffer size
    uint16_t                buff_size_tx; // TX buffer size
    uint8_t*                p_buff_tx_start; // Pointer to the start of the TX buffer
    uint8_t*                p_buff_rx_start; // Pointer to the start of the RX buffer
    uint8_t*                p_buff_tx; // Pointer to the TX buffer
    uint8_t*                p_buff_rx; // Pointer to the RX buffer  
    uint8_t                 byte_rx;

} UVX_SPI;

/**
 * @brief Predefined macro for easy SPI1 configuration
 * SPI1 Master mode, Half-Duplex (1-line), 8-bit data
 * Baudrate: 5Mbps (prescaler /16 @ 80MHz)
 * CPOL: Low, CPHA: 1 Edge
 * MSB First, No CRC
 * GPIO: SCK=PA5, MOSI=PA7
 */
#define UVX_SETUP_SPI_1 ((UVX_SPI)\
{\
    .ID = 0,\
    .hal_spi = {\
        .hspi = {\
            .Instance = SPI1,\
            .Init.Mode = SPI_MODE_MASTER,\
            .Init.Direction = SPI_DIRECTION_1LINE,\
            .Init.DataSize = SPI_DATASIZE_8BIT,\
            .Init.CLKPolarity = SPI_POLARITY_LOW,\
            .Init.CLKPhase = SPI_PHASE_1EDGE,\
            .Init.NSS = SPI_NSS_SOFT,\
            .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,\
            .Init.FirstBit = SPI_FIRSTBIT_MSB,\
            .Init.TIMode = SPI_TIMODE_DISABLE,\
            .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,\
        },\
        .hdma_tx = {\
            .Instance = DMA1_Channel3,\
            .Init.Request = DMA_REQUEST_1,\
            .Init.Direction = DMA_MEMORY_TO_PERIPH,\
            .Init.PeriphInc = DMA_PINC_DISABLE,\
            .Init.MemInc = DMA_MINC_ENABLE,\
            .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,\
            .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,\
            .Init.Mode = DMA_NORMAL,\
            .Init.Priority = DMA_PRIORITY_HIGH,\
        },\
        .hdma_rx = {\
            .Instance = DMA1_Channel2,\
        },\
        .dma_interrupt_line_tx = DMA1_Channel3_IRQn,\
        .dma_priority_tx = 2,\
        .dma_subpriority_tx = 0,\
        .spi_interrupt_rx = false,\
        .spi_interrupt_tx = false,\
        .spi_priority_rx = 0,\
        .spi_subpriority_rx = 0,\
        .spi_priority_tx = 5,\
        .spi_subpriority_tx = 0,\
        .spi_interrupt_line = SPI1_IRQn,\
        .dma_tx_enabled = true,\
        .dma_rx_enabled = false,\
        .dma_interrupt_tx = true,\
        .dma_interrupt_rx = false,\
    },\
    .gpio_sck = {\
        .Port = 'A',\
        .Pin = 5,\
        .Mode = GPIO_MODE_OUTPUT_PP,\
        .Pull = GPIO_PULLDOWN,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF5_SPI1,\
    },\
    .gpio_mosi = {\
        .Port = 'A',\
        .Pin = 7,\
        .Mode = GPIO_MODE_AF_PP,\
        .Pull = GPIO_NOPULL,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF5_SPI1,\
    },\
    .Enable = true,\
    .is_Initialized = false,\
    .Error = false,\
    .RX_Ready_Buffer = false,\
    .TX_Ready_Buffer = false,\
    .buff_size_tx = WS2812_BUFFER_SIZE,\
    .buff_size_rx = 0,\
})
// DMA2_Stream2
/**
 * @brief Initializes the SPI peripheral and its associated GPIO pins
 * @param spi Pointer to the UVX_SPI structure containing SPI configuration
 * @retval UVX_SPI_STATE Status of the initialization process
 */
UVX_SPI_STATE uvx_spi_init(UVX_SPI* spi, uint8_t* p_buff_tx, uint8_t* p_buff_rx);

/**
 * @brief Transmits data via SPI using interrupt-driven mode
 * @param spi Pointer to the UVX_SPI structure
 * @param data Pointer to data buffer to transmit
 * @param size Number of bytes to transmit
 * @retval UVX_SPI_STATE Status of the transmission
 */
UVX_SPI_STATE uvx_spi_transmit(UVX_SPI* spi, uint8_t* data, uint16_t size);

/**
 * @brief Callback function called when TX transmission is complete
 * Should be called from HAL_SPI_TxCpltCallback()
 * @param spi Pointer to the UVX_SPI structure
 */
void uvx_spi_tx_complete_callback(UVX_SPI* spi);

/**
 * @brief Callback function called when SPI error occurs
 * Should be called from HAL_SPI_ErrorCallback()
 * @param spi Pointer to the UVX_SPI structure
 */
void uvx_spi_error_callback(UVX_SPI* spi);

#endif /* __UVX_SPI_H */
