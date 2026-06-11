
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_UART
#define __UVX_UART


#include "stm32l4xx_hal.h"
#include "uvx_gpio.h"

#define UVX_UART_MAX_DMA_ERRORS 10 // Maximum number of consecutive DMA errors before resetting the UART

typedef enum
{
    UVX_UART_OK = 0,
    UVX_UART_ERROR, 
    UVX_UART_BUSY,
    UVX_UART_TIMEOUT,
    UVX_UART_INIT_ERROR,
    UVX_UART_INIT_ERROR_TX_MEMORY,
    UVX_UART_INIT_ERROR_RX_MEMORY,
    UVX_UART_INIT_ERROR_TX_BUFF_SIZE,
    UVX_UART_INIT_ERROR_RX_BUFF_SIZE,
    UVX_UART_INIT_ERROR_GPIO,
    UVX_UART_INIT_ERROR_DMA,
    UVX_UART_ALREADY_INITIALIZED
} UVX_UART_STATE;

typedef struct UVX_UART_HAL
{
    UART_HandleTypeDef  huart;
    DMA_HandleTypeDef   hdma_tx;
    DMA_HandleTypeDef   hdma_rx;

    uint8_t     uart_priority_rx; // Interrupt priority
    uint8_t     uart_subpriority_rx; // Interrupt subpriority
    uint8_t     uart_priority_tx; // Interrupt priority
    uint8_t     uart_subpriority_tx; // Interrupt subpriority
    IRQn_Type   uart_interrupt_line; // Interrupt line

    uint8_t dma_priority_rx; // DMA RX interrupt priority
    uint8_t dma_subpriority_rx; // DMA RX interrupt subpriority
    uint8_t dma_priority_tx; // DMA TX interrupt priority
    uint8_t dma_subpriority_tx; // DMA TX interrupt subpriority
    IRQn_Type dma_interrupt_line_tx; // DMA interrupt line
    IRQn_Type dma_interrupt_line_rx; // DMA interrupt line

    uint32_t error_dma_cnt; // DMA error counter
    uint32_t error_cnt; // DMA error counter

    uint8_t uart_interrupt_rx 	: 1; // Flag to indicate if RX interrupt is enabled
    uint8_t uart_interrupt_tx 	: 1; // Flag to indicate if TX interrupt is enabled
    uint8_t dma_rx_enabled 	    : 1; // Flag to indicate if DMA is enabled
    uint8_t dma_tx_enabled 	    : 1; // Flag to indicate if DMA is enabled
    uint8_t dma_interrupt_rx 	: 1; // Flag to indicate if RX DMA interrupt is enabled
    uint8_t dma_interrupt_tx 	: 1; // Flag to indicate if TX DMA interrupt is enabled
    uint8_t tx_ready 		    : 1; //
    uint8_t rx_ready 		    : 1; //

}UVX_UART_HAL;

typedef struct UVX_UART
{
	uint8_t 		        ID;		  // UART ID
    uint32_t                baudrate; // Baudrate

    UVX_UART_HAL            hal_uart; // HAL UART structure
    UVX_GPIO                gpio_rx; // GPIO structure
    UVX_GPIO                gpio_tx; // GPIO structure

    uint8_t Enable 		    : 1; // Flag to indicate if UART is enabled
    uint8_t is_Initilized 	: 1; // Flag to indicate if HAL timer is initialied
    uint8_t Error 		    : 1; // Error flag
    uint8_t RX_Ready 	    : 1; // RX byte ready
    uint8_t TX_Ready 	    : 1; // TX byte ready
    uint8_t TX_Ready_Buffer : 1; // TX buffer ready
    uint8_t RX_Ready_Buffer : 1; // RX buffer ready        
    uint8_t Reserve 	    : 1; // Reserved for future use

    uint16_t                buff_size_rx; // RX buffer size
    uint16_t                buff_size_tx; // TX buffer size
    uint8_t*                p_buff_tx_start; // Pointer to the start of the TX buffer
    uint8_t*                p_buff_rx_start; // Pointer to the start of the RX buffer
    uint8_t*                p_buff_tx; // Pointer to the TX buffer
    uint8_t*                p_buff_rx; // Pointer to the RX buffer  
    uint8_t                 byte_rx;
}UVX_UART;

#define UVX_SETUP_UART_1 ((UVX_UART)\
{\
    .ID = 0,\
    .baudrate = 921600,\
    .hal_uart = {\
        .huart = {\
            .Instance = USART1,\
            .Init.BaudRate = 921600,\
            .Init.WordLength = UART_WORDLENGTH_8B,\
            .Init.StopBits = UART_STOPBITS_1,\
            .Init.Parity = UART_PARITY_NONE,\
            .Init.Mode = UART_MODE_TX_RX,\
            .Init.HwFlowCtl = UART_HWCONTROL_NONE,\
            .Init.OverSampling = UART_OVERSAMPLING_8,\
            .Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE,\
        },\
        .hdma_tx = {\
            .Instance = DMA1_Channel4,\
            .Init.Request = DMA_REQUEST_2,\
            .Init.Direction = DMA_MEMORY_TO_PERIPH,\
            .Init.PeriphInc = DMA_PINC_DISABLE,\
            .Init.MemInc = DMA_MINC_ENABLE,\
            .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,\
            .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,\
            .Init.Mode = DMA_NORMAL,\
            .Init.Priority = DMA_PRIORITY_LOW,\
        },\
        .hdma_rx = {\
            .Instance = DMA1_Channel3,\
        },\
        .dma_interrupt_line_tx = DMA1_Channel4_IRQn,\
        .dma_priority_tx = 1,\
        .dma_subpriority_tx = 0,\
        .uart_interrupt_rx = true,\
        .uart_interrupt_tx = false,\
        .uart_priority_rx = 13,\
        .uart_subpriority_rx = 0,\
        .uart_priority_tx = 12,\
        .uart_subpriority_tx = 0,\
        .uart_interrupt_line = USART1_IRQn,\
        .dma_tx_enabled = true,\
        .dma_rx_enabled = false,\
        .dma_interrupt_tx = true,\
        .dma_interrupt_rx = false,\
    },\
    .gpio_tx = {\
        .Port = 'A',\
        .Pin = 9,\
        .Mode = GPIO_MODE_AF_PP,\
        .Pull = GPIO_NOPULL,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF7_USART1,\
    },\
    .gpio_rx = {\
        .Port = 'A',\
        .Pin = 10,\
        .Mode = GPIO_MODE_AF_PP,\
        .Pull = GPIO_NOPULL,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF7_USART1,\
    },\
    .Enable = true,\
    .is_Initilized = false,\
    .Error = false,\
    .RX_Ready_Buffer = false,\
    .TX_Ready_Buffer = false,\
    .buff_size_tx = BUFF_SIZE_TX_M2M,\
    .buff_size_rx = BUFF_SIZE_RX_M2M,\
})

#define UVX_SETUP_UART_2 ((UVX_UART)\
{\
    .ID = 1,\
    .baudrate = 576000,\
    .hal_uart = {\
        .huart = {\
            .Instance = USART2,\
            .Init.BaudRate = 576000,\
            .Init.WordLength = UART_WORDLENGTH_8B,\
            .Init.StopBits = UART_STOPBITS_1,\
            .Init.Parity = UART_PARITY_NONE,\
            .Init.Mode = UART_MODE_TX_RX,\
            .Init.HwFlowCtl = UART_HWCONTROL_NONE,\
            .Init.OverSampling = UART_OVERSAMPLING_8,\
            .Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE,\
        },\
        .hdma_tx = {\
            .Instance = DMA1_Channel7,\
            .Init.Request = DMA_REQUEST_2,\
            .Init.Direction = DMA_MEMORY_TO_PERIPH,\
            .Init.PeriphInc = DMA_PINC_DISABLE,\
            .Init.MemInc = DMA_MINC_ENABLE,\
            .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,\
            .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,\
            .Init.Mode = DMA_NORMAL,\
            .Init.Priority = DMA_PRIORITY_LOW,\
        },\
        .hdma_rx = {\
            .Instance = DMA1_Channel5,\
        },\
        .dma_interrupt_line_tx = DMA1_Channel7_IRQn,\
        .dma_priority_tx = 1,\
        .dma_subpriority_tx = 0,\
        .uart_interrupt_rx = true,\
        .uart_interrupt_tx = false,\
        .uart_priority_rx = 14,\
        .uart_subpriority_rx = 0,\
        .uart_priority_tx = 14,\
        .uart_subpriority_tx = 0,\
        .uart_interrupt_line = USART2_IRQn,\
        .dma_tx_enabled = true,\
        .dma_rx_enabled = false,\
        .dma_interrupt_tx = true,\
        .dma_interrupt_rx = false,\
    },\
    .gpio_tx = {\
        .Port = 'A',\
        .Pin = 2,\
        .Mode = GPIO_MODE_AF_PP,\
        .Pull = GPIO_NOPULL,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF7_USART2,\
    },\
    .gpio_rx = {\
        .Port = 'A',\
        .Pin = 3,\
        .Mode = GPIO_MODE_AF_PP,\
        .Pull = GPIO_NOPULL,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF7_USART2,\
    },\
    .Enable= true, \
    .is_Initilized = false,\
    .buff_size_tx = BUFF_SIZE_TX_M2JMB,\
    .buff_size_rx = BUFF_SIZE_RX_M2JMB\
})

UVX_UART_STATE uvx_uart_init(UVX_UART* uart, uint8_t* p_buff_tx, uint8_t* p_buff_rx);
UVX_UART_STATE uvx_uart_send(UVX_UART_HAL* p_uart, uint8_t *data, uint16_t size);

//=====================================OLD============================================

#define USART1_TX_FC_RX_Pin 						GPIO_PIN_9
#define USART1_TX_FC_RX_GPIO_Port 					GPIOA
#define USART1_RX_FC_TX_Pin 						GPIO_PIN_10
#define USART1_RX_FC_TX_GPIO_Port 					GPIOA

#define UART4_TX_BATT_RB_RX_Pin 					GPIO_PIN_10
#define UART4_TX_BATT_RB_RX_GPIO_Port 				GPIOC
#define UART4_RX_BATT_RB_TX_Pin 					GPIO_PIN_11
#define UART4_RX_BATT_RB_TX_GPIO_Port 				GPIOC

#define UART5_TX_BATT_RF_RX_Pin 					GPIO_PIN_12
#define UART5_TX_BATT_RF_RX_GPIO_Port 				GPIOC
#define UART5_RX_BATT_RF_TX_Pin 					GPIO_PIN_2
#define UART5_RX_BATT_RF_TX_GPIO_Port 				GPIOD

#define USART2_TX_LF_BATT_RX_Pin 					GPIO_PIN_2
#define USART2_TX_LF_BATT_RX_GPIO_Port 				GPIOA
#define USART2_RX_LF_BATT_TX_Pin 					GPIO_PIN_3
#define USART2_RX_LF_BATT_TX_GPIO_Port 				GPIOA

#define USART3_TX_LB_BATT_RX_Pin 					GPIO_PIN_4
#define USART3_TX_LB_BATT_RX_GPIO_Port 				GPIOC
#define USART3_RX_LB_BATT_TX_Pin 					GPIO_PIN_5
#define USART3_RX_LB_BATT_TX_GPIO_Port 				GPIOC

#define UART_START_BYTE								0x0F
#define BATT_UART_BAUDRATE							19200
#define ESC_UART_BAUDRATE							38400

static __inline	void uart_TXE_Enable( USART_TypeDef* uart );
static __inline	void uart_RXNE_Enable( USART_TypeDef* uart );
static __inline	void uart_TXE_Disable( USART_TypeDef* uart );
static __inline	void uart_RXNE_Disable( USART_TypeDef* uart );

// enable the USART transmitter empty interrupt
static __inline	void uart_TXE_Enable( USART_TypeDef* uart )
{
	uart->CR1 |= USART_CR1_TXEIE;
}

// enable the USART receiver not empty interrupt
static __inline	void uart_RXNE_Enable( USART_TypeDef* uart )
{
    uart->CR1 |= USART_CR1_RXNEIE;	
}

// disable the USART transmitter empty interrupt
static __inline	void uart_TXE_Disable( USART_TypeDef* uart )
{
	uart->CR1 &= ~USART_CR1_TXEIE;
}

// disable the USART receiver not empty interrupt
static __inline	void uart_RXNE_Disable( USART_TypeDef* uart )
{
    uart->CR1 &= ~USART_CR1_RXNEIE;	
}

#endif
