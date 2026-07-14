
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_I2C
#define __UVX_I2C


#include "stm32l4xx_hal.h"
#include "uvx_gpio.h"

#ifndef UVX_I2C_TX_IT_BUFFER_SIZE
#define UVX_I2C_TX_IT_BUFFER_SIZE 32U
#endif


typedef enum
{
    UVX_I2C_OK = 0,
    UVX_I2C_ERROR, 
    UVX_I2C_BUSY,
    UVX_I2C_TIMEOUT,
    UVX_I2C_INIT_ERROR,
    UVX_I2C_INIT_ERROR_TX_MEMORY,
    UVX_I2C_INIT_ERROR_RX_MEMORY,
    UVX_I2C_INIT_ERROR_TX_BUFF_SIZE,
    UVX_I2C_INIT_ERROR_RX_BUFF_SIZE,
    UVX_I2C_INIT_ERROR_GPIO,
    UVX_I2C_INIT_ERROR_DMA,
    UVX_I2C_TX_READY,
    UVX_I2C_RX_READY,
    UVX_I2C_ALREADY_INITIALIZED
} UVX_I2C_STATE;

typedef struct UVX_I2C_HAL
{
    I2C_HandleTypeDef   hi2c;
    DMA_HandleTypeDef   hdma_tx;
    DMA_HandleTypeDef   hdma_rx;

    IRQn_Type   i2c_interrupt_ev;  // Interrupt event line
    uint8_t     i2c_interrupt_ev_priority; // Interrupt priority
    uint8_t     i2c_interrupt_ev_subpriority; // Interrupt subpriority
    IRQn_Type   i2c_interrupt_err;  // Interrupt error line
    uint8_t     i2c_interrupt_err_priority; // Interrupt priority
    uint8_t     i2c_interrupt_err_subpriority; // Interrupt subpriority

    uint8_t dma_priority_rx; // DMA RX interrupt priority
    uint8_t dma_subpriority_rx; // DMA RX interrupt subpriority
    uint8_t dma_priority_tx; // DMA TX interrupt priority
    uint8_t dma_subpriority_tx; // DMA TX interrupt subpriority
    IRQn_Type dma_interrupt_line_tx; // DMA interrupt line
    IRQn_Type dma_interrupt_line_rx; // DMA interrupt line

    uint32_t error_dma_cnt; // DMA error counter
    uint8_t tx_it_buffer[UVX_I2C_TX_IT_BUFFER_SIZE];
    //interrupt event line
    uint8_t i2c_interrupt_rx 	    : 1; // Flag to indicate if RX interrupt is enabled
    uint8_t i2c_interrupt_tx 	    : 1; // Flag to indicate if TX interrupt is enabled
    uint8_t i2c_interrupt_tc 	    : 1; // Flag to indicate if TC interrupt is enabled
    //interrupt error line 
    uint8_t i2c_interrupt_berr 	    : 1; // Flag to indicate if buss error interrupt is enabled
    uint8_t i2c_interrupt_arlo 	    : 1; // Flag to indicate if arbitration lost interrupt is enabled
    uint8_t i2c_interrupt_ovr 	    : 1; // Flag to indicate if overrun/underrun interrupt is enabled
    uint8_t i2c_interrupt_pecerr 	: 1; // Flag to indicate if PEC Error in reception interrupt is enabled
    uint8_t i2c_interrupt_timeout 	: 1; // Flag to indicate if timeout interrupt is enabled
    uint8_t i2c_interrupt_alert 	: 1; // Flag to indicate if alert interrupt is enabled
    //interrupt dma lines
    uint8_t dma_rx_enabled 	        : 1; // Flag to indicate if DMA is enabled
    uint8_t dma_tx_enabled 	        : 1; // Flag to indicate if DMA is enabled
    uint8_t dma_interrupt_rx 	    : 1; // Flag to indicate if RX DMA interrupt is enabled
    uint8_t dma_interrupt_tx 	    : 1; // Flag to indicate if TX DMA interrupt is enabled

    uint8_t RX_Ready 	            : 1; // RX byte ready
    uint8_t TX_Ready 	            : 1; // TX byte ready    

}UVX_I2C_HAL;

typedef struct UVX_I2C
{
	uint8_t 		        ID;		  // I2C ID
    UVX_I2C_HAL             hal_i2c; // HAL I2C structure
    UVX_GPIO                gpio_sda; // GPIO structure
    UVX_GPIO                gpio_scl; // GPIO structure

    uint8_t Enable 		    : 1; // Flag to indicate if I2C is enabled
    uint8_t is_Initilized 	: 1; // Flag to indicate if HAL timer is initialied
    uint8_t Error 		    : 1; // Error flag
    // uint8_t RX_Ready 	    : 1; // RX byte ready
    // uint8_t TX_Ready 	    : 1; // TX byte ready
    uint8_t TX_Ready_Buffer : 1; // TX buffer ready
    uint8_t RX_Ready_Buffer : 1; // RX buffer ready        
    uint8_t Reserve 	    : 1; // Reserved for future use

    uint32_t                cnt_error_busy;
    uint16_t                buff_size_rx; // RX buffer size
    uint16_t                buff_size_tx; // TX buffer size
    uint8_t*                p_buff_tx_start; // Pointer to the start of the TX buffer
    uint8_t*                p_buff_rx_start; // Pointer to the start of the RX buffer
    uint8_t*                p_buff_tx; // Pointer to the TX buffer
    uint8_t*                p_buff_rx; // Pointer to the RX buffer  
    uint8_t                 byte_rx;
}UVX_I2C;

#define UVX_SETUP_I2C_1 ((UVX_I2C)\
{\
    .ID = 0,\
    .hal_i2c = {\
        .hi2c = {\
            .Instance = I2C1,\
            .Init.Timing = 0x1042C7C7,\
            .Init.OwnAddress1 = 0,\
            .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,\
            .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,\
            .Init.OwnAddress2 = 0,\
            .Init.OwnAddress2Masks = I2C_OA2_NOMASK,\
            .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,\
            .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE,\
            },\
        .hdma_tx = {\
            .Instance = DMA1_Channel4,\
        },\
        .hdma_rx = {\
            .Instance = DMA1_Channel3,\
        },\
        .i2c_interrupt_ev = I2C1_EV_IRQn,\
        .i2c_interrupt_ev_priority = 12,\
        .i2c_interrupt_ev_subpriority = 0,\
        .i2c_interrupt_err = I2C1_ER_IRQn,\
        .i2c_interrupt_err_priority = 13,\
        .i2c_interrupt_err_subpriority = 0,\
        .i2c_interrupt_rx = true,\
        .i2c_interrupt_tx = true,\
        .i2c_interrupt_tc = true,\
        .i2c_interrupt_berr = true,\
        .i2c_interrupt_arlo = true,\
        .i2c_interrupt_ovr = true,\
        .i2c_interrupt_pecerr = true,\
        .i2c_interrupt_timeout = true,\
        .i2c_interrupt_alert = true,\
        .dma_interrupt_line_tx = DMA1_Channel4_IRQn,\
        .dma_priority_tx = 1,\
        .dma_subpriority_tx = 0,\
        .dma_tx_enabled = false,\
        .dma_rx_enabled = false,\
        .dma_interrupt_tx = false,\
        .dma_interrupt_rx = false,\
    },\
    .gpio_sda = {\
        .Port = 'B',\
        .Pin = 9,\
        .Mode = GPIO_MODE_AF_OD,\
        .Pull = GPIO_PULLUP,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF4_I2C1,\
    },\
    .gpio_scl = {\
        .Port = 'B',\
        .Pin = 8,\
        .Mode = GPIO_MODE_AF_OD,\
        .Pull = GPIO_PULLUP,\
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,\
        .Alternate = GPIO_AF4_I2C1,\
    },\
    .Enable = true,\
    .is_Initilized = false,\
    .Error = false,\
    .RX_Ready_Buffer = false,\
    .TX_Ready_Buffer = false,\
    .buff_size_tx = BUFF_SIZE_TX_M2M,\
    .buff_size_rx = BUFF_SIZE_RX_M2M,\
})

#define UVX_SETUP_I2C_2 ((UVX_I2C)\
{\
    .ID = 1,\
    .baudrate = 576000,\
    .hal_i2c = {\
        .hi2c = {\
            .Instance = USART2,\
            .Init.BaudRate = 576000,\
            .Init.WordLength = I2C_WORDLENGTH_8B,\
            .Init.StopBits = I2C_STOPBITS_1,\
            .Init.Parity = I2C_PARITY_NONE,\
            .Init.Mode = I2C_MODE_TX_RX,\
            .Init.HwFlowCtl = I2C_HWCONTROL_NONE,\
            .Init.OverSampling = I2C_OVERSAMPLING_8,\
            .Init.OneBitSampling = I2C_ONE_BIT_SAMPLE_DISABLE,\
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
        .i2c_interrupt_rx = true,\
        .i2c_interrupt_tx = false,\
        .i2c_interrupt_ev = I2C1_EV_IRQn,\
        .i2c_interrupt_line_priority = 12,\
        .i2c_interrupt_line_subpriority = 0,\
        .i2c_interrupt_err = I2C1_ER_IRQn,\
        .i2c_interrupt_err_priority = 13,\
        .i2c_interrupt_err_subpriority = 0,\
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

UVX_I2C_STATE uvx_i2c_init(UVX_I2C* i2c/*, uint8_t* p_buff_tx, uint8_t* p_buff_rx*/);
UVX_I2C_STATE uvx_i2c_send_mem(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint16_t reg_addr, uint16_t reg_size, uint8_t* data, uint16_t size);
UVX_I2C_STATE uvx_i2c_send(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint8_t* data, uint16_t size);
UVX_I2C_STATE uvx_i2c_read_mem(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint16_t reg_addr, uint16_t reg_size, uint8_t* data, uint16_t size);
UVX_I2C_STATE uvx_i2c_read(UVX_I2C_HAL* p_i2c, uint8_t dev_addr, uint8_t* data, uint16_t size);
UVX_I2C_STATE uvx_i2c_check_state(UVX_I2C_HAL* p_i2c);


#define I2C_START_BYTE								0x0F
#define BATT_I2C_BAUDRATE							19200
#define ESC_I2C_BAUDRATE							38400

static __inline	void i2c_TXE_Enable( USART_TypeDef* i2c );
static __inline	void i2c_RXNE_Enable( USART_TypeDef* i2c );
static __inline	void i2c_TXE_Disable( USART_TypeDef* i2c );
static __inline	void i2c_RXNE_Disable( USART_TypeDef* i2c );

// enable the USART transmitter empty interrupt
static __inline	void i2c_TXE_Enable( USART_TypeDef* i2c )
{
	i2c->CR1 |= USART_CR1_TXEIE;
}

// enable the USART receiver not empty interrupt
static __inline	void i2c_RXNE_Enable( USART_TypeDef* i2c )
{
    i2c->CR1 |= USART_CR1_RXNEIE;	
}

// disable the USART transmitter empty interrupt
static __inline	void i2c_TXE_Disable( USART_TypeDef* i2c )
{
	i2c->CR1 &= ~USART_CR1_TXEIE;
}

// disable the USART receiver not empty interrupt
static __inline	void i2c_RXNE_Disable( USART_TypeDef* i2c )
{
    i2c->CR1 &= ~USART_CR1_RXNEIE;	
}

#endif
