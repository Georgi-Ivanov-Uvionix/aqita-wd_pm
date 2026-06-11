
/**
  ******************************************************************************
  * @file    uvx_comm_m2m.h
  * @brief   Header file for microcontroller-to-microcontroller (M2M) communication.
  *
  * This file defines constants and includes required headers for implementing
  * a communication protocol between two microcontrollers. The protocol uses
  * a start byte, control codes, and a maximum packet size for data exchange.
  *
  * - UVX_START_BYTE: Start-of-packet marker for framing.
  * - UVX_CTRL_READ:  Example control code for read operations.
  * - UVX_MAX_PACKET: Maximum allowed packet size in bytes.
  *
  *   * Protocol Packet Structure:
  * ----------------------------------------------------------------------------
  * | Byte Index |    Name        | Size |         Description                 |
  * |------------|--------------- |------|-------------------------------------|
  * |    0       | Start Byte     |  1   | Fixed value to mark packet start    |
  * |            |                |      | (0x02)                              |
  * |    1       | Control Byte   |  1   | Command type:                       |
  * |            |                |      |   0x00 = Write                      |
  * |            |                |      |   0x01 = Read                       |
  * |            |                |      |   0x02 = Send                       |
  * |            |                |      |   0x03 = Send Error                 |
  * |    2       | Command Byte   |  1   | Specifies the action or register    |
  * |  3 - 4     | Payload Size   |  2   | Payload size (number of bytes)      |
  * |  5 ...     | Payload        |  N   | Optional data depending on command  |
  * | 5+N        | CRC Byte       |  1   | XOR or CRC-8 over all previous bytes|
  * ----------------------------------------------------------------------------
  *
  ******************************************************************************
  */
#ifndef __UVX_COMM_M2M_H
#define __UVX_COMM_M2M_H

/* Includes ------------------------------------------------------------------*/
#include "uvx_universal_types.h"
#include <stdint.h>
#include "main.h"

#define BUFF_SIZE_TX_M2M			100
#define BUFF_SIZE_RX_M2M			100
#define BUFF_SIZE_TX_JMB      100
#define BUFF_SIZE_RX_JMB      100
#define BUFF_SIZE_TX_ESC      250
#define BUFF_SIZE_RX_ESC      100

#define UVX_PACKET_SIZE     1000  // Maximum packet size
#define UVX_START_BYTE      0x02  // Start byte for framing packets

#define M2M_SENT_DATA_PACKET_SIZE  (82)	// Number of data bytes to send over the UART interface

#define M2M_BYTE_CTRL       1 // Control byte index in the packet
#define M2M_BYTE_CMD        2 // Command byte index in the packet
#define M2M_BYTE_SIZE_MSB   3 // Payload size MSB index in the packet
#define M2M_BYTE_SIZE_LSB   4 // Payload size LSB index in the packet
#define M2M_BYTE_DATA       5 // Data byte index in the packet
#define M2M_BYTE_DATA_FLAGS 81 // Data byte index in the packet

#define M2M_DATA_FLAGS_INIT               0x01 // Bit mask for init status in the data flags
#define M2M_DATA_FLAGS_ARM_BIT            0x02 // Bit mask for arm status in the data flags
#define M2M_DATA_FLAGS_LAND_COMPLETE      0x04 // Bit mask for land complete status in the data flags
#define M2M_DATA_FLAGS_FLYING             0x08 // Bit mask for flying status in the data flags
#define M2M_DATA_FLAGS_PSYS_ARM           0x80 // Bit mask for powered system arm status in the data flags

typedef struct UVX_COMM_M2M
{
	  uint8_t 		            ID;		  // UART ID 
    UVX_UART_HAL*           p_hal_uart; // HAL UART structure

    uint8_t Enable 		      : 1; // Flag to indicate if UART is enabled
    uint8_t is_Initilized 	: 1; // Flag to indicate if HAL timer is initialied
    uint8_t Error 		      : 1; // Error flag
    uint8_t RX_Ready 	      : 1; // RX byte ready
    uint8_t TX_Ready 	      : 1; // TX byte ready
    uint8_t TX_Ready_Buffer : 1; // TX buffer ready
    uint8_t RX_Ready_Buffer : 1; // RX buffer ready        
    uint8_t Reserve 	      : 1; // Reserved for future use

    uint16_t                buff_size_rx; // RX buffer size
    uint16_t                buff_size_tx; // TX buffer size
    uint16_t                buff_tx_cnt;  // RX buffer count
    uint16_t                buff_rx_cnt;  // TX buffer count
    uint8_t*                p_buff_tx_start; // Pointer to the start of the TX buffer
    uint8_t*                p_buff_rx_start; // Pointer to the start of the RX buffer
    uint8_t*                p_buff_tx; // Pointer to the TX buffer
    uint8_t*                p_buff_rx; // Pointer to the RX buffer  
    uint16_t                size_frame;
    uint16_t                size_payload;
}UVX_COMM_M2M;

 typedef enum
 {
   UVX_M2M_OK = 0x00,
   UVX_M2M_ERROR,
   UVX_M2M_ERROR_INIT,
   UVX_M2M_ERROR_INIT_UART,
   UVX_M2M_BUSY,
   UVX_M2M_ERROR_UNKNOWN_CMD,
   UVX_M2M_ERROR_CRC,
   UVX_M2M_TIMEOUT
 } UVX_COMM_M2M_STATE;

typedef enum 
{
    M2M_MODE_IDLE = 0x00,
    M2M_MODE_READ_DATA_ESC,
    M2M_MODE_READ_DATA_JMB,
    M2M_MODE_WRITE,    
    M2M_MODE_SEND,
    M2M_MODE_SEND_ERROR,
    M2M_MODE_WAIT_RESPONSE
} UVX_COMM_M2M_MODE;

typedef struct 
{
  UVX_COMM_M2M_MODE state_previous; // Previous state of the M2M communication
  UVX_COMM_M2M_MODE state_current;  // Current state of the M2M communication
  UVX_COMM_M2M_MODE state_next; // Next state of the M2M communication
} UVX_COMM_M2M_STATE_MACHINE;

typedef enum
{
    CMD_READ_DATA_ESC = 0x01, // Read data from ESC
    CMD_READ_DATA_JMB // Read data from JMB
} UVX_M2M_CMD_BYTE;

/* Command types for M2M communication */
typedef enum
{
    CTRL_NONE = 0x00, // No command
    CTRL_READ      ,
    CTRL_WRITE     ,
    CTRL_SEND      ,
    CTRL_SEND_ERROR
} UVX_M2M_CTRL_BYTE;

extern UVX_COMM_M2M comm_m2m; // M2M communication structure
extern UVX_COMM_M2M_STATE_MACHINE comm_m2m_state; // M2M communication state machine

extern uint8_t buff_tx_m2m[BUFF_SIZE_TX_M2M]; // TX buffer for M2M communication
extern uint8_t buff_rx_m2m[BUFF_SIZE_RX_M2M]; // RX buffer for M2M communication

/* Function prototypes */
UVX_COMM_M2M_STATE uvx_comm_m2m_init(UVX_UART* uart_m2m);
UVX_COMM_M2M_STATE uvx_comm_m2m_read(uint8_t data);
UVX_COMM_M2M_STATE uvx_comm_m2m_write(uint8_t data);
UVX_COMM_M2M_STATE uvx_comm_m2m_send(uint8_t cmd_echo, uint8_t* data, uint16_t size);
UVX_COMM_M2M_STATE uvx_comm_m2m_send_error(uint8_t data);
UVX_COMM_M2M_STATE uvx_comm_m2m_process_rx(uint8_t byte_rx);
UVX_COMM_M2M_STATE uvx_comm_m2m_process_rx_data(void);

#endif /* __UVX_COMM_M2M_H */
