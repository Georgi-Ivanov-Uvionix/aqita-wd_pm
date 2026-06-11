
/*
 * ----------------------------------------------------------------------------
 * UVX_COMM_M2JMB PROTOCOL PACKET STRUCTURE
 * ----------------------------------------------------------------------------
 *
 * Send Packet Structure:
 * ----------------------------------------------------------------------------
 * | Byte Index | Field Name     | Size (Bytes) | Description                  |
 * |------------|----------------|--------------|------------------------------|
 * | 0          | Start Byte     | 1            | Fixed value (0x02)           |
 * | 1          | Command Byte   | 1            | Specifies command type       |
 * | 2          | Header Size    | 1            | Size of header field (N)     |
 * | 3          | Header         | N            | Optional, depends on command |
 * | 3+N        | Payload    LSB | 1            | LSB of payload length (M)    |
 * | 4+N        | Payload    MSB | 1            | MSB of payload length (M)    |
 * | 5+N        | Payload        | M            | Payload data (if any)        |
 * | 5+N+M      | CRC Byte       | 1            | Checksum of all previous     |
 * ----------------------------------------------------------------------------
 *
 * Response Packet Structure:
 * ----------------------------------------------------------------------------
 * | Byte Index | Field Name     | Size (Bytes) | Description                  |
 * |------------|----------------|--------------|------------------------------|
 * | 0          | Start Byte     | 1            | Fixed value (0x02)           |
 * | 1          | Command Byte   | 1            | Echoed command or error code |
 * | 2          | Response Type  | 1            | 0x02 = Send, 0x03 = Error    |
 * | 3          | Payload    LSB | 1            | LSB of payload length (M)    |
 * | 4          | Payload    MSB | 1            | MSB of payload length (M)    |
 * | 5          | Payload        | M            | Payload data or error info   |
 * | 5+M        | CRC Byte       | 1            | Checksum of all previous     |
 * ----------------------------------------------------------------------------
 *
 * Notes:
 * - All CRC bytes are calculated using XOR or CRC-8 over all preceding bytes.
 * - Payload and header sections are optional and determined by the command type.
 * - Maximum packet length must be defined and bounded as per MCU limits.
 *
 */

#ifndef __UVX_COMM_M2JMB_H
#define __UVX_COMM_M2JMB_H


/* Includes ------------------------------------------------------------------*/
#include "uvx_universal_types.h"
#include <stdint.h>
#include "main.h"

#ifdef UVX_COMM_M2JMB_V1_0

#define BUFF_SIZE_TX_M2JMB			250
#define BUFF_SIZE_RX_M2JMB			250
#define BUFF_SIZE_TX_JMB      100
#define BUFF_SIZE_RX_JMB      100

#define UVX_PACKET_SIZE     1000  // Maximum packet size
#define UVX_START_BYTE      0x02  // Start byte for framing packets

#define M2JMB_BYTE_CTRL       1 // Control byte index in the packet
#define M2JMB_BYTE_CMD        2 // Command byte index in the packet
#define M2JMB_BYTE_SIZE_MSB   3 // Payload size MSB index in the packet
#define M2JMB_BYTE_SIZE_LSB   4 // Payload size LSB index in the packet
#define M2JMB_BYTE_DATA       5 // Data byte index in the packet

typedef struct UVX_COMM_M2JMB
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
    uint8_t Heartbeat	      : 1; // Heartbeat flag

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
    uint32_t                timeout_heartbeat; // Heartbeat timeout in milliseconds
    uint32_t                timeout_power_on; // Power on timeout in milliseconds
    uint32_t                timeout_power_off; // Power off timeout in milliseconds
}UVX_COMM_M2JMB;

 typedef enum
 {
    UVX_M2JMB_TEMP_UM = 0x01,
    UVX_M2JMB_TEMP_UC,
    UVX_M2JMB_TEMP_LM,
    UVX_M2JMB_TEMP_LC,
    UVX_M2JMB_CURRENT_UM,
    UVX_M2JMB_CURRENT_UC,
    UVX_M2JMB_CURRENT_LM,
    UVX_M2JMB_CURRENT_LC,
    UVX_M2JMB_RPM_UM,
    UVX_M2JMB_RPM_UC,
    UVX_M2JMB_RPM_LM,
    UVX_M2JMB_RPM_LC,
    UVX_M2JMB_VOLTAGE,
    UVX_M2JMB_CYCLIC_MAGN_UP,
    UVX_M2JMB_CYCLIC_ERROR_UP,
    UVX_M2JMB_CYCLIC_MAGN_LP,
    UVX_M2JMB_CYCLIC_ERROR_LP,
    UVX_M2JMB_CYCLIC_SP_UP,    
    UVX_M2JMB_CYCLIC_UP,
    UVX_M2JMB_CYCLIC_FF_UP,
    UVX_M2JMB_CYCLIC_IQ_UP,
    UVX_M2JMB_MAIN_IQ_UP,    
    UVX_M2JMB_CYCLIC_SP_LP,
    UVX_M2JMB_CYCLIC_LP,    
    UVX_M2JMB_CYCLIC_FF_LP,    
    UVX_M2JMB_CYCLIC_IQ_LP,    
    UVX_M2JMB_MAIN_IQ_LP,
    UVX_M2JMB_ANGL_HALL_UM,
    UVX_M2JMB_ANGL_HALL_UC, 
    UVX_M2JMB_ANGL_HALL_LM,
    UVX_M2JMB_ANGL_HALL_LC,
    UVX_M2JMB_TRUST_RU,
    UVX_M2JMB_AUTO_RESET,
    UVX_M2JMB_CYCLIC_ID_UP,
    UVX_M2JMB_MAIN_ID_UP,    
    UVX_M2JMB_CYCLIC_ID_LP,    
    UVX_M2JMB_MAIN_ID_LP,
    UVX_M2JMB_MOD_ID_UM,
    UVX_M2JMB_MOD_ID_UC,
    UVX_M2JMB_MOD_ID_LM,
    UVX_M2JMB_MOD_ID_LC,
    UVX_M2JMB_BATT_SOC,
    UVX_M2JMB_BATT_SOH,    
    UVX_M2JMB_TRUST_ROLL,
    UVX_M2JMB_TRUST_PITCH,
    UVX_M2JMB_TRUST_YAW,
    UVX_M2JMB_TRUST_THROTLE,
    UVX_M2JMB_ALTITUDE,
    UVX_M2JMB_RC_INPUT,
    UVX_M2JMB_FLAGS,
    // UVX_M2JMB_BATT_SOH,
    // UVX_M2JMB_BATT_VOLTAGE,
    // UVX_M2JMB_BATT_TEMPERATURE,
    UVX_M2JMB_BATT_CURRENT,
    // UVX_M2JMB_BATT_CHARGE,
    // UVX_M2JMB_BATT_CAPACITY,
    // UVX_M2JMB_BATT_DESIGN_CAPACITY,
    // UVX_M2JMB_BATT_PERCENTAGE,
    // UVX_M2JMB_BATT_SUPPLY_STATUS,
    // UVX_M2JMB_BATT_SUPPLY_HEALTH,
    // UVX_M2JMB_BATT_CELL_VOLTAGE,    
    // UVX_M2JMB_BATT_CELL_TEMPERATURE,    
 } UVX_COMM_M2JMB_HEADER_TYPES;


 typedef enum
 {
   UVX_M2JMB_OK = 0x00,
   UVX_M2JMB_ERROR,
   UVX_M2JMB_ERROR_INIT,
   UVX_M2JMB_ERROR_INIT_UART,
   UVX_M2JMB_BUSY,
   UVX_M2JMB_ERROR_UNKNOWN_CMD,
   UVX_M2JMB_ERROR_CRC,
   UVX_M2JMB_TIMEOUT
 } UVX_COMM_M2JMB_STATE;

typedef enum 
{
    M2JMB_MODE_IDLE = 0x00,
    M2JMB_MODE_TURN_ON,
    M2JMB_MODE_TURN_OFF,
    M2JMB_MODE_SLEEP,
    M2JMB_MODE_WAIT_RESPONSE
} UVX_COMM_M2JMB_MODE;

typedef struct 
{
  UVX_COMM_M2JMB_MODE state_previous; // Previous state of the M2JMB communication
  UVX_COMM_M2JMB_MODE state_current;  // Current state of the M2JMB communication
  UVX_COMM_M2JMB_MODE state_next; // Next state of the M2JMB communication
} UVX_COMM_M2JMB_STATE_MACHINE;

typedef enum
{
    CMD_NONE = 0x00, // No command
    CMD_SEND_DATA_ESC,
    CMD_RESET,
    CMD_TURN_OFF,
    CMD_SLEEP,
    CMD_HEARTBEAT,
    CMD_PWR_ON_FC,
    CMD_PWR_OFF_FC, 
//acknowledgment commands
    CMD_PWR_ON_FC_ACK,
    CMD_PWR_OFF_FC_ACK,
} UVX_M2JMB_CMD_BYTE;

typedef enum
{
  RSP_ACK = 1,
  RSP_ERROR,
  RSP_TIMEOUT
} UVX_M2JMB_RESPONSE_BYTE;

extern UVX_COMM_M2JMB comm_m2jmb; // M2JMB communication structure
extern UVX_COMM_M2JMB_STATE_MACHINE comm_m2jmb_state; // M2JMB communication state machine

extern uint8_t buff_tx_m2jmb[BUFF_SIZE_TX_M2JMB]; // TX buffer for M2JMB communication
extern uint8_t buff_rx_m2jmb[BUFF_SIZE_RX_M2JMB]; // RX buffer for M2JMB communication

/* Function prototypes */
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_init(UVX_UART* uart_m2jmb);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_read(uint8_t data);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_write(uint8_t data);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_send(uint8_t cmd_echo, uint8_t* data, uint16_t size);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_send_error(uint8_t data);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_process_rx(uint8_t byte_rx);
UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_process_rx_data(void);

#endif /* UVX_COMM_M2JMB_V10 */

#endif /* __UVX_COMM_M2JMB_H */
