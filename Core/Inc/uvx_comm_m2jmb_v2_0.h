
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

#ifndef __UVX_COMM_M2JMB_V2_0_H
#define __UVX_COMM_M2JMB_V2_0_H


/* Includes ------------------------------------------------------------------*/
#include "uvx_universal_types.h"
#include <stdint.h>
#include "main.h"

#ifdef UVX_COMM_M2JMB_V2_0

#define BUFF_SIZE_TX_M2JMB			400
#define BUFF_SIZE_RX_M2JMB			250
#define BUFF_SIZE_TX_JMB      100
#define BUFF_SIZE_RX_JMB      100

#define UVX_PACKET_SIZE     1000  // Maximum packet size
#define UVX_HEADER_SIZE     200  // Header size
#define UVX_START_BYTE      0x02  // Start byte for framing packets

#define M2JMB_BYTE_CMD        5 // Command byte index in the packet
#define M2JMB_BYTE_SIZE_MSB   3 // Payload size MSB index in the packet
#define M2JMB_BYTE_SIZE_LSB   4 // Payload size LSB index in the packet
#define M2JMB_BYTE_DATA       5 // Data byte index in the packet

#define PROTOCOL_START_BYTES_SIZE     3 // Size of start bytes in the packet
#define PROTOCOL_MIN_SIZE             7 // Minimum packet size (start byte + command byte + size bytes + CRC)
#define MAX_ERROR_COUNT_TX            250 // Maximum number of consecutive transmission errors before resetting the error count and allowing new transmissions


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
    uint32_t                error_count_tx; // Error count for TX
    uint32_t                error_count_rx; // Error count for RX
    uint32_t                error_no_response_tx; //
    uint32_t                error_no_response_rx; //
}UVX_COMM_M2JMB;

 typedef enum
 {
    UVX_M2JMB_TEMP_UM = 0x01,
    UVX_M2JMB_TEMP_UC = 0x02,
    UVX_M2JMB_TEMP_LM = 0x03,
    UVX_M2JMB_TEMP_LC = 0x04,
    UVX_M2JMB_CURRENT_UM = 0x05,
    UVX_M2JMB_CURRENT_UC = 0x06,
    UVX_M2JMB_CURRENT_LM = 0x07,
    UVX_M2JMB_CURRENT_LC = 0x08,
    UVX_M2JMB_RPM_UM = 0x09,
    UVX_M2JMB_RPM_UC = 0x0A,
    UVX_M2JMB_RPM_LM = 0x0B,
    UVX_M2JMB_RPM_LC = 0x0C,
    UVX_M2JMB_VOLTAGE = 0x0D,
    UVX_M2JMB_CYCLIC_MAGN_UP = 0x0E,
    UVX_M2JMB_CYCLIC_ERROR_UP = 0x0F,
    UVX_M2JMB_CYCLIC_MAGN_LP = 0x10,
    UVX_M2JMB_CYCLIC_ERROR_LP = 0x11,
    UVX_M2JMB_CYCLIC_SP_UP = 0x12,
    UVX_M2JMB_CYCLIC_UP = 0x13,
    UVX_M2JMB_CYCLIC_FF_UP = 0x14,
    UVX_M2JMB_CYCLIC_IQ_UP = 0x15,
    UVX_M2JMB_MAIN_IQ_UP = 0x16,
    UVX_M2JMB_CYCLIC_SP_LP = 0x17,
    UVX_M2JMB_CYCLIC_LP = 0x18,
    UVX_M2JMB_CYCLIC_FF_LP = 0x19,
    UVX_M2JMB_CYCLIC_IQ_LP = 0x1A,
    UVX_M2JMB_MAIN_IQ_LP = 0x1B,
    UVX_M2JMB_ANGL_HALL_UM = 0x1C,
    UVX_M2JMB_ANGL_HALL_UC = 0x1D,
    UVX_M2JMB_ANGL_HALL_LM = 0x1E,
    UVX_M2JMB_ANGL_HALL_LC = 0x1F,
    UVX_M2JMB_TRUST_RU = 0x20,
    UVX_M2JMB_AUTO_RESET = 0x21,
    UVX_M2JMB_CYCLIC_ID_UP = 0x22,
    UVX_M2JMB_MAIN_ID_UP = 0x23,
    UVX_M2JMB_CYCLIC_ID_LP = 0x24,
    UVX_M2JMB_MAIN_ID_LP = 0x25,
    UVX_M2JMB_MOD_ID_UM = 0x26,
    UVX_M2JMB_MOD_ID_UC = 0x27,
    UVX_M2JMB_MOD_ID_LM = 0x28,
    UVX_M2JMB_MOD_ID_LC = 0x29,
    UVX_M2JMB_TRUST_ROLL = 0x2A,
    UVX_M2JMB_TRUST_PITCH = 0x2B,
    UVX_M2JMB_TRUST_YAW = 0x2C,
    UVX_M2JMB_TRUST_THROTLE = 0x2D,
    UVX_M2JMB_ALTITUDE = 0x2E,
    UVX_M2JMB_RC_INPUT = 0x2F,
    UVX_M2JMB_FLAGS = 0x30,
    UVX_M2JMB_BATT_SOH = 0x31,
    UVX_M2JMB_BATT_VOLTAGE = 0x32,
    UVX_M2JMB_BATT_TEMPERATURE = 0x33,
    UVX_M2JMB_BATT_CURRENT = 0x34,
    UVX_M2JMB_BATT_CHARGE = 0x35,
    UVX_M2JMB_BATT_CAPACITY = 0x36,
    UVX_M2JMB_BATT_DESIGN_CAPACITY = 0x37,
    UVX_M2JMB_BATT_PERCENTAGE_REL = 0x38,
    UVX_M2JMB_BATT_SUPPLY_STATUS = 0x39,
    UVX_M2JMB_BATT_SUPPLY_HEALTH      = 0x3A,
    UVX_M2JMB_BATT_CELL_VOLTAGE       = 0x3B,   
    UVX_M2JMB_BATT_CELL_TEMPERATURE   = 0x3C, //
    UVX_M2JMB_BATT_VOLTAGE_DIFF_PACK  = 0x3D,
    UVX_M2JMB_BATT_VOLTAGE_DELTA_CELL = 0x3E,
    UVX_M2JMB_BATT_VOLTAGE_MIN_CELL   = 0x3F,
    UVX_M2JMB_BATT_VOLTAGE_MAX_CELL   = 0x40,     
    UVX_M2JMB_BATT_CELLS_BALLANCE     = 0x41,   
    UVX_M2JMB_BATT_PERCENTAGE_ABS     = 0x42,
    UVX_M2JMB_BATT_POWER              = 0x43,
    UVX_M2JMB_BATT_CYCLE_COUNT        = 0x44,
    UVX_M2JMB_BATT_PACK_V             = 0x45,
    UVX_M2JMB_BATT_AVG_TIME_TO_EMPTY  = 0x46,
    UVX_M2JMB_BATT_AVG_TIME_TO_FULL   = 0x47,
    UVX_M2JMB_BATT_STATE_TIME_L       = 0x48,
    UVX_M2JMB_BATT_STATE_TIME_H       = 0x49,
    UVX_M2JMB_BATT_ERROR              = 0x4A,  
    UVX_M2JMB_BATT_QMAX_PASSED_BQ_L   = 0x4B,  
    UVX_M2JMB_BATT_QMAX_PASSED_BQ_H   = 0x4C,  
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
    M2JMB_MODE_SEND_FC_ON_ACK,
    M2JMB_MODE_SEND_FC_OFF_ACK,
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

#endif /* UVX_COMM_M2JMB_V2_0 */

#endif /* __UVX_COMM_M2JMB_V2_0_H */
