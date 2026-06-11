
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

#include "uvx_comm_m2jmb_v2_0.h"
#include "uvx_crc8.h"
#include <string.h>

#ifdef UVX_COMM_M2JMB_V2_0

uint8_t buff_tx_m2jmb[BUFF_SIZE_TX_M2JMB];
uint8_t buff_rx_m2jmb[BUFF_SIZE_RX_M2JMB];

UVX_COMM_M2JMB comm_m2jmb; // M2JMB communication structure
UVX_COMM_M2JMB_STATE_MACHINE comm_m2jmb_state;

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_init(UVX_UART* uart) 
{
	comm_m2jmb.p_buff_tx_start = buff_tx_m2jmb; // Set the start pointer for the TX buffer
	comm_m2jmb.p_buff_rx_start = buff_rx_m2jmb; // Set the start pointer for the RX buffer
	comm_m2jmb.p_buff_tx = buff_tx_m2jmb; // Set the pointer to the TX buffer
	comm_m2jmb.p_buff_rx = buff_rx_m2jmb; // Set the pointer to the RX buffer
	comm_m2jmb.buff_size_tx = BUFF_SIZE_TX_M2JMB; // Set the size of the TX buffer
	comm_m2jmb.buff_size_rx = BUFF_SIZE_RX_M2JMB; // Set the size of the RX buffer
	comm_m2jmb.timeout_heartbeat = APP_TIMEOUT_JMB_HEARTBEAT; // Set the heartbeat timeout in milliseconds
	comm_m2jmb.timeout_power_on = APP_TIMEOUT_JMB_POWER_ON; // Set the power on timeout in milliseconds
	comm_m2jmb.timeout_power_off = APP_TIMEOUT_JMB_POWER_OFF; // Set the power off timeout in milliseconds
	comm_m2jmb.buff_rx_cnt = 0; // Initialize RX buffer count
	comm_m2jmb.buff_tx_cnt = 0; // Initialize TX buffer count

	if(uart != NULL)
	{
		comm_m2jmb.p_hal_uart = &uart->hal_uart; // Set the pointer to the UART HAL structure

		#ifdef PROJECT_AQITA_PM
			if(uvx_uart_init(uart, buff_tx_m2jmb, buff_rx_m2jmb)) // Initialize the UART peripheral
			{
				return UVX_M2JMB_ERROR_INIT_UART; // Return error if initialization fails
			}
		#endif
	}

	return UVX_M2JMB_OK; // Return success
}


UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_read(UVX_M2JMB_CMD_BYTE cmd_byte) 
{    
    buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
    buff_tx_m2jmb[1] = UVX_START_BYTE;       // Start byte
    buff_tx_m2jmb[2] = UVX_START_BYTE;       // Start byte
    buff_tx_m2jmb[3] = cmd_byte;              // Target command byte
    buff_tx_m2jmb[4] = 0x00;                 // Payload size MSB (0 for read request)
    buff_tx_m2jmb[5] = 0x00;                 // Payload size LSB (0 for read request)
    buff_tx_m2jmb[6] = uvx_crc_8(buff_tx_m2jmb, 6);   // CRC over the 7 bytes before it
    
	uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 7); // Send the command over UART
	return UVX_M2JMB_OK; // Return success
}  

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_write(uint8_t data) 
{    
	buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[1] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[2] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[3] = CTRL_WRITE;       // Write command
	buff_tx_m2jmb[4] = data;              // Target command byte
	buff_tx_m2jmb[5] = 0x00;                 // Payload size MSB (0 for write request)
	buff_tx_m2jmb[6] = 0x00;                 // Payload size LSB (0 for write request)
	buff_tx_m2jmb[7] = uvx_crc_8(buff_tx_m2jmb, 7);   // CRC over the 8 bytes before it
	
	uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 8); // Send the command over UART
	return UVX_M2JMB_OK; // Return success
}

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_send(uint8_t cmd, uint8_t* data, uint16_t size) 
{    
	uint16_t header_size = 0;
	uint16_t total_size = 0;
	uint16_t crc_16 = 0;
	uint8_t header_size_offset = 0;
	buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[1] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[2] = UVX_START_BYTE;       // Start byte
	

	switch (cmd)
	{
		case CMD_SEND_DATA_ESC:
			
			header_size_offset = 6; // Header starts after the command and header size bytes
			header_size = (&buff_tx_m2jmb[header_size_offset + 74] - &buff_tx_m2jmb[header_size_offset]); // Calculate header size based on the defined header fields	

			total_size = header_size + size + 2; // Calculate total payload size (header + data)
			buff_tx_m2jmb[3] = (uint8_t)(total_size >> 8);      // Payload size LSB
			buff_tx_m2jmb[4] = (uint8_t)total_size;    // Payload size MSB
			buff_tx_m2jmb[5] = cmd;			  // Command byte
			buff_tx_m2jmb[header_size_offset] = header_size;

			buff_tx_m2jmb[header_size_offset + 1]  = UVX_M2JMB_TEMP_UM; 				// 1 byte   sum:   1 byte
			buff_tx_m2jmb[header_size_offset + 2]  = UVX_M2JMB_CURRENT_UM; 				// 2 bytes 	sum:   3 bytes
			buff_tx_m2jmb[header_size_offset + 3]  = UVX_M2JMB_RPM_UM; 					// 2 bytes 	sum:   5 bytes
			buff_tx_m2jmb[header_size_offset + 4]  = UVX_M2JMB_TEMP_UC; 				// 1 byte 	sum:   6 bytes
			buff_tx_m2jmb[header_size_offset + 5] = UVX_M2JMB_CURRENT_UC; 				// 2 bytes 	sum:   8 bytes
			buff_tx_m2jmb[header_size_offset + 6] = UVX_M2JMB_RPM_UC;  					// 2 bytes 	sum:  10 bytes
			buff_tx_m2jmb[header_size_offset + 7] = UVX_M2JMB_TEMP_LM; 					// 1 byte 	sum:  11 bytes
			buff_tx_m2jmb[header_size_offset + 8] = UVX_M2JMB_CURRENT_LM; 				// 2 bytes 	sum:  13 bytes
			buff_tx_m2jmb[header_size_offset + 9] = UVX_M2JMB_RPM_LM; 					// 2 bytes 	sum:  15 bytes
			buff_tx_m2jmb[header_size_offset + 10] = UVX_M2JMB_TEMP_LC; 				// 1 byte 	sum:  16 bytes
			buff_tx_m2jmb[header_size_offset + 11] = UVX_M2JMB_CURRENT_LC; 				// 2 bytes	sum:  18 bytes
			buff_tx_m2jmb[header_size_offset + 12] = UVX_M2JMB_RPM_LC; 					// 2 bytes 	sum:  20 bytes
			buff_tx_m2jmb[header_size_offset + 13] = UVX_M2JMB_VOLTAGE; 				// 2 bytes  sum:  22 bytes
			buff_tx_m2jmb[header_size_offset + 14] = UVX_M2JMB_CYCLIC_MAGN_UP; 			// 2 bytes  sum:  24 bytes		
			buff_tx_m2jmb[header_size_offset + 15] = UVX_M2JMB_CYCLIC_ERROR_UP; 		// 2 bytes 	sum:  26 bytes
			buff_tx_m2jmb[header_size_offset + 16] = UVX_M2JMB_CYCLIC_MAGN_LP; 			// 2 bytes  sum:  28 bytes
			buff_tx_m2jmb[header_size_offset + 17] = UVX_M2JMB_CYCLIC_ERROR_LP; 		// 2 bytes  sum:  30 bytes		
			buff_tx_m2jmb[header_size_offset + 18] = UVX_M2JMB_CYCLIC_SP_UP; 			// 2 bytes 	sum:  32 bytes
			buff_tx_m2jmb[header_size_offset + 19] = UVX_M2JMB_CYCLIC_UP; 				// 2 bytes	sum:  34 bytes
			buff_tx_m2jmb[header_size_offset + 20] = UVX_M2JMB_CYCLIC_FF_UP; 			// 2 bytes 	sum: 36 bytes
			buff_tx_m2jmb[header_size_offset + 21] = UVX_M2JMB_CYCLIC_IQ_UP; 			// 2 bytes 	sum: 38 bytes
			buff_tx_m2jmb[header_size_offset + 22] = UVX_M2JMB_MAIN_IQ_UP; 				// 2 bytes 	sum: 40 bytes		
			buff_tx_m2jmb[header_size_offset + 23] = UVX_M2JMB_CYCLIC_SP_LP; 			// 2 bytes 	sum: 42 bytes
			buff_tx_m2jmb[header_size_offset + 24] = UVX_M2JMB_CYCLIC_LP; 				// 2 bytes 	sum: 44 bytes
			buff_tx_m2jmb[header_size_offset + 25] = UVX_M2JMB_CYCLIC_FF_LP;			// 2 bytes 	sum: 46 bytes
			buff_tx_m2jmb[header_size_offset + 26] = UVX_M2JMB_CYCLIC_IQ_LP;			// 2 bytes 	sum: 48 bytes
			buff_tx_m2jmb[header_size_offset + 27] = UVX_M2JMB_MAIN_IQ_LP;				// 2 bytes 	sum: 50 bytes
			buff_tx_m2jmb[header_size_offset + 28] = UVX_M2JMB_ANGL_HALL_UM; 			// 1 byte 	sum: 51 bytes
			buff_tx_m2jmb[header_size_offset + 29] = UVX_M2JMB_ANGL_HALL_UC; 			// 1 byte	sum: 52 bytes
			buff_tx_m2jmb[header_size_offset + 30] = UVX_M2JMB_ANGL_HALL_LM; 			// 1 byte	sum: 53 bytes
			buff_tx_m2jmb[header_size_offset + 31] = UVX_M2JMB_ANGL_HALL_LC; 			// 1 byte 	sum: 54 bytes
			buff_tx_m2jmb[header_size_offset + 32] = UVX_M2JMB_TRUST_RU; 				// 2 bytes	sum: 56 bytes
			buff_tx_m2jmb[header_size_offset + 33] = UVX_M2JMB_AUTO_RESET; 				// 1 byte 	sum: 57 bytes
			buff_tx_m2jmb[header_size_offset + 34] = UVX_M2JMB_CYCLIC_ID_UP; 			// 2 bytes	sum: 59 bytes
			buff_tx_m2jmb[header_size_offset + 35] = UVX_M2JMB_MAIN_ID_UP; 				// 2 bytes 	sum: 61 bytes
			buff_tx_m2jmb[header_size_offset + 36] = UVX_M2JMB_CYCLIC_ID_LP;			// 2 bytes 	sum: 63 bytes
			buff_tx_m2jmb[header_size_offset + 37] = UVX_M2JMB_MAIN_ID_LP;				// 2 bytes 	sum: 65 bytes
			buff_tx_m2jmb[header_size_offset + 38] = UVX_M2JMB_MOD_ID_UM; 				// 1 byte	sum: 66 bytes
			buff_tx_m2jmb[header_size_offset + 39] = UVX_M2JMB_MOD_ID_UC; 				// 1 byte	sum: 67 bytes
			buff_tx_m2jmb[header_size_offset + 40] = UVX_M2JMB_MOD_ID_LM; 				// 1 byte	sum: 68 bytes
			buff_tx_m2jmb[header_size_offset + 41] = UVX_M2JMB_MOD_ID_LC; 				// 1 byte 	sum: 69 bytes
			buff_tx_m2jmb[header_size_offset + 42] = UVX_M2JMB_TRUST_ROLL; 				// 2 bytes	sum: 71 bytes
			buff_tx_m2jmb[header_size_offset + 43] = UVX_M2JMB_TRUST_PITCH; 			// 2 bytes	sum: 73 bytes
			buff_tx_m2jmb[header_size_offset + 44] = UVX_M2JMB_TRUST_YAW; 				// 2 bytes	sum: 75 bytes
			buff_tx_m2jmb[header_size_offset + 45] = UVX_M2JMB_TRUST_THROTLE; 			// 2 bytes	sum: 77 bytes
			buff_tx_m2jmb[header_size_offset + 46] = UVX_M2JMB_ALTITUDE; 				// 2 bytes	sum: 79 bytes
			buff_tx_m2jmb[header_size_offset + 47] = UVX_M2JMB_RC_INPUT; 				// 2 bytes	sum: 81 bytes
			buff_tx_m2jmb[header_size_offset + 48] = UVX_M2JMB_FLAGS; 					// 1 byte	sum: 82 bytes

			buff_tx_m2jmb[header_size_offset + 49] = UVX_M2JMB_BATT_SOH; 				// 1 bytes  sum: 83 bytes buff index 163
			buff_tx_m2jmb[header_size_offset + 50] = UVX_M2JMB_BATT_VOLTAGE; 			// 2 bytes  sum: 85 bytes buff index 164-165
			buff_tx_m2jmb[header_size_offset + 51] = UVX_M2JMB_BATT_CURRENT; 			// 2 bytes  sum: 87 bytes buff index 166-167
			buff_tx_m2jmb[header_size_offset + 52] = UVX_M2JMB_BATT_CHARGE;     		// 2 bytes  sum: 89 bytes buff index 168-169
			buff_tx_m2jmb[header_size_offset + 53] = UVX_M2JMB_BATT_CAPACITY;			// 2 bytes  sum: 91 bytes buff index 170-171
			buff_tx_m2jmb[header_size_offset + 54] = UVX_M2JMB_BATT_DESIGN_CAPACITY;	// 2 bytes  sum: 93 bytes buff index 172-173
			buff_tx_m2jmb[header_size_offset + 55] = UVX_M2JMB_BATT_PERCENTAGE_REL;		// 1 byte  	sum: 94 bytes buff index 174
			buff_tx_m2jmb[header_size_offset + 56] = UVX_M2JMB_BATT_SUPPLY_STATUS;		// 1 byte  	sum: 95 bytes buff index 175
			buff_tx_m2jmb[header_size_offset + 57] = UVX_M2JMB_BATT_CELL_VOLTAGE;		// 20 bytes sum: 115 bytes buff index 176-195
			buff_tx_m2jmb[header_size_offset + 58] = UVX_M2JMB_BATT_CELL_TEMPERATURE;	// 20 bytes sum: 135 bytes buff index 196-215
			buff_tx_m2jmb[header_size_offset + 59] = UVX_M2JMB_BATT_VOLTAGE_DIFF_PACK; 	// 2 bytes  sum: 137 bytes buff index 216-217
			buff_tx_m2jmb[header_size_offset + 60] = UVX_M2JMB_BATT_VOLTAGE_DELTA_CELL; // 2 bytes  sum: 139 bytes buff index 218-219
			buff_tx_m2jmb[header_size_offset + 61] = UVX_M2JMB_BATT_VOLTAGE_MIN_CELL; 	// 2 bytes  sum: 141 bytes buff index 220-221
			buff_tx_m2jmb[header_size_offset + 62] = UVX_M2JMB_BATT_VOLTAGE_MAX_CELL; 	// 2 bytes  sum: 143 bytes buff index 222-223
			buff_tx_m2jmb[header_size_offset + 63] = UVX_M2JMB_BATT_CELLS_BALLANCE; 	// 20 bytes sum: 163 bytes buff index 224-243
			buff_tx_m2jmb[header_size_offset + 64] = UVX_M2JMB_BATT_PERCENTAGE_ABS; 	// 2 byte   sum: 165 bytes buff index 244-245
			buff_tx_m2jmb[header_size_offset + 65] = UVX_M2JMB_BATT_POWER; 				// 4 bytes  sum: 169 bytes buff index 246-249
			buff_tx_m2jmb[header_size_offset + 66] = UVX_M2JMB_BATT_CYCLE_COUNT; 		// 2 bytes  sum: 171 bytes buff index 250-251
			buff_tx_m2jmb[header_size_offset + 67] = UVX_M2JMB_BATT_PACK_V; 			// 2 bytes  sum: 173 bytes buff index 252-253
			buff_tx_m2jmb[header_size_offset + 68] = UVX_M2JMB_BATT_AVG_TIME_TO_EMPTY; 	// 2 bytes  sum: 175 bytes buff index 254-255
			buff_tx_m2jmb[header_size_offset + 69] = UVX_M2JMB_BATT_AVG_TIME_TO_FULL; 	// 2 bytes  sum: 177 bytes buff index 256-257
			buff_tx_m2jmb[header_size_offset + 70] = UVX_M2JMB_BATT_STATE_TIME_L; 		// 2 bytes  sum: 179 bytes buff index 258-259
			buff_tx_m2jmb[header_size_offset + 71] = UVX_M2JMB_BATT_STATE_TIME_H; 		// 2 bytes  sum: 181 bytes buff index 260-261
			buff_tx_m2jmb[header_size_offset + 72] = UVX_M2JMB_BATT_ERROR; 				// 1 byte  	sum: 182 bytes buff index 262
			buff_tx_m2jmb[header_size_offset + 73] = UVX_M2JMB_BATT_QMAX_PASSED_BQ_L; 	// 2 byte  	sum: 184 bytes buff index 263-264
			buff_tx_m2jmb[header_size_offset + 74] = UVX_M2JMB_BATT_QMAX_PASSED_BQ_H; 	// 2 byte  	sum: 186 bytes buff index 265-266

			// for (uint16_t i = 0; i < size; i++)
			// {
			// 	data[i] = (uint8_t)(i + 1);
			// }

			memcpy(&buff_tx_m2jmb[header_size_offset + 75], data, size); 			// Copy the payload data to the TX buffer

			// for (uint16_t i = 0; i < (total_size - 1); i++)
			// {
			// 	buff_tx_m2jmb[6 + i] = (uint8_t)(i + 1);
			// }

			crc_16 = uvx_crc_16(&buff_tx_m2jmb[5], total_size); // CRC over the 4 bytes before it and the data
			buff_tx_m2jmb[total_size + 5] = (uint8_t)(crc_16 >> 8); 				// CRC LSB
			buff_tx_m2jmb[total_size + 6] = (uint8_t)crc_16; 	// CRC MSB

			comm_m2jmb.TX_Ready = 0; // Clear the TX ready flag before sending
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, total_size + 7);			
			
		break;

		case CMD_TURN_OFF:
			total_size = 1;
			buff_tx_m2jmb[3] = (uint8_t)(total_size >> 8);      // Payload size LSB
			buff_tx_m2jmb[4] = (uint8_t)total_size;    // Payload size MSB
			buff_tx_m2jmb[5] = cmd;

			crc_16 = uvx_crc_16(&buff_tx_m2jmb[5], total_size); // CRC over the 4 bytes before it and the data
			buff_tx_m2jmb[6] = (uint8_t)(crc_16 >> 8); 				// CRC LSB
			buff_tx_m2jmb[7] = (uint8_t)crc_16; 	// CRC MSB
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 8); // Send the command over UART
		break;

		case CMD_PWR_ON_FC_ACK:
			total_size = 2;
			buff_tx_m2jmb[3] = (uint8_t)(total_size >> 8);      // Payload size LSB
			buff_tx_m2jmb[4] = (uint8_t)total_size;    // Payload size MSB
			buff_tx_m2jmb[5] = CMD_PWR_ON_FC;
			buff_tx_m2jmb[6] = RSP_ACK; // Acknowledgment response
			crc_16 = uvx_crc_16(&buff_tx_m2jmb[5], total_size); // CRC over the 4 bytes before it and the data
			buff_tx_m2jmb[7] = (uint8_t)(crc_16 >> 8); 				// CRC LSB
			buff_tx_m2jmb[8] = (uint8_t)crc_16; 	// CRC MSB
			if(uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 9) != UVX_UART_OK) // Send the command over UART
			{
				comm_m2jmb.error_count_tx++;
				if(comm_m2jmb.error_count_tx >= MAX_ERROR_COUNT_TX)
				{
					comm_m2jmb.error_count_tx = 0; // Reset error count after reaching the maximum
					comm_m2jmb.p_hal_uart->tx_ready = true; // Reset UART TX ready flag to allow new transmissions
					comm_m2jmb.error_no_response_tx++; // Increment the no response error count
				}
				return UVX_M2JMB_ERROR;
			}
		break;

		case CMD_PWR_OFF_FC_ACK:
			total_size = 2;
			buff_tx_m2jmb[3] = (uint8_t)(total_size >> 8);      // Payload size LSB
			buff_tx_m2jmb[4] = (uint8_t)total_size;    // Payload size MSB
			buff_tx_m2jmb[5] = CMD_PWR_OFF_FC;
			buff_tx_m2jmb[6] = RSP_ACK; // Acknowledgment response
			crc_16 = uvx_crc_16(&buff_tx_m2jmb[5], total_size); // CRC over the 4 bytes before it and the data
			buff_tx_m2jmb[7] = (uint8_t)(crc_16 >> 8); 				// CRC LSB
			buff_tx_m2jmb[8] = (uint8_t)crc_16; 	// CRC MSB
			if(uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 9) != UVX_UART_OK) // Send the command over UART
			{
				comm_m2jmb.error_count_tx++;
				if(comm_m2jmb.error_count_tx >= MAX_ERROR_COUNT_TX)
				{
					comm_m2jmb.error_count_tx = 0; // Reset error count after reaching the maximum
					comm_m2jmb.p_hal_uart->tx_ready = true; // Reset UART TX ready flag to allow new transmissions
					comm_m2jmb.error_no_response_tx++; // Increment the no response error count
				}				
				return UVX_M2JMB_ERROR;
			}
		break;

		default:
			return UVX_M2JMB_ERROR_UNKNOWN_CMD; // Return error for unknown command
	}
		
	return UVX_M2JMB_OK; // Return success
}

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_send_error(uint8_t data) 
{    
	buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[1] = CTRL_SEND_ERROR;  // Send error command
	buff_tx_m2jmb[2] = data;              // Target command byte
	buff_tx_m2jmb[3] = 0x00;                 // Payload size MSB (0 for send error request)
	buff_tx_m2jmb[4] = 0x00;                 // Payload size LSB (0 for send error request)
	buff_tx_m2jmb[5] = uvx_crc_8(buff_tx_m2jmb, 5);   // CRC over the 5 bytes before it

	uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 6);
	return UVX_M2JMB_OK; // Return success
}

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_process_rx(uint8_t byte_rx)
{
	if(!comm_m2jmb.RX_Ready) // Process incoming byte only if RX is not already ready with a complete frame
	{
		comm_m2jmb.p_buff_rx[comm_m2jmb.buff_rx_cnt] = byte_rx; // Store the received byte in the RX buffer

		if(comm_m2jmb.buff_rx_cnt < comm_m2jmb.buff_size_rx) // Check if RX buffer count is within bounds
		{
			if((byte_rx == UVX_START_BYTE) && (comm_m2jmb.buff_rx_cnt < PROTOCOL_START_BYTES_SIZE)) // If the first byte is the start byte and it's the first byte received
			{
				comm_m2jmb.buff_rx_cnt++; // Increment the RX buffer count
			}		
			else
			{
				if (((buff_rx_m2jmb[0]) == UVX_START_BYTE) &&
					((buff_rx_m2jmb[1]) == UVX_START_BYTE) &&
					((buff_rx_m2jmb[2]) == UVX_START_BYTE)) // Check if the first byte is the start byte and RX is not ready
				{
					if(comm_m2jmb.buff_rx_cnt < comm_m2jmb.buff_size_rx) // Check if RX buffer count is within bounds before incrementing
					{
						comm_m2jmb.buff_rx_cnt++; // Increment the RX buffer count
					}

					if(comm_m2jmb.buff_rx_cnt == 5) // After receiving the command byte, calculate expected payload size
					{
						comm_m2jmb.size_payload = (comm_m2jmb.p_buff_rx[M2JMB_BYTE_SIZE_MSB] << 8) | comm_m2jmb.p_buff_rx[M2JMB_BYTE_SIZE_LSB]; // Calculate payload size from the received bytes
						if(comm_m2jmb.size_payload == 0)
						{
							comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count
						}
						else
						{
							comm_m2jmb.size_frame = comm_m2jmb.size_payload + PROTOCOL_MIN_SIZE; // full payload is received (including CRC) 
							if(comm_m2jmb.size_frame >= comm_m2jmb.buff_size_rx) // Check if the expected frame size exceeds the RX buffer size
							{
								comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count
								comm_m2jmb.size_frame = 0; // Reset expected frame size
								comm_m2jmb.RX_Ready = false; // Reset RX ready flag
							}
						}
					}
					
					if(comm_m2jmb.size_frame > PROTOCOL_MIN_SIZE)
					{
						if(comm_m2jmb.buff_rx_cnt == comm_m2jmb.size_frame) // Wait until the full payload is received (including CRC)
						{
							//check CRC
							uint16_t crc_calculated = uvx_crc_16(&comm_m2jmb.p_buff_rx[M2JMB_BYTE_CMD], comm_m2jmb.size_payload); // CRC over the payload data
							uint16_t crc_received = (comm_m2jmb.p_buff_rx[comm_m2jmb.size_frame - 1] | (comm_m2jmb.p_buff_rx[comm_m2jmb.size_frame - 2] << 8)); // CRC received in the last 2 bytes of the frame
							
							if (crc_calculated == crc_received)
							{
								comm_m2jmb.RX_Ready = true; // Set RX buffer ready flag
							}
							else
							{
								comm_m2jmb.RX_Ready = false; // Reset RX ready flag if CRC check fails
								comm_m2jmb.size_frame = 0; // Reset expected frame size
								comm_m2jmb.size_payload = 0; // Reset expected payload size								
							}
								
							comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count							
						}
					}
				}
			}
		}	    
		else 
		{
			// If RX buffer is full, reset the count
			comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count
			comm_m2jmb.size_frame = 0; // Reset expected frame size
			comm_m2jmb.RX_Ready = 0; // Reset RX ready flag
		}	
	}
	




	// if (((buff_rx_m2jmb[0]) == UVX_START_BYTE) &&
	//  	((buff_rx_m2jmb[1]) == UVX_START_BYTE) &&
	//     ((buff_rx_m2jmb[2]) == UVX_START_BYTE) &&
	//   	(comm_m2jmb.RX_Ready == false) ) // Check if the first byte is the start byte and RX is not ready
	// {


	// 	// After receiving payload size bytes, calculate expected frame size
	// 	if (comm_m2jmb.buff_rx_cnt == 2) 
	// 	{
	// 		switch (buff_rx_m2jmb[1]) // Check the command byte
	// 		{
	// 			case CTRL_READ:
	// 			case CTRL_WRITE:
	// 			case CTRL_SEND:
	// 			case CMD_HEARTBEAT:
	// 				comm_m2jmb.size_payload = 0; // No payload for these commands
	// 				comm_m2jmb.size_frame = 3; 
	// 			break;

	// 			case CMD_PWR_ON_FC:
	// 			case CMD_PWR_OFF_FC:
	// 				comm_m2jmb.size_payload = 0; // No payload for these commands
	// 				comm_m2jmb.size_frame = 2; 
	// 			break;
	// 		}
	// 	}

	// 	// Wait until the full frame is received
	// 	if (comm_m2jmb.size_frame && comm_m2jmb.buff_rx_cnt == comm_m2jmb.size_frame + 1) 
	// 	{
	// 		if(buff_rx_m2jmb[1] == CMD_PWR_ON_FC)
	// 		{
	// 			buff_rx_m2jmb[1] = CMD_PWR_ON_FC;
	// 		}

	// 		// Check CRC
	// 		uint8_t crc = uvx_crc_8(buff_rx_m2jmb, comm_m2jmb.size_frame);

	// 		if (buff_rx_m2jmb[comm_m2jmb.size_frame] == crc) 
	// 		{
	// 			//comm_m2jmb.buff_rx_cnt = 0;
	// 			comm_m2jmb.RX_Ready = 1; // Notify RX ready				
	// 		} 
	// 		else 
	// 		{
	// 			comm_m2jmb.buff_rx_cnt = 0;
	// 			comm_m2jmb.size_frame = 0;
	// 		}
	// 	}		
	// }

    return UVX_M2JMB_OK;
}

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_process_rx_data(void)
{
	uint8_t resp = 0x05;

	if(comm_m2jmb.RX_Ready == 1) // If RX is not ready, process the received byte
	{
		switch (comm_m2jmb.p_buff_rx[1]) // Check the control byte
		{
			case CTRL_READ:

				// Prepare and send a response for READ
				switch(comm_m2jmb.p_buff_rx[2]) // Check the command byte
				{
					case CMD_READ_DATA_ESC:

						resp = 0x01; // Example response for CMD_READ_DATA_ESC
						uvx_comm_m2jmb_send(CMD_READ_DATA_ESC, &resp, 1); // Send the command byte as response
					break;

					case CMD_READ_DATA_JMB:		
						resp = 0x02; // Example response for CMD_READ_DATA_JMB								
						uvx_comm_m2jmb_send(CMD_READ_DATA_ESC, &resp, 1); // Send the command byte as response
					break;
					

					default:
						// Unknown command, send error
						return UVX_M2JMB_ERROR_UNKNOWN_CMD;
				}
			break;

			case CTRL_WRITE:
				// Handle write, maybe store data, then ACK
				// ... (store data as needed)

				break;

			case CTRL_SEND:
				// Handle send command
				// ... (process as needed)
				break;

			case CTRL_SEND_ERROR:
				// Handle error
				// ... (process as needed)
				break;

			default:
				// Unknown command, send error
				return UVX_M2JMB_ERROR_UNKNOWN_CMD;
		}
		
		comm_m2jmb.p_buff_rx[1] = CTRL_NONE; // Reset the command byte to NONE after processing
	}
	
	
	return UVX_M2JMB_OK; // Return success
}
#endif // UVX_COMM_M2JMB_V10
