
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

#include "uvx_comm_m2jmb.h"
#include "uvx_crc8.h"

#ifdef UVX_COMM_M2JMB_V1_0

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
    buff_tx_m2jmb[1] = cmd_byte;              // Target command byte
    buff_tx_m2jmb[2] = 0x00;                 // Payload size MSB (0 for read request)
    buff_tx_m2jmb[3] = 0x00;                 // Payload size LSB (0 for read request)
    buff_tx_m2jmb[4] = uvx_crc_8(buff_tx_m2jmb, 4);   // CRC over the 5 bytes before it
    
	uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 5); // Send the command over UART
	return UVX_M2JMB_OK; // Return success
}  

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_write(uint8_t data) 
{    
	buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[1] = CTRL_WRITE;       // Write command
	buff_tx_m2jmb[2] = data;              // Target command byte
	buff_tx_m2jmb[3] = 0x00;                 // Payload size MSB (0 for write request)
	buff_tx_m2jmb[4] = 0x00;                 // Payload size LSB (0 for write request)
	buff_tx_m2jmb[5] = uvx_crc_8(buff_tx_m2jmb, 5);   // CRC over the 5 bytes before it
	
	uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 6); // Send the command over UART
	return UVX_M2JMB_OK; // Return success
}

UVX_COMM_M2JMB_STATE uvx_comm_m2jmb_send(uint8_t cmd, uint8_t* data, uint16_t size) 
{    
	buff_tx_m2jmb[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2jmb[1] = cmd;

	switch (cmd)
	{
		case CMD_SEND_DATA_ESC:
			//buff_tx_m2jmb[2]  = 60;
			buff_tx_m2jmb[2]  = 49;
			buff_tx_m2jmb[3]  = UVX_M2JMB_TEMP_UM; //1 byte
			buff_tx_m2jmb[4]  = UVX_M2JMB_CURRENT_UM; // 2 bytes
			buff_tx_m2jmb[5]  = UVX_M2JMB_RPM_UM; // 2 bytes

			buff_tx_m2jmb[6]  = UVX_M2JMB_TEMP_UC; // 1 byte
			buff_tx_m2jmb[7]  = UVX_M2JMB_CURRENT_UC; // 2 bytes
			buff_tx_m2jmb[8]  = UVX_M2JMB_RPM_UC;  // 2 bytes

			buff_tx_m2jmb[9]  = UVX_M2JMB_TEMP_LM; // 1 byte
			buff_tx_m2jmb[10] = UVX_M2JMB_CURRENT_LM; // 2 bytes
			buff_tx_m2jmb[11] = UVX_M2JMB_RPM_LM; // 2 bytes

			buff_tx_m2jmb[12] = UVX_M2JMB_TEMP_LC; // 1 byte
			buff_tx_m2jmb[13] = UVX_M2JMB_CURRENT_LC; // 2 bytes
			buff_tx_m2jmb[14] = UVX_M2JMB_RPM_LC; // 2 bytes

			buff_tx_m2jmb[15] = UVX_M2JMB_VOLTAGE; // 2 bytes 22

			buff_tx_m2jmb[16] = UVX_M2JMB_CYCLIC_MAGN_UP; // 2 bytes
			buff_tx_m2jmb[17] = UVX_M2JMB_CYCLIC_ERROR_UP; // 2 bytes

			buff_tx_m2jmb[18] = UVX_M2JMB_CYCLIC_MAGN_LP; // 2 bytes
			buff_tx_m2jmb[19] = UVX_M2JMB_CYCLIC_ERROR_LP; // 2 bytes 30

			buff_tx_m2jmb[20] = UVX_M2JMB_CYCLIC_SP_UP; // 2 bytes
			buff_tx_m2jmb[21] = UVX_M2JMB_CYCLIC_UP; // 2 bytes
			buff_tx_m2jmb[22] = UVX_M2JMB_CYCLIC_FF_UP; // 2 bytess
			buff_tx_m2jmb[23] = UVX_M2JMB_CYCLIC_IQ_UP; // 2 bytess
			buff_tx_m2jmb[24] = UVX_M2JMB_MAIN_IQ_UP; // 2 bytess 40

			buff_tx_m2jmb[25] = UVX_M2JMB_CYCLIC_SP_LP; // 2 bytess
			buff_tx_m2jmb[26] = UVX_M2JMB_CYCLIC_LP; // 2 bytess
			buff_tx_m2jmb[27] = UVX_M2JMB_CYCLIC_FF_LP;// 2 bytess
			buff_tx_m2jmb[28] = UVX_M2JMB_CYCLIC_IQ_LP;// 2 bytess
			buff_tx_m2jmb[29] = UVX_M2JMB_MAIN_IQ_LP;// 2 bytess 50

			buff_tx_m2jmb[30] = UVX_M2JMB_ANGL_HALL_UM; // 1 byte
			buff_tx_m2jmb[31] = UVX_M2JMB_ANGL_HALL_UC; // 1 byte
			buff_tx_m2jmb[32] = UVX_M2JMB_ANGL_HALL_LM; // 1 byte
			buff_tx_m2jmb[33] = UVX_M2JMB_ANGL_HALL_LC; // 1 byte // 54
			buff_tx_m2jmb[34] = UVX_M2JMB_TRUST_RU; // 2 bytess
			buff_tx_m2jmb[35] = UVX_M2JMB_AUTO_RESET; // 1 byte 57

			buff_tx_m2jmb[36] = UVX_M2JMB_CYCLIC_ID_UP; // 2 bytess
			buff_tx_m2jmb[37] = UVX_M2JMB_MAIN_ID_UP; // 2 bytess 61

			buff_tx_m2jmb[38] = UVX_M2JMB_CYCLIC_ID_LP;// 2 bytess 63
			buff_tx_m2jmb[39] = UVX_M2JMB_MAIN_ID_LP;// 2 bytess 65

			buff_tx_m2jmb[40] = UVX_M2JMB_MOD_ID_UM; // 1 byte
			buff_tx_m2jmb[41] = UVX_M2JMB_MOD_ID_UC; // 1 byte
			buff_tx_m2jmb[42] = UVX_M2JMB_MOD_ID_LM; // 1 byte
			buff_tx_m2jmb[43] = UVX_M2JMB_MOD_ID_LC; // 1 byte 69

			buff_tx_m2jmb[44] = UVX_M2JMB_TRUST_ROLL; // 2 bytess
			buff_tx_m2jmb[45] = UVX_M2JMB_TRUST_PITCH; // 2 bytess
			buff_tx_m2jmb[46] = UVX_M2JMB_TRUST_YAW; // 2 bytess
			buff_tx_m2jmb[47] = UVX_M2JMB_TRUST_THROTLE; // 2 bytes
			buff_tx_m2jmb[48] = UVX_M2JMB_ALTITUDE; // 2 bytess
			buff_tx_m2jmb[49] = UVX_M2JMB_RC_INPUT; // 2 bytess
			buff_tx_m2jmb[50] = UVX_M2JMB_FLAGS; // 1 byte
			buff_tx_m2jmb[51] = UVX_M2JMB_BATT_CURRENT; // 3 bytes - 85bytes total			
			// buff_tx_m2jmb[50] = UVX_M2JMB_FLAGS; // 1 byte - 82bytes total

			// buff_tx_m2jmb[51] = UVX_M2JMB_BATT_SOH; 				// 1 bytes 
			// buff_tx_m2jmb[52] = UVX_M2JMB_BATT_VOLTAGE; 			// 2 bytes 
			// buff_tx_m2jmb[53] = UVX_M2JMB_BATT_TEMPERATURE; 		// 2 bytes 
			// buff_tx_m2jmb[54] = UVX_M2JMB_BATT_CURRENT; 			// 2 bytes 89
			// buff_tx_m2jmb[55] = UVX_M2JMB_BATT_CHARGE; 				// 2 bytes 91
			// buff_tx_m2jmb[56] = UVX_M2JMB_BATT_CAPACITY; 			// 2 bytes 
			// buff_tx_m2jmb[57] = UVX_M2JMB_BATT_DESIGN_CAPACITY; 	// 2 bytes 
			// buff_tx_m2jmb[58] = UVX_M2JMB_BATT_PERCENTAGE; 			// 1 bytes 
			// buff_tx_m2jmb[59] = UVX_M2JMB_BATT_SUPPLY_STATUS; 		// 1 bytes 
			// buff_tx_m2jmb[60] = UVX_M2JMB_BATT_SUPPLY_HEALTH; 		// 1 bytes 98
			// buff_tx_m2jmb[61] = UVX_M2JMB_BATT_CELL_VOLTAGE; 		// 20 bytes 118
			// buff_tx_m2jmb[62] = UVX_M2JMB_BATT_CELL_TEMPERATURE; 	// 20 bytes - 138 bytes total

			for(uint16_t i = 0; i < size; i++) 
			{
				//buff_tx_m2jmb[63 + i] = data[i]; // Copy the data to the buffer
				buff_tx_m2jmb[52 + i] = data[i]; // Copy the data to the buffer
			}

			//check crc of all data
			buff_tx_m2jmb[52 + size] = uvx_crc_8(buff_tx_m2jmb, 52 + size); // CRC over the 4 bytes before it and the data			
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 53 + size);
		break;

		case CMD_TURN_OFF:
			buff_tx_m2jmb[2] = uvx_crc_8(buff_tx_m2jmb, 2); // CRC over the 2 bytes before it
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 3); // Send the command over UART
		break;

		case CMD_PWR_ON_FC_ACK:
			buff_tx_m2jmb[1] = CMD_PWR_ON_FC; // Change command byte to power on command
			buff_tx_m2jmb[2] = RSP_ACK; // Acknowledgment response
			buff_tx_m2jmb[3] = uvx_crc_8(buff_tx_m2jmb, 3); // CRC over the 2 bytes before it
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 4); // Send the command over UART
		break;

		case CMD_PWR_OFF_FC_ACK:
			buff_tx_m2jmb[1] = CMD_PWR_OFF_FC; // Change command byte to power off command
			buff_tx_m2jmb[2] = RSP_ACK; // Acknowledgment response
			buff_tx_m2jmb[3] = uvx_crc_8(buff_tx_m2jmb, 3); // CRC over the 2 bytes before it
			uvx_uart_send(comm_m2jmb.p_hal_uart, buff_tx_m2jmb, 4); // Send the command over UART
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
	comm_m2jmb.p_buff_rx[comm_m2jmb.buff_rx_cnt] = byte_rx; // Store the received byte in the RX buffer

	if (((buff_rx_m2jmb[0]) == UVX_START_BYTE)  && (comm_m2jmb.RX_Ready == false)) // Check if the first byte is the start byte and RX is not ready
	{
		if(comm_m2jmb.buff_rx_cnt <= comm_m2jmb.buff_size_rx) // Check if RX buffer count is within bounds
		{
			comm_m2jmb.buff_rx_cnt++; // Increment the RX buffer count
		}	    
		else 
		{
			// If RX buffer is full, reset the count
			comm_m2jmb.buff_rx_cnt = 0; // Reset RX buffer count
			comm_m2jmb.size_frame = 0; // Reset expected frame size
			comm_m2jmb.RX_Ready = 0; // Reset RX ready flag
		}

		// After receiving payload size bytes, calculate expected frame size
		if (comm_m2jmb.buff_rx_cnt == 2) 
		{
			switch (buff_rx_m2jmb[1]) // Check the command byte
			{
				case CTRL_READ:
				case CTRL_WRITE:
				case CTRL_SEND:
				case CMD_HEARTBEAT:
					comm_m2jmb.size_payload = 0; // No payload for these commands
					comm_m2jmb.size_frame = 3; 
				break;

				case CMD_PWR_ON_FC:
				case CMD_PWR_OFF_FC:
					comm_m2jmb.size_payload = 0; // No payload for these commands
					comm_m2jmb.size_frame = 2; 
				break;
			}
		}

		// Wait until the full frame is received
		if (comm_m2jmb.size_frame && comm_m2jmb.buff_rx_cnt == comm_m2jmb.size_frame + 1) 
		{
			if(buff_rx_m2jmb[1] == CMD_PWR_ON_FC)
			{
				buff_rx_m2jmb[1] = CMD_PWR_ON_FC;
			}

			// Check CRC
			uint8_t crc = uvx_crc_8(buff_rx_m2jmb, comm_m2jmb.size_frame);

			if (buff_rx_m2jmb[comm_m2jmb.size_frame] == crc) 
			{
				//comm_m2jmb.buff_rx_cnt = 0;
				comm_m2jmb.RX_Ready = 1; // Notify RX ready				
			} 
			else 
			{
				comm_m2jmb.buff_rx_cnt = 0;
				comm_m2jmb.size_frame = 0;
			}
		}		
	}

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
