
#include "uvx_comm_m2m.h"
#include "uvx_crc8.h"


uint8_t buff_tx_m2m[BUFF_SIZE_TX_M2M];
uint8_t buff_rx_m2m[BUFF_SIZE_RX_M2M];

UVX_COMM_M2M comm_m2m; // M2M communication structure
UVX_COMM_M2M_STATE_MACHINE comm_m2m_state;

UVX_COMM_M2M_STATE uvx_comm_m2m_init(UVX_UART* uart) 
{
	comm_m2m.p_buff_tx_start = buff_tx_m2m; // Set the start pointer for the TX buffer
	comm_m2m.p_buff_rx_start = buff_rx_m2m; // Set the start pointer for the RX buffer
	comm_m2m.p_buff_tx = buff_tx_m2m; // Set the pointer to the TX buffer
	comm_m2m.p_buff_rx = buff_rx_m2m; // Set the pointer to the RX buffer
	comm_m2m.buff_size_tx = BUFF_SIZE_TX_M2M; // Set the size of the TX buffer
	comm_m2m.buff_size_rx = BUFF_SIZE_RX_M2M; // Set the size of the RX buffer
	comm_m2m.buff_rx_cnt = 0; // Initialize RX buffer count
	comm_m2m.buff_tx_cnt = 0; // Initialize TX buffer count

	if(uart != NULL)
	{
		comm_m2m.p_hal_uart = &uart->hal_uart; // Set the pointer to the UART HAL structure

		#ifdef PROJECT_AQITA_PM
			if(uvx_uart_init(uart, buff_tx_m2m, buff_rx_m2m)) // Initialize the UART peripheral
			{
				return UVX_M2M_ERROR_INIT_UART; // Return error if initialization fails
			}
		#endif
	}

	comm_m2m.TX_Ready = 1;
	return UVX_M2M_OK; // Return success
}


UVX_COMM_M2M_STATE uvx_comm_m2m_read(UVX_M2M_CMD_BYTE cmd_byte) 
{    
    buff_tx_m2m[0] = UVX_START_BYTE;       // Start byte
    buff_tx_m2m[1] = CTRL_READ;        // Read command
    buff_tx_m2m[2] = cmd_byte;              // Target command byte
    buff_tx_m2m[3] = 0x00;                 // Payload size MSB (0 for read request)
    buff_tx_m2m[4] = 0x00;                 // Payload size LSB (0 for read request)
    buff_tx_m2m[5] = uvx_crc_8(buff_tx_m2m, 5);   // CRC over the 5 bytes before it

	//if(comm_m2m.TX_Ready == 1)
	{		
		uvx_uart_send(comm_m2m.p_hal_uart, buff_tx_m2m, 6);
		comm_m2m.TX_Ready = 0; // Reset TX ready flag
	}
    
	return UVX_M2M_OK; // Return success
}  

UVX_COMM_M2M_STATE uvx_comm_m2m_write(uint8_t data) 
{    
	buff_tx_m2m[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2m[1] = CTRL_WRITE;       // Write command
	buff_tx_m2m[2] = data;              // Target command byte
	buff_tx_m2m[3] = 0x00;                 // Payload size MSB (0 for write request)
	buff_tx_m2m[4] = 0x00;                 // Payload size LSB (0 for write request)
	buff_tx_m2m[5] = uvx_crc_8(buff_tx_m2m, 5);   // CRC over the 5 bytes before it

	if(comm_m2m.TX_Ready == 1)
	{
		comm_m2m.TX_Ready = 0; // Reset TX ready flag
		uvx_uart_send(comm_m2m.p_hal_uart, buff_tx_m2m, 6);
	}
	
	return UVX_M2M_OK; // Return success
}

UVX_COMM_M2M_STATE uvx_comm_m2m_send(uint8_t cmd_echo, uint8_t* data, uint16_t size) 
{    
	buff_tx_m2m[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2m[1] = CTRL_SEND;        // Send command
	buff_tx_m2m[2] = cmd_echo;
	buff_tx_m2m[3] = size >> 8; // Payload size MSB
	buff_tx_m2m[4] = (uint8_t)size;

	for(uint16_t i = 0; i < size; i++) 
	{
		buff_tx_m2m[5 + i] = data[i]; // Copy the data to the buffer
	}

	//check crc of all data
	buff_tx_m2m[5 + size] = uvx_crc_8(buff_tx_m2m, 5 + size); // CRC over the 4 bytes before it and the data
	
	if(comm_m2m.TX_Ready == 1)
	{
		comm_m2m.TX_Ready = 0; // Reset TX ready flag
		uvx_uart_send(comm_m2m.p_hal_uart, buff_tx_m2m, 6);
	}
	
	return UVX_M2M_OK; // Return success
}

UVX_COMM_M2M_STATE uvx_comm_m2m_send_error(uint8_t data) 
{    
	buff_tx_m2m[0] = UVX_START_BYTE;       // Start byte
	buff_tx_m2m[1] = CTRL_SEND_ERROR;  // Send error command
	buff_tx_m2m[2] = data;              // Target command byte
	buff_tx_m2m[3] = 0x00;                 // Payload size MSB (0 for send error request)
	buff_tx_m2m[4] = 0x00;                 // Payload size LSB (0 for send error request)
	buff_tx_m2m[5] = uvx_crc_8(buff_tx_m2m, 5);   // CRC over the 5 bytes before it

	if(comm_m2m.TX_Ready == 1)
	{
		comm_m2m.TX_Ready = 0; // Reset TX ready flag
		uvx_uart_send(comm_m2m.p_hal_uart, buff_tx_m2m, 6);
	}

	return UVX_M2M_OK; // Return success
}



UVX_COMM_M2M_STATE uvx_comm_m2m_process_rx(uint8_t byte_rx)
{
	comm_m2m.p_buff_rx[comm_m2m.buff_rx_cnt] = byte_rx; // Store the received byte in the RX buffer

	if (((buff_rx_m2m[0]) == UVX_START_BYTE)  && (comm_m2m.RX_Ready == false)) // Check if the first byte is the start byte and RX is not ready
	{
		if(comm_m2m.buff_rx_cnt <= comm_m2m.buff_size_rx) // Check if RX buffer count is within bounds
		{
			comm_m2m.buff_rx_cnt++; // Increment the RX buffer count
		}	    
		else 
		{
			// If RX buffer is full, reset the count
			comm_m2m.buff_rx_cnt = 0; // Reset RX buffer count
			comm_m2m.size_frame = 0; // Reset expected frame size
			comm_m2m.RX_Ready = 0; // Reset RX ready flag
		}

		// After receiving payload size bytes, calculate expected frame size
		if (comm_m2m.buff_rx_cnt == 5) 
		{
			comm_m2m.size_payload = ((uint16_t)buff_rx_m2m[3] << 8) | buff_rx_m2m[4];
			comm_m2m.size_frame = 6 + comm_m2m.size_payload - 1; // 6 = header(5) + CRC(1)
		}

		// Wait until the full frame is received
		if (comm_m2m.size_frame && comm_m2m.buff_rx_cnt == comm_m2m.size_frame + 1) 
		{
			// Check CRC
			uint8_t crc = uvx_crc_8(buff_rx_m2m, comm_m2m.size_frame);
			if (buff_rx_m2m[comm_m2m.size_frame] == crc) 
			{
				//comm_m2m.buff_rx_cnt = 0;
				comm_m2m.RX_Ready = 1; // Notify RX ready				
			} 
			else 
			{
				comm_m2m.buff_rx_cnt = 0;
				comm_m2m.size_frame = 0;
			}
		}		
	}

    return UVX_M2M_OK;
}

UVX_COMM_M2M_STATE uvx_comm_m2m_process_rx_data(void)
{
	uint8_t resp = 0x05;

	if(comm_m2m.RX_Ready == 1) // If RX is not ready, process the received byte
	{
		switch (comm_m2m.p_buff_rx[1]) // Check the control byte
		{
			case CTRL_READ:

				// Prepare and send a response for READ
				switch(comm_m2m.p_buff_rx[2]) // Check the command byte
				{
					case CMD_READ_DATA_ESC:

						resp = 0x01; // Example response for CMD_READ_DATA_ESC
						uvx_comm_m2m_send(CMD_READ_DATA_ESC, &resp, 1); // Send the command byte as response
					break;

					case CMD_READ_DATA_JMB:		
						resp = 0x02; // Example response for CMD_READ_DATA_JMB								
						uvx_comm_m2m_send(CMD_READ_DATA_ESC, &resp, 1); // Send the command byte as response
					break;
					

					default:
						// Unknown command, send error
						return UVX_M2M_ERROR_UNKNOWN_CMD;
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
				return UVX_M2M_ERROR_UNKNOWN_CMD;
		}
		
		comm_m2m.p_buff_rx[1] = CTRL_NONE; // Reset the command byte to NONE after processing
	}
	
	
	return UVX_M2M_OK; // Return success
}
