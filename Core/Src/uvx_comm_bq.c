
#include "uvx_comm_bq.h"
#include "uvx_crc8.h"
#include <string.h>


SRAM1 UVX_BQ_DATA bq_data_1;
SRAM1 UVX_BQ_DATA bq_data_2;
SRAM1 UVX_BQ_DATA bq_data_3;

SRAM1 UVX_COMM_BQ comm_bq_1; // BQ communication structure
SRAM1 UVX_COMM_BQ comm_bq_2; // BQ communication structure
SRAM1 UVX_COMM_BQ comm_bq_3; // BQ communication structure
SRAM1 UVX_COMM_BQ_STATE_MACHINE comm_bq_state;

// Array of all BQ_1 registers with address and size
UVX_BQ_REGISTER bq_1_register_list_read[] = {
    {TEMPERATURE, 				2, (uint8_t*) &bq_data_1.temperature},
    {VOLTAGE, 					2, (uint8_t*) &bq_data_1.voltage},
    {CURRENT, 					2, (uint8_t*) &bq_data_1.current},
    {RELATIVE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_1.relative_state_of_charge},
    {ABSOLUTE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_1.absolute_state_of_charge},
    {REMAINING_CAPACITY, 		2, (uint8_t*) &bq_data_1.remaining_capacity},
    {FULL_CHARGE_CAPACITY, 		2, (uint8_t*) &bq_data_1.full_charge_capacity},
    {AVERAGE_TIME_TO_EMPTY, 	2, (uint8_t*) &bq_data_1.average_time_to_empty},
    {AVERAGE_TIME_TO_FULL, 		2, (uint8_t*) &bq_data_1.average_time_to_full},
    {CHARGING_CURRENT, 			2, (uint8_t*) &bq_data_1.charging_current},
    {CHARGING_VOLTAGE, 			2, (uint8_t*) &bq_data_1.charging_voltage},
	{DA_STATUS1, 			   32, (uint8_t*) &bq_data_1.dastatus1_size},
	{DA_STATUS2, 			   16, (uint8_t*) &bq_data_1.dastatus2_size},
	{DA_STATUS3, 			   18, (uint8_t*) &bq_data_1.dastatus3_size},
	{OPERATION_STATUS, 		    5, (uint8_t*) &bq_data_1.operation_status.raw},
    {GAUGING_STATUS, 		    4, (uint8_t*) &bq_data_1.gauging_status.raw},
    {GAUGING_STATUS_2, 		   32, (uint8_t*) &bq_data_1.gauging_status_2_size},
    {GAUGING_STATUS_3, 		   32, (uint8_t*) &bq_data_1.gauging_status_3_size},
	{MANUFACTURING_STATUS,   	3, (uint8_t*) &bq_data_1.manufacturing_status.raw},	
	{CBSTATUS,   			   30, (uint8_t*) &bq_data_1.CBSTATUS_size},	
	{STATE_OF_HEALTH,   	    1, (uint8_t*) &bq_data_1.SOH},	
	{CYCLE_COUNT,   	    	2, (uint8_t*) &bq_data_1.cycle_count},
    {END_REGISTER, 				0, NULL} // End marker
};

// Array of all BQ registers with address and size
UVX_BQ_REGISTER bq_1_register_list_read_once[] = {
    {DESIGN_CAPACITY, 			2, (uint8_t*) &bq_data_1.design_capacity},
    {DESIGN_VOLTAGE, 			2, (uint8_t*) &bq_data_1.design_voltage},
    {SPECIFICATION_INFO, 		2, (uint8_t*) &bq_data_1.specification_info},
    {MANUFACTURER_DATE, 		2, (uint8_t*) &bq_data_1.manufacturer_date},
    {SERIAL_NUMBER, 			2, (uint8_t*) &bq_data_1.serial_number},
    {END_REGISTER,		 		0, NULL} // End marker
};

//====================================================================================================

// Array of all BQ_2 registers with address and size
UVX_BQ_REGISTER bq_2_register_list_read[] = {
    {TEMPERATURE, 				2, (uint8_t*) &bq_data_2.temperature},
    {VOLTAGE, 					2, (uint8_t*) &bq_data_2.voltage},
    {CURRENT, 					2, (uint8_t*) &bq_data_2.current},
    {RELATIVE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_2.relative_state_of_charge},
    {ABSOLUTE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_2.absolute_state_of_charge},
    {REMAINING_CAPACITY, 		2, (uint8_t*) &bq_data_2.remaining_capacity},
    {FULL_CHARGE_CAPACITY, 		2, (uint8_t*) &bq_data_2.full_charge_capacity},
    {AVERAGE_TIME_TO_EMPTY, 	2, (uint8_t*) &bq_data_2.average_time_to_empty},
    {AVERAGE_TIME_TO_FULL, 		2, (uint8_t*) &bq_data_2.average_time_to_full},
    {CHARGING_CURRENT, 			2, (uint8_t*) &bq_data_2.charging_current},
    {CHARGING_VOLTAGE, 			2, (uint8_t*) &bq_data_2.charging_voltage},
	{DA_STATUS1, 			   32, (uint8_t*) &bq_data_2.dastatus1_size},
	{DA_STATUS2, 			   16, (uint8_t*) &bq_data_2.dastatus2_size},
	{DA_STATUS3, 			   18, (uint8_t*) &bq_data_2.dastatus3_size},
	{OPERATION_STATUS, 		    5, (uint8_t*) &bq_data_2.operation_status.raw},	
    {GAUGING_STATUS, 		    4, (uint8_t*) &bq_data_2.gauging_status.raw},
	{GAUGING_STATUS_2, 		   32, (uint8_t*) &bq_data_2.gauging_status_2_size},
	{GAUGING_STATUS_3, 		   32, (uint8_t*) &bq_data_2.gauging_status_3_size},
	{MANUFACTURING_STATUS,   	3, (uint8_t*) &bq_data_2.manufacturing_status.raw},
	{CBSTATUS,   			   30, (uint8_t*) &bq_data_2.CBSTATUS_size},
	{STATE_OF_HEALTH,   	    1, (uint8_t*) &bq_data_2.SOH},
	{CYCLE_COUNT,   	    	2, (uint8_t*) &bq_data_2.cycle_count},
    {END_REGISTER, 				0, NULL} // End marker
};

// Array of all BQ registers with address and size
UVX_BQ_REGISTER bq_2_register_list_read_once[] = {
    {DESIGN_CAPACITY, 			2, (uint8_t*) &bq_data_2.design_capacity},
    {DESIGN_VOLTAGE, 			2, (uint8_t*) &bq_data_2.design_voltage},
    {SPECIFICATION_INFO, 		2, (uint8_t*) &bq_data_2.specification_info},
    {MANUFACTURER_DATE, 		2, (uint8_t*) &bq_data_2.manufacturer_date},
    {SERIAL_NUMBER, 			2, (uint8_t*) &bq_data_2.serial_number},
    {END_REGISTER,		 		0, NULL} // End marker
};

UVX_COMM_BQ_STATE uvx_comm_bq_init(UVX_COMM_BQ* p_comm_bq, UVX_I2C* i2c, uint8_t i2c_addr, UVX_BQ_REGISTER* p_register_list)
{
	if(p_comm_bq != NULL && i2c != NULL) 
	{
		p_comm_bq->p_register_list = p_register_list; // Set the pointer to the register list
		p_comm_bq->addr_i2c = i2c_addr; // Set the I2C device address
		p_comm_bq->p_hal_i2c = &i2c->hal_i2c; // Set the pointer to the I2C HAL structure
		p_comm_bq->i2c = i2c; // Set the pointer to the I2C structure
		p_comm_bq->TX_Ready = 1;
		p_comm_bq->RX_Ready = 1;
		p_comm_bq->Force_balance_old = 1; // Initialize Force_balance_old to a different value to ensure the first write occurs

		#ifdef PROJECT_AQITA_PM
			if(uvx_i2c_init(p_comm_bq->i2c/*, buff_tx_bq, buff_rx_bq*/)) // Initialize the I2C peripheral
			{
				return UVX_BQ_ERROR_INIT_I2C; // Return error if initialization fails
			}
		#endif
	}

	return UVX_BQ_OK; // Return success
}


UVX_COMM_BQ_STATE uvx_comm_bq_change_list(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTER* p_register_list)
{
	if((p_comm_bq != NULL) && (p_register_list != NULL))
	{
		p_comm_bq->p_register_list = p_register_list; // Set the pointer to the register list

		// #ifdef PROJECT_AQITA_PM
		// 	if(uvx_i2c_init(p_comm_bq->i2c/*, buff_tx_bq, buff_rx_bq*/)) // Initialize the I2C peripheral
		// 	{
		// 		return UVX_BQ_ERROR_INIT_I2C; // Return error if initialization fails
		// 	}
		// #endif
	}

	return UVX_BQ_OK; // Return success
}

UVX_COMM_BQ_STATE uvx_comm_bq_read_list(UVX_COMM_BQ* p_comm_bq, uint16_t reg_index) 
{    
	if(p_comm_bq->RX_Ready == 1)
	{					
		if(p_comm_bq->p_register_list[reg_index].reg_addr == END_REGISTER)
		{
			return UVX_BQ_REG_END;
		}

		if(p_comm_bq->p_register_list[reg_index].reg_addr == DA_STATUS1)
		{
			p_comm_bq->RX_Ready = 1; // Reset TX ready flag	
		}		
		
		p_comm_bq->RX_Ready = 0; // Reset TX ready flag	

		if(uvx_i2c_read_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c,
			p_comm_bq->p_register_list[reg_index].reg_addr, 1,
			p_comm_bq->p_register_list[reg_index].p_data,
			p_comm_bq->p_register_list[reg_index].size_data) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR;
		}			
		
	}
	else
	{
		return UVX_BQ_ERROR_BUSY;
	}
    
	return UVX_BQ_OK; // Return success
}  

UVX_COMM_BQ_STATE uvx_comm_bq_read_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTERS reg_addr) 
{    
	uint8_t reg_index = 0;


	UVX_COMM_BQ_STATE state = UVX_BQ_OK;
	if(p_comm_bq->RX_Ready == 1)
	{		
		p_comm_bq->RX_Ready = 0; // Reset TX ready flag				
		
		state = uvx_comm_bq_get_index_register(p_comm_bq->p_register_list, reg_addr, &reg_index);
		if(state != UVX_BQ_OK)
		{
			return state;
		}

		if(uvx_i2c_read_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c,
			 p_comm_bq->p_register_list[reg_index].reg_addr, 1,
			 p_comm_bq->p_register_list[reg_index].p_data,
			 p_comm_bq->p_register_list[reg_index].size_data) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR;
		}
	}
	else
	{
		return UVX_BQ_ERROR_BUSY;
	}
    
	return UVX_BQ_OK; // Return success
}  

//read ManufactureAddress() function only for test
UVX_COMM_BQ_STATE uvx_comm_bq_read_ma_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTERS reg_addr) 
{    
	uint8_t reg_index = 0;
	UVX_COMM_BQ_STATE state = UVX_BQ_OK;
	uint8_t i2c_data[10] = {0};
	
	p_comm_bq->RX_Ready = 0; // Reset TX ready flag		
	
	state = uvx_comm_bq_get_index_register(p_comm_bq->p_register_list, reg_addr, &reg_index);
	if(state != UVX_BQ_OK)
	{
		return state;
	}

	i2c_data[0] = 0x00;
	i2c_data[1] = 0x00;		
	i2c_data[2] = reg_addr;

	if(uvx_i2c_send(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, i2c_data, 3) != UVX_I2C_OK)
	{
		return UVX_BQ_ERROR;
	}

	if(uvx_i2c_read(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, /*addr, 1,*/
			p_comm_bq->p_register_list[reg_index].p_data,
			p_comm_bq->p_register_list[reg_index].size_data) != UVX_I2C_OK)
	{
		return UVX_BQ_ERROR;
	}	
    
	return UVX_BQ_OK; // Return success
}  

UVX_COMM_BQ_STATE uvx_comm_bq_get_index_register(UVX_BQ_REGISTER *list, UVX_BQ_REGISTERS reg_addr, uint8_t* p_index)
{
	uint8_t index = 0;

	while(list[index].reg_addr != END_REGISTER)
	{
		if(list[index].reg_addr == reg_addr)
		{
			*p_index = index;
			return UVX_BQ_OK; // Return success
		}

		index++;
	}

	return UVX_BQ_ERROR; // Return error if register not found
}

UVX_COMM_BQ_STATE uvx_comm_bq_write_register(UVX_COMM_BQ* p_comm_bq, uint16_t reg_addr, uint8_t* data, uint16_t size) 
{    
	if((p_comm_bq == NULL) || (data == NULL) || (size == 0) || (size > UVX_BQ_I2C_TX_STAGING_SIZE))
	{
		return UVX_BQ_ERROR;
	}

	if(p_comm_bq->TX_Ready == 1)
	{
		p_comm_bq->TX_Ready = 0; // Reset TX ready flag

		memcpy(p_comm_bq->i2c_tx_staging, data, size);

		if(uvx_i2c_send_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, reg_addr, 1, p_comm_bq->i2c_tx_staging, size) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR;
		}
	}
	else
	{
		if(uvx_i2c_check_state(p_comm_bq->p_hal_i2c) == UVX_I2C_TX_READY)
		{
			p_comm_bq->TX_Ready = 1;
		}

		return UVX_BQ_ERROR_BUSY;
	}
	
	return UVX_BQ_OK; // Return success
}

UVX_COMM_BQ_STATE uvx_comm_bq_write_mba_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_MA_REGISTERS reg_addr, uint8_t* data, uint16_t size) 
{    
	uint8_t i2c_data[20] = {0};

	if(p_comm_bq->TX_Ready == 1)
	{
		p_comm_bq->TX_Ready = 0; // Reset TX ready flag

		i2c_data[0] = 0x44;
		i2c_data[1] = 0x02;
		i2c_data[2] = reg_addr;
		i2c_data[3] = 0x00;

		if(uvx_i2c_send(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, i2c_data, 4) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR;
		}
	}
	else
	{
		if(uvx_i2c_check_state(p_comm_bq->p_hal_i2c) == UVX_I2C_TX_READY)
		{
			p_comm_bq->TX_Ready = 1;
		}

		return UVX_BQ_ERROR_BUSY;
	}
	
	return UVX_BQ_OK; // Return success
}

UVX_COMM_BQ_STATE uvx_comm_bq_force_balance(UVX_COMM_BQ* p_comm_bq, uint8_t enable)
{
	uint8_t data[2];

	if(enable)
	{
		data[0] = 0x00;
		data[1] = 0x10;
		p_comm_bq->Force_balance = true;
	}
	else
	{
		data[0] = 0xFF;
		data[1] = 0xFF;
		p_comm_bq->Force_balance = false;
	}

	if(p_comm_bq->Force_balance_old != p_comm_bq->Force_balance)
	{
		p_comm_bq->Force_balance_old = p_comm_bq->Force_balance;
		return uvx_comm_bq_write_register(p_comm_bq, GPIO_WRITE, data, 2);

	}
	else
	{
		return UVX_BQ_OK; // No change, return success
	}	
}

UVX_COMM_BQ_STATE uvx_comm_bq_charge_fet(UVX_COMM_BQ* p_comm_bq, uint8_t state)
{
	if(state)
	{
		if(!batt_data.CHG_fet_stat)
		{
			uvx_comm_bq_write_mba_register(&comm_bq_2, BQ_MA_CHG_FET_TOGGLE, NULL, 0);
		}
	}
	else
	{
		if(batt_data.CHG_fet_stat)
		{
			uvx_comm_bq_write_mba_register(&comm_bq_2, BQ_MA_CHG_FET_TOGGLE, NULL, 0);
		}
	}

	return UVX_BQ_OK; // Return success
}

uint8_t uvx_comm_bq_swap_u8(uint8_t* v)
{
	return *v;
}

uint16_t uvx_comm_bq_swap_u16_pointer(uint16_t* v)
{
	return *v;
}

uint16_t uvx_comm_bq_swap_u16_value(uint16_t v)
{
	 return (uint16_t)((v >> 8) | (v << 8));
}

uint32_t uvx_comm_bq_swap_u32_pointer(uint32_t* v)
{
	return *v;		   
}

uint32_t uvx_comm_bq_swap_u32_value(uint32_t v)
{
	return ((v >> 24) & 0x000000FF) | 
		   ((v >> 8)  & 0x0000FF00) | 
		   ((v << 8)  & 0x00FF0000) | 
		   ((v << 24) & 0xFF000000);
}
