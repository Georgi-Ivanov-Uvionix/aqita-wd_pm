
#include "uvx_comm_bq.h"
#include "uvx_crc8.h"
#include <string.h>

static void uvx_comm_bq_unlock_i2c(UVX_COMM_BQ* p_comm_bq);

static UVX_I2C_STATE uvx_comm_bq_lock_i2c(UVX_COMM_BQ* p_comm_bq)
{
	UVX_I2C_STATE state;
	UVX_I2C_TRANSFER_STATE transfer_state;

	if((p_comm_bq == NULL) || (p_comm_bq->p_hal_i2c == NULL))
	{
		return UVX_I2C_ERROR;
	}

	/*
	 * Synchronize the module ownership flag with the asynchronous transfer.
	 * A completion can occur between application state-machine iterations.
	 * In that case the old BQ transaction is finished and its lock must be
	 * released before this module attempts the next request.
	 */
	if(p_comm_bq->owns_i2c_lock != 0U)
	{
		transfer_state = uvx_i2c_get_transfer_state(p_comm_bq->p_hal_i2c);

		if((transfer_state == UVX_I2C_TRANSFER_COMPLETE) ||
		   (transfer_state == UVX_I2C_TRANSFER_ERROR))
		{
			p_comm_bq->RX_Ready = 1U;
			p_comm_bq->TX_Ready = 1U;
			uvx_comm_bq_unlock_i2c(p_comm_bq);
		}
		else if(transfer_state == UVX_I2C_TRANSFER_PENDING)
		{
			return UVX_I2C_BUSY;
		}
		else if(p_comm_bq->p_hal_i2c->lock == 0U)
		{
			/* Recover a stale local ownership flag after bus reset/re-init. */
			p_comm_bq->owns_i2c_lock = 0U;
			p_comm_bq->RX_Ready = 1U;
			p_comm_bq->TX_Ready = 1U;
		}
		else
		{
			return UVX_I2C_BUSY;
		}
	}

	state = uvx_i2c_lock(p_comm_bq->p_hal_i2c);

	if(state == UVX_I2C_OK)
	{
		p_comm_bq->owns_i2c_lock = 1U;
	}

	return state;
}

static void uvx_comm_bq_unlock_i2c(UVX_COMM_BQ* p_comm_bq)
{
	if((p_comm_bq != NULL) && (p_comm_bq->owns_i2c_lock != 0U))
	{
		if(uvx_i2c_unlock(p_comm_bq->p_hal_i2c) == UVX_I2C_OK)
		{
			p_comm_bq->owns_i2c_lock = 0U;
		}
	}
}


SRAM1 UVX_BQ_DATA bq_data_l;
SRAM1 UVX_BQ_DATA bq_data_h;

SRAM1 UVX_COMM_BQ comm_bq_l; // BQ communication structure
SRAM1 UVX_COMM_BQ comm_bq_h; // BQ communication structure
SRAM1 UVX_COMM_BQ_STATE_MACHINE comm_bq_state;

// Array of all BQ_L registers with address and size
UVX_BQ_REGISTER bq_l_register_list_read[] = {
    {TEMPERATURE, 				2, (uint8_t*) &bq_data_l.temperature},
    {VOLTAGE, 					2, (uint8_t*) &bq_data_l.voltage},
    {CURRENT, 					2, (uint8_t*) &bq_data_l.current},
    {RELATIVE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_l.relative_state_of_charge},
    {ABSOLUTE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_l.absolute_state_of_charge},
    {REMAINING_CAPACITY, 		2, (uint8_t*) &bq_data_l.remaining_capacity},
    {FULL_CHARGE_CAPACITY, 		2, (uint8_t*) &bq_data_l.full_charge_capacity},
    {AVERAGE_TIME_TO_EMPTY, 	2, (uint8_t*) &bq_data_l.average_time_to_empty},
    {AVERAGE_TIME_TO_FULL, 		2, (uint8_t*) &bq_data_l.average_time_to_full},
    {CHARGING_CURRENT, 			2, (uint8_t*) &bq_data_l.charging_current},
    {CHARGING_VOLTAGE, 			2, (uint8_t*) &bq_data_l.charging_voltage},
	{DA_STATUS1, 			   32, (uint8_t*) &bq_data_l.dastatus1_size},
	{DA_STATUS2, 			   16, (uint8_t*) &bq_data_l.dastatus2_size},
	{DA_STATUS3, 			   18, (uint8_t*) &bq_data_l.dastatus3_size},
	{OPERATION_STATUS, 		    5, (uint8_t*) &bq_data_l.operation_status.raw},
    {GAUGING_STATUS, 		    4, (uint8_t*) &bq_data_l.gauging_status.raw},
    {GAUGING_STATUS_2, 		   32, (uint8_t*) &bq_data_l.gauging_status_2_size},
    {GAUGING_STATUS_3, 		   32, (uint8_t*) &bq_data_l.gauging_status_3_size},
	{MANUFACTURING_STATUS,   	3, (uint8_t*) &bq_data_l.manufacturing_status.raw},	
	{CBSTATUS,   			   30, (uint8_t*) &bq_data_l.CBSTATUS_size},	
	{STATE_OF_HEALTH,   	    1, (uint8_t*) &bq_data_l.SOH},	
	{CYCLE_COUNT,   	    	2, (uint8_t*) &bq_data_l.cycle_count},
    {END_REGISTER, 				0, NULL} // End marker
};

// Array of all BQ registers with address and size
UVX_BQ_REGISTER bq_l_register_list_read_once[] = {
    {DESIGN_CAPACITY, 			2, (uint8_t*) &bq_data_l.design_capacity},
    {DESIGN_VOLTAGE, 			2, (uint8_t*) &bq_data_l.design_voltage},
    {SPECIFICATION_INFO, 		2, (uint8_t*) &bq_data_l.specification_info},
    {MANUFACTURER_DATE, 		2, (uint8_t*) &bq_data_l.manufacturer_date},
    {SERIAL_NUMBER, 			2, (uint8_t*) &bq_data_l.serial_number},
    {END_REGISTER,		 		0, NULL} // End marker
};

//====================================================================================================

// Array of all BQ_L registers with address and size
UVX_BQ_REGISTER bq_h_register_list_read[] = {
    {TEMPERATURE, 				2, (uint8_t*) &bq_data_h.temperature},
    {VOLTAGE, 					2, (uint8_t*) &bq_data_h.voltage},
    {CURRENT, 					2, (uint8_t*) &bq_data_h.current},
    {RELATIVE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_h.relative_state_of_charge},
    {ABSOLUTE_STATE_OF_CHARGE, 	1, (uint8_t*) &bq_data_h.absolute_state_of_charge},
    {REMAINING_CAPACITY, 		2, (uint8_t*) &bq_data_h.remaining_capacity},
    {FULL_CHARGE_CAPACITY, 		2, (uint8_t*) &bq_data_h.full_charge_capacity},
    {AVERAGE_TIME_TO_EMPTY, 	2, (uint8_t*) &bq_data_h.average_time_to_empty},
    {AVERAGE_TIME_TO_FULL, 		2, (uint8_t*) &bq_data_h.average_time_to_full},
    {CHARGING_CURRENT, 			2, (uint8_t*) &bq_data_h.charging_current},
    {CHARGING_VOLTAGE, 			2, (uint8_t*) &bq_data_h.charging_voltage},
	{DA_STATUS1, 			   32, (uint8_t*) &bq_data_h.dastatus1_size},
	{DA_STATUS2, 			   16, (uint8_t*) &bq_data_h.dastatus2_size},
	{DA_STATUS3, 			   18, (uint8_t*) &bq_data_h.dastatus3_size},
	{OPERATION_STATUS, 		    5, (uint8_t*) &bq_data_h.operation_status.raw},	
    {GAUGING_STATUS, 		    4, (uint8_t*) &bq_data_h.gauging_status.raw},
	{GAUGING_STATUS_2, 		   32, (uint8_t*) &bq_data_h.gauging_status_2_size},
	{GAUGING_STATUS_3, 		   32, (uint8_t*) &bq_data_h.gauging_status_3_size},
	{MANUFACTURING_STATUS,   	3, (uint8_t*) &bq_data_h.manufacturing_status.raw},
	{CBSTATUS,   			   30, (uint8_t*) &bq_data_h.CBSTATUS_size},
	{STATE_OF_HEALTH,   	    1, (uint8_t*) &bq_data_h.SOH},
	{CYCLE_COUNT,   	    	2, (uint8_t*) &bq_data_h.cycle_count},
    {END_REGISTER, 				0, NULL} // End marker
};

// Array of all BQ registers with address and size
UVX_BQ_REGISTER bq_h_register_list_read_once[] = {
    {DESIGN_CAPACITY, 			2, (uint8_t*) &bq_data_h.design_capacity},
    {DESIGN_VOLTAGE, 			2, (uint8_t*) &bq_data_h.design_voltage},
    {SPECIFICATION_INFO, 		2, (uint8_t*) &bq_data_h.specification_info},
    {MANUFACTURER_DATE, 		2, (uint8_t*) &bq_data_h.manufacturer_date},
    {SERIAL_NUMBER, 			2, (uint8_t*) &bq_data_h.serial_number},
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
		p_comm_bq->owns_i2c_lock = 0U;
		p_comm_bq->busy_reason = UVX_BQ_BUSY_NONE;

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
	UVX_I2C_STATE i2c_state;

	if((p_comm_bq == NULL) || (p_comm_bq->p_register_list == NULL) ||
	   (p_comm_bq->p_hal_i2c == NULL))
	{
		return UVX_BQ_ERROR;
	}

	if(p_comm_bq->p_register_list[reg_index].reg_addr == END_REGISTER)
	{
		return UVX_BQ_REG_END;
	}

	/*
	 * owns_i2c_lock and the shared I2C lock are the transaction authority.
	 * RX_Ready is retained as a compatibility/status flag, but must not act
	 * as a second mutex: it can be stale after another device used the bus.
	 */
	if(uvx_comm_bq_lock_i2c(p_comm_bq) != UVX_I2C_OK)
	{
		p_comm_bq->busy_reason = (p_comm_bq->owns_i2c_lock != 0U) ?
			UVX_BQ_BUSY_OWNS_PREVIOUS_TRANSFER :
			((p_comm_bq->p_hal_i2c->lock != 0U) ?
			 UVX_BQ_BUSY_SHARED_I2C_LOCK :
			 UVX_BQ_BUSY_I2C_TRANSFER_NOT_IDLE);
		return UVX_BQ_ERROR_BUSY;
	}

	p_comm_bq->busy_reason = UVX_BQ_BUSY_NONE;
	p_comm_bq->RX_Ready = 0U;

	i2c_state = uvx_i2c_read_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c,
		p_comm_bq->p_register_list[reg_index].reg_addr, 1,
		p_comm_bq->p_register_list[reg_index].p_data,
		p_comm_bq->p_register_list[reg_index].size_data);

	if(i2c_state != UVX_I2C_OK)
	{
		p_comm_bq->RX_Ready = 1U;
		uvx_comm_bq_unlock_i2c(p_comm_bq);
		return (i2c_state == UVX_I2C_BUSY) ?
			UVX_BQ_ERROR_BUSY : UVX_BQ_ERROR;
	}
    
	return UVX_BQ_OK; // Return success
}  

UVX_COMM_BQ_STATE uvx_comm_bq_read_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTERS reg_addr) 
{   
	uint8_t reg_index = 0;
	UVX_I2C_STATE i2c_state;
	UVX_COMM_BQ_STATE state = UVX_BQ_OK;

	if((p_comm_bq == NULL) || (p_comm_bq->p_register_list == NULL) ||
	   (p_comm_bq->p_hal_i2c == NULL))
	{
		return UVX_BQ_ERROR;
	}

	state = uvx_comm_bq_get_index_register(p_comm_bq->p_register_list, reg_addr, &reg_index);
	if(state != UVX_BQ_OK)
	{
		return state;
	}

	if(uvx_comm_bq_lock_i2c(p_comm_bq) != UVX_I2C_OK)
	{
		p_comm_bq->busy_reason = (p_comm_bq->owns_i2c_lock != 0U) ?
			UVX_BQ_BUSY_OWNS_PREVIOUS_TRANSFER :
			((p_comm_bq->p_hal_i2c->lock != 0U) ?
			 UVX_BQ_BUSY_SHARED_I2C_LOCK :
			 UVX_BQ_BUSY_I2C_TRANSFER_NOT_IDLE);
		return UVX_BQ_ERROR_BUSY;
	}

	p_comm_bq->busy_reason = UVX_BQ_BUSY_NONE;
	p_comm_bq->RX_Ready = 0U;

	i2c_state = uvx_i2c_read_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c,
		 p_comm_bq->p_register_list[reg_index].reg_addr, 1,
		 p_comm_bq->p_register_list[reg_index].p_data,
		 p_comm_bq->p_register_list[reg_index].size_data);

	if(i2c_state != UVX_I2C_OK)
	{
		p_comm_bq->RX_Ready = 1U;
		uvx_comm_bq_unlock_i2c(p_comm_bq);
		return (i2c_state == UVX_I2C_BUSY) ?
			UVX_BQ_ERROR_BUSY : UVX_BQ_ERROR;
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
		if(uvx_comm_bq_lock_i2c(p_comm_bq) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR_BUSY;
		}

		p_comm_bq->TX_Ready = 0; // Reset TX ready flag

		memcpy(p_comm_bq->i2c_tx_staging, data, size);

		if(uvx_i2c_send_mem(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, reg_addr, 1, p_comm_bq->i2c_tx_staging, size) != UVX_I2C_OK)
		{
			p_comm_bq->TX_Ready = 1;
			uvx_comm_bq_unlock_i2c(p_comm_bq);
			return UVX_BQ_ERROR;
		}
	}
	else
	{
		return UVX_BQ_ERROR_BUSY;
	}
	
	return UVX_BQ_OK; // Return success
}

UVX_COMM_BQ_STATE uvx_comm_bq_write_mba_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_MA_REGISTERS reg_addr, uint8_t* data, uint16_t size) 
{    
	uint8_t i2c_data[20] = {0};

	if(p_comm_bq->TX_Ready == 1)
	{
		if(uvx_comm_bq_lock_i2c(p_comm_bq) != UVX_I2C_OK)
		{
			return UVX_BQ_ERROR_BUSY;
		}

		p_comm_bq->TX_Ready = 0; // Reset TX ready flag

		i2c_data[0] = 0x44;
		i2c_data[1] = 0x02;
		i2c_data[2] = reg_addr;
		i2c_data[3] = 0x00;

		if(uvx_i2c_send(p_comm_bq->p_hal_i2c, p_comm_bq->addr_i2c, i2c_data, 4) != UVX_I2C_OK)
		{
			p_comm_bq->TX_Ready = 1;
			uvx_comm_bq_unlock_i2c(p_comm_bq);
			return UVX_BQ_ERROR;
		}
	}
	else
	{
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
			uvx_comm_bq_write_mba_register(&comm_bq_h, BQ_MA_CHG_FET_TOGGLE, NULL, 0);
		}
	}
	else
	{
		if(batt_data.CHG_fet_stat)
		{
			uvx_comm_bq_write_mba_register(&comm_bq_h, BQ_MA_CHG_FET_TOGGLE, NULL, 0);
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
