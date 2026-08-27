
#include "uvx_batt.h"

SRAM1 UVX_BATT_DATA batt_data;

extern uint32_t g_Drone_Started;

UVX_BATT_STATE_MACHINE batt_state;

static UVX_BATT_STATE uvx_batt_send_learn_commands(UVX_COMM_BQ* p_comm_bq)
{
	static const UVX_BQ_MA_REGISTERS learn_commands[] =
	{
		BQ_MA_LIFETIME_DATA_RESET,
		BQ_MA_PF_DATA_RESET,
		BQ_MA_BLACK_BOX_RESET,
		BQ_MA_DEVICE_RESET
	};
	uint8_t i;

	for(i = 0U; i < (sizeof(learn_commands) / sizeof(learn_commands[0])); i++)
	{
		if(uvx_comm_bq_write_mba_register(p_comm_bq, learn_commands[i], NULL, 0U) != UVX_BQ_OK)
		{
			return UVX_BATT_ERROR;
		}

		p_comm_bq->TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent

		if(learn_commands[i] == BQ_MA_DEVICE_RESET)
		{
			HAL_Delay(100);
		}
		else
		{
			HAL_Delay(10);
		}
	}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_learn(void)
{
	comm_bq_1.TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent
	
	if(uvx_batt_send_learn_commands(&comm_bq_1) != UVX_BATT_OK)
	{
		return UVX_BATT_ERROR;
	}

	comm_bq_2.TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent

	if(uvx_batt_send_learn_commands(&comm_bq_2) != UVX_BATT_OK)
	{
		return UVX_BATT_ERROR;
	}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_parse_data(void)
{
	batt_data.cell_voltage_1  = bq_data_1.cell_voltage_1 - (bq_data_1.current * CELL_1_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_2  = bq_data_1.cell_voltage_2 - (bq_data_1.current * CELL_2_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_3  = bq_data_1.cell_voltage_3 - (bq_data_1.current * CELL_3_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_4  = bq_data_1.cell_voltage_4 - (bq_data_1.current * CELL_4_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_5  = bq_data_1.cell_voltage_5 - (bq_data_1.current * CELL_5_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_6  = bq_data_2.cell_voltage_1 - (bq_data_2.current * CELL_6_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_7  = bq_data_2.cell_voltage_2 - (bq_data_2.current * CELL_7_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_8  = bq_data_2.cell_voltage_3 - (bq_data_2.current * CELL_8_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_9  = bq_data_2.cell_voltage_4 - (bq_data_2.current * CELL_9_INTERCONNECT_RESISTANCE)/1000;
    batt_data.cell_voltage_10 = bq_data_2.cell_voltage_5 - (bq_data_2.current * CELL_10_INTERCONNECT_RESISTANCE)/1000;

	//convert to little endian 
	batt_data.payload.cell_voltage_1  = uvx_comm_bq_swap_u16_value(bq_data_1.cell_voltage_1);
	batt_data.payload.cell_voltage_2  = uvx_comm_bq_swap_u16_value(bq_data_1.cell_voltage_2);
	batt_data.payload.cell_voltage_3  = uvx_comm_bq_swap_u16_value(bq_data_1.cell_voltage_3);
	batt_data.payload.cell_voltage_4  = uvx_comm_bq_swap_u16_value(bq_data_1.cell_voltage_4);
	batt_data.payload.cell_voltage_5  = uvx_comm_bq_swap_u16_value(bq_data_1.cell_voltage_5);
	batt_data.payload.cell_voltage_6  = uvx_comm_bq_swap_u16_value(bq_data_2.cell_voltage_1);
	batt_data.payload.cell_voltage_7  = uvx_comm_bq_swap_u16_value(bq_data_2.cell_voltage_2);
	batt_data.payload.cell_voltage_8  = uvx_comm_bq_swap_u16_value(bq_data_2.cell_voltage_3);
	batt_data.payload.cell_voltage_9  = uvx_comm_bq_swap_u16_value(bq_data_2.cell_voltage_4);
    batt_data.payload.cell_voltage_10 = uvx_comm_bq_swap_u16_value(bq_data_2.cell_voltage_5);

	batt_data.cell_current_1  = bq_data_1.cell_current_1;
	batt_data.cell_current_2  = bq_data_1.cell_current_2;
	batt_data.cell_current_3  = bq_data_1.cell_current_3;
	batt_data.cell_current_4  = bq_data_1.cell_current_4;
	batt_data.cell_current_5  = bq_data_1.cell_current_5;
	batt_data.cell_current_6  = bq_data_2.cell_current_1;
	batt_data.cell_current_7  = bq_data_2.cell_current_2;
	batt_data.cell_current_8  = bq_data_2.cell_current_3;
	batt_data.cell_current_9  = bq_data_2.cell_current_4;
	batt_data.cell_current_10 = bq_data_2.cell_current_5;

	batt_data.cell_power_1  = bq_data_1.cell_power_1;
	batt_data.cell_power_2  = bq_data_1.cell_power_2;
	batt_data.cell_power_3  = bq_data_1.cell_power_3;
	batt_data.cell_power_4  = bq_data_1.cell_power_4;
	batt_data.cell_power_5  = bq_data_1.cell_power_5;
	batt_data.cell_power_6  = bq_data_2.cell_power_1;
	batt_data.cell_power_7  = bq_data_2.cell_power_2;
	batt_data.cell_power_8  = bq_data_2.cell_power_3;
	batt_data.cell_power_9  = bq_data_2.cell_power_4;
	batt_data.cell_power_10 = bq_data_2.cell_power_5;

	uvx_batt_detect_cells();
	uvx_batt_search_cell_diff();

	batt_data.payload.batt_voltage = uvx_comm_bq_swap_u16_value(bq_data_1.voltage + bq_data_2.voltage);
	batt_data.batt_voltage = bq_data_1.voltage + bq_data_2.voltage;

	batt_data.payload.current = uvx_comm_bq_swap_u16_value((bq_data_1.current + bq_data_2.current) / 2);
	batt_data.current = (bq_data_1.current + bq_data_2.current) / 2;

	batt_data.payload.design_capacity = uvx_comm_bq_swap_u16_value(batt_data.design_capacity);

	if(batt_data.voltage_min_cell <= BATT_CELL_MIN_VOLTAGE)
	{
		if(timer_app_batt_low_voltage.Timeout == 0)
		{
			timer_app_batt_low_voltage.Timeout = APP_TIMEOUT_BATT_LOW_VOLTAGE;
			timer_app_batt_low_voltage.Enable = true;
			UVX_APP_Shutdown_JMB();
			drone_state.state_current = DRONE_SLEEP;
		}
	}
	else
	{
		timer_app_batt_low_voltage.Timeout = APP_TIMEOUT_BATT_LOW_VOLTAGE;
		timer_app_batt_low_voltage.Enable = true;
	}


	batt_data.payload.temperature_l_int  = uvx_comm_bq_swap_u16_value(bq_data_1.int_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_1 = uvx_comm_bq_swap_u16_value(bq_data_1.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_2 = uvx_comm_bq_swap_u16_value(bq_data_1.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_3 = uvx_comm_bq_swap_u16_value(bq_data_1.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_4 = uvx_comm_bq_swap_u16_value(bq_data_1.ts4_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_h_int  = uvx_comm_bq_swap_u16_value(bq_data_2.int_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_5 = uvx_comm_bq_swap_u16_value(bq_data_2.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_6 = uvx_comm_bq_swap_u16_value(bq_data_2.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_7 = uvx_comm_bq_swap_u16_value(bq_data_2.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_8 = uvx_comm_bq_swap_u16_value(bq_data_2.ts4_temperature - KELVIN_TO_DEG_C);	

	batt_data.temperature_cell_l = bq_data_1.temperature - KELVIN_TO_DEG_C; // kelvin to C -273.15 ~2732
	batt_data.temperature_cell_h = bq_data_2.temperature - KELVIN_TO_DEG_C; // kelvin to C -273.15 ~2732	
	batt_data.temperature_l_int  = (bq_data_1.int_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_1 = (bq_data_1.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_2 = (bq_data_1.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_3 = (bq_data_1.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_4 = (bq_data_1.ts4_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_h_int  = (bq_data_2.int_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_5 = (bq_data_2.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_6 = (bq_data_2.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_7 = (bq_data_2.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_8 = (bq_data_2.ts4_temperature - KELVIN_TO_DEG_C);		

	batt_data.power = (bq_data_1.power + bq_data_2.power) * 10; // convert to mW
	batt_data.payload.power = uvx_comm_bq_swap_u32_value(batt_data.power);

	batt_data.cell_bal_time_1 =  bq_data_1.cell_bal_time_1;
	batt_data.cell_bal_time_2 =  bq_data_1.cell_bal_time_2;
	batt_data.cell_bal_time_3 =  bq_data_1.cell_bal_time_3;
	batt_data.cell_bal_time_4 =  bq_data_1.cell_bal_time_4;
	batt_data.cell_bal_time_5 =  bq_data_1.cell_bal_time_5;
	batt_data.cell_bal_time_6 =  bq_data_2.cell_bal_time_1;
	batt_data.cell_bal_time_7 =  bq_data_2.cell_bal_time_2;
	batt_data.cell_bal_time_8 =  bq_data_2.cell_bal_time_3;
	batt_data.cell_bal_time_9 =  bq_data_2.cell_bal_time_4;
	batt_data.cell_bal_time_10 = bq_data_2.cell_bal_time_5;	

	batt_data.payload.cell_bal_time_1 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_1);
	batt_data.payload.cell_bal_time_2 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_2);
	batt_data.payload.cell_bal_time_3 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_3);
	batt_data.payload.cell_bal_time_4 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_4);
	batt_data.payload.cell_bal_time_5 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_5);
	batt_data.payload.cell_bal_time_6 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_6);
	batt_data.payload.cell_bal_time_7 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_7);
	batt_data.payload.cell_bal_time_8 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_8);
	batt_data.payload.cell_bal_time_9 =  uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_9);
	batt_data.payload.cell_bal_time_10 = uvx_comm_bq_swap_u16_value(batt_data.cell_bal_time_10);



	batt_data.cell_dod_1 = bq_data_1.cell_dod_1;
	batt_data.cell_dod_2 = bq_data_1.cell_dod_2;
	batt_data.cell_dod_3 = bq_data_1.cell_dod_3;
	batt_data.cell_dod_4 = bq_data_1.cell_dod_4;
	batt_data.cell_dod_5 = bq_data_1.cell_dod_5;
	batt_data.cell_dod_6 = bq_data_2.cell_dod_1;
	batt_data.cell_dod_7 = bq_data_2.cell_dod_2;
	batt_data.cell_dod_8 = bq_data_2.cell_dod_3;
	batt_data.cell_dod_9 = bq_data_2.cell_dod_4;
	batt_data.cell_dod_10 =bq_data_2.cell_dod_5;

	bq_data_1.manufacturing_status.reg.data = uvx_comm_bq_swap_u16_pointer((uint16_t*)&bq_data_1.manufacturing_status.raw[1]);
	bq_data_1.operation_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.operation_status.raw[1]);
	bq_data_1.gauging_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.gauging_status.raw[1]);

	bq_data_2.manufacturing_status.reg.data = uvx_comm_bq_swap_u16_pointer((uint16_t*)&bq_data_2.manufacturing_status.raw[1]);
	bq_data_2.operation_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.operation_status.raw[1]);	
	bq_data_2.gauging_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.gauging_status.raw[1]);

	batt_data.CHG_fet_en = bq_data_2.manufacturing_status.reg.bits.FET_EN;
	batt_data.CHG_fet_stat = bq_data_2.operation_status.reg.bits.CHG;

	batt_data.payload.error = 	bq_data_1.No_response << BATT_ERROR_BQ_1_NO_RESPONSE_BIT |
							 	bq_data_2.No_response << BATT_ERROR_BQ_2_NO_RESPONSE_BIT |
								batt_data.batt_max_temp << BATT_ERROR_BATT_MAX_TEMP_BIT |
								drone_status.charge_overvoltage << BATT_ERROR_CHARGE_OVERVOLTAGE_BIT |
								drone_status.charge_cell_count_error << BATT_ERROR_CHARGE_CELL_COUNT_ERROR_BIT;

	batt_data.error = batt_data.payload.error;

	batt_data.Qmax_passed_BQ_1 = (int16_t)bq_data_1.Qmax_passed;
	batt_data.Qmax_passed_BQ_2 = (int16_t)bq_data_2.Qmax_passed; 

	batt_data.payload.Qmax_passed_BQ_1 = uvx_comm_bq_swap_u16_value(batt_data.Qmax_passed_BQ_1);
	batt_data.payload.Qmax_passed_BQ_2 = uvx_comm_bq_swap_u16_value(batt_data.Qmax_passed_BQ_2); 



	if(bq_data_2.SOH < bq_data_1.SOH)
	{
		batt_data.SOH = bq_data_2.SOH;
	}
	else
	{
		batt_data.SOH = bq_data_1.SOH;
	}

	batt_data.payload.SOH = batt_data.SOH;

	if(bq_data_2.cycle_count > bq_data_1.cycle_count)
	{
		batt_data.cycle_count = bq_data_2.cycle_count;
	}
	else
	{
		batt_data.cycle_count = bq_data_1.cycle_count;
	}

	batt_data.payload.cycle_count = uvx_comm_bq_swap_u16_value(batt_data.cycle_count);

	if(uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.state_time_s) >= 60)
	{
		batt_data.state_time_l_m = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.state_time_s) / 60U;
	}
	else
	{
		batt_data.state_time_l_m = 0;
	}

	batt_data.payload.state_time_l_m = uvx_comm_bq_swap_u16_value(batt_data.state_time_l_m);

	if(uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.state_time_s) >= 60)
	{
		batt_data.state_time_h_m = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.state_time_s) / 60U;
	}
	else
	{
		batt_data.state_time_h_m = 0;
	}

	batt_data.payload.state_time_h_m = uvx_comm_bq_swap_u16_value(batt_data.state_time_h_m);

	if(bq_data_2.remaining_capacity < bq_data_1.remaining_capacity)
	{
		batt_data.remaining_capacity = (bq_data_2.remaining_capacity);
	}
	else
	{
		batt_data.remaining_capacity = (bq_data_1.remaining_capacity);
	}

	batt_data.payload.remaining_capacity = uvx_comm_bq_swap_u16_value(batt_data.remaining_capacity);


	if(bq_data_2.full_charge_capacity < bq_data_1.full_charge_capacity)
	{
		batt_data.full_charge_capacity = (bq_data_2.full_charge_capacity);
	}
	else
	{
		batt_data.full_charge_capacity = (bq_data_1.full_charge_capacity);
	}

	batt_data.payload.full_charge_capacity = uvx_comm_bq_swap_u16_value(batt_data.full_charge_capacity);


	if(bq_data_2.relative_state_of_charge < bq_data_1.relative_state_of_charge)
	{
		batt_data.relative_state_of_charge = (bq_data_2.relative_state_of_charge);
	}
	else
	{
		batt_data.relative_state_of_charge = (bq_data_1.relative_state_of_charge);
	}

	batt_data.payload.relative_state_of_charge = batt_data.relative_state_of_charge;


	if(bq_data_2.absolute_state_of_charge < bq_data_1.absolute_state_of_charge)
	{
		batt_data.absolute_state_of_charge = (bq_data_2.absolute_state_of_charge);
	}
	else
	{
		batt_data.absolute_state_of_charge = (bq_data_1.absolute_state_of_charge);
	}

	batt_data.payload.absolute_state_of_charge = batt_data.absolute_state_of_charge;


	if(bq_data_2.cells_count > 0U)
	{
		bq_data_2.voltage_per_cell = bq_data_2.voltage / bq_data_2.cells_count;
	}
	else
	{
		bq_data_2.voltage_per_cell = 0;
	}

	if(bq_data_1.cells_count > 0U)
	{
		bq_data_1.voltage_per_cell = bq_data_1.voltage / bq_data_1.cells_count;
	}
	else
	{
		bq_data_1.voltage_per_cell = 0;
	}

	batt_data.voltage_diff_pack = (bq_data_2.voltage_per_cell > bq_data_1.voltage_per_cell) ?
									(bq_data_2.voltage_per_cell - bq_data_1.voltage_per_cell) : 
									(bq_data_1.voltage_per_cell - bq_data_2.voltage_per_cell);	

	batt_data.payload.voltage_diff_pack = uvx_comm_bq_swap_u16_value(batt_data.voltage_diff_pack);		
	batt_data.payload.voltage_delta_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_delta_cell);
	batt_data.payload.voltage_min_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_min_cell);
	batt_data.payload.voltage_max_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_max_cell);
						

	if(bq_data_2.average_time_to_empty < bq_data_1.average_time_to_empty)
	{
		batt_data.avg_time_to_empty_m = bq_data_2.average_time_to_empty;
	}
	else
	{
		batt_data.avg_time_to_empty_m = bq_data_1.average_time_to_empty;
	}

	batt_data.payload.avg_time_to_empty_m = uvx_comm_bq_swap_u16_value(batt_data.avg_time_to_empty_m);

	if(bq_data_2.average_time_to_full < bq_data_1.average_time_to_full)
	{
		batt_data.avg_time_to_full_m = bq_data_2.average_time_to_full;
	}
	else
	{
		batt_data.avg_time_to_full_m = bq_data_1.average_time_to_full;
	}

	batt_data.payload.avg_time_to_full_m = uvx_comm_bq_swap_u16_value(batt_data.avg_time_to_full_m);

	if(batt_data.voltage_max_cell > BATT_CELL_MAX_VOLTAGE)
	{
		drone_status.charge_overvoltage = 1U;
	}
	else if(batt_data.voltage_max_cell < BATT_CELL_CHARGE_RESUME_VOLTAGE)
	{
		drone_status.charge_overvoltage = 0U;
	}

	if(batt_data.cells_count != BATT_EXPECTED_CELLS)
	{
		drone_status.charge_cell_count_error = 1U;
	}
	else
	{
		drone_status.charge_cell_count_error = 0U;
	}

	if( (drone_status.charge_overvoltage) || 
		(drone_status.charge_cell_count_error) ||
		(batt_data.batt_max_temp) ||
		(drone_status.pwr_fc))
	{
		batt_data.tc = 1;
	}
	else
	{
		batt_data.tc = 0;
	}

	batt_data.payload.supply_status = 	drone_status.pwr_fet 								<< BATT_SUPPLY_STATUS_PWR_FET_BIT |
										batt_data.CHG_fet_stat 								<< BATT_SUPPLY_STATUS_CHG_FET_BIT |
										batt_data.tc 										<< BATT_SUPPLY_STATUS_TC_BIT |
										((batt_data.cell_ball_l || batt_data.cell_ball_h) 	<< BATT_SUPPLY_STATUS_BALANCE_BIT) |
										((comm_bq_1.Force_balance) 							<< BATT_SUPPLY_STATUS_FORCE_BALANCE_L_BIT) |
										((comm_bq_2.Force_balance) 							<< BATT_SUPPLY_STATUS_FORCE_BALANCE_H_BIT) |
										((batt_data.adc_pack_v_stable_low)  				<< BATT_SUPPLY_STATUS_STABLE_L_BIT) |
										((batt_data.adc_pack_v_stable_high) 				<< BATT_SUPPLY_STATUS_STABLE_H_BIT);

	batt_data.supply_status = batt_data.payload.supply_status;

	if((batt_data.temperature_cell_h > MAX_HIS_CELL_TEMPERATURE) || (batt_data.temperature_cell_l > MAX_HIS_CELL_TEMPERATURE))
	{
		//UVX_APP_PWR_FET(0); // power off FC
		batt_data.batt_max_temp = 1;
	}
	else
	{
		if((batt_data.temperature_cell_h < MIN_HIS_CELL_TEMPERATURE) && (batt_data.temperature_cell_l < MIN_HIS_CELL_TEMPERATURE))
		{
			batt_data.batt_max_temp = 0;
		}
	}

	batt_data.cell_ball_h = bq_data_2.operation_status.reg.bits.CB;
	batt_data.cell_ball_l = bq_data_1.operation_status.reg.bits.CB;
	if((bq_data_2.operation_status.reg.bits.CB) || (bq_data_1.operation_status.reg.bits.CB))
	{
		uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_RESET); //charging
	}	

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_search_cell_diff(void)
{
	uint16_t *p_cell = (uint16_t *)&batt_data.cell_voltage_1;
	uint16_t min_cell = 0xFFFFU;
	uint16_t max_cell = 0U;
	uint16_t cell_voltage = 0U;
	uint8_t i;
	uint8_t valid_cells = 0;

	for(i = 0; i < BATT_CELLS_MAX; i++)
	{
		cell_voltage = p_cell[i];

		if(cell_voltage > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			valid_cells++;

			if(cell_voltage < min_cell)
			{
				min_cell = cell_voltage;
			}

			if(cell_voltage > max_cell)
			{
				max_cell = cell_voltage;
			}
		}
	}

	if(valid_cells == 0U)
	{
		batt_data.voltage_min_cell = 0;
		batt_data.voltage_max_cell = 0;
		batt_data.voltage_delta_cell = 0;
		return UVX_BATT_ERROR;
	}

	batt_data.voltage_min_cell = (int16_t)min_cell;
	batt_data.voltage_max_cell = (int16_t)max_cell;
	batt_data.voltage_delta_cell = (int16_t)(max_cell - min_cell);
	batt_data.payload.adc_pack_v = uvx_comm_bq_swap_u16_value(batt_data.adc_pack_v);
	

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_read_pack_v(void)
{
	if(&hadc1 != NULL)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		HAL_ADC_Stop(&hadc1);

		batt_data.adc_pack_v = HAL_ADC_GetValue(&hadc1) * PACK_V_GAIN;

		if((batt_data.adc_pack_v > batt_data.pwr_min_voltage) &&
		   (batt_data.adc_pack_v < batt_data.pwr_max_voltage))
		{								
			if(timer_app_batt_pwr_high.Timeout == 0)
			{
				batt_data.adc_pack_v_stable_high = 1;

				if( (drone_status.pwr_fet == 0) &&
					(drone_status.pwr_fc == 0) &&
				    (!batt_data.tc))				   
				{					
					UVX_APP_PWR_FET(1); // power on FC
					batt_data.adc_pack_v_stable_low = 0;
					uvx_gpio_set_pin(GPIO_OUT_LED_STRIP_ENABLE, GPIO_PIN_SET);	
					if(!batt_data.init)
					{
						batt_state.state_current = BATT_MODE_INIT;
					}
					else
					{
						batt_state.state_current = BATT_MODE_READ_BQ_1;
					}
				}				
				else if(drone_status.pwr_fc == 1)
				{
					UVX_APP_PWR_FET(0); // power off FC
					uvx_gpio_set_pin(GPIO_OUT_LED_STRIP_ENABLE, GPIO_PIN_RESET);
				}
			}
			else
			{
				timer_app_comm_jmb.Timeout = comm_m2jmb.timeout_heartbeat; // Reset heartbeat timeout
				timer_app_comm_jmb.Enable = true;			
			}
			
			batt_data.adc_pack_v_stable_low = 0;
			timer_app_batt_pwr_low.Timeout = APP_TIMEOUT_PACK_V_STABLE_LOW; // Reset timeout for power off
			timer_app_batt_pwr_low.Enable = true;
		}
		else
		{				
			if(timer_app_batt_pwr_low.Timeout == 0)
			{
				batt_data.adc_pack_v_stable_low = 1;
				batt_data.adc_pack_v_stable_high = 0;			
			}
			
			timer_app_batt_pwr_high.Timeout = APP_TIMEOUT_PACK_V_STABLE_HIGH; // Reset timeout for power on
			timer_app_batt_pwr_high.Enable = true; 
			if((comm_bq_2.RX_Ready) && (comm_bq_1.RX_Ready))
			{
				batt_data.adc_pack_v_stable_high = 0;
				UVX_APP_PWR_FET(0); // power off
				// if((drone_state.state_current == DRONE_SLEEP))
				// {
				// 	batt_state.state_current = BATT_MODE_CHECK_STATUS;
				// }				
			}				
		}		
	}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_read_data(uint16_t reg_index)
{
		if(uvx_comm_bq_read_list(&comm_bq_2, reg_index) == UVX_BQ_REG_END)
		{
			return UVX_BATT_REG_END;
		}	
		else
		{
			HAL_Delay(100);
			comm_bq_1.RX_Ready = 1; // Set RX ready flag
		}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_detect_cells(void)
{
	uint16_t *p_cell = (uint16_t *)&batt_data.cell_voltage_1;
	uint8_t i;

	bq_data_1.cells_count = 0;
	for(i = 0; i < (BATT_CELLS_MAX / 2U); i++)
	{
		if(p_cell[i] > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			bq_data_1.cells_count++;
		}
	}

	bq_data_2.cells_count = 0;
	for(i = (BATT_CELLS_MAX / 2U); i < BATT_CELLS_MAX; i++)
	{
		if(p_cell[i] > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			bq_data_2.cells_count++;
		}
	}

	if(unit_test.cell_count)
	{
		batt_data.cells_count = BATT_EXPECTED_CELLS - 1;
	}
	else
	{
		batt_data.cells_count = (uint8_t)(bq_data_1.cells_count + bq_data_2.cells_count);
	}
	
	batt_data.pwr_min_voltage = batt_data.cells_count * BATT_CELL_MIN_VOLTAGE;
	batt_data.pwr_max_voltage = batt_data.cells_count * BATT_CELL_MAX_VOLTAGE + BATT_DELTA_VOLTAGE;

	if(batt_data.cells_count > 0U)
	{
		return UVX_BATT_OK;
	}

	return UVX_BATT_ERROR;
}
