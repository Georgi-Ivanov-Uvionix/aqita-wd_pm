
#include "uvx_batt.h"

SRAM1 UVX_BATT_DATA batt_data;

extern uint32_t g_Drone_Started;

UVX_BATT_STATE_MACHINE batt_state;

static uint16_t uvx_batt_min_u16_3(uint16_t a, uint16_t b, uint16_t c)
{
	uint16_t min = (a < b) ? a : b;
	return (c < min) ? c : min;
}

static uint16_t uvx_batt_max_u16_3(uint16_t a, uint16_t b, uint16_t c)
{
	uint16_t max = (a > b) ? a : b;
	return (c > max) ? c : max;
}

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

		p_comm_bq->BQ_TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent

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
	comm_bq_1.BQ_TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent
	
	if(uvx_batt_send_learn_commands(&comm_bq_1) != UVX_BATT_OK)
	{
		return UVX_BATT_ERROR;
	}

	comm_bq_2.BQ_TX_Ready = true; // Set TX_Ready to true to indicate that the command has been sent

	if(uvx_batt_send_learn_commands(&comm_bq_2) != UVX_BATT_OK)
	{
		return UVX_BATT_ERROR;
	}

	comm_bq_3.BQ_TX_Ready = true;
	if(uvx_batt_send_learn_commands(&comm_bq_3) != UVX_BATT_OK)
	{
		return UVX_BATT_ERROR;
	}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_find_highest_average_pack_voltage(void)
{
	uint16_t highest = 0U;
	uint16_t lowest = UINT16_MAX;
	uint8_t i;

	for(i = 0U; i < BQ_DEVICES; i++)
	{
		if(bq_data[i].voltage_per_cell > highest)
		{
			highest = bq_data[i].voltage_per_cell;
		}
		if(bq_data[i].voltage_per_cell < lowest)
		{
			lowest = bq_data[i].voltage_per_cell;
		}
	}

	for(i = 0U; i < BQ_DEVICES; i++)
	{
		bq_data[i].bps_need_discharge = ( ((highest - lowest) > BATT_BPS_DISCHARGE_DIFF_MV) && (bq_data[i].voltage_per_cell == highest) );
		bq_data[i].bps_need_charge = (bq_data[i].voltage_per_cell == lowest);
	}

	return UVX_BATT_OK;
}

// Match the three-cell pack mapping and corrected voltages used by batt_data.
static void uvx_batt_update_pack_cell_delta(void)
{
	uint16_t cells[3][3] = {
		{batt_data.cell_voltage_1, batt_data.cell_voltage_2, batt_data.cell_voltage_3},
		{batt_data.cell_voltage_4, batt_data.cell_voltage_5, batt_data.cell_voltage_6},
		{batt_data.cell_voltage_7, batt_data.cell_voltage_8, batt_data.cell_voltage_9}
	};
	uint32_t pack;
	uint32_t cell;
	uint16_t minimum;
	uint16_t maximum;
	uint16_t voltage;

	for(pack = 0; pack < BQ_DEVICES; pack++)
	{
		minimum = 0xFFFFU;
		maximum = 0U;
		for(cell = 0; cell < 3U; cell++)
		{
			voltage = cells[pack][cell];
			if(voltage > BATT_CELL_DETECT_THRESHOLD_MV)
			{
				if(voltage < minimum)
				{
					minimum = voltage;
				}
				if(voltage > maximum)
				{
					maximum = voltage;
				}
			}
		}
		// Zero or one detected cell has no measurable cell-to-cell spread.
		bq_data[pack].voltage_delta_cell =
			(maximum > 0U) ? (uint16_t)(maximum - minimum) : 0U;
	}
}

UVX_BATT_STATE uvx_batt_parse_data(void)
{
	batt_data.cell_voltage_1  = bq_data_1.cell_voltage_1 - (bq_data_1.current * CELL_1_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_2  = bq_data_1.cell_voltage_2 - (bq_data_1.current * CELL_2_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_3  = bq_data_1.cell_voltage_3 - (bq_data_1.current * CELL_3_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_4  = bq_data_2.cell_voltage_1 - (bq_data_2.current * CELL_4_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_5  = bq_data_2.cell_voltage_2 - (bq_data_2.current * CELL_5_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_6  = bq_data_2.cell_voltage_3 - (bq_data_2.current * CELL_6_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_7  = bq_data_3.cell_voltage_1 - (bq_data_3.current * CELL_7_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_8  = bq_data_3.cell_voltage_2 - (bq_data_3.current * CELL_8_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_9  = bq_data_3.cell_voltage_3 - (bq_data_3.current * CELL_9_INTERCONNECT_RESISTANCE)/1000;
	batt_data.cell_voltage_10 = 0U;
	uvx_batt_update_pack_cell_delta();

	//convert to little endian 
	batt_data.payload.cell_voltage_1 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_1);
	batt_data.payload.cell_voltage_2 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_2);
	batt_data.payload.cell_voltage_3 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_3);
	batt_data.payload.cell_voltage_4 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_4);
	batt_data.payload.cell_voltage_5 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_5);
	batt_data.payload.cell_voltage_6 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_6);
	batt_data.payload.cell_voltage_7 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_7);
	batt_data.payload.cell_voltage_8 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_8);
	batt_data.payload.cell_voltage_9 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_9);
	batt_data.payload.cell_voltage_10 = uvx_comm_bq_swap_u16_value(batt_data.cell_voltage_10);

	uvx_batt_detect_cells();
	uvx_batt_search_cell_diff();

	batt_data.batt_voltage = bq_data_1.pack_voltage + bq_data_2.pack_voltage + bq_data_3.pack_voltage;
	batt_data.payload.batt_voltage = uvx_comm_bq_swap_u16_value(batt_data.batt_voltage);	

	batt_data.payload.current = uvx_comm_bq_swap_u16_value((bq_data_1.current + bq_data_2.current + bq_data_3.current) / 3);
	batt_data.current = (bq_data_1.current + bq_data_2.current + bq_data_3.current) / 3;

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


	batt_data.payload.temperature_1_int  = uvx_comm_bq_swap_u16_value((	(bq_data_1.int_temperature - KELVIN_TO_DEG_C) +
																		(bq_data_2.int_temperature - KELVIN_TO_DEG_C) +
																	 	(bq_data_3.int_temperature - KELVIN_TO_DEG_C) ) / 3); //combine all three internal temperatures and convert to little endian
	batt_data.payload.temperature_cell_1 = uvx_comm_bq_swap_u16_value(bq_data_1.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_2 = uvx_comm_bq_swap_u16_value(bq_data_1.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_3 = uvx_comm_bq_swap_u16_value(bq_data_1.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_4 = uvx_comm_bq_swap_u16_value(bq_data_2.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_2_int  = uvx_comm_bq_swap_u16_value(bq_data_3.ts3_temperature - KELVIN_TO_DEG_C); // this is cell 9 temperature, just for now, because we have to change the protocol
	batt_data.payload.temperature_cell_5 = uvx_comm_bq_swap_u16_value(bq_data_2.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_6 = uvx_comm_bq_swap_u16_value(bq_data_2.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_7 = uvx_comm_bq_swap_u16_value(bq_data_3.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.payload.temperature_cell_8 = uvx_comm_bq_swap_u16_value(bq_data_3.ts2_temperature - KELVIN_TO_DEG_C);	

	batt_data.temperature_bq_1 = bq_data_1.temperature - KELVIN_TO_DEG_C; // kelvin to C -273.15 ~2732
	batt_data.temperature_bq_2 = bq_data_2.temperature - KELVIN_TO_DEG_C; // kelvin to C -273.15 ~2732	
	batt_data.temperature_bq_3 = bq_data_3.temperature - KELVIN_TO_DEG_C; // kelvin to C -273.15 ~2732
	batt_data.temperature_1_int  = (bq_data_1.int_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_1 = (bq_data_1.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_2 = (bq_data_1.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_3 = (bq_data_1.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_4 = (bq_data_2.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_2_int  = (bq_data_2.int_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_3_int  = (bq_data_3.int_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_5 = (bq_data_2.ts2_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_6 = (bq_data_2.ts3_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_7 = (bq_data_3.ts1_temperature - KELVIN_TO_DEG_C);
	batt_data.temperature_cell_8 = (bq_data_3.ts2_temperature - KELVIN_TO_DEG_C);		
	batt_data.temperature_cell_9 = (bq_data_3.ts3_temperature - KELVIN_TO_DEG_C);		

	batt_data.power = (bq_data_1.power + bq_data_2.power + bq_data_3.power) * 10; // convert to mW
	batt_data.payload.power = uvx_comm_bq_swap_u32_value(batt_data.power);

	batt_data.cell_bal_time_1 =  bq_data_1.cell_bal_time_1;
	batt_data.cell_bal_time_2 =  bq_data_1.cell_bal_time_2;
	batt_data.cell_bal_time_3 =  bq_data_1.cell_bal_time_3;
	batt_data.cell_bal_time_4 =  bq_data_2.cell_bal_time_1;
	batt_data.cell_bal_time_5 =  bq_data_2.cell_bal_time_2;
	batt_data.cell_bal_time_6 =  bq_data_2.cell_bal_time_3;
	batt_data.cell_bal_time_7 =  bq_data_3.cell_bal_time_1;
	batt_data.cell_bal_time_8 =  bq_data_3.cell_bal_time_2;
	batt_data.cell_bal_time_9 =  bq_data_3.cell_bal_time_3;
	batt_data.cell_bal_time_10 = 0U;

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

	bq_data_1.manufacturing_status.reg.data = uvx_comm_bq_swap_u16_pointer((uint16_t*)&bq_data_1.manufacturing_status.raw[1]);
	bq_data_1.operation_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.operation_status.raw[1]);
	bq_data_1.gauging_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.gauging_status.raw[1]);

	bq_data_2.manufacturing_status.reg.data = uvx_comm_bq_swap_u16_pointer((uint16_t*)&bq_data_2.manufacturing_status.raw[1]);
	bq_data_2.operation_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.operation_status.raw[1]);	
	bq_data_2.gauging_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.gauging_status.raw[1]);
	bq_data_3.manufacturing_status.reg.data = uvx_comm_bq_swap_u16_pointer((uint16_t*)&bq_data_3.manufacturing_status.raw[1]);
	bq_data_3.operation_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_3.operation_status.raw[1]);
	bq_data_3.gauging_status.reg.data = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_3.gauging_status.raw[1]);

	bq_data_1.gpio_pin_12 = (bq_data_1.gpio_status & BQ_GPIO_PIN12_MASK) >> 0;
	bq_data_1.gpio_pin_13 = (bq_data_1.gpio_status & BQ_GPIO_PIN13_MASK) >> 1;
	bq_data_1.gpio_pin_17 = (bq_data_1.gpio_status & BQ_GPIO_PIN17_MASK) >> 2;
	bq_data_1.gpio_pin_16 = (bq_data_1.gpio_status & BQ_GPIO_PIN16_MASK) >> 3;
	bq_data_1.gpio_pin_15 = (bq_data_1.gpio_status & BQ_GPIO_PIN15_MASK) >> 4;
	bq_data_1.gpio_pin_20 = (bq_data_1.gpio_status & BQ_GPIO_PIN20_MASK) >> 5;
	bq_data_1.gpio_pin_21 = (bq_data_1.gpio_status & BQ_GPIO_PIN21_MASK) >> 6;
	bq_data_1.gpio_pin_22 = (bq_data_1.gpio_status & BQ_GPIO_PIN22_MASK) >> 7;

	bq_data_2.gpio_pin_12 = (bq_data_2.gpio_status & BQ_GPIO_PIN12_MASK) >> 0;
	bq_data_2.gpio_pin_13 = (bq_data_2.gpio_status & BQ_GPIO_PIN13_MASK) >> 1;
	bq_data_2.gpio_pin_17 = (bq_data_2.gpio_status & BQ_GPIO_PIN17_MASK) >> 2;
	bq_data_2.gpio_pin_16 = (bq_data_2.gpio_status & BQ_GPIO_PIN16_MASK) >> 3;
	bq_data_2.gpio_pin_15 = (bq_data_2.gpio_status & BQ_GPIO_PIN15_MASK) >> 4;
	bq_data_2.gpio_pin_20 = (bq_data_2.gpio_status & BQ_GPIO_PIN20_MASK) >> 5;
	bq_data_2.gpio_pin_21 = (bq_data_2.gpio_status & BQ_GPIO_PIN21_MASK) >> 6;
	bq_data_2.gpio_pin_22 = (bq_data_2.gpio_status & BQ_GPIO_PIN22_MASK) >> 7;

	bq_data_3.gpio_pin_12 = (bq_data_3.gpio_status & BQ_GPIO_PIN12_MASK) >> 0;
	bq_data_3.gpio_pin_13 = (bq_data_3.gpio_status & BQ_GPIO_PIN13_MASK) >> 1;
	bq_data_3.gpio_pin_17 = (bq_data_3.gpio_status & BQ_GPIO_PIN17_MASK) >> 2;
	bq_data_3.gpio_pin_16 = (bq_data_3.gpio_status & BQ_GPIO_PIN16_MASK) >> 3;
	bq_data_3.gpio_pin_15 = (bq_data_3.gpio_status & BQ_GPIO_PIN15_MASK) >> 4;
	bq_data_3.gpio_pin_20 = (bq_data_3.gpio_status & BQ_GPIO_PIN20_MASK) >> 5;
	bq_data_3.gpio_pin_21 = (bq_data_3.gpio_status & BQ_GPIO_PIN21_MASK) >> 6;
	bq_data_3.gpio_pin_22 = (bq_data_3.gpio_status & BQ_GPIO_PIN22_MASK) >> 7;

	bq_data_1.FET_BPS_STAT = !bq_data_1.gpio_pin_15;
	bq_data_2.FET_BPS_STAT = !bq_data_2.gpio_pin_15;
	bq_data_3.FET_BPS_STAT = !bq_data_3.gpio_pin_15;

	// Bypass code [Force_balance_2:Force_balance_1]: 00 off, 01 BQ1, 10 BQ2, 11 BQ3.
	comm_bq_1.Force_balance = bq_data_1.FET_BPS_STAT || bq_data_3.FET_BPS_STAT;
	comm_bq_2.Force_balance = bq_data_2.FET_BPS_STAT || bq_data_3.FET_BPS_STAT;

	if(batt_data.init)
	{
		batt_data.BPS_Passive = bq_data_1.FET_BPS_STAT || bq_data_2.FET_BPS_STAT || bq_data_3.FET_BPS_STAT;
	}

	bq_data_1.FET_DSG_CHG_EN = bq_data_1.manufacturing_status.reg.bits.FET_EN;
	bq_data_2.FET_DSG_CHG_EN = bq_data_2.manufacturing_status.reg.bits.FET_EN;
	bq_data_3.FET_DSG_CHG_EN = bq_data_3.manufacturing_status.reg.bits.FET_EN;

	bq_data_1.FET_CHG_STAT = bq_data_1.operation_status.reg.bits.CHG;
	bq_data_2.FET_CHG_STAT = bq_data_2.operation_status.reg.bits.CHG;
	bq_data_3.FET_CHG_STAT = bq_data_3.operation_status.reg.bits.CHG;

	bq_data_1.FET_DSG_STAT = bq_data_1.operation_status.reg.bits.DSG;
	bq_data_2.FET_DSG_STAT = bq_data_2.operation_status.reg.bits.DSG;
	bq_data_3.FET_DSG_STAT = bq_data_3.operation_status.reg.bits.DSG;

	batt_data.CHG_FET_1_STAT = bq_data_1.FET_CHG_STAT;
	batt_data.DSG_FET_1_STAT = bq_data_1.FET_DSG_STAT;
	batt_data.CHG_FET_2_STAT = bq_data_2.FET_CHG_STAT;
	batt_data.DSG_FET_2_STAT = bq_data_2.FET_DSG_STAT;
	batt_data.CHG_FET_3_STAT = bq_data_3.FET_CHG_STAT;
	batt_data.DSG_FET_3_STAT = bq_data_3.FET_DSG_STAT;

	batt_data.Qmax_passed_BQ_1 = (int16_t)bq_data_1.Qmax_passed;
	batt_data.Qmax_passed_BQ_2 = (int16_t)bq_data_2.Qmax_passed; 
	batt_data.Qmax_passed_BQ_3 = (int16_t)bq_data_3.Qmax_passed;

	batt_data.payload.Qmax_passed_BQ_1 = uvx_comm_bq_swap_u16_value(batt_data.Qmax_passed_BQ_1);
	batt_data.payload.Qmax_passed_BQ_2 = uvx_comm_bq_swap_u16_value(batt_data.Qmax_passed_BQ_2); 
	batt_data.payload.Qmax_passed_BQ_3 = uvx_comm_bq_swap_u16_value(batt_data.Qmax_passed_BQ_3);



	batt_data.SOH = (uint8_t)uvx_batt_min_u16_3(bq_data_1.SOH, bq_data_2.SOH, bq_data_3.SOH);

	batt_data.payload.SOH = batt_data.SOH;

	batt_data.cycle_count = uvx_batt_max_u16_3(bq_data_1.cycle_count, bq_data_2.cycle_count, bq_data_3.cycle_count);

	batt_data.payload.cycle_count = uvx_comm_bq_swap_u16_value(batt_data.cycle_count);

	if(uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.state_time_s) >= 60)
	{
		batt_data.state_time_1_m = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_1.state_time_s) / 60U;
	}
	else
	{
		batt_data.state_time_1_m = 0;
	}

	batt_data.payload.state_time_1_m = uvx_comm_bq_swap_u16_value(batt_data.state_time_1_m);

	if(uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.state_time_s) >= 60)
	{
		batt_data.state_time_2_m = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_2.state_time_s) / 60U;
	}
	else
	{
		batt_data.state_time_2_m = 0;
	}

	batt_data.payload.state_time_2_m = uvx_comm_bq_swap_u16_value(batt_data.state_time_2_m);

	if(uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_3.state_time_s) >= 60)
	{
		batt_data.state_time_3_m = uvx_comm_bq_swap_u32_pointer((uint32_t *)&bq_data_3.state_time_s) / 60U;
	}
	else
	{
		batt_data.state_time_3_m = 0;
	}
	batt_data.payload.state_time_3_m = uvx_comm_bq_swap_u16_value(batt_data.state_time_3_m);

	batt_data.remaining_capacity = uvx_batt_min_u16_3(bq_data_1.remaining_capacity, bq_data_2.remaining_capacity, bq_data_3.remaining_capacity);

	batt_data.payload.remaining_capacity = uvx_comm_bq_swap_u16_value(batt_data.remaining_capacity);


	batt_data.full_charge_capacity = uvx_batt_min_u16_3(bq_data_1.full_charge_capacity, bq_data_2.full_charge_capacity, bq_data_3.full_charge_capacity);

	batt_data.payload.full_charge_capacity = uvx_comm_bq_swap_u16_value(batt_data.full_charge_capacity);


	batt_data.relative_state_of_charge = (uint8_t)uvx_batt_min_u16_3(bq_data_1.relative_state_of_charge, bq_data_2.relative_state_of_charge, bq_data_3.relative_state_of_charge);

	batt_data.payload.relative_state_of_charge = batt_data.relative_state_of_charge;


	batt_data.absolute_state_of_charge = (uint8_t)uvx_batt_min_u16_3(bq_data_1.absolute_state_of_charge, bq_data_2.absolute_state_of_charge, bq_data_3.absolute_state_of_charge);

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

	if(bq_data_3.cells_count > 0U)
	{
		bq_data_3.voltage_per_cell = bq_data_3.voltage / bq_data_3.cells_count;
	}
	else
	{
		bq_data_3.voltage_per_cell = 0;
	}

	//uvx_batt_find_highest_average_pack_voltage();

	batt_data.voltage_delta_cell = (int16_t)uvx_batt_max_u16_3(bq_data_1.voltage_delta_cell, bq_data_2.voltage_delta_cell, bq_data_3.voltage_delta_cell);
	batt_data.voltage_diff_pack = (int16_t)( uvx_batt_max_u16_3(bq_data_1.voltage_per_cell, bq_data_2.voltage_per_cell, bq_data_3.voltage_per_cell) -
											 uvx_batt_min_u16_3(bq_data_1.voltage_per_cell, bq_data_2.voltage_per_cell, bq_data_3.voltage_per_cell));

	batt_data.payload.voltage_diff_pack = uvx_comm_bq_swap_u16_value(batt_data.voltage_diff_pack);		
	batt_data.payload.voltage_delta_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_delta_cell);
	batt_data.payload.voltage_min_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_min_cell);
	batt_data.payload.voltage_max_cell = uvx_comm_bq_swap_u16_value(batt_data.voltage_max_cell);
						

	batt_data.avg_time_to_empty_m = uvx_batt_min_u16_3(bq_data_1.average_time_to_empty, bq_data_2.average_time_to_empty, bq_data_3.average_time_to_empty);

	batt_data.payload.avg_time_to_empty_m = uvx_comm_bq_swap_u16_value(batt_data.avg_time_to_empty_m);

	batt_data.avg_time_to_full_m = uvx_batt_max_u16_3(bq_data_1.average_time_to_full, bq_data_2.average_time_to_full, bq_data_3.average_time_to_full);

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

	if((batt_data.temperature_bq_2 > MAX_HIS_CELL_TEMPERATURE) ||
	   (batt_data.temperature_bq_1 > MAX_HIS_CELL_TEMPERATURE) ||
	   (batt_data.temperature_bq_3 > MAX_HIS_CELL_TEMPERATURE))
	{
		//UVX_APP_PWR_FET(0); // power off FC
		batt_data.batt_max_temp = 1;
	}
	else
	{
		if((batt_data.temperature_bq_2 < MIN_HIS_CELL_TEMPERATURE) &&
		   (batt_data.temperature_bq_1 < MIN_HIS_CELL_TEMPERATURE) &&
		   (batt_data.temperature_bq_3 < MIN_HIS_CELL_TEMPERATURE))
		{
			batt_data.batt_max_temp = 0;
		}
	}

	batt_data.cell_ball_2 = bq_data_2.operation_status.reg.bits.CB;
	batt_data.cell_ball_1 = bq_data_1.operation_status.reg.bits.CB;
	batt_data.cell_ball_3 = bq_data_3.operation_status.reg.bits.CB;
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

	bq_data_1.No_response = comm_bq_1.No_response || comm_bq_3.No_response;
	bq_data_2.No_response = comm_bq_2.No_response || comm_bq_3.No_response;

	batt_data.payload.error = 	bq_data_1.No_response << BATT_ERROR_BQ_L_NO_RESPONSE_BIT |
								bq_data_2.No_response << BATT_ERROR_BQ_H_NO_RESPONSE_BIT |
								batt_data.batt_max_temp << BATT_ERROR_BATT_MAX_TEMP_BIT |
								drone_status.charge_overvoltage << BATT_ERROR_CHARGE_OVERVOLTAGE_BIT |
								drone_status.charge_cell_count_error << BATT_ERROR_CHARGE_CELL_COUNT_ERROR_BIT;

	batt_data.payload.supply_status = 	drone_status.pwr_fet 								<< BATT_SUPPLY_STATUS_PWR_FET_BIT |
										batt_data.CHG_FET_1_STAT							<< BATT_SUPPLY_STATUS_CHG_FET_BIT |
										batt_data.tc 										<< BATT_SUPPLY_STATUS_TC_BIT |
										((batt_data.cell_ball_1 || batt_data.cell_ball_2 || batt_data.cell_ball_3) << BATT_SUPPLY_STATUS_BALANCE_BIT) |
										((comm_bq_1.Force_balance) 							<< BATT_SUPPLY_STATUS_FORCE_BALANCE_1_BIT) |
										((comm_bq_2.Force_balance) 							<< BATT_SUPPLY_STATUS_FORCE_BALANCE_2_BIT) |
										((batt_data.adc_pack_v_stable_low)  				<< BATT_SUPPLY_STATUS_STABLE_L_BIT) |
										((batt_data.adc_pack_v_stable_high) 				<< BATT_SUPPLY_STATUS_STABLE_H_BIT);

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
	batt_data.payload.adc_pack_v = uvx_comm_bq_swap_u16_value(batt_data.adc_pack_v);
	

	return UVX_BATT_OK;
}

/* Measure ADC1 with enough sampling time for VREFINT and the divider. */
static HAL_StatusTypeDef uvx_batt_adc_read(uint32_t channel, uint16_t *raw)
{
	ADC_ChannelConfTypeDef config = {0};
	HAL_StatusTypeDef status;
	config.Channel = channel;
	config.Rank = ADC_REGULAR_RANK_1;
	config.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	config.SingleDiff = ADC_SINGLE_ENDED;
	config.OffsetNumber = ADC_OFFSET_NONE;
	status = HAL_ADC_ConfigChannel(&hadc1, &config);
	if(status != HAL_OK) return status;
	// ConfigChannel enables the VREFINT input path, not VREFBUF.
	if(channel == ADC_CHANNEL_VREFINT) HAL_Delay(1);
	status = HAL_ADC_Start(&hadc1);
	if(status == HAL_OK)
	{
		status = HAL_ADC_PollForConversion(&hadc1, 10U);
		if(status == HAL_OK) *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
	}
	if(HAL_ADC_Stop(&hadc1) != HAL_OK) return HAL_ERROR;
	return status;
}

UVX_BATT_STATE uvx_batt_read_pack_v(void)
{
	uint16_t vref_raw;
	uint16_t pack_raw;
	uint32_t vref_mv;
	float pack_mv;

	if(uvx_batt_adc_read(ADC_CHANNEL_VREFINT, &vref_raw) != HAL_OK ||
	   vref_raw == 0U || vref_raw >= 4095U ||
	   *VREFINT_CAL_ADDR == 0U || *VREFINT_CAL_ADDR == 0xFFFFU)
	{
		return UVX_BATT_ERROR;
	}
	if(uvx_batt_adc_read(ADC_CHANNEL_9, &pack_raw) != HAL_OK || pack_raw >= 4095U)
	{
		return UVX_BATT_ERROR;
	}
	vref_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_12B);
	pack_mv = ((float)pack_raw * (float)vref_mv) / (ADC_MAX * DIV_RATIO);
	if(pack_mv > 65535.0f) return UVX_BATT_ERROR;
	batt_data.adc_vrefint_raw = vref_raw;
	batt_data.adc_pack_raw = pack_raw;
	batt_data.adc_vref_mv = (uint16_t)vref_mv;
	{
		batt_data.adc_pack_v = (uint16_t)(pack_mv + 0.5f);

		if((batt_data.adc_pack_v > batt_data.pwr_min_voltage) &&
		   (batt_data.adc_pack_v < batt_data.pwr_max_voltage))
		{								
			if(timer_app_batt_pwr_high.Timeout == 0)
			{
				batt_data.adc_pack_v_stable_high = 1;

				if( (drone_status.pwr_fet == 0) &&
					(drone_status.pwr_fc == 0) &&
				    (!batt_data.tc) && (!batt_data.BPS_Passive))				   
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
						batt_state.state_current = BATT_MODE_READ_BQ;
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
			if((comm_bq_3.BQ_RX_Ready) && (comm_bq_2.BQ_RX_Ready) && (comm_bq_1.BQ_RX_Ready))
			{
				batt_data.adc_pack_v_stable_high = 0;

				if(batt_data.init)
				{
					UVX_APP_PWR_FET(0); // power off
				}
				
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
		if(uvx_comm_bq_read_list(&comm_bq_3, reg_index) == UVX_BQ_REG_END)
		{
			return UVX_BATT_REG_END;
		}	
		else
		{
			HAL_Delay(100);
			comm_bq_3.BQ_RX_Ready = 1; // Set RX ready flag
		}

	return UVX_BATT_OK;
}

UVX_BATT_STATE uvx_batt_detect_cells(void)
{
	uint16_t *p_cell = (uint16_t *)&batt_data.cell_voltage_1;
	uint8_t i;

	bq_data_1.cells_count = 0;
	for(i = 0; i < 3U; i++)
	{
		if(p_cell[i] > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			bq_data_1.cells_count++;
		}
	}

	bq_data_2.cells_count = 0;
	for(i = 3U; i < 6U; i++)
	{
		if(p_cell[i] > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			bq_data_2.cells_count++;
		}
	}

	bq_data_3.cells_count = 0;
	for(i = 6U; i < 9U; i++)
	{
		if(p_cell[i] > BATT_CELL_DETECT_THRESHOLD_MV)
		{
			bq_data_3.cells_count++;
		}
	}

	if(unit_test.cell_count)
	{
		batt_data.cells_count = BATT_EXPECTED_CELLS - 1;
	}
	else
	{
		batt_data.cells_count = (uint8_t)(bq_data_1.cells_count + bq_data_2.cells_count + bq_data_3.cells_count);
	}
	
	batt_data.pwr_min_voltage = batt_data.cells_count * BATT_CELL_MIN_VOLTAGE;
	batt_data.pwr_max_voltage = batt_data.cells_count * BATT_CELL_MAX_VOLTAGE + BATT_DELTA_VOLTAGE;

	if(batt_data.cells_count > 0U)
	{
		return UVX_BATT_OK;
	}

	return UVX_BATT_ERROR;
}
