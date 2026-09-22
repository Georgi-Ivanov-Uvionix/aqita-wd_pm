#include "uvx_app_battery.h"
#include "main.h"

extern UVX_I2C i2c_bq;

static uint16_t batt_reg_cnt = 0;

void UVX_APP_Batt(void)
{
	switch(batt_state.state_current)
	{
		case BATT_MODE_INIT:
			uvx_comm_bq_change_list(&comm_bq_1, bq_1_register_list_read_once);
			uvx_comm_bq_change_list(&comm_bq_2, bq_2_register_list_read_once);
			uvx_comm_bq_change_list(&comm_bq_3, bq_3_register_list_read_once);
			batt_reg_cnt = 0;
			batt_state.state_current = BATT_MODE_READ_ONCE_BQ_1;
		break;

		case BATT_MODE_READ_ONCE_BQ_1:
			if(uvx_comm_bq_read_list(&comm_bq_1, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_ONCE_BQ_2;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_ONCE_BQ_1;
				batt_state.state_when_fail = BATT_MODE_READ_ONCE_BQ_2;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_READ_ONCE_BQ_2:
			if(uvx_comm_bq_read_list(&comm_bq_2, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_ONCE_BQ_3;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_ONCE_BQ_2;
				batt_state.state_when_fail = BATT_MODE_READ_ONCE_BQ_3;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_READ_ONCE_BQ_3:
			if(uvx_comm_bq_read_list(&comm_bq_3, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_data.design_capacity = (bq_data_1.design_capacity + bq_data_2.design_capacity + bq_data_3.design_capacity) / 3U;
				batt_data.design_voltage = bq_data_1.design_voltage + bq_data_2.design_voltage + bq_data_3.design_voltage;
				uvx_comm_bq_change_list(&comm_bq_1, bq_1_register_list_read);
				uvx_comm_bq_change_list(&comm_bq_2, bq_2_register_list_read);
				uvx_comm_bq_change_list(&comm_bq_3, bq_3_register_list_read);
				batt_state.state_current = BATT_MODE_INIT_BALANCE_1;
				uvx_batt_parse_data();				
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_ONCE_BQ_3;
				batt_state.state_when_fail = BATT_MODE_INIT_BALANCE_1;
				HAL_Delay(1);
			}
		break;		

		case BATT_MODE_INIT_BALANCE_1:
			uvx_comm_bq_force_balance(&comm_bq_1, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_INIT_BALANCE_2;
			batt_state.state_when_fail = BATT_MODE_INIT_BALANCE_2;
		break;

		case BATT_MODE_INIT_BALANCE_2:
			uvx_comm_bq_force_balance(&comm_bq_2, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_INIT_BALANCE_3;
			batt_state.state_when_fail = BATT_MODE_INIT_BALANCE_3;
		break;

		case BATT_MODE_INIT_BALANCE_3:
			uvx_comm_bq_force_balance(&comm_bq_3, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_BQ_1;
			batt_state.state_when_fail = BATT_MODE_READ_BQ_1;
			batt_reg_cnt = 0;
		break;

		case BATT_MODE_READ_BQ_1:
			if(uvx_comm_bq_read_list(&comm_bq_1, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_BQ_2;
				uvx_batt_parse_data();

				if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
				{
					uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_SET);
				}
				uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_SET);
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_BQ_1;
				batt_state.state_when_fail = BATT_MODE_READ_BQ_2;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_READ_BQ_2:
			if(uvx_comm_bq_read_list(&comm_bq_2, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_READ_BQ_3;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_BQ_2;
				batt_state.state_when_fail = BATT_MODE_READ_BQ_3;
				HAL_Delay(1);
			}
		break;	

		case BATT_MODE_READ_BQ_3:
			if(uvx_comm_bq_read_list(&comm_bq_3, batt_reg_cnt) == UVX_BQ_REG_END)
			{
				batt_reg_cnt = 0;
				batt_state.state_current = BATT_MODE_CHECK_STATUS;
				uvx_batt_parse_data();
			}
			else
			{
				batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
				batt_state.state_next = BATT_MODE_READ_BQ_3;
				batt_state.state_when_fail = BATT_MODE_CHECK_STATUS;
				HAL_Delay(1);
			}
		break;

		case BATT_MODE_CHECK_STATUS:
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_when_fail = BATT_MODE_READ_CHECK_PACK_V;

			if(bq_data_1.CHG_FET_STAT)
			{
				uvx_comm_bq_write_mba_register(&bq_data_1, BQ_MA_FET_CONTROL, NULL, 0);
			}
			else
			{
				// if(batt_data.tc)
				// {
				// 	uvx_comm_bq_charge_fet(&bq_data_1, 0);

				// }
				// else
				// {
				// 	if((batt_data.adc_pack_v_stable_high) && (drone_status.pwr_fet))
				// 	{
				// 		uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_RESET);
				// 		if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
				// 		{
				// 			uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
				// 		}

				// 		uvx_comm_bq_charge_fet(&bq_data_2, 1);
				// 	}
				// 	else
				// 	{
				// 		uvx_comm_bq_charge_fet(&bq_data_2, 0);
				// 	}		
				// }

				if(!batt_data.cell_ball_2 && !batt_data.cell_ball_1 && !batt_data.cell_ball_3)
				{
					// if((batt_data.voltage_diff_pack > BATT_CELL_VOLTAGE_DIFF) && (batt_data.CHG_fet_stat))
					// {
					// 	if((bq_data_1.voltage_per_cell >= bq_data_2.voltage_per_cell) &&
					// 	   (bq_data_1.voltage_per_cell >= bq_data_3.voltage_per_cell))
					// 	{
					// 		uvx_comm_bq_force_balance(&comm_bq_1, 1);
					// 		batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
					// 	}
					// 	else if(bq_data_2.voltage_per_cell >= bq_data_3.voltage_per_cell)
					// 	{
					// 		uvx_comm_bq_force_balance(&comm_bq_2, 1);
					// 		batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
					// 	}
					// 	else
					// 	{
					// 		uvx_comm_bq_force_balance(&comm_bq_3, 1);
					// 		batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
					// 	}
					// }
					// else
					// {
					// 	if(comm_bq_3.Force_balance)
					// 	{
					// 		batt_state.state_next = BATT_MODE_OFF_BALANCE_3;
					// 	}
					// 	else if(comm_bq_2.Force_balance)
					// 	{
					// 		batt_state.state_next = BATT_MODE_OFF_BALANCE_2;
					// 	}
					// 	else if(comm_bq_1.Force_balance)
					// 	{
					// 		batt_state.state_next = BATT_MODE_OFF_BALANCE_1;
					// 	}
					// 	else
					// 	{
					// 		batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
					// 	}					
					// }	
				}
				else
				{
					if(comm_bq_3.Force_balance)
					{
						batt_state.state_next = BATT_MODE_OFF_BALANCE_3;
					}
					else if(comm_bq_2.Force_balance)
					{
						batt_state.state_next = BATT_MODE_OFF_BALANCE_2;
					}
					else if(comm_bq_1.Force_balance)
					{
						batt_state.state_next = BATT_MODE_OFF_BALANCE_1;
					}
					else
					{
						batt_state.state_current = BATT_MODE_READ_CHECK_PACK_V;
					}		
				}
			}			
		break;

		case BATT_MODE_OFF_BALANCE_1:
			uvx_comm_bq_force_balance(&comm_bq_1, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
			batt_state.state_when_fail = BATT_MODE_READ_CHECK_PACK_V;
		break;

		case BATT_MODE_OFF_BALANCE_2:
			uvx_comm_bq_force_balance(&comm_bq_2, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
			batt_state.state_when_fail = BATT_MODE_READ_CHECK_PACK_V;
			batt_reg_cnt = 0;
		break;				

		case BATT_MODE_OFF_BALANCE_3:
			uvx_comm_bq_force_balance(&comm_bq_3, 0);
			batt_state.state_current = BATT_MODE_WAIT_RESPONSE;
			batt_state.state_next = BATT_MODE_READ_CHECK_PACK_V;
			batt_state.state_when_fail = BATT_MODE_READ_CHECK_PACK_V;
			batt_reg_cnt = 0;
		break;
		
		case BATT_MODE_READ_CHECK_PACK_V:
			batt_data.init = true;
			batt_state.state_current = BATT_MODE_READ_BQ_1;
			if((led_strip_state.state_current < LED_STRIP_MODE_BATTERY_LEVEL))
			{
				led_strip_state.state_next = LED_STRIP_MODE_BATTERY_LEVEL;				
			}

			if(enable) //manual learn new battery
			{
				uvx_batt_learn();	
			}			

			HAL_Delay(1);
		break;

		case BATT_MODE_WAIT_RESPONSE:
			if((i2c_bq.hal_i2c.RX_Ready) || (batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE))
			{
				switch(batt_state.state_next)
				{
					case BATT_MODE_READ_CHECK_PACK_V:
						if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
						{
							batt_data.cnt_no_response = 0;
							batt_state.state_current = batt_state.state_when_fail;
							batt_reg_cnt = 0;
							HAL_Delay(1);
						}
						else
						{
							batt_state.state_current = batt_state.state_when_fail;
							batt_data.cnt_no_response++;
						}
					break;

					case BATT_MODE_INIT_BALANCE_2:
					case BATT_MODE_OFF_BALANCE_1:
					case BATT_MODE_READ_ONCE_BQ_1:
					case BATT_MODE_READ_BQ_1:
						if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
						{
							bq_data_1.No_response = true;
							batt_reg_cnt = 0;
							comm_bq_1.RX_Ready = 1; // Force ready to avoid blocking
							comm_bq_1.p_hal_i2c->RX_Ready = 1; // Force ready to avoid blocking
							batt_state.state_current = batt_state.state_when_fail;
						}						
						else
						{
							bq_data_1.No_response = false;
							batt_state.state_current = batt_state.state_next;
						}

						batt_data.cnt_no_response = 0;						
						
						comm_bq_1.RX_Ready = 1;
						
						// if(bq_1_register_list_read[batt_reg_cnt].reg_addr == CBSTATUS)
						// {
						// 	for_test = 0;
						// }

						batt_reg_cnt++;							
						HAL_Delay(1);						
					break;
					
					case BATT_MODE_OFF_BALANCE_2:
					case BATT_MODE_INIT_BALANCE_3:
					case BATT_MODE_READ_ONCE_BQ_2:
					case BATT_MODE_READ_BQ_2:
							if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
							{
								bq_data_2.No_response = true;
								batt_reg_cnt = 0;
								comm_bq_2.RX_Ready = 1; // Force ready to avoid blocking
								comm_bq_2.p_hal_i2c->RX_Ready = 1; // Force ready to avoid blocking
								batt_state.state_current = batt_state.state_when_fail;
							}						
							else
							{
								bq_data_2.No_response = false;
								batt_state.state_current = batt_state.state_next;
							}				

							batt_data.cnt_no_response = 0;							
							comm_bq_2.RX_Ready = 1;
							batt_reg_cnt++;
							HAL_Delay(1);
					break;

					case BATT_MODE_INIT_BALANCE_DONE:
					case BATT_MODE_OFF_BALANCE_3:
					case BATT_MODE_READ_ONCE_BQ_3:
					case BATT_MODE_READ_BQ_3:
						if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
						{
							bq_data_3.No_response = true;
							batt_reg_cnt = 0;
							comm_bq_3.RX_Ready = 1;
							comm_bq_3.p_hal_i2c->RX_Ready = 1;
							batt_state.state_current = batt_state.state_when_fail;
						}
						else
						{
							bq_data_3.No_response = false;
							batt_state.state_current = batt_state.state_next;
						}
						batt_data.cnt_no_response = 0;						
						comm_bq_3.RX_Ready = 1;
						batt_reg_cnt++;
						HAL_Delay(1);
					break;
				}			
			}
			else
			{
				batt_data.cnt_no_response++;
			}	
		break;

		case BATT_MODE_STOP:
			//batt_data.init = false;
			if((batt_data.tc) && (batt_data.adc_pack_v_stable_high))
			{
				batt_state.state_current = BATT_MODE_READ_BQ_1;
			}
			HAL_Delay(10);
		break;		
	}

	uvx_batt_read_pack_v();


}
