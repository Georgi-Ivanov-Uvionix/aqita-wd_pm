#include "uvx_app_battery.h"
#include "main.h"

extern UVX_I2C i2c_bq;
static UVX_BQ_DATA *p_app_bq_data;
static UVX_COMM_BQ *p_app_comm_bq;
static UVX_COMM_BQ_STATE bq_state;

#define BATT_BPS_SWITCH_INTERVAL_MS 10000U
static uint32_t bps_active_pack = BQ_DEVICES;
static uint32_t bps_next_pack;
static uint32_t bps_start_ms;
static bool bps_timer_running;
static bool bps_wait_off;

static void batt_rotate_fet_bps(void);

static UVX_COMM_BQ_STATE batt_mode_init_bypass(UVX_COMM_BQ *p_comm_bq);
static UVX_COMM_BQ_STATE batt_mode_read_BQ(UVX_COMM_BQ *p_comm_bq);
static UVX_COMM_BQ_STATE batt_mode_check_fet_enable(UVX_BQ_DATA *p_bq_data);
static UVX_COMM_BQ_STATE batt_mode_check_fet_bps(UVX_BQ_DATA *p_bq_data);
static UVX_COMM_BQ_STATE batt_mode_check_fet_dsg(UVX_BQ_DATA *p_bq_data);
static UVX_COMM_BQ_STATE batt_mode_check_fet_chg(UVX_BQ_DATA *p_bq_data);
static UVX_COMM_BQ_STATE batt_check_response(UVX_COMM_BQ *p_comm_bq);

void UVX_APP_Batt(void)
{
	switch(batt_state.state_current)
	{
		case BATT_MODE_INIT:
			bps_active_pack = BQ_DEVICES;
			bps_next_pack = 0;
			bps_timer_running = false;
			bps_wait_off = false;
			batt_data.BPS_Passive = true;
			uvx_comm_bq_change_list(&comm_bq_1, bq_1_register_list_read_once);
			uvx_comm_bq_change_list(&comm_bq_2, bq_2_register_list_read_once);
			uvx_comm_bq_change_list(&comm_bq_3, bq_3_register_list_read_once);			
			comm_bq_1.ID = 1;
			comm_bq_2.ID = 2;
			comm_bq_3.ID = 3;
			batt_state.state_current = BATT_MODE_INIT_BYPASS;
			p_app_bq_data = bq_data;
			p_app_comm_bq = comm_bq;
			bq_data_1.FET_DSG_CHG_EN = 0;
			bq_data_1.FET_BPS_EN = 0;
			bq_data_1.FET_DSG_STAT_NEW = 1;
			bq_data_1.FET_CHG_STAT_NEW = 1;

			bq_data_2.FET_DSG_CHG_EN = 0;
			bq_data_2.FET_BPS_EN = 0;
			bq_data_2.FET_DSG_STAT_NEW = 1;
			bq_data_2.FET_CHG_STAT_NEW = 1;

			bq_data_3.FET_DSG_CHG_EN = 0;
			bq_data_3.FET_BPS_EN = 0;
			bq_data_3.FET_DSG_STAT_NEW = 1;
			bq_data_3.FET_CHG_STAT_NEW = 1;
		break;

		case BATT_MODE_INIT_BYPASS:
			bq_state = batt_mode_init_bypass(p_app_comm_bq);

			if(bq_state == UVX_BQ_OK)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq->batt_reg_cnt = 0;
					p_app_comm_bq++;
				}
				else
				{			
					batt_state.state_current = BATT_MODE_READ_ONCE_BQ;
					p_app_comm_bq->batt_reg_cnt = 0;
					p_app_comm_bq = comm_bq;					
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}
		break;

		case BATT_MODE_READ_ONCE_BQ:
			bq_state = batt_mode_read_BQ(p_app_comm_bq);

			if(bq_state == UVX_BQ_REG_END)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
				else
				{
					uvx_comm_bq_change_list(&comm_bq_1, bq_1_register_list_read);
					uvx_comm_bq_change_list(&comm_bq_2, bq_2_register_list_read);
					uvx_comm_bq_change_list(&comm_bq_3, bq_3_register_list_read);						
					batt_state.state_current = BATT_MODE_READ_BQ;
					p_app_comm_bq = comm_bq;
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}
		break;	

		case BATT_MODE_READ_BQ:
			bq_state = batt_mode_read_BQ(p_app_comm_bq);

			if(bq_state == UVX_BQ_REG_END)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
				else
				{
					p_app_comm_bq = comm_bq;
					batt_state.state_current = BATT_MODE_CHECK_STATUS_FET_EN;
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
				else
				{
					p_app_comm_bq = comm_bq;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;	
				
				if(p_app_comm_bq < &comm_bq[BQ_DEVICES - 1])
				{
					p_app_comm_bq++;
				}
				else
				{
					p_app_comm_bq = comm_bq;
				}

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}
		break;

		case BATT_MODE_CHECK_STATUS_FET_EN:
			bq_state = batt_mode_check_fet_enable(p_app_bq_data);
			
			if(bq_state == UVX_BQ_OK)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
					batt_state.state_current = BATT_MODE_CHECK_STATUS_BYPASS;
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq = p_app_bq_data->p_comm_bq;
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;	
				
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}			
		break;

		case BATT_MODE_CHECK_STATUS_BYPASS:			
			bq_state = batt_mode_check_fet_bps(p_app_bq_data);
			
			if(bq_state == UVX_BQ_OK)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
					batt_state.state_current = BATT_MODE_CHECK_STATUS_FET_DSG;
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq = p_app_bq_data->p_comm_bq;
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;	
				
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}	
		break;

		case BATT_MODE_CHECK_STATUS_FET_DSG:
			bq_state = batt_mode_check_fet_dsg(p_app_bq_data);
			
			if(bq_state == UVX_BQ_OK)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
					batt_state.state_current = BATT_MODE_CHECK_STATUS_FET_CHG;
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq = p_app_bq_data->p_comm_bq;
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;	
				
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}			
		break;

		case BATT_MODE_CHECK_STATUS_FET_CHG:
			bq_state = batt_mode_check_fet_chg(p_app_bq_data);
			
			if(bq_state == UVX_BQ_OK)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
					batt_state.state_current = BATT_MODE_CHECK_STATUS;
					uvx_gpio_set_pin(GPIO_OUTPUT_BLUE_LED, GPIO_PIN_RESET);
				}
			}
			else if(bq_state == UVX_BQ_TIMEOUT)
			{
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}
			}
			else if(bq_state == UVX_BQ_ERROR_NACK)
			{
				p_app_comm_bq = p_app_bq_data->p_comm_bq;
				p_app_comm_bq->RX_Pending = 0;
				p_app_comm_bq->cnt_nack++;
				p_app_comm_bq->No_response = 1;	
				
				if(p_app_bq_data < &bq_data[BQ_DEVICES - 1])
				{
					p_app_bq_data++;
				}
				else
				{
					p_app_bq_data = bq_data;
				}

				if(HAL_GetTick() - p_app_comm_bq->time_stamp > 50)
				{
					p_app_comm_bq->time_stamp = HAL_GetTick();
					uvx_gpio_toggle_pin(GPIO_OUTPUT_PWR_LED);
				}
			}			
		break;		

		case BATT_MODE_CHECK_STATUS:

			// if(batt_data.tc)
			// {
			// 	bq_data_1.FET_CHG_STAT_NEW = false;
			// 	bq_data_2.FET_CHG_STAT_NEW = false;
			// 	bq_data_3.FET_CHG_STAT_NEW = false;
			// }
			// else
			// {
			// 	if((batt_data.adc_pack_v_stable_high) && (drone_status.pwr_fet))
			// 	{				
			// 		if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
			// 		{
			// 			uvx_gpio_set_pin(GPIO_OUTPUT_PWR_LED, GPIO_PIN_RESET);
			// 		}

			// 		bq_data_1.FET_CHG_STAT_NEW = true;
			// 		bq_data_2.FET_CHG_STAT_NEW = true;
			// 		bq_data_3.FET_CHG_STAT_NEW = true;
			// 	}
			// 	else
			// 	{
			// 		bq_data_1.FET_CHG_STAT_NEW = false;
			// 		bq_data_2.FET_CHG_STAT_NEW = false;
			// 		bq_data_3.FET_CHG_STAT_NEW = false;
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
						
			batt_state.state_current = BATT_MODE_READ_CHECK_PACK_V;


	
		break;
		
		case BATT_MODE_READ_CHECK_PACK_V:
			batt_data.init = true;
			batt_state.state_current = BATT_MODE_READ_BQ;
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
			// if((i2c_bq.hal_i2c.I2C_RX_Ready) || (batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE))
			// {
			// 	switch(batt_state.state_next)
			// 	{
			// 		case BATT_MODE_READ_CHECK_PACK_V:
			// 			if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
			// 			{
			// 				batt_data.cnt_no_response = 0;
			// 				batt_state.state_current = batt_state.state_when_fail;
			// 				batt_reg_cnt = 0;
			// 				HAL_Delay(1);
			// 			}
			// 			else
			// 			{
			// 				batt_state.state_current = batt_state.state_when_fail;
			// 				batt_data.cnt_no_response++;
			// 			}
			// 		break;

			// 		case BATT_MODE_INIT_BALANCE_2:
			// 		case BATT_MODE_OFF_BALANCE_1:
			// 		case BATT_MODE_READ_ONCE_BQ_1:
			// 		case BATT_MODE_READ_BQ_1:
			// 			if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
			// 			{
			// 				bq_data_1.No_response = true;
			// 				batt_reg_cnt = 0;
			// 				comm_bq_1.RX_Ready = 1; // Force ready to avoid blocking
			// 				comm_bq_1.p_hal_i2c->I2C_RX_Ready = 1; // Force ready to avoid blocking
			// 				batt_state.state_current = batt_state.state_when_fail;
			// 			}						
			// 			else
			// 			{
			// 				bq_data_1.No_response = false;
			// 				batt_state.state_current = batt_state.state_next;
			// 			}

			// 			batt_data.cnt_no_response = 0;						
						
			// 			comm_bq_1.RX_Ready = 1;
						
			// 			// if(bq_1_register_list_read[batt_reg_cnt].reg_addr == CBSTATUS)
			// 			// {
			// 			// 	for_test = 0;
			// 			// }

			// 			batt_reg_cnt++;							
			// 			HAL_Delay(1);						
			// 		break;
					
			// 		case BATT_MODE_OFF_BALANCE_2:
			// 		case BATT_MODE_INIT_BALANCE_3:
			// 		case BATT_MODE_READ_ONCE_BQ_2:
			// 		case BATT_MODE_READ_BQ_2:
			// 				if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
			// 				{
			// 					bq_data_2.No_response = true;
			// 					batt_reg_cnt = 0;
			// 					comm_bq_2.RX_Ready = 1; // Force ready to avoid blocking
			// 					comm_bq_2.p_hal_i2c->I2C_RX_Ready = 1; // Force ready to avoid blocking
			// 					batt_state.state_current = batt_state.state_when_fail;
			// 				}						
			// 				else
			// 				{
			// 					bq_data_2.No_response = false;
			// 					batt_state.state_current = batt_state.state_next;
			// 				}				

			// 				batt_data.cnt_no_response = 0;							
			// 				comm_bq_2.RX_Ready = 1;
			// 				batt_reg_cnt++;
			// 				HAL_Delay(1);
			// 		break;

			// 		case BATT_MODE_INIT_BALANCE_DONE:
			// 		case BATT_MODE_OFF_BALANCE_3:
			// 		case BATT_MODE_READ_ONCE_BQ_3:
			// 		case BATT_MODE_READ_BQ_3:
			// 			if(batt_data.cnt_no_response > BQ_MAX_NO_RESPONSE)
			// 			{
			// 				bq_data_3.No_response = true;
			// 				batt_reg_cnt = 0;
			// 				comm_bq_3.RX_Ready = 1;
			// 				comm_bq_3.p_hal_i2c->I2C_RX_Ready = 1;
			// 				batt_state.state_current = batt_state.state_when_fail;
			// 			}
			// 			else
			// 			{
			// 				bq_data_3.No_response = false;
			// 				batt_state.state_current = batt_state.state_next;
			// 			}
			// 			batt_data.cnt_no_response = 0;						
			// 			comm_bq_3.RX_Ready = 1;
			// 			batt_reg_cnt++;
			// 			HAL_Delay(1);
			// 		break;
			// 	}			
			// }
			// else
			// {
			// 	batt_data.cnt_no_response++;
			// }	
		break;

		case BATT_MODE_STOP:
			//batt_data.init = false;
			if((batt_data.tc) && (batt_data.adc_pack_v_stable_high))
			{
				batt_state.state_current = BATT_MODE_READ_BQ;
			}
			HAL_Delay(10);
		break;		
	}

	uvx_batt_read_pack_v();
}

static UVX_COMM_BQ_STATE batt_mode_init_bypass(UVX_COMM_BQ *p_comm_bq)
{
	UVX_COMM_BQ_STATE result;
	if(p_comm_bq == NULL)
	{
		return UVX_BQ_ERROR;
	}

	if(!p_comm_bq->RX_Pending)
	{
		p_comm_bq->Bypass_old = 1;
		result = uvx_comm_bq_bypass(p_comm_bq, 0);
		p_comm_bq->RX_Pending = 1;

		if(result == UVX_BQ_ERROR_NACK)
		{			
			return UVX_BQ_ERROR_NACK;
		}

		return UVX_BQ_ERROR_BUSY;
	}
	else
	{
		result = batt_check_response(p_comm_bq);
	}

	return result;
}

static UVX_COMM_BQ_STATE batt_mode_read_BQ(UVX_COMM_BQ *p_comm_bq)
{
	if(p_comm_bq == NULL)
	{
		return UVX_BQ_ERROR;
	}

	if(!p_comm_bq->RX_Pending)
	{
		if(uvx_comm_bq_read_list(p_comm_bq, p_comm_bq->batt_reg_cnt) == UVX_BQ_REG_END)
		{
			p_comm_bq->batt_reg_cnt = 0;
			uvx_batt_parse_data();
			return UVX_BQ_REG_END;
		}

		p_comm_bq->RX_Pending = 1; // Set RX pending flag to indicate that a read operation is in progress
	}
	else
	{
		return batt_check_response(p_comm_bq);
	}	

	return UVX_BQ_ERROR_BUSY;
}

static UVX_COMM_BQ_STATE batt_mode_check_fet_enable(UVX_BQ_DATA *p_bq_data)
{
	UVX_COMM_BQ *p_comm_bq = p_bq_data->p_comm_bq;

	if((p_bq_data == NULL) || (p_comm_bq == NULL))
	{
		return UVX_BQ_ERROR;
	}

	if(!p_comm_bq->RX_Pending)
	{
		if(p_bq_data->FET_DSG_CHG_EN != p_bq_data->FET_DSG_CHG_EN_NEW) 
		{
			uvx_comm_bq_write_mba_register(p_comm_bq, BQ_MA_FET_CONTROL, NULL, p_bq_data->FET_DSG_CHG_EN_NEW);
			p_comm_bq->RX_Pending = 1; // Set RX pending flag to indicate that a read operation is in progress
			return UVX_BQ_ERROR_BUSY;
		}		

		return UVX_BQ_OK;
	}
	else
	{
		return batt_check_response(p_comm_bq);
	}	
}

// Rotate bypass through packs that do not need discharge, one pack at a time.
static void batt_rotate_fet_bps(void)
{
	uint32_t i;
	uint32_t pack;
	uint32_t now = HAL_GetTick();

	if(bps_active_pack < BQ_DEVICES)
	{
		if(!bps_wait_off)
		{
			if(!bps_timer_running && bq_data[bps_active_pack].FET_BPS_STAT)
			{
				bps_start_ms = now;
				bps_timer_running = true;
			}

			if(bq_data[bps_active_pack].bps_need_discharge || !bq_data[bps_active_pack].FET_BPS_EN || (bps_timer_running && (uint32_t)(now - bps_start_ms) >= BATT_BPS_SWITCH_INTERVAL_MS))
			{
				bq_data[bps_active_pack].FET_BPS_EN = false;
				bps_wait_off = true;
			}
		}

		if(!bps_wait_off)
		{
			return;
		}

		// Finish disabling the previous pack before enabling another.
		if(bq_data[bps_active_pack].FET_BPS_STAT || bq_data[bps_active_pack].p_comm_bq->RX_Pending)
		{
			return;
		}

		bps_next_pack = (bps_active_pack + 1U) % BQ_DEVICES;
		bps_active_pack = BQ_DEVICES;
		bps_timer_running = false;
		bps_wait_off = false;
	}

	// Also clear any previous bypass requests before starting the sequence.
	for(i = 0; i < BQ_DEVICES; i++)
	{
		bq_data[i].FET_BPS_EN = false;
	}

	for(i = 0; i < BQ_DEVICES; i++)
	{
		if(bq_data[i].FET_BPS_STAT || (bq_data[i].p_comm_bq != NULL && bq_data[i].p_comm_bq->RX_Pending))
		{
			return;
		}
	}

	for(i = 0; i < BQ_DEVICES; i++)
	{
		pack = (bps_next_pack + i) % BQ_DEVICES;
		if(!bq_data[pack].bps_need_discharge && bq_data[pack].p_comm_bq != NULL)
		{
			bps_active_pack = pack;

			if((!drone_status.pwr_fet) && (!drone_status.esc_arm) && (!drone_status.esc_psys_arm) && (batt_data.adc_pack_v < batt_data.pwr_min_voltage))
			{
				bq_data[pack].FET_BPS_EN = true;
			}
			else
			{
				bq_data[pack].FET_BPS_EN = false;
			}
			return;
		}
	}
}

static UVX_COMM_BQ_STATE batt_mode_check_fet_bps(UVX_BQ_DATA *p_bq_data)
{
	UVX_COMM_BQ_STATE comm_state;
	UVX_COMM_BQ *p_comm_bq = (p_bq_data != NULL) ? p_bq_data->p_comm_bq : NULL;

	if((p_bq_data == NULL) || (p_comm_bq == NULL))
	{
		return UVX_BQ_ERROR;
	}

	batt_rotate_fet_bps();

	if(p_bq_data->FET_BPS_EN) //if we enable bypass FET
	{
		//if((!drone_status.pwr_fet) && (!drone_status.esc_arm) && (!drone_status.esc_psys_arm))
		{
			if(!p_bq_data->FET_BPS_STAT)
			{
				p_bq_data->FET_DSG_STAT_NEW = false;
				p_bq_data->FET_CHG_STAT_NEW = false;

				comm_state = uvx_comm_bq_discharge_fet(p_bq_data, p_bq_data->FET_DSG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_DSG_STAT = false;
				}

				comm_state = uvx_comm_bq_charge_fet(p_bq_data, p_bq_data->FET_CHG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_CHG_STAT = false;
				}

				p_bq_data->FET_CHG_STAT = false;
				p_bq_data->FET_BPS_STAT_NEW = true;

				comm_state = uvx_comm_bq_bypass(p_comm_bq, p_bq_data->FET_BPS_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_BPS_STAT = true;
				}

				return UVX_BQ_OK;
			}
		}
	}
	else
	{
		if((p_bq_data->FET_BPS_STAT))
		{			
				p_bq_data->FET_BPS_STAT_NEW = false;
				comm_state = uvx_comm_bq_bypass(p_comm_bq, p_bq_data->FET_BPS_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_BPS_STAT = false;
				}

				p_bq_data->FET_DSG_STAT_NEW = true;
				p_bq_data->FET_CHG_STAT_NEW = true;

				comm_state = uvx_comm_bq_charge_fet(p_bq_data, p_bq_data->FET_CHG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_CHG_STAT = true;
				}


				comm_state = uvx_comm_bq_discharge_fet(p_bq_data, p_bq_data->FET_DSG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					while(batt_check_response(p_comm_bq) != UVX_BQ_OK)
					{				
						HAL_Delay(1);
					}		
					p_bq_data->FET_DSG_STAT = true;
				}
				
				return UVX_BQ_OK;				
		}					
	}

	return UVX_BQ_OK;
}

static UVX_COMM_BQ_STATE batt_mode_check_fet_dsg(UVX_BQ_DATA *p_bq_data)
{
	UVX_COMM_BQ *p_comm_bq = p_bq_data->p_comm_bq;

	if((p_bq_data == NULL) || (p_comm_bq == NULL))
	{
		return UVX_BQ_ERROR;
	}

	if(!p_comm_bq->RX_Pending)
	{
		if(p_bq_data->FET_DSG_STAT != p_bq_data->FET_DSG_STAT_NEW)
		{
			uvx_comm_bq_discharge_fet(p_bq_data, p_bq_data->FET_DSG_STAT_NEW);
			p_comm_bq->RX_Pending = 1; // Set RX pending flag to indicate that a read operation is in progress
			return UVX_BQ_ERROR_BUSY;
		}		

		return UVX_BQ_OK;
	}
	else
	{
		return batt_check_response(p_comm_bq);
	}	
}

static UVX_COMM_BQ_STATE batt_mode_check_fet_chg(UVX_BQ_DATA *p_bq_data)
{
	UVX_COMM_BQ_STATE state;
	UVX_COMM_BQ *p_comm_bq = p_bq_data->p_comm_bq;

	if((p_bq_data == NULL) || (p_comm_bq == NULL))
	{
		return UVX_BQ_ERROR;
	}

	if(!p_comm_bq->RX_Pending)
	{
		if((p_bq_data->FET_CHG_STAT) != (p_bq_data->FET_CHG_STAT_NEW))
		{
			uvx_comm_bq_charge_fet(p_bq_data, p_bq_data->FET_CHG_STAT_NEW);
			p_comm_bq->RX_Pending = 1; // Set RX pending flag to indicate that a read operation is in progress
			return UVX_BQ_ERROR_BUSY;
		}

		return UVX_BQ_OK;
	}
	else
	{
		state = batt_check_response(p_comm_bq);
		if(state == UVX_BQ_OK)
		{
			if(p_bq_data->bypass_time_start_ms != 0)
			{
				p_bq_data->bypass_time_ms = HAL_GetTick() - p_bq_data->bypass_time_start_ms;	
				p_bq_data->bypass_time_start_ms = 0;	
			}
		}
		return state;
	}	
}

/* Process one pending response. The caller remains busy until its operation
 * (such as the complete register list) is finished. */
static UVX_COMM_BQ_STATE batt_check_response(UVX_COMM_BQ *p_comm_bq)
{
	p_comm_bq->i2c_state = uvx_i2c_check_response(&i2c_bq.hal_i2c, (uint32_t*)p_comm_bq);

	if(p_comm_bq->i2c_state == UVX_I2C_UNLOCKED)
	{
		p_comm_bq->RX_Pending = 0;
		p_comm_bq->BQ_RX_Ready = 1;
		p_comm_bq->BQ_TX_Ready = 1;		
		return UVX_BQ_ERROR_UNLOCKED;
	}

	if(p_comm_bq->i2c_state == UVX_I2C_LOCK_ERROR)
	{
		return UVX_BQ_ERROR_LOCK;
	}

	if(p_comm_bq->i2c_state == UVX_I2C_NACK)
	{		
		p_comm_bq->No_response = true;
		p_comm_bq->cnt_no_response = 0;
		p_comm_bq->RX_Pending = 0;
		p_comm_bq->BQ_RX_Ready = 1;
		p_comm_bq->BQ_TX_Ready = 1;
		uvx_i2c_unlock(&i2c_bq.hal_i2c, (uint32_t*)p_comm_bq);

		return UVX_BQ_ERROR_NACK;
	}

	if(p_comm_bq->i2c_state == UVX_I2C_STOP_DETECTED)
	{
			
		p_comm_bq->cnt_no_response = 0;
		p_comm_bq->RX_Pending = 0;
		p_comm_bq->BQ_RX_Ready = 1;
		p_comm_bq->BQ_TX_Ready = 1;

		p_comm_bq->batt_reg_cnt++;
		uvx_i2c_unlock(&i2c_bq.hal_i2c, (uint32_t*)p_comm_bq);
		return UVX_BQ_OK;
	}	

	if(p_comm_bq->i2c_state == UVX_I2C_ERROR)
	{
		p_comm_bq->No_response = true;
		p_comm_bq->cnt_no_response = 0;
		p_comm_bq->RX_Pending = 0;
		p_comm_bq->BQ_RX_Ready = 1;
		p_comm_bq->BQ_TX_Ready = 1;
		uvx_i2c_unlock(&i2c_bq.hal_i2c, (uint32_t*)p_comm_bq);
		return UVX_BQ_ERROR;
	}	

	if(p_comm_bq->i2c_state == UVX_I2C_OK)
	{
		p_comm_bq->No_response = false;
		p_comm_bq->batt_reg_cnt++;
		p_comm_bq->cnt_no_response = 0;
		p_comm_bq->RX_Pending = 0;
		p_comm_bq->BQ_RX_Ready = 1;
		return UVX_BQ_OK;
	}
	else
	{
		if(p_comm_bq == &comm_bq_3)
		{
			p_comm_bq->cnt_no_response++;
		}

		p_comm_bq->cnt_no_response++;
		if(p_comm_bq->cnt_no_response > BQ_MAX_NO_RESPONSE)
		{

			// if((p_comm_bq->p_hal_i2c->hi2c.Instance->ISR) != 0x01)
			// {
			// 	HAL_I2C_ER_IRQHandler(&p_comm_bq->p_hal_i2c->hi2c);	
			// }
			p_comm_bq->cnt_no_response = 0;
			p_comm_bq->No_response = true;
			p_comm_bq->RX_Pending = 0;
			p_comm_bq->BQ_RX_Ready = 1;
			uvx_i2c_unlock(&i2c_bq.hal_i2c, (uint32_t*)p_comm_bq);				
			return UVX_BQ_TIMEOUT;
		}						
		else
		{
			p_comm_bq->No_response = false;
		}			
	}

	return UVX_BQ_ERROR_BUSY;
}
