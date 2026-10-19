#include "uvx_app_battery.h"
#include "main.h"

extern UVX_I2C i2c_bq;
static UVX_BQ_DATA *p_app_bq_data;
static UVX_COMM_BQ *p_app_comm_bq;
static UVX_COMM_BQ_STATE bq_state;

#define BATT_BPS_SWITCH_INTERVAL_MS 15000U
#define BATT_BPS_MIN_CELL_VOLTAGE_MV 2700
static uint32_t bps_active_pack = BQ_DEVICES;
static uint32_t bps_next_pack;
#define BATT_BPS_DISCHARGE_MAH 20U
#define BATT_BPS_REST_INTERVAL_MS 20000U

typedef enum
{
	BPS_IDLE,
	BPS_SELECT_BYPASS,
	BPS_WAIT_ON,
	BPS_HOLD,
	BPS_WAIT_OFF,
	BPS_COMPLETE,
	BPS_REST,
	BPS_REASSESS
} BATT_BPS_STATE;

static BATT_BPS_STATE bps_state = BPS_IDLE;
static uint32_t bps_discharge_pack = BQ_DEVICES;
static uint16_t bps_qmax_start[BQ_DEVICES];
static uint16_t bps_qmax_time[BQ_DEVICES];
static bool bps_discharge_selected[BQ_DEVICES];
static uint32_t bps_read_sequence[BQ_DEVICES];
static uint32_t bps_rest_start_ms;
static uint32_t bps_rest_read_sequence[BQ_DEVICES];

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
			bps_discharge_pack = BQ_DEVICES;
			bps_state = BPS_IDLE;
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
			bq_data_1.FET_DSG_STAT_NEW = 0;
			bq_data_1.FET_CHG_STAT_NEW = 0;

			bq_data_2.FET_DSG_CHG_EN = 0;
			bq_data_2.FET_BPS_EN = 0;
			bq_data_2.FET_DSG_STAT_NEW = 0;
			bq_data_2.FET_CHG_STAT_NEW = 0;

			bq_data_3.FET_DSG_CHG_EN = 0;
			bq_data_3.FET_BPS_EN = 0;
			bq_data_3.FET_DSG_STAT_NEW = 0;
			bq_data_3.FET_CHG_STAT_NEW = 0;
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
			else if((bq_state == UVX_BQ_ERROR_NACK)/* || (bq_state == UVX_BQ_ERROR_BUSY)*/)
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
					batt_state.state_current = BATT_MODE_CHECK_STATUS_FET_EN;
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

			if(batt_data.tc)
			{
				// bq_data_1.FET_CHG_STAT_NEW = false;
				// bq_data_2.FET_CHG_STAT_NEW = false;
				// bq_data_3.FET_CHG_STAT_NEW = false;
				bq_data_1.FET_CHG_STAT_NEW = true;
				bq_data_2.FET_CHG_STAT_NEW = true;
				bq_data_3.FET_CHG_STAT_NEW = true;
				bq_data_1.FET_DSG_STAT_NEW = true;
				bq_data_2.FET_DSG_STAT_NEW = true;
				bq_data_3.FET_DSG_STAT_NEW = true;
				UVX_APP_PWR_FET(0);				
			}
			else
			{
				if((batt_data.adc_pack_v_stable_high))
				{				
					if(drone_state.state_current == DRONE_CHECK_BUTTON_PRESS_ONCE)
					{
						uvx_app_pwr_led(1);
					}

					bq_data_1.FET_CHG_STAT_NEW = true;
					bq_data_2.FET_CHG_STAT_NEW = true;
					bq_data_3.FET_CHG_STAT_NEW = true;
					bq_data_1.FET_DSG_STAT_NEW = true;
					bq_data_2.FET_DSG_STAT_NEW = true;
					bq_data_3.FET_DSG_STAT_NEW = true;
				}
				else
				{
					// bq_data_1.FET_CHG_STAT_NEW = false;
					// bq_data_2.FET_CHG_STAT_NEW = false;
					// bq_data_3.FET_CHG_STAT_NEW = false;
				}		
			}

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
			if(p_comm_bq->ID > 0U && p_comm_bq->ID <= BQ_DEVICES)
			{
				bps_read_sequence[p_comm_bq->ID - 1U]++;
			}
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

// Discharge all selected high packs; rotate bypass only among the lower packs.
static void batt_rotate_fet_bps(void)
{
	uint32_t i;
	uint32_t pack;
	uint32_t now = HAL_GetTick();
	int32_t passed_mah;

	// Sleep must never select another pack for bypass.
	if(drone_state.state_current == DRONE_SLEEP)
	{
		for(i = 0; i < BQ_DEVICES; i++)
		{
			bq_data[i].FET_BPS_EN = false;
			bq_data[i].FET_BPS_STAT_NEW = false;
		}
		bps_active_pack = BQ_DEVICES;
		bps_next_pack = 0;
		bps_discharge_pack = BQ_DEVICES;
		bps_state = BPS_IDLE;
		return;
	}

	// Keep prepare_fc set until the packs are ready for FC power-on.
	if(drone_status.pwr_fc || comm_m2jmb_state.state_current == M2JMB_MODE_SEND_FC_ON_ACK)
	{
		for(i = 0; i < BQ_DEVICES; i++)
		{
			bq_data[i].FET_BPS_EN = false;
			bq_data[i].FET_BPS_STAT_NEW = false;
			bq_data[i].FET_CHG_STAT_NEW = true;
			bq_data[i].FET_DSG_STAT_NEW = true;
		}

		// Use reported status, not just the requested FET settings.
		for(i = 0; i < BQ_DEVICES; i++)
		{
			if(!bq_data[i].operation_status.reg.bits.CHG ||
			   !bq_data[i].operation_status.reg.bits.DSG ||
			   bq_data[i].FET_BPS_STAT ||
			   bq_data[i].p_comm_bq == NULL ||
			   bq_data[i].p_comm_bq->No_response ||
			   bq_data[i].p_comm_bq->RX_Pending)
			{
				break;
			}
		}
		
		if(i == BQ_DEVICES)
		{
			drone_status.prepare_fc = false;
		}

		bps_active_pack = BQ_DEVICES;
		bps_next_pack = 0;
		bps_discharge_pack = BQ_DEVICES;
		bps_state = BPS_IDLE;
		return;
	}

	// Stop bypass whenever the minimum detected cell voltage is too low.
	if(batt_data.voltage_min_cell < BATT_BPS_MIN_CELL_VOLTAGE_MV)
	{
		for(i = 0; i < BQ_DEVICES; i++)
		{
			bq_data[i].FET_BPS_EN = false;
			bq_data[i].FET_BPS_STAT_NEW = false;
		}
		bps_active_pack = BQ_DEVICES;
		bps_next_pack = 0;
		bps_discharge_pack = BQ_DEVICES;
		bps_state = BPS_IDLE;
		return;
	}

	// Abort balancing if operating conditions or pack communication are invalid.
	for(i = 0; i < BQ_DEVICES; i++)
	{
		if((batt_data.current >= 0) || drone_status.esc_arm || drone_status.esc_psys_arm ||
		   bq_data[i].p_comm_bq == NULL || bq_data[i].p_comm_bq->No_response)
		{
			uint32_t j;
			for(j = 0; j < BQ_DEVICES; j++)
			{
				bq_data[j].FET_BPS_EN = false;
				bq_data[j].FET_BPS_STAT_NEW = false;
			}
			bps_state = BPS_IDLE;
			bps_active_pack = BQ_DEVICES;
			bps_discharge_pack = BQ_DEVICES;
			return;
		}
	}

	if(bps_state == BPS_WAIT_ON || bps_state == BPS_HOLD)
	{
		// End the interval when any selected discharge pack reaches 10 mAh.
		for(i = 0; i < BQ_DEVICES; i++)
		{
			if(!bps_discharge_selected[i])
			{
				continue;
			}
			passed_mah = (uint16_t)(bq_data[i].Qmax_passed - bps_qmax_start[i]);
			if(passed_mah > 32767)
			{
				passed_mah -= 65536;
			}
			if(passed_mah < 0 || passed_mah >= (int32_t)BATT_BPS_DISCHARGE_MAH ||
			   bq_data[i].Qmax_time < bps_qmax_time[i])
			{
				bps_state = BPS_COMPLETE;
				break;
			}
			bps_qmax_time[i] = bq_data[i].Qmax_time;
		}
	}

	switch(bps_state)
	{
		case BPS_IDLE:
			for(i = 0; i < BQ_DEVICES; i++)
			{
				bq_data[i].FET_BPS_EN = false;				
			}
			for(i = 0; i < BQ_DEVICES; i++)
			{
				if(bq_data[i].FET_BPS_STAT || bq_data[i].p_comm_bq->RX_Pending)
				{
					return;
				}
			}
			// Initialize/recompute flags while all bypasses are off.
			// Otherwise zero startup flags prevent ever reaching BPS_REASSESS.
			uvx_batt_find_highest_average_pack_voltage();
			bps_discharge_pack = BQ_DEVICES;
			for(i = 0; i < BQ_DEVICES; i++)
			{
				if(bq_data[i].bps_need_discharge &&
				   (bps_discharge_pack == BQ_DEVICES ||
				    bq_data[i].voltage_per_cell > bq_data[bps_discharge_pack].voltage_per_cell))
				{
					bps_discharge_pack = i;
				}
			}
			if(bps_discharge_pack == BQ_DEVICES)
			{
				return;
			}
			// Latch all targets for this interval; reassess only after recovery.
			for(i = 0; i < BQ_DEVICES; i++)
			{
				bps_discharge_selected[i] = bq_data[i].bps_need_discharge;
				bps_qmax_start[i] = bq_data[i].Qmax_passed;
				bps_qmax_time[i] = bq_data[i].Qmax_time;
			}
			bps_state = BPS_SELECT_BYPASS;
			break;

		case BPS_SELECT_BYPASS:
			for(i = 0; i < BQ_DEVICES; i++)
			{
				if(bq_data[i].FET_BPS_STAT || bq_data[i].p_comm_bq->RX_Pending) // if any pack is still bypassing, wait for it to finish before selecting the next pack
				{
					return;
				}
			}

			// Clear the bypass status flags so that the next pack can be selected for bypass.
			for(i = 0; i < BQ_DEVICES; i++)
			{
				bq_data[i].FET_CHG_STAT_NEW = true;
				bq_data[i].FET_DSG_STAT_NEW = true;
			}

			for(i = 0; i < BQ_DEVICES; i++)
			{
				pack = (bps_next_pack + i) % BQ_DEVICES; // Rotate through the packs in order, starting with the next pack after the last one that was bypassed.
				if(!bps_discharge_selected[pack])
				{
					bps_active_pack = pack;
					bq_data[pack].bypass_time_start_ms = now;
					bq_data[pack].bypass_time_ms = 0U;
					bq_data[pack].FET_BPS_EN = true;
					bps_state = BPS_WAIT_ON;
					break;
				}
			}
			break;

		case BPS_WAIT_ON:
			bq_data[bps_active_pack].bypass_time_ms = (uint32_t)(now - bq_data[bps_active_pack].bypass_time_start_ms);

			if(bq_data[bps_active_pack].FET_BPS_STAT)
			{
				bq_data[bps_active_pack].bypass_time_start_ms = now;
				bq_data[bps_active_pack].bypass_time_ms = 0U;
				bps_state = BPS_HOLD;
			}
			else if(!bq_data[bps_active_pack].FET_BPS_EN || bq_data[bps_active_pack].bypass_time_ms >= BATT_BPS_SWITCH_INTERVAL_MS)
			{
				bq_data[bps_active_pack].FET_BPS_EN = false;
				bps_state = BPS_WAIT_OFF;
			}
			break;

		case BPS_HOLD:
			bq_data[bps_active_pack].bypass_time_ms = (uint32_t)(now - bq_data[bps_active_pack].bypass_time_start_ms);

			if(!bq_data[bps_active_pack].FET_BPS_EN || bq_data[bps_active_pack].bypass_time_ms >= BATT_BPS_SWITCH_INTERVAL_MS)
			{
				bq_data[bps_active_pack].FET_BPS_EN = false;
				bps_state = BPS_WAIT_OFF;
			}
			break;

		case BPS_WAIT_OFF:
		case BPS_COMPLETE:
			for(i = 0; i < BQ_DEVICES; i++)
			{
				bq_data[i].FET_BPS_EN = false;
				bq_data[i].FET_BPS_STAT_NEW = false;
				bq_data[i].FET_CHG_STAT_NEW = true;
				bq_data[i].FET_DSG_STAT_NEW = true;
			}
			for(i = 0; i < BQ_DEVICES; i++)
			{
				if(bq_data[i].FET_BPS_STAT || bq_data[i].p_comm_bq->RX_Pending ||
				   !bq_data[i].FET_CHG_STAT || !bq_data[i].FET_DSG_STAT)
				{
					return;
				}
			}
			if(bps_active_pack < BQ_DEVICES)
			{
				bps_next_pack = (bps_active_pack + 1U) % BQ_DEVICES;
			}
			bps_active_pack = BQ_DEVICES;
			bps_discharge_pack = BQ_DEVICES;
			bps_rest_start_ms = now;
			bps_state = BPS_REST;
			break;

		case BPS_REST:
			// Begin recovery only after every bypass is off and FETs are restored.
			if((uint32_t)(now - bps_rest_start_ms) >= BATT_BPS_REST_INTERVAL_MS)
			{
				for(i = 0; i < BQ_DEVICES; i++)
				{
					bps_rest_read_sequence[i] = bps_read_sequence[i];
				}
				bps_state = BPS_REASSESS;
			}
			break;

		case BPS_REASSESS:
			// All packs must have fresh readings after the recovery interval.
			for(i = 0; i < BQ_DEVICES; i++)
			{
				if(bps_read_sequence[i] == bps_rest_read_sequence[i])
				{
					return;
				}
			}
			uvx_batt_find_highest_average_pack_voltage();
			bps_state = BPS_IDLE;
			break;

		default:
			bps_state = BPS_IDLE;
			break;
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
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
					}		
					p_bq_data->FET_DSG_STAT = false;
				}

				comm_state = uvx_comm_bq_charge_fet(p_bq_data, p_bq_data->FET_CHG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
					}		
					p_bq_data->FET_CHG_STAT = false;
				}

				p_bq_data->FET_CHG_STAT = false;
				p_bq_data->FET_BPS_STAT_NEW = true;

				comm_state = uvx_comm_bq_bypass(p_comm_bq, p_bq_data->FET_BPS_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
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
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
					}		
					p_bq_data->FET_BPS_STAT = false;
				}

				p_bq_data->FET_DSG_STAT_NEW = true;
				p_bq_data->FET_CHG_STAT_NEW = true;

				comm_state = uvx_comm_bq_charge_fet(p_bq_data, p_bq_data->FET_CHG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
					}		
					p_bq_data->FET_CHG_STAT = true;
				}


				comm_state = uvx_comm_bq_discharge_fet(p_bq_data, p_bq_data->FET_DSG_STAT_NEW);
				if(comm_state != UVX_BQ_OK)
				{
					do
					{
						comm_state = batt_check_response(p_comm_bq);
						if(comm_state == UVX_BQ_ERROR_BUSY)
						{
							HAL_Delay(1);
						}
					} while(comm_state == UVX_BQ_ERROR_BUSY);
					if(comm_state != UVX_BQ_OK)
					{
						// Retry the desired GPIO state after a failed transaction.
						p_comm_bq->Bypass_old = !p_comm_bq->Bypass;
						return (comm_state == UVX_BQ_ERROR_NACK) ? comm_state : UVX_BQ_TIMEOUT;
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
