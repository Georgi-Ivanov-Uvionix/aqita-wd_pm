/**
  ******************************************************************************
  * @file    uvx_dock_charger.c
  * @brief   Dock charger control using two TPL0401x-10 devices.
  *
  * TPL0401A-10 (I2C address 0x2E) regulates charge current.
  * TPL0401B-10 (I2C address 0x3E) regulates charge voltage.
  *
  * The application calls this non-blocking driver from its main state loop.
  ******************************************************************************
  */

#include "main.h"
#include "uvx_dock_charger.h"

#define UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS    0x2EU
#define UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS    0x3EU
#define UVX_DOCK_CHARGER_WIPER_MAX              0x7FU
#define UVX_DOCK_CHARGER_VOLTAGE_OFFSET_MV      1000U
#define UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV    200U
#define UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MV    200U
#define UVX_DOCK_CHARGER_RAMP_INTERVAL_MS         500U

/* Temporary declarations until uvx_tpl0401x_10.h is added after review. */
UVX_I2C_STATE uvx_tpl0401x_10_init(UVX_I2C *i2c);
UVX_I2C_STATE uvx_tpl0401x_10_write(UVX_I2C *i2c,
                                    uint8_t device_address,
                                    uint8_t wiper_position);
UVX_I2C_STATE uvx_tpl0401x_10_read(UVX_I2C *i2c,
                                   uint8_t device_address,
                                   uint8_t *wiper_position);

UVX_DOCK_CHARGER_STATE_MACHINE dock_charger_state;
UVX_DOCK_CHARGER_STATUS dock_charger_status;

static void uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE state_next)
{
    dock_charger_state.state_next = state_next;
    dock_charger_state.state_current = dock_charger_state.state_next;
}

static UVX_I2C_STATE uvx_dock_charger_lock_i2c(void)
{
    UVX_I2C_STATE state = uvx_i2c_lock(&dock_charger_status.i2c->hal_i2c);

    if(state == UVX_I2C_OK)
    {
        dock_charger_status.owns_i2c_lock = 1U;
    }

    return state;
}

static void uvx_dock_charger_unlock_i2c(void)
{
    if(dock_charger_status.owns_i2c_lock != 0U)
    {
        if(uvx_i2c_unlock(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_OK)
        {
            dock_charger_status.owns_i2c_lock = 0U;
        }
    }
}

/**
 * @brief Convert an engineering-unit value to a calibrated wiper code.
 */
static UVX_DOCK_CHARGER_RESULT uvx_dock_charger_value_to_code(
    uint32_t value,
    uint32_t value_min,
    uint32_t value_max,
    uint8_t code_at_min,
    uint8_t code_at_max,
    uint8_t *code)
{
    int64_t numerator;
    int64_t code_result;
    uint32_t value_range;

    if((code == NULL) ||
       (value_min >= value_max) ||
       (value < value_min) ||
       (value > value_max) ||
       (code_at_min > UVX_DOCK_CHARGER_WIPER_MAX) ||
       (code_at_max > UVX_DOCK_CHARGER_WIPER_MAX))
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    value_range = value_max - value_min;
    numerator = (int64_t)(value - value_min) *
                ((int64_t)code_at_max - (int64_t)code_at_min);

    /* Round to the nearest wiper position for either calibration direction. */
    if(numerator >= 0)
    {
        numerator += (int64_t)(value_range / 2U);
    }
    else
    {
        numerator -= (int64_t)(value_range / 2U);
    }

    code_result = (int64_t)code_at_min + (numerator / (int64_t)value_range);
    if((code_result < 0) || (code_result > UVX_DOCK_CHARGER_WIPER_MAX))
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    *code = (uint8_t)code_result;
    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Initialize the dock charger control module.
 * @param i2c I2C bus shared by the two TPL0401x-10 devices.
 * @param config Physical-value to wiper-code calibration.
 * @param current_ma Initial requested charging current in milliamperes.
 * @param voltage_mv Initial requested charging voltage in millivolts.
 * @retval Charger module status.
 */

static uint32_t uvx_dock_charger_battery_target_mv(void)
{
    uint32_t battery_target_mv = (uint32_t)batt_data.batt_voltage + UVX_DOCK_CHARGER_VOLTAGE_OFFSET_MV;

    if(battery_target_mv > dock_charger_status.config.voltage_max_mv)
    {
        battery_target_mv = dock_charger_status.config.voltage_max_mv;
    }

    if((batt_data.pwr_max_voltage != 0U) && (battery_target_mv > batt_data.pwr_max_voltage))
    {
        battery_target_mv = batt_data.pwr_max_voltage;
    }

    return battery_target_mv;
}

UVX_DOCK_CHARGER_RESULT uvx_dock_charger_init(UVX_I2C *i2c, const UVX_DOCK_CHARGER_CONFIG *config)
{
    UVX_I2C_STATE i2c_state;

    if((i2c == NULL) || (config == NULL))
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    i2c_state = uvx_tpl0401x_10_init(i2c);
    if((i2c_state != UVX_I2C_OK) && (i2c_state != UVX_I2C_ALREADY_INITIALIZED))
    {
        return UVX_DOCK_CHARGER_ERROR_I2C;
    }

    dock_charger_status.i2c = i2c;
    dock_charger_status.config = *config;
    dock_charger_status.target_current_ma = DOCK_CHARGER_CURRENT_MIN_MA;
    dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
    dock_charger_status.current_code = DOCK_CHARGER_WIPER_MIN;
    dock_charger_status.voltage_code = DOCK_CHARGER_WIPER_MIN;
    dock_charger_status.owns_i2c_lock = 0U;
    dock_charger_status.voltage_ready = 0U;
    dock_charger_status.current_ready = 0U;
    dock_charger_status.current_ramp_tick = HAL_GetTick();
    dock_charger_status.voltage_ramp_tick = HAL_GetTick();
    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
    dock_charger_status.initialized = true;

    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Set the desired charging current in milliamperes.
 */
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_set_current(uint32_t current_ma)
{
    uint8_t validated_code;
    uint32_t maximum_current_ma;

    if(dock_charger_state.state_current == UVX_DOCK_CHARGER_STATE_NOT_INITIALIZED)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    maximum_current_ma = dock_charger_status.config.current_max_ma;
    if((batt_data.pwr_max_current != 0U) &&
       (maximum_current_ma > batt_data.pwr_max_current))
    {
        maximum_current_ma = batt_data.pwr_max_current;
    }
    if(current_ma > maximum_current_ma)
    {
        current_ma = maximum_current_ma;
    }

    if(uvx_dock_charger_value_to_code(current_ma,
                                      dock_charger_status.config.current_min_ma,
                                      dock_charger_status.config.current_max_ma,
                                      dock_charger_status.config.current_code_at_min,
                                      dock_charger_status.config.current_code_at_max,
                                      &validated_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(dock_charger_status.target_current_ma != current_ma)
    {
        dock_charger_status.target_current_ma = current_ma;
        dock_charger_status.current_ready = 0U;
    }

    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Set the desired charging voltage in millivolts.
 */
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_set_voltage(uint32_t voltage_mv)
{
    uint8_t validated_code;
    uint32_t maximum_voltage_mv;

    if(dock_charger_state.state_current == UVX_DOCK_CHARGER_STATE_NOT_INITIALIZED)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    maximum_voltage_mv = dock_charger_status.config.voltage_max_mv;
    if((batt_data.pwr_max_voltage != 0U) && (maximum_voltage_mv > batt_data.pwr_max_voltage))
    {
        maximum_voltage_mv = batt_data.pwr_max_voltage;
    }
    if(voltage_mv > maximum_voltage_mv)
    {
        voltage_mv = maximum_voltage_mv;
    }
    if(voltage_mv < dock_charger_status.config.voltage_min_mv)
    {
        voltage_mv = dock_charger_status.config.voltage_min_mv;
    }

    if(uvx_dock_charger_value_to_code(voltage_mv,
                                      dock_charger_status.config.voltage_min_mv,
                                      dock_charger_status.config.voltage_max_mv,
                                      dock_charger_status.config.voltage_code_at_min,
                                      dock_charger_status.config.voltage_code_at_max,
                                      &validated_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(dock_charger_status.target_voltage_mv != voltage_mv)
    {
        dock_charger_status.target_voltage_mv = voltage_mv;
    }

    return UVX_DOCK_CHARGER_OK;
}

static UVX_DOCK_CHARGER_RESULT uvx_dock_charger_regulate_voltage_and_current(void)
{
    uint32_t now = HAL_GetTick();

    if( (dock_charger_status.voltage_ready == 0U) &&    
        ((uint32_t)batt_data.adc_pack_v >= dock_charger_status.target_voltage_mv - UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV) &&
        ((uint32_t)batt_data.adc_pack_v <= dock_charger_status.target_voltage_mv + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV) )
    {
        dock_charger_status.voltage_ready = 1U;
    }

    if(dock_charger_status.voltage_ready == 0U)
    {
        UVX_APP_PWR_FET(0U);

        if((uint32_t)(now - dock_charger_status.voltage_ramp_tick) < UVX_DOCK_CHARGER_RAMP_INTERVAL_MS)
        {
            return UVX_DOCK_CHARGER_BUSY;
        }

        if( ((uint32_t)batt_data.adc_pack_v < dock_charger_status.target_voltage_mv + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV) &&
            (dock_charger_status.voltage_code < UVX_DOCK_CHARGER_WIPER_MAX) )
        {
            dock_charger_status.voltage_code++;
        }
        else if(((uint32_t)batt_data.adc_pack_v > dock_charger_status.target_voltage_mv - UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV) &&
                (dock_charger_status.voltage_code > 0U) )
        {
            dock_charger_status.voltage_code--;
        }
        else
        {
            return UVX_DOCK_CHARGER_BUSY;
        }

        dock_charger_status.voltage_ramp_tick = now;
        dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_WRITE_CURRENT;
        return UVX_DOCK_CHARGER_BUSY;
    }

    dock_charger_status.target_current_ma = 1000; //for testing only, remove later

    if((dock_charger_status.target_current_ma > batt_data.pwr_max_current))
    {
        dock_charger_status.target_current_ma = batt_data.pwr_max_current;
    }

    if(dock_charger_status.target_voltage_mv > batt_data.pwr_max_voltage)
    {
        dock_charger_status.target_voltage_mv = batt_data.pwr_max_voltage;
    }

    if( (!dock_charger_status.current_ready) && (dock_charger_status.current_ma > 0) && (dock_charger_status.voltage_ready) )
    {
        if(dock_charger_status.current_ma > (dock_charger_status.target_current_ma + UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MV))
        {
            /* Overcurrent has priority over all normal ramp operations. */
            UVX_APP_PWR_FET(0U);
            dock_charger_status.current_ready = 0U;
            dock_charger_status.current_code = 0U;
            dock_charger_status.current_code = 0U;
            dock_charger_status.current_ramp_tick = now;
            return UVX_DOCK_CHARGER_BUSY;
        }

        UVX_APP_PWR_FET(1U);

        if( (dock_charger_status.current_ma >= dock_charger_status.target_current_ma - UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MV) &&
            (dock_charger_status.current_ma <= dock_charger_status.target_current_ma + UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MV) )    
        {
            dock_charger_status.current_ready = true;
        }
        else if( (dock_charger_status.current_ma < dock_charger_status.target_current_ma) &&
                ((uint32_t)(now - dock_charger_status.current_ramp_tick) >= UVX_DOCK_CHARGER_RAMP_INTERVAL_MS) )
        {
            if(dock_charger_status.current_code < UVX_DOCK_CHARGER_WIPER_MAX)
            {
                dock_charger_status.current_code++;
            }

            dock_charger_status.target_voltage_mv = dock_charger_status.voltage_mv + 500; //for testing only, remove later
            dock_charger_status.voltage_ready = 0U;
            dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_WRITE_CURRENT;
            dock_charger_status.current_ramp_tick = now;
            return UVX_DOCK_CHARGER_BUSY;
        }        
    }   
    
    return (dock_charger_status.current_ready != 0U) ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
}

/**
 * @brief Apply charger settings after the landing Hall sensor is active.
 * @retval OK when both settings are applied, BUSY while I2C is in progress,
 *         or an error status.
 *
 * Call this function repeatedly from the application loop.  The two writes
 * are sequenced because uvx_i2c uses interrupt-driven transfers on one bus.
 */
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_process(void)
{
    UVX_I2C_STATE i2c_state;

    if(dock_charger_status.i2c == NULL)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    if(drone_status.hall_land_2 == false)
    {
        dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
        dock_charger_status.voltage_ready = 0U;
        dock_charger_status.current_ready = 0U;
        dock_charger_status.voltage_code = DOCK_CHARGER_WIPER_MIN;
        dock_charger_status.current_code = DOCK_CHARGER_WIPER_MIN;        
        dock_charger_status.current_ramp_tick = HAL_GetTick();
        dock_charger_status.voltage_ramp_tick = HAL_GetTick();
        UVX_APP_PWR_FET(0U);

        if(dock_charger_status.owns_i2c_lock != 0U)
        {
            if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_PENDING)
            {
                return UVX_DOCK_CHARGER_BUSY;
            }

            uvx_dock_charger_unlock_i2c();
        }

        dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_WRITE_CURRENT;
        return UVX_DOCK_CHARGER_BUSY;
    }
    else
    {                
        switch(dock_charger_state.state_current)
        {
            case UVX_DOCK_CHARGER_STATE_WRITE_CURRENT:
                if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK)
                {
                    break;
                }

                i2c_state = uvx_tpl0401x_10_write(dock_charger_status.i2c, UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS, dock_charger_status.current_code);

                if(i2c_state == UVX_I2C_OK)
                {
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WAIT_CURRENT);
                }
                else if(i2c_state != UVX_I2C_BUSY)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                else
                {
                    uvx_dock_charger_unlock_i2c();
                }
                break;

            case UVX_DOCK_CHARGER_STATE_WAIT_CURRENT:
                if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_COMPLETE)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_CURRENT);
                }
                else if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_ERROR)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                break;

            case UVX_DOCK_CHARGER_STATE_READ_CURRENT:
                if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK)
                {
                    break;
                }

                i2c_state = uvx_tpl0401x_10_read(dock_charger_status.i2c, UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS, &dock_charger_status.current_readback_code);

                if(i2c_state == UVX_I2C_OK)
                {
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WAIT_CURRENT_READ);
                }
                else
                {
                    uvx_dock_charger_unlock_i2c();
                    if(i2c_state != UVX_I2C_BUSY)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR_I2C;
                    }
                }
                break;

            case UVX_DOCK_CHARGER_STATE_WAIT_CURRENT_READ:
                if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_COMPLETE)
                {
                    uvx_dock_charger_unlock_i2c();

                    if((dock_charger_status.current_readback_code & 0x7FU) != dock_charger_status.current_code)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR;
                    }

                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
                }
                else if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_ERROR)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                break;

            case UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE:
                if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK)
                {
                    break;
                }

                i2c_state = uvx_tpl0401x_10_write(dock_charger_status.i2c, UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS, dock_charger_status.voltage_code);

                if(i2c_state == UVX_I2C_OK)
                {
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE);
                }
                else if(i2c_state != UVX_I2C_BUSY)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                else
                {
                    uvx_dock_charger_unlock_i2c();
                }
                break;

            case UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE:
                if(uvx_i2c_get_transfer_state(
                    &dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_COMPLETE)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
                }
                else if(uvx_i2c_get_transfer_state(
                            &dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_ERROR)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                break;

            case UVX_DOCK_CHARGER_STATE_READ_VOLTAGE:
                if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK)
                {
                    break;
                }

                i2c_state = uvx_tpl0401x_10_read(dock_charger_status.i2c, UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS, &dock_charger_status.voltage_readback_code);

                if(i2c_state == UVX_I2C_OK)
                {
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE_READ);
                }
                else
                {
                    uvx_dock_charger_unlock_i2c();
                    if(i2c_state != UVX_I2C_BUSY)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR_I2C;
                    }
                }
                break;

            case UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE_READ:
                if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_COMPLETE)
                {
                    uvx_dock_charger_unlock_i2c();

                    if((dock_charger_status.voltage_readback_code & 0x7FU) != dock_charger_status.voltage_code)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR;
                    }

                    if(batt_data.adc_pack_v_stable_high)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ACTIVE);
                    }
                    else
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
                    }
                    
                    return UVX_DOCK_CHARGER_OK;
                }
                else if(uvx_i2c_get_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_TRANSFER_ERROR)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                break;

            case UVX_DOCK_CHARGER_STATE_ACTIVE:
                return uvx_dock_charger_regulate_voltage_and_current();

            case UVX_DOCK_CHARGER_STATE_ERROR:
                dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_WRITE_CURRENT;
                return UVX_DOCK_CHARGER_ERROR;

            default:
                return UVX_DOCK_CHARGER_ERROR;
        }
        
    }

    return UVX_DOCK_CHARGER_BUSY;
}
