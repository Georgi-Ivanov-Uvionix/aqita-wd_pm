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

#define UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS        0x2EU
#define UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS        0x3EU
#define UVX_DOCK_CHARGER_WIPER_MAX                  0x7FU
#define UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV       200U
#define UVX_DOCK_CHARGER_RAMP_INTERVAL_MS           500U
#define UVX_DOCK_CHARGER_BATTERY_OFFSET_MV          500U
#define UVX_DOCK_CHARGER_CURRENT_INTERVAL_MS        2000U
#define UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA       100

/* TPL0401x-10 wiper register: TI datasheet SLIS144B, section 9.6.2. */
static UVX_I2C_STATE uvx_tpl0401x_10_init(UVX_I2C *i2c)
{
    return uvx_i2c_init(i2c);
}

static UVX_I2C_STATE uvx_tpl0401x_10_write(UVX_I2C *i2c,
                                         uint8_t device_address,
                                         uint8_t wiper_position)
{
    /* The interrupt-driven write needs storage that outlives this call. */
    i2c->hal_i2c.tx_it_buffer[0] = wiper_position;
    return uvx_i2c_send_mem(&i2c->hal_i2c, device_address, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, i2c->hal_i2c.tx_it_buffer, 1U);
}

static UVX_I2C_STATE uvx_tpl0401x_10_read(UVX_I2C *i2c,
                                        uint8_t device_address,
                                        uint8_t *wiper_position)
{
    return uvx_i2c_read_mem(&i2c->hal_i2c, device_address, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, wiper_position, 1U);
}

/* Observe completion without releasing ownership before readback is consumed. */
static UVX_I2C_STATE uvx_dock_charger_transfer_state(UVX_I2C_HAL *i2c)
{
    if(HAL_I2C_GetState(&i2c->hi2c) != HAL_I2C_STATE_READY)
    {
        return UVX_I2C_BUSY;
    }

    return (HAL_I2C_GetError(&i2c->hi2c) == HAL_I2C_ERROR_NONE) ?
           UVX_I2C_OK : UVX_I2C_ERROR;
}
UVX_DOCK_CHARGER_STATE_MACHINE dock_charger_state;
UVX_DOCK_CHARGER_STATUS dock_charger_status;

static void uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE state_next)
{
    if(state_next == UVX_DOCK_CHARGER_STATE_ERROR)
    {
        dock_charger_status.voltage_detected = 0U;
        dock_charger_status.voltage_ready = 0U;
        dock_charger_status.current_ready = 0U;
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        UVX_APP_PWR_FET(0U);
    }
    dock_charger_state.state_next = state_next;
    dock_charger_state.state_current = dock_charger_state.state_next;
}

static UVX_I2C_STATE uvx_dock_charger_lock_i2c(void)
{
    UVX_I2C_STATE state = uvx_i2c_lock(&dock_charger_status.i2c->hal_i2c, (uint32_t*)&dock_charger_status);

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
        if(uvx_i2c_unlock(&dock_charger_status.i2c->hal_i2c, (uint32_t*)&dock_charger_status) == UVX_I2C_OK)
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
 * @retval Charger module status.
 */

static uint32_t uvx_dock_charger_battery_target_mv(void)
{
    uint32_t battery_target_mv = (uint32_t)batt_data.batt_voltage + UVX_DOCK_CHARGER_BATTERY_OFFSET_MV;

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

    if((i2c == NULL) || (config == NULL) ||
       (config->current_min_ma > config->current_max_ma) ||
       (config->current_code_at_min > UVX_DOCK_CHARGER_WIPER_MAX) ||
       (config->voltage_min_mv >= config->voltage_max_mv) ||
       (config->voltage_code_at_min >= config->voltage_code_at_max) ||
       (config->voltage_code_at_max > UVX_DOCK_CHARGER_WIPER_MAX))
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
    dock_charger_status.target_current_ma = config->current_min_ma;
    dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
    dock_charger_status.current_code = dock_charger_status.config.current_code_at_min;
    dock_charger_status.voltage_code = config->voltage_code_at_min;
    dock_charger_status.owns_i2c_lock = 0U;
    dock_charger_status.voltage_detected = 0U;
    dock_charger_status.voltage_ready = 0U;
    dock_charger_status.current_ready = 0U;
    dock_charger_status.current_ramp_tick = HAL_GetTick();
    dock_charger_status.voltage_ramp_tick = HAL_GetTick();
    dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
    UVX_APP_PWR_FET(0U);
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

/* Adjust the PC wiper slowly from measured battery current (positive = charging). */
static UVX_DOCK_CHARGER_RESULT uvx_dock_charger_regulate_current(void)
{
    uint32_t now = HAL_GetTick();
    int32_t error_ma = (int32_t)dock_charger_status.target_current_ma -
                       (int32_t)dock_charger_status.current_ma;

    if(!drone_status.pwr_fet || batt_data.tc || batt_data.BPS_Passive ||
       drone_status.pwr_fc || comm_bq_1.No_response ||
       comm_bq_2.No_response || comm_bq_3.No_response)
    {
        UVX_APP_PWR_FET(0U);
        dock_charger_status.current_ready = 0U;
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        return UVX_DOCK_CHARGER_BUSY;
    }

    dock_charger_status.current_ready =
        (error_ma >= -UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA) &&
        (error_ma <= UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA);
    if((uint32_t)(now - dock_charger_status.current_ramp_tick) >=
       UVX_DOCK_CHARGER_CURRENT_INTERVAL_MS)
    {
        dock_charger_status.current_ramp_tick = now;
        if(error_ma > UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA &&
           dock_charger_status.current_code < dock_charger_status.config.current_code_at_max)
        {
            dock_charger_status.current_code++;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
            return UVX_DOCK_CHARGER_BUSY;
        }
        if(error_ma < -UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA &&
           dock_charger_status.current_code > 0U)
        {
            /* Code 4 is a starting point; allow reduction below it on overshoot. */
            dock_charger_status.current_code--;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
            return UVX_DOCK_CHARGER_BUSY;
        }
    }

    /* Hold the voltage code during current limiting; keep checking dock presence. */
    if((uint32_t)(now - dock_charger_status.voltage_ramp_tick) >= UVX_DOCK_CHARGER_RAMP_INTERVAL_MS)
    {
        dock_charger_status.voltage_ramp_tick = now;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
    }
    return dock_charger_status.current_ready ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
}

/* Track battery + 500 mV, subject to the existing supply-voltage limits. */
static UVX_DOCK_CHARGER_RESULT uvx_dock_charger_regulate_voltage(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t measured = dock_charger_status.voltage_mv;
    uint32_t target = uvx_dock_charger_battery_target_mv();

    dock_charger_status.target_voltage_mv = target;
    dock_charger_status.voltage_ready = (measured + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV >= target) && (measured <= target + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV);
    if(dock_charger_status.voltage_ready && dock_charger_status.regulation_state == UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE)
    {
        /* Keep the input disconnected until minimum current is written and verified. */
        UVX_APP_PWR_FET(0U);
        dock_charger_status.current_code = dock_charger_status.config.current_code_at_min;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
        return UVX_DOCK_CHARGER_BUSY;
    }

    if((uint32_t)(now - dock_charger_status.voltage_ramp_tick) < UVX_DOCK_CHARGER_RAMP_INTERVAL_MS)
    {
        return dock_charger_status.voltage_ready ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
    }
    dock_charger_status.voltage_ramp_tick = now;
    if(!dock_charger_status.voltage_ready && measured < target && dock_charger_status.voltage_code < dock_charger_status.config.voltage_code_at_max)
    {
        dock_charger_status.voltage_code++;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
    }
    else if(!dock_charger_status.voltage_ready && measured > target && dock_charger_status.voltage_code > dock_charger_status.config.voltage_code_at_min)
    {
        dock_charger_status.voltage_code--;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
    }
    else
    {
        /* Keep polling presence even while holding the target or a code limit. */
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
    }
    return dock_charger_status.voltage_ready ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
}

/* Read the voltage potentiometer first; successful completion detects the dock. */
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_process(void)
{
    UVX_I2C_STATE i2c_state;

    if(dock_charger_status.i2c == NULL)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    if(batt_data.batt_voltage == 0U)
    {
        dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
        dock_charger_status.voltage_detected = 0U;
        dock_charger_status.voltage_ready = 0U;
        dock_charger_status.current_ready = 0U;
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        UVX_APP_PWR_FET(0U);
        dock_charger_status.voltage_code = dock_charger_status.config.voltage_code_at_min;
        dock_charger_status.current_code = dock_charger_status.config.current_code_at_min;
        dock_charger_status.current_ramp_tick = HAL_GetTick();
        dock_charger_status.voltage_ramp_tick = HAL_GetTick();
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        UVX_APP_PWR_FET(0U);

        if(dock_charger_status.owns_i2c_lock != 0U)
        {
            if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_BUSY)
            {
                return UVX_DOCK_CHARGER_BUSY;
            }

            uvx_dock_charger_unlock_i2c();
        }

        dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_READ_VOLTAGE;
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
                if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_OK)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_CURRENT);
                }
                else if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_ERROR)
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
                i2c_state = uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c);
                if(i2c_state == UVX_I2C_OK)
                {
                    uvx_dock_charger_unlock_i2c();
                    if((dock_charger_status.current_readback_code & 0x7FU) !=
                       dock_charger_status.current_code)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR;
                    }

                    /* Each adjustment must be acknowledged and read back before the next step. */
                    dock_charger_status.current_ramp_tick = HAL_GetTick();
                    if(dock_charger_status.regulation_state == UVX_DOCK_CHARGER_REGULATION_CONTROL_CURRENT)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
                        return UVX_DOCK_CHARGER_BUSY;
                    }

                    /* Recheck feedback: voltage can move during the I2C sequence. */
                    dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
                    dock_charger_status.voltage_ready =
                        (dock_charger_status.voltage_mv + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV >=
                         dock_charger_status.target_voltage_mv) &&
                        (dock_charger_status.voltage_mv <= dock_charger_status.target_voltage_mv +
                         UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV);
                    if(dock_charger_status.voltage_detected && dock_charger_status.voltage_ready &&
                       !batt_data.tc && !batt_data.BPS_Passive && !drone_status.pwr_fc &&
                       !comm_bq_1.No_response && !comm_bq_2.No_response && !comm_bq_3.No_response)
                    {
                        UVX_APP_PWR_FET(1U);
                        dock_charger_status.current_ready = 0U;
                        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_CONTROL_CURRENT;
                    }
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ACTIVE);
                    return dock_charger_status.current_ready ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
                }
                else if(i2c_state == UVX_I2C_ERROR)
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
                if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_OK)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
                }
                else if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_ERROR)
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
                if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_OK)
                {
                    uvx_dock_charger_unlock_i2c();

                    if(!dock_charger_status.voltage_detected)
                    {
                        /* Reset the actual potentiometer after startup/reconnection.
                         * WRITE -> WAIT -> READ must verify this before regulation. */
                        UVX_APP_PWR_FET(0U);
                        dock_charger_status.current_ready = 0U;
                        dock_charger_status.voltage_ready = 0U;
                        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
                        dock_charger_status.voltage_code = dock_charger_status.config.voltage_code_at_min;
                        dock_charger_status.voltage_detected = 1U;
                        dock_charger_status.voltage_ramp_tick = HAL_GetTick();
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
                        return UVX_DOCK_CHARGER_BUSY;
                    }

                    if((dock_charger_status.voltage_readback_code & 0x7FU) != dock_charger_status.voltage_code)
                    {
                        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                        return UVX_DOCK_CHARGER_ERROR;
                    }

                    /* Let the regulation state machine decide the next ramp step. */
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ACTIVE);
                    
                    return UVX_DOCK_CHARGER_OK;
                }
                else if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_ERROR)
                {
                    uvx_dock_charger_unlock_i2c();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
                    return UVX_DOCK_CHARGER_ERROR_I2C;
                }
                break;

            case UVX_DOCK_CHARGER_STATE_ACTIVE:
                if(dock_charger_status.regulation_state == UVX_DOCK_CHARGER_REGULATION_CONTROL_CURRENT)
                {
                    return uvx_dock_charger_regulate_current();
                }
                return uvx_dock_charger_regulate_voltage();

            case UVX_DOCK_CHARGER_STATE_ERROR:
                dock_charger_status.voltage_detected = 0U;
                dock_charger_status.voltage_ready = 0U;
                if((uint32_t)(HAL_GetTick() - dock_charger_status.voltage_ramp_tick) >= UVX_DOCK_CHARGER_RAMP_INTERVAL_MS)
                {
                    dock_charger_status.voltage_ramp_tick = HAL_GetTick();
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
                }
                return UVX_DOCK_CHARGER_ERROR;

            default:
                return UVX_DOCK_CHARGER_ERROR;
        }
        
    }

    return UVX_DOCK_CHARGER_BUSY;
}
