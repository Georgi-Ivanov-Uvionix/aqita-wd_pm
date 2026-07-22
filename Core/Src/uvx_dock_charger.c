/**
  ******************************************************************************
  * @file    uvx_dock_charger.c
  * @brief   Review-only dock charger control using two TPL0401x-10 devices.
  *
  * TPL0401A-10 (I2C address 0x2E) regulates charge current.
  * TPL0401B-10 (I2C address 0x3E) regulates charge voltage.
  *
  * This file is intentionally not connected to the application or build yet.
  ******************************************************************************
  */

#include "main.h"
#include "uvx_i2c.h"

#define UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS    0x2EU
#define UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS    0x3EU
#define UVX_DOCK_CHARGER_WIPER_MAX              0x7FU

/* Temporary declarations until uvx_tpl0401x_10.h is added after review. */
UVX_I2C_STATE uvx_tpl0401x_10_init(UVX_I2C *i2c);
UVX_I2C_STATE uvx_tpl0401x_10_write(UVX_I2C *i2c,
                                    uint8_t device_address,
                                    uint8_t wiper_position);

typedef enum
{
    UVX_DOCK_CHARGER_OK = 0,
    UVX_DOCK_CHARGER_BUSY,
    UVX_DOCK_CHARGER_ERROR,
    UVX_DOCK_CHARGER_ERROR_PARAMETER,
    UVX_DOCK_CHARGER_ERROR_I2C,
    UVX_DOCK_CHARGER_NOT_INITIALIZED
} UVX_DOCK_CHARGER_STATE;

typedef enum
{
    UVX_DOCK_CHARGER_SEQUENCE_NOT_INITIALIZED = 0,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_FOR_DRONE,
    UVX_DOCK_CHARGER_SEQUENCE_WRITE_CURRENT,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_CURRENT,
    UVX_DOCK_CHARGER_SEQUENCE_WRITE_VOLTAGE,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_VOLTAGE,
    UVX_DOCK_CHARGER_SEQUENCE_ACTIVE,
    UVX_DOCK_CHARGER_SEQUENCE_ERROR
} UVX_DOCK_CHARGER_SEQUENCE;

/**
 * @brief Two-point charger calibration in engineering units.
 *
 * The code endpoints may be ascending or descending.  This allows the dock
 * charger abstraction to hide the feedback circuit polarity from callers.
 */
typedef struct
{
    uint32_t current_min_ma;
    uint32_t current_max_ma;
    uint8_t current_code_at_min;
    uint8_t current_code_at_max;
    uint32_t voltage_min_mv;
    uint32_t voltage_max_mv;
    uint8_t voltage_code_at_min;
    uint8_t voltage_code_at_max;
} UVX_DOCK_CHARGER_CONFIG;

typedef struct
{
    UVX_I2C *i2c;
    UVX_DOCK_CHARGER_CONFIG config;
    UVX_DOCK_CHARGER_SEQUENCE sequence;
    uint8_t current_code;
    uint8_t voltage_code;
    uint8_t transfer_current_code;
    uint8_t transfer_voltage_code;
    uint8_t settings_changed : 1;
} UVX_DOCK_CHARGER;

static UVX_DOCK_CHARGER dock_charger;

/**
 * @brief Convert an engineering-unit value to a calibrated wiper code.
 */
static UVX_DOCK_CHARGER_STATE uvx_dock_charger_value_to_code(
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
UVX_DOCK_CHARGER_STATE uvx_dock_charger_init(UVX_I2C *i2c,
                                             const UVX_DOCK_CHARGER_CONFIG *config,
                                             uint32_t current_ma,
                                             uint32_t voltage_mv)
{
    UVX_I2C_STATE i2c_state;
    uint8_t current_code;
    uint8_t voltage_code;

    if((i2c == NULL) || (config == NULL))
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(uvx_dock_charger_value_to_code(current_ma,
                                      config->current_min_ma,
                                      config->current_max_ma,
                                      config->current_code_at_min,
                                      config->current_code_at_max,
                                      &current_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(uvx_dock_charger_value_to_code(voltage_mv,
                                      config->voltage_min_mv,
                                      config->voltage_max_mv,
                                      config->voltage_code_at_min,
                                      config->voltage_code_at_max,
                                      &voltage_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    i2c_state = uvx_tpl0401x_10_init(i2c);
    if((i2c_state != UVX_I2C_OK) &&
       (i2c_state != UVX_I2C_ALREADY_INITIALIZED))
    {
        return UVX_DOCK_CHARGER_ERROR_I2C;
    }

    dock_charger.i2c = i2c;
    dock_charger.config = *config;
    dock_charger.current_code = current_code;
    dock_charger.voltage_code = voltage_code;
    dock_charger.settings_changed = 1U;
    dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WAIT_FOR_DRONE;

    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Set the desired charging current in milliamperes.
 */
UVX_DOCK_CHARGER_STATE uvx_dock_charger_set_current(uint32_t current_ma)
{
    uint8_t current_code;

    if(dock_charger.sequence == UVX_DOCK_CHARGER_SEQUENCE_NOT_INITIALIZED)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    if(uvx_dock_charger_value_to_code(current_ma,
                                      dock_charger.config.current_min_ma,
                                      dock_charger.config.current_max_ma,
                                      dock_charger.config.current_code_at_min,
                                      dock_charger.config.current_code_at_max,
                                      &current_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(dock_charger.current_code != current_code)
    {
        dock_charger.current_code = current_code;
        dock_charger.settings_changed = 1U;
    }

    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Set the desired charging voltage in millivolts.
 */
UVX_DOCK_CHARGER_STATE uvx_dock_charger_set_voltage(uint32_t voltage_mv)
{
    uint8_t voltage_code;

    if(dock_charger.sequence == UVX_DOCK_CHARGER_SEQUENCE_NOT_INITIALIZED)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    if(uvx_dock_charger_value_to_code(voltage_mv,
                                      dock_charger.config.voltage_min_mv,
                                      dock_charger.config.voltage_max_mv,
                                      dock_charger.config.voltage_code_at_min,
                                      dock_charger.config.voltage_code_at_max,
                                      &voltage_code) != UVX_DOCK_CHARGER_OK)
    {
        return UVX_DOCK_CHARGER_ERROR_PARAMETER;
    }

    if(dock_charger.voltage_code != voltage_code)
    {
        dock_charger.voltage_code = voltage_code;
        dock_charger.settings_changed = 1U;
    }

    return UVX_DOCK_CHARGER_OK;
}

/**
 * @brief Apply charger settings after the landing Hall sensor is active.
 * @retval OK when both settings are applied, BUSY while I2C is in progress,
 *         or an error status.
 *
 * Call this function repeatedly from the application loop.  The two writes
 * are sequenced because uvx_i2c uses interrupt-driven transfers on one bus.
 */
UVX_DOCK_CHARGER_STATE uvx_dock_charger_process(void)
{
    UVX_I2C_STATE i2c_state;

    if((dock_charger.sequence == UVX_DOCK_CHARGER_SEQUENCE_NOT_INITIALIZED) ||
       (dock_charger.i2c == NULL))
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    if(drone_status.hall_land_2 == false)
    {
        dock_charger.settings_changed = 1U;
        dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WAIT_FOR_DRONE;
        return UVX_DOCK_CHARGER_BUSY;
    }

    if((dock_charger.sequence == UVX_DOCK_CHARGER_SEQUENCE_WAIT_FOR_DRONE) ||
       ((dock_charger.sequence == UVX_DOCK_CHARGER_SEQUENCE_ACTIVE) &&
        (dock_charger.settings_changed != 0U)))
    {
        /* Keep one coherent pair throughout the asynchronous sequence. */
        dock_charger.transfer_current_code = dock_charger.current_code;
        dock_charger.transfer_voltage_code = dock_charger.voltage_code;
        dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WRITE_CURRENT;
    }

    switch(dock_charger.sequence)
    {
        case UVX_DOCK_CHARGER_SEQUENCE_WRITE_CURRENT:
            i2c_state = uvx_tpl0401x_10_write(
                dock_charger.i2c,
                UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS,
                dock_charger.transfer_current_code);

            if(i2c_state == UVX_I2C_OK)
            {
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WAIT_CURRENT;
            }
            else if(i2c_state != UVX_I2C_BUSY)
            {
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_ERROR;
                return UVX_DOCK_CHARGER_ERROR_I2C;
            }
            break;

        case UVX_DOCK_CHARGER_SEQUENCE_WAIT_CURRENT:
            if(dock_charger.i2c->hal_i2c.TX_Ready != 0U)
            {
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WRITE_VOLTAGE;
            }
            break;

        case UVX_DOCK_CHARGER_SEQUENCE_WRITE_VOLTAGE:
            i2c_state = uvx_tpl0401x_10_write(
                dock_charger.i2c,
                UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS,
                dock_charger.transfer_voltage_code);

            if(i2c_state == UVX_I2C_OK)
            {
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_WAIT_VOLTAGE;
            }
            else if(i2c_state != UVX_I2C_BUSY)
            {
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_ERROR;
                return UVX_DOCK_CHARGER_ERROR_I2C;
            }
            break;

        case UVX_DOCK_CHARGER_SEQUENCE_WAIT_VOLTAGE:
            if(dock_charger.i2c->hal_i2c.TX_Ready != 0U)
            {
                dock_charger.settings_changed =
                    (uint8_t)((dock_charger.current_code != dock_charger.transfer_current_code) ||
                              (dock_charger.voltage_code != dock_charger.transfer_voltage_code));
                dock_charger.sequence = UVX_DOCK_CHARGER_SEQUENCE_ACTIVE;
                return UVX_DOCK_CHARGER_OK;
            }
            break;

        case UVX_DOCK_CHARGER_SEQUENCE_ACTIVE:
            return UVX_DOCK_CHARGER_OK;

        case UVX_DOCK_CHARGER_SEQUENCE_ERROR:
            return UVX_DOCK_CHARGER_ERROR;

        default:
            return UVX_DOCK_CHARGER_ERROR;
    }

    return UVX_DOCK_CHARGER_BUSY;
}
