#ifndef __UVX_DOCK_CHARGER_H
#define __UVX_DOCK_CHARGER_H

#include <stdint.h>
#include "uvx_i2c.h"

typedef enum
{
    UVX_DOCK_CHARGER_OK = 0,
    UVX_DOCK_CHARGER_BUSY,
    UVX_DOCK_CHARGER_ERROR,
    UVX_DOCK_CHARGER_ERROR_PARAMETER,
    UVX_DOCK_CHARGER_ERROR_I2C,
    UVX_DOCK_CHARGER_NOT_INITIALIZED
} UVX_DOCK_CHARGER_STATE;

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

typedef enum
{
    UVX_DOCK_CHARGER_SEQUENCE_NOT_INITIALIZED = 0,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_FOR_DRONE,
    UVX_DOCK_CHARGER_SEQUENCE_WRITE_CURRENT,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_CURRENT,
    UVX_DOCK_CHARGER_SEQUENCE_READ_CURRENT,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_CURRENT_READ,
    UVX_DOCK_CHARGER_SEQUENCE_WRITE_VOLTAGE,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_VOLTAGE,
    UVX_DOCK_CHARGER_SEQUENCE_READ_VOLTAGE,
    UVX_DOCK_CHARGER_SEQUENCE_WAIT_VOLTAGE_READ,
    UVX_DOCK_CHARGER_SEQUENCE_ACTIVE,
    UVX_DOCK_CHARGER_SEQUENCE_ERROR
} UVX_DOCK_CHARGER_SEQUENCE;

/**
 * @brief Internal dock charger state and pending I2C values.
 */
typedef struct
{
    UVX_I2C *i2c;
    UVX_DOCK_CHARGER_CONFIG config;
    UVX_DOCK_CHARGER_SEQUENCE sequence;
    uint8_t current_code;
    uint8_t voltage_code;
    uint8_t transfer_current_code;
    uint8_t transfer_voltage_code;
    uint8_t current_readback_code;
    uint8_t voltage_readback_code;
    uint8_t settings_changed : 1;
    uint8_t owns_i2c_lock : 1;
} UVX_DOCK_CHARGER;

UVX_DOCK_CHARGER_STATE uvx_dock_charger_init(
    UVX_I2C *i2c,
    const UVX_DOCK_CHARGER_CONFIG *config,
    uint32_t current_ma,
    uint32_t voltage_mv);

extern UVX_DOCK_CHARGER dock_charger;
UVX_DOCK_CHARGER_STATE uvx_dock_charger_set_current(uint32_t current_ma);
UVX_DOCK_CHARGER_STATE uvx_dock_charger_set_voltage(uint32_t voltage_mv);
UVX_DOCK_CHARGER_STATE uvx_dock_charger_process(void);

#endif /* __UVX_DOCK_CHARGER_H */
