#ifndef __UVX_DOCK_CHARGER_H
#define __UVX_DOCK_CHARGER_H

#include <stdint.h>
#include "uvx_i2c.h"

#define DOCK_CHARGER_CURRENT_MIN_MA              500U
#define DOCK_CHARGER_CURRENT_MAX_MA             15500U
#define DOCK_CHARGER_VOLTAGE_MIN_MV              5670U
#define DOCK_CHARGER_VOLTAGE_MAX_MV             40490U
#define DOCK_CHARGER_WIPER_MIN                     32U
#define DOCK_CHARGER_WIPER_MAX                     127U

typedef enum
{
    UVX_DOCK_CHARGER_OK = 0,
    UVX_DOCK_CHARGER_BUSY,
    UVX_DOCK_CHARGER_ERROR,
    UVX_DOCK_CHARGER_ERROR_PARAMETER,
    UVX_DOCK_CHARGER_ERROR_I2C,
    UVX_DOCK_CHARGER_NOT_INITIALIZED
} UVX_DOCK_CHARGER_RESULT;

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
    UVX_DOCK_CHARGER_STATE_NOT_INITIALIZED = 0,
    UVX_DOCK_CHARGER_STATE_WRITE_CURRENT,
    UVX_DOCK_CHARGER_STATE_WAIT_CURRENT,
    UVX_DOCK_CHARGER_STATE_READ_CURRENT,
    UVX_DOCK_CHARGER_STATE_WAIT_CURRENT_READ,
    UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE,
    UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE,
    UVX_DOCK_CHARGER_STATE_READ_VOLTAGE,
    UVX_DOCK_CHARGER_STATE_WAIT_VOLTAGE_READ,
    UVX_DOCK_CHARGER_STATE_ACTIVE,
    UVX_DOCK_CHARGER_STATE_ERROR
} UVX_DOCK_CHARGER_STATE;

typedef struct
{
    UVX_DOCK_CHARGER_STATE state_current;
    UVX_DOCK_CHARGER_STATE state_next;
} UVX_DOCK_CHARGER_STATE_MACHINE;

/**
 * @brief Dock charger runtime status, calibration and pending I2C values.
 */
typedef struct
{
    UVX_I2C *i2c;
    UVX_DOCK_CHARGER_CONFIG config;
    int16_t current_ma;
    uint32_t voltage_mv;
    uint32_t target_current_ma;
    uint32_t target_voltage_mv;    
    uint8_t current_code;
    uint8_t voltage_code;
    uint8_t current_readback_code;
    uint8_t voltage_readback_code;
    uint32_t voltage_ramp_tick;
    uint32_t current_ramp_tick;

    uint8_t initialized : 1;
    uint8_t owns_i2c_lock : 1;
    uint8_t voltage_ready : 1; /* ADC verified at battery voltage + test offset */
    uint8_t current_ready : 1;
} UVX_DOCK_CHARGER_STATUS;

UVX_DOCK_CHARGER_RESULT uvx_dock_charger_init(UVX_I2C *i2c, const UVX_DOCK_CHARGER_CONFIG *config);

extern UVX_DOCK_CHARGER_STATE_MACHINE dock_charger_state;
extern UVX_DOCK_CHARGER_STATUS dock_charger_status;
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_set_current(uint32_t current_ma);
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_set_voltage(uint32_t voltage_mv);
UVX_DOCK_CHARGER_RESULT uvx_dock_charger_process(void);

#endif /* __UVX_DOCK_CHARGER_H */
