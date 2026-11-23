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
#define UVX_DOCK_GPIO_I2C_ADDRESS                   0x21U
#define UVX_DOCK_LED_MASK                           0x78U /* P3..P6: R,G,B,W */
#define UVX_DOCK_LED_RED                            0x08U
#define UVX_DOCK_LED_GREEN                          0x10U
#define UVX_DOCK_LED_BLUE                           0x20U
#define UVX_DOCK_LED_FLASH_HALF_PERIOD_MS           500U
#define UVX_DOCK_SUPPLY_ON                          0x01U /* P0 ON/OFF */
#define UVX_DOCK_SUPPLY_DC_OK                       0x02U /* P1, active low */
#define UVX_DOCK_SUPPLY_STARTUP_MS                  1000U
#define UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS        0x3EU
#define UVX_DOCK_CHARGER_WIPER_MAX                  0x7FU
#define UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV       200U
#define UVX_DOCK_CHARGER_RAMP_INTERVAL_MS           200U
#define UVX_DOCK_CHARGER_BATTERY_OFFSET_MV          1000U
#define UVX_DOCK_CHARGER_CURRENT_INTERVAL_MS        2000U
#define UVX_DOCK_CHARGER_CV_CELL_MV                 4200
#define UVX_DOCK_CHARGER_CV_HYSTERESIS_MV            10
#define UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA       100
#define UVX_DOCK_CHARGER_CURRENT_RESPONSE_MA        50
#define UVX_DOCK_CHARGER_CURRENT_STALL_STEPS        2U
#define UVX_DOCK_CHARGER_TEMP_REDUCE_DECIC          490
#define UVX_DOCK_CHARGER_TEMP_RESUME_DECIC          485

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
typedef struct
{
    uint8_t step;
    uint8_t pending;
    uint8_t data;
    uint32_t retry_tick;
} UVX_DOCK_GPIO_TRANSACTION;

static UVX_DOCK_GPIO_TRANSACTION dock_led_io;
static UVX_DOCK_GPIO_TRANSACTION dock_supply_io;
static uint8_t dock_supply_requested = 1U;
/* Output shadow is shared with supply RMW; no LED write may overlap it. */
static uint8_t dock_gpio_output;
static uint8_t dock_led_pending, dock_led_data;
static uint16_t dock_led_hue, dock_led_red_acc, dock_led_green_acc;
static uint32_t dock_led_tick, dock_led_color_tick;
static uint8_t dock_voltage_adjusting;
static uint8_t dock_led_mode;
static uint32_t dock_led_mode_tick;
static UVX_DOCK_GPIO_TRANSACTION dock_fc_supply_io;
static uint8_t dock_fc_access;
static uint8_t dock_fc_pending;
static uint8_t dock_fc_required;
static uint8_t dock_fc_resume;
static uint8_t dock_fc_saved_supply;
#define UVX_DOCK_I2C_TIMEOUT_MS 50U
static void uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE state_next)
{
    if(state_next == UVX_DOCK_CHARGER_STATE_ERROR)
    {
        /* Retain the failure before retries replace HAL's state/error flags. */
        dock_charger_status.error_count++;
        dock_charger_status.last_error_state = dock_charger_state.state_current;
        if(dock_charger_status.i2c != NULL)
        {
            dock_charger_status.last_error_hal_state = HAL_I2C_GetState(&dock_charger_status.i2c->hal_i2c.hi2c);
            dock_charger_status.last_error_hal_code = HAL_I2C_GetError(&dock_charger_status.i2c->hal_i2c.hi2c);
            dock_charger_status.last_error_isr = dock_charger_status.i2c->hal_i2c.hi2c.Instance->ISR;
        }
        dock_voltage_adjusting = 0U;
        dock_charger_status.supply_enabled = 0U;
        dock_charger_status.supply_ready = 0U;
        dock_charger_status.led_ready = 0U;
        dock_led_io.step = 0U;
        dock_led_io.pending = 0U;
        dock_supply_io.step = 0U;
        dock_supply_io.pending = 0U;
        dock_charger_status.voltage_detected = 0U;
        dock_charger_status.voltage_ready = 0U;
        dock_charger_status.current_ready = 0U;
        dock_charger_status.cv_active = 0U;
        dock_charger_status.balance_required = 0U;
        dock_charger_status.current_stall_count = 0U;
        dock_charger_status.current_step_pending = 0U;
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        UVX_APP_PWR_FET(0U);
    }
    dock_charger_state.state_next = state_next;
    dock_charger_state.state_current = dock_charger_state.state_next;
}

static UVX_I2C_STATE uvx_dock_charger_lock_i2c(void)
{
    /* Complete existing dock transfers, but give the next bus slot to FC
     * preparation. Every dock start (including LED setup) comes through here. */
    if(!dock_fc_access && drone_status.prepare_fc &&
       comm_m2jmb_state.state_current == M2JMB_MODE_SEND_FC_ON_ACK)
    {
        return UVX_I2C_BUSY;
    }
    UVX_I2C_STATE state = uvx_i2c_lock(&dock_charger_status.i2c->hal_i2c, (uint32_t*)&dock_charger_status);

    if(state == UVX_I2C_OK)
    {
        dock_charger_status.owns_i2c_lock = 1U;
        dock_charger_status.i2c_lock_tick = HAL_GetTick();
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

/* Dock transfers use interrupts, not DMA. On a deadline, stop the peripheral
 * before releasing its buffer/lock. A missing STOP cannot be repaired by an
 * interrupt-based abort alone (the bundled HAL also rejects abort in MEM mode).
 * No GPIO bus-clear pulses: a slave holding SDA low remains a hardware fault. */
static UVX_I2C_STATE uvx_dock_charger_recover_i2c(void)
{
    UVX_I2C_HAL *i2c = &dock_charger_status.i2c->hal_i2c;
    uint32_t primask;
    uint32_t filters;
    uint32_t timeout;
    HAL_StatusTypeDef result;

    if(i2c->hi2c.Instance != I2C1 ||
       i2c->p_lock_owner != (uint32_t*)&dock_charger_status)
        return UVX_I2C_ERROR;

    primask = __get_PRIMASK();
    __disable_irq();
    dock_charger_status.i2c_timeout_isr = i2c->hi2c.Instance->ISR;
    dock_charger_status.i2c_timeout_hal_state = i2c->hi2c.State;
    filters = i2c->hi2c.Instance->CR1 & (I2C_CR1_ANFOFF | I2C_CR1_DNF);
    timeout = i2c->hi2c.Instance->TIMEOUTR;
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    i2c->hi2c.State = HAL_I2C_STATE_RESET;
    i2c->hi2c.XferISR = NULL;
    i2c->hi2c.pBuffPtr = NULL;
    i2c->hi2c.XferCount = 0U;
    i2c->hi2c.XferSize = 0U;
    result = HAL_I2C_Init(&i2c->hi2c);
    if(result == HAL_OK)
    {
        __HAL_I2C_DISABLE(&i2c->hi2c);
        MODIFY_REG(i2c->hi2c.Instance->CR1, I2C_CR1_ANFOFF | I2C_CR1_DNF, filters);
        i2c->hi2c.Instance->TIMEOUTR = timeout;
        __HAL_I2C_ENABLE(&i2c->hi2c);
        i2c->STOP_Detected = 0U;
        i2c->i2c_err_timeout_lock = 0U;
        uvx_dock_charger_unlock_i2c();
    }
    __set_PRIMASK(primask);
    return result == HAL_OK ? UVX_I2C_OK : UVX_I2C_ERROR;
}

/* Shared non-blocking read/modify/write engine. Each caller owns its state.
 * The application serializes supply and LED transactions to preserve register bits. */
static UVX_I2C_STATE uvx_dock_gpio_update(UVX_DOCK_GPIO_TRANSACTION *io,
                                        uint8_t mask, uint8_t value, uint8_t supply)
{
    static const uint8_t registers[7] = {1U, 1U, 3U, 3U, 1U, 3U, 0U};
    UVX_I2C_STATE state;
    uint8_t input_mask = supply ? UVX_DOCK_SUPPLY_DC_OK : 0U;
    UVX_I2C_HAL *i2c = &dock_charger_status.i2c->hal_i2c;

    if(!io->pending && io->step == 0U &&
       (uint32_t)(HAL_GetTick() - io->retry_tick) < 1000U)
    {
        return UVX_I2C_OK;
    }
    if(io->pending)
    {
        state = uvx_dock_charger_transfer_state(i2c);
        if(state == UVX_I2C_BUSY)
        {
            return UVX_I2C_BUSY;
        }
        uvx_dock_charger_unlock_i2c();
        io->pending = 0U;
        if(state != UVX_I2C_OK)
        {
            goto failed;
        }
        if(io->step == 0U || io->step == 1U || io->step == 4U)
        {
            dock_gpio_output = io->data;
        }
        if(io->step == 0U)
        {
            io->data = (io->data & (uint8_t)~mask) | value;
        }
        else if(io->step == 2U)
        {
            io->data = (io->data & (uint8_t)~mask) | input_mask;
        }
        else if((io->step == 4U && (io->data & mask) != value) ||
                (io->step == 5U && (io->data & (mask | input_mask)) != input_mask))
        {
            goto failed;
        }
        if(supply && io->step == 5U)
        {
            if(io->data & UVX_DOCK_LED_MASK)
            {
                dock_charger_status.led_ready = 0U;
            }
            if(value && !dock_charger_status.supply_enabled)
            {
                dock_charger_status.supply_enable_tick = HAL_GetTick();
            }
            dock_charger_status.supply_enabled = value != 0U;
        }
        if(supply && io->step == 6U)
        {
            dock_charger_status.supply_ready = value &&
                ((io->data & UVX_DOCK_SUPPLY_DC_OK) == 0U);
        }
        io->step++;
        if(io->step == (supply ? 7U : 6U))
        {
            if(!supply)
            {
                dock_charger_status.led_ready = 1U;
            }
            io->step = 0U;
            io->retry_tick = HAL_GetTick();
            return UVX_I2C_OK;
        }
        return UVX_I2C_BUSY;
    }
    if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK)
    {
        return UVX_I2C_BUSY;
    }
    if(io->step == 1U || io->step == 3U)
    {
        state = uvx_i2c_send_mem(i2c, UVX_DOCK_GPIO_I2C_ADDRESS, registers[io->step],
                                I2C_MEMADD_SIZE_8BIT, &io->data, 1U);
    }
    else
    {
        state = uvx_i2c_read_mem(i2c, UVX_DOCK_GPIO_I2C_ADDRESS, registers[io->step],
                                I2C_MEMADD_SIZE_8BIT, &io->data, 1U);
    }
    if(state == UVX_I2C_OK)
    {
        io->pending = 1U;
        return UVX_I2C_BUSY;
    }
    uvx_dock_charger_unlock_i2c();
    if(state == UVX_I2C_BUSY)
    {
        return UVX_I2C_BUSY;
    }
failed:
    if(supply)
    {
        dock_charger_status.supply_enabled = 0U;
        dock_charger_status.supply_ready = 0U;
    }
    else
    {
        dock_charger_status.led_ready = 0U;
    }
    io->step = 0U;
    io->pending = 0U;
    io->retry_tick = HAL_GetTick();
    return UVX_I2C_ERROR;
}

/* Request ON/OFF; the application loop performs and verifies the I2C writes. */
void uvx_dock_supply_set_enabled(uint8_t enabled)
{
    dock_supply_requested = enabled != 0U;
    if(!enabled)
    {
        UVX_APP_PWR_FET(0U);
        dock_charger_status.supply_ready = 0U;
    }
}

void uvx_dock_charger_begin_fc(void)
{
    if(dock_charger_status.fc_shutdown_state != UVX_DOCK_FC_IDLE)
        return; /* A repeated request must not overwrite an in-flight buffer. */
    dock_fc_saved_supply = dock_supply_requested;
    dock_fc_required = dock_charger_status.voltage_detected || dock_charger_status.supply_enabled;
    dock_fc_resume = 0U;
    dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_DRAIN;
    if(!dock_charger_status.initialized && !dock_fc_required)
        dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_DONE;
    UVX_APP_PWR_FET(0U);
}

uint8_t uvx_dock_charger_fc_ready(void)
{
    return dock_charger_status.fc_shutdown_state == UVX_DOCK_FC_DONE;
}

void uvx_dock_charger_end_fc(void)
{
    if(dock_charger_status.fc_shutdown_state != UVX_DOCK_FC_IDLE)
        dock_fc_resume = 1U;
}

static UVX_DOCK_CHARGER_RESULT uvx_dock_fc_fail(void)
{
    dock_fc_pending = 0U;
    dock_fc_supply_io.pending = 0U;
    dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_ERROR;
    UVX_APP_PWR_FET(0U);
    return UVX_DOCK_CHARGER_ERROR_I2C;
}

static UVX_DOCK_CHARGER_RESULT uvx_dock_fc_process(void)
{
    UVX_I2C_STATE result;
    UVX_DOCK_FC_STATE stage = dock_charger_status.fc_shutdown_state;
    uint8_t current;
    uint8_t address;
    uint8_t code;
    uint8_t *readback;

    /* Drain an old dock transaction before replacing its state or buffers. */
    if(stage == UVX_DOCK_FC_DRAIN || dock_fc_resume)
    {
        if(dock_charger_status.owns_i2c_lock)
        {
            if(uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_BUSY)
                return UVX_DOCK_CHARGER_BUSY;
            uvx_dock_charger_unlock_i2c();
        }
        dock_led_pending = 0U;
        dock_led_io = (UVX_DOCK_GPIO_TRANSACTION){0};
        dock_supply_io = (UVX_DOCK_GPIO_TRANSACTION){0};
        dock_fc_supply_io = (UVX_DOCK_GPIO_TRANSACTION){0};
        dock_fc_pending = 0U;
        dock_charger_status.led_ready = 0U;
        if(dock_fc_resume)
        {
            dock_fc_resume = 0U;
            dock_supply_requested = dock_fc_saved_supply;
            dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_IDLE;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
            return UVX_DOCK_CHARGER_BUSY;
        }
        dock_charger_status.fc_shutdown_state = dock_fc_required ?
            UVX_DOCK_FC_CURRENT_WRITE : UVX_DOCK_FC_DONE;
        return UVX_DOCK_CHARGER_BUSY;
    }
    if(stage == UVX_DOCK_FC_DONE)
    {
        /* Indication is best effort: supply OFF was already verified, so an
         * LED failure must not delay the FC acknowledgement. Preserve P0. */
        if(dock_fc_required && !dock_charger_status.led_ready)
        {
            if(uvx_dock_gpio_update(&dock_led_io, UVX_DOCK_LED_MASK, UVX_DOCK_LED_RED, 0U) == UVX_I2C_OK &&
               dock_charger_status.led_ready)
            {
                dock_charger_status.led_red = 255U;
                dock_charger_status.led_green = 0U;
            }
        }
        return UVX_DOCK_CHARGER_OK;
    }
    if(stage == UVX_DOCK_FC_ERROR) return UVX_DOCK_CHARGER_ERROR_I2C;
    if(stage == UVX_DOCK_FC_SUPPLY_OFF)
    {
        result = uvx_dock_gpio_update(&dock_fc_supply_io, UVX_DOCK_SUPPLY_ON, 0U, 1U);
        if(result == UVX_I2C_BUSY) return UVX_DOCK_CHARGER_BUSY;
        if(result != UVX_I2C_OK) return uvx_dock_fc_fail();
        /* The GPIO engine read back both OFF and output direction. */
        dock_supply_requested = 0U;
        dock_led_io.retry_tick = HAL_GetTick() - 1000U;
        dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_DONE;
        return UVX_DOCK_CHARGER_OK;
    }
    current = stage == UVX_DOCK_FC_CURRENT_WRITE || stage == UVX_DOCK_FC_CURRENT_READ;
    address = current ? UVX_DOCK_CHARGER_CURRENT_I2C_ADDRESS : UVX_DOCK_CHARGER_VOLTAGE_I2C_ADDRESS;
    code = current ? dock_charger_status.config.current_code_at_min : dock_charger_status.config.voltage_code_at_min;
    readback = current ? &dock_charger_status.current_readback_code : &dock_charger_status.voltage_readback_code;
    if(dock_fc_pending)
    {
        result = uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c);
        if(result == UVX_I2C_BUSY) return UVX_DOCK_CHARGER_BUSY;
        uvx_dock_charger_unlock_i2c();
        dock_fc_pending = 0U;
        if(result != UVX_I2C_OK) return uvx_dock_fc_fail();
        if(stage == UVX_DOCK_FC_CURRENT_WRITE)
            dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_CURRENT_READ;
        else if(stage == UVX_DOCK_FC_VOLTAGE_WRITE)
            dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_VOLTAGE_READ;
        else
        {
            if((*readback & 0x7FU) != code) return uvx_dock_fc_fail();
            if(current)
            {
                dock_charger_status.current_code = code;
                dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_VOLTAGE_WRITE;
            }
            else
            {
                dock_charger_status.voltage_code = code;
                dock_fc_supply_io.retry_tick = HAL_GetTick() - 1000U;
                dock_charger_status.fc_shutdown_state = UVX_DOCK_FC_SUPPLY_OFF;
            }
        }
        return UVX_DOCK_CHARGER_BUSY;
    }
    if(uvx_dock_charger_lock_i2c() != UVX_I2C_OK) return UVX_DOCK_CHARGER_BUSY;
    if(stage == UVX_DOCK_FC_CURRENT_WRITE || stage == UVX_DOCK_FC_VOLTAGE_WRITE)
        result = uvx_tpl0401x_10_write(dock_charger_status.i2c, address, code);
    else
        result = uvx_tpl0401x_10_read(dock_charger_status.i2c, address, readback);
    if(result == UVX_I2C_OK)
        dock_fc_pending = 1U;
    else
    {
        uvx_dock_charger_unlock_i2c();
        if(result != UVX_I2C_BUSY) return uvx_dock_fc_fail();
    }
    return UVX_DOCK_CHARGER_BUSY;
}

static UVX_I2C_STATE uvx_dock_supply_process(void)
{
    static uint8_t applied_request = 1U;
    /* Finish an in-flight command before changing its data/verification value. */
    if(!dock_supply_io.pending && dock_supply_io.step == 0U &&
       applied_request != dock_supply_requested)
    {
        applied_request = dock_supply_requested;
        dock_supply_io.retry_tick = HAL_GetTick() - 1000U;
    }
    return uvx_dock_gpio_update(&dock_supply_io, UVX_DOCK_SUPPLY_ON,
                               applied_request ? UVX_DOCK_SUPPLY_ON : 0U, 1U);
}

static UVX_I2C_STATE uvx_dock_led_setup_process(void)
{
    if(dock_charger_status.led_ready)
    {
        return UVX_I2C_OK;
    }
    return uvx_dock_gpio_update(&dock_led_io, UVX_DOCK_LED_MASK, UVX_DOCK_LED_RED, 0U);
}

/* Best-effort 1 ms pulse-density slots: U1 has no hardware PWM.
 * Bus contention stretches slots; charging transactions take priority. */
static void uvx_dock_led_color_process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t cell_mv = 0U;
    uint16_t target = 0U;
    uint8_t leds = 0U;
    uint8_t mode = 0U; /* Red, flashing red, flashing blue, charging blend, purple */
    UVX_I2C_STATE state;

    if(!dock_charger_status.led_ready || dock_supply_io.pending ||
       dock_supply_io.step || dock_led_io.pending || dock_led_io.step)
    {
        return;
    }
    if(batt_data.cells_count != 0U)
    {
        cell_mv = (uint32_t)batt_data.batt_voltage / batt_data.cells_count;
    }
    dock_charger_status.led_cell_mv = cell_mv;
    if(dock_charger_status.voltage_detected && dock_supply_requested)
    {
        mode = dock_voltage_adjusting ? 2U : 1U;
        if(dock_voltage_adjusting && dock_charger_status.voltage_ready)
        {
            mode = 4U; /* Voltage matched; waiting for charging to start. */
        }
    }
    if(drone_status.pwr_fet && dock_supply_requested &&
       dock_charger_status.regulation_state == UVX_DOCK_CHARGER_REGULATION_CONTROL_CURRENT)
    {
        mode = (dock_charger_status.current_ma > UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA) ? 3U : 4U;
        if(cell_mv >= 4200U) target = 510U;
        else if(cell_mv > 2500U) target = (uint16_t)((cell_mv - 2500U) * 510U / 1700U);
    }
    if(mode != dock_led_mode)
    {
        dock_led_mode = mode;
        dock_led_mode_tick = now; /* Each flashing phase starts lit. */
    }
    if(mode != 3U)
    {
        dock_led_hue = dock_led_red_acc = dock_led_green_acc = 0U;
        dock_led_color_tick = now;
        if(mode == 4U)
        {
            leds = UVX_DOCK_LED_RED | UVX_DOCK_LED_BLUE;
        }
        else if(mode == 0U || ((uint32_t)(now - dock_led_mode_tick) /
                          UVX_DOCK_LED_FLASH_HALF_PERIOD_MS) % 2U == 0U)
        {
            leds = (mode == 2U) ? UVX_DOCK_LED_BLUE : UVX_DOCK_LED_RED;
        }
        dock_charger_status.led_red = (leds & UVX_DOCK_LED_RED) ? 255U : 0U;
        dock_charger_status.led_green = 0U;
        goto write_output;
    }
    if((uint32_t)(now - dock_led_color_tick) >= 100U)
    {
        dock_led_color_tick = now;
        if(target > dock_led_hue)
            dock_led_hue += (target - dock_led_hue > 8U) ? 8U : target - dock_led_hue;
        else
            dock_led_hue -= (dock_led_hue - target > 8U) ? 8U : dock_led_hue - target;
    }
    dock_charger_status.led_red = (dock_led_hue <= 255U) ? 255U : 510U - dock_led_hue;
    dock_charger_status.led_green = (dock_led_hue >= 255U) ? 255U : dock_led_hue;
    if((uint32_t)(now - dock_led_tick) < 1U) return;
    dock_led_tick = now;
    dock_led_red_acc += dock_charger_status.led_red;
    dock_led_green_acc += dock_charger_status.led_green;
    if(dock_led_red_acc >= 255U) { dock_led_red_acc -= 255U; leds |= UVX_DOCK_LED_RED; }
    if(dock_led_green_acc >= 255U) { dock_led_green_acc -= 255U; leds |= UVX_DOCK_LED_GREEN; }
write_output:
    dock_led_data = (dock_gpio_output & (uint8_t)~UVX_DOCK_LED_MASK) | leds;
    if(dock_led_data == dock_gpio_output || uvx_dock_charger_lock_i2c() != UVX_I2C_OK) return;
    state = uvx_i2c_send_mem(&dock_charger_status.i2c->hal_i2c, UVX_DOCK_GPIO_I2C_ADDRESS,
                            1U, I2C_MEMADD_SIZE_8BIT, &dock_led_data, 1U);
    if(state == UVX_I2C_OK) dock_led_pending = 1U;
    else
    {
        uvx_dock_charger_unlock_i2c();
        if(state != UVX_I2C_BUSY) dock_charger_status.led_ready = 0U;
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
    dock_charger_status.cv_active = 0U;
    dock_charger_status.balance_required = 0U;
    dock_charger_status.target_current_ma = config->current_min_ma;
    dock_charger_status.target_voltage_mv = uvx_dock_charger_battery_target_mv();
    dock_charger_status.current_code = dock_charger_status.config.current_code_at_min;
    dock_charger_status.voltage_code = config->voltage_code_at_min;
    dock_charger_status.owns_i2c_lock = 0U;
    dock_charger_status.i2c_lock_tick = 0U;
    dock_charger_status.i2c_timeout_count = 0U;
    dock_charger_status.voltage_detected = 0U;
    dock_charger_status.voltage_ready = 0U;
    dock_charger_status.current_ready = 0U;
    dock_charger_status.current_ramp_tick = HAL_GetTick();
    dock_charger_status.current_stall_count = 0U;
    dock_charger_status.current_step_pending = 0U;
    dock_charger_status.voltage_ramp_tick = HAL_GetTick();
    dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
    UVX_APP_PWR_FET(0U);
    dock_charger_status.initialized = true;
    dock_charger_status.supply_enabled = 0U;
    dock_charger_status.supply_ready = 0U;
    dock_charger_status.supply_enable_tick = 0U;
    dock_charger_status.led_ready = 0U;
    dock_led_pending = 0U;
    dock_voltage_adjusting = 0U;
    dock_led_mode = 0U;
    dock_led_mode_tick = HAL_GetTick();
    dock_led_hue = dock_led_red_acc = dock_led_green_acc = 0U;
    dock_led_tick = dock_led_color_tick = HAL_GetTick();
    dock_charger_status.led_cell_mv = 0U;
    dock_charger_status.led_red = 255U;
    dock_charger_status.led_green = 0U;
    dock_led_io.step = 0U;
    dock_led_io.pending = 0U;
    dock_supply_io.step = 0U;
    dock_supply_io.pending = 0U;
    dock_led_io.retry_tick = HAL_GetTick() - 1000U;
    dock_supply_io.retry_tick = HAL_GetTick() - 1000U;
    uvx_dock_supply_set_enabled(1U);
    dock_charger_status.temperature_limited = 0U;

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
    uint32_t pack;
    uint32_t now = HAL_GetTick();
    uint32_t voltage_limit_mv = dock_charger_status.config.voltage_max_mv;
    uint32_t voltage_step_mv =
        (dock_charger_status.config.voltage_max_mv - dock_charger_status.config.voltage_min_mv +
         dock_charger_status.config.voltage_code_at_max - dock_charger_status.config.voltage_code_at_min - 1U) /
        (dock_charger_status.config.voltage_code_at_max - dock_charger_status.config.voltage_code_at_min);
    int32_t error_ma = (int32_t)dock_charger_status.target_current_ma -
                       (int32_t)dock_charger_status.current_ma;

    if(batt_data.pwr_max_voltage != 0U && batt_data.pwr_max_voltage < voltage_limit_mv)
    {
        voltage_limit_mv = batt_data.pwr_max_voltage;
    }
    if(dock_charger_status.target_voltage_mv > voltage_limit_mv)
    {
        dock_charger_status.target_voltage_mv = voltage_limit_mv;
    }

    if(!drone_status.pwr_fet || batt_data.tc || batt_data.BPS_Passive ||
       batt_data.voltage_max_cell <= 0 || batt_data.cells_count != BATT_EXPECTED_CELLS ||
       drone_status.pwr_fc || comm_bq_1.No_response ||
       comm_bq_2.No_response || comm_bq_3.No_response)
    {
        UVX_APP_PWR_FET(0U);
        dock_charger_status.current_ready = 0U;
        dock_charger_status.cv_active = 0U;
        dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
        return UVX_DOCK_CHARGER_BUSY;
    }

    /* Refresh flags while charging; bypass keeps its separately latched targets.
     * Compare pack averages for balancing, but the highest individual cell
     * controls taper so an average cannot hide a cell above the CV target. */
    uvx_batt_find_highest_average_pack_voltage();
    dock_charger_status.balance_required = 0U;
    for(pack = 0U; pack < BQ_DEVICES; pack++)
    {
        if(bq_data[pack].bps_need_discharge)
        {
            dock_charger_status.balance_required = 1U;
        }
    }

    if(dock_charger_status.balance_required)
    {
        dock_charger_status.cv_active = 0U;
    }
    else if(batt_data.voltage_max_cell >= UVX_DOCK_CHARGER_CV_CELL_MV - UVX_DOCK_CHARGER_CV_HYSTERESIS_MV)
    {
        dock_charger_status.cv_active = 1U;
    }

    /* Temperature fields contain tenths of Celsius, stored in uint16_t.
     * Interpret negative converted temperatures as signed values. */
    if((int16_t)batt_data.temperature_bq_1 > UVX_DOCK_CHARGER_TEMP_REDUCE_DECIC ||
       (int16_t)batt_data.temperature_bq_2 > UVX_DOCK_CHARGER_TEMP_REDUCE_DECIC ||
       (int16_t)batt_data.temperature_bq_3 > UVX_DOCK_CHARGER_TEMP_REDUCE_DECIC)
    {
        dock_charger_status.temperature_limited = 1U;
    }
    else if((int16_t)batt_data.temperature_bq_1 <= UVX_DOCK_CHARGER_TEMP_RESUME_DECIC &&
            (int16_t)batt_data.temperature_bq_2 <= UVX_DOCK_CHARGER_TEMP_RESUME_DECIC &&
            (int16_t)batt_data.temperature_bq_3 <= UVX_DOCK_CHARGER_TEMP_RESUME_DECIC)
    {
        dock_charger_status.temperature_limited = 0U;
    }

    dock_charger_status.current_ready =
        !dock_charger_status.temperature_limited &&
        !dock_charger_status.cv_active &&
        (error_ma >= -UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA) &&
        (error_ma <= UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA);
    if((uint32_t)(now - dock_charger_status.current_ramp_tick) >=
       UVX_DOCK_CHARGER_CURRENT_INTERVAL_MS)
    {
        dock_charger_status.current_ramp_tick = now;
        /* Evaluate the response only after the previous write has settled. */
        if(dock_charger_status.current_step_pending)
        {
            if((int32_t)dock_charger_status.current_ma - dock_charger_status.current_before_step_ma <
               UVX_DOCK_CHARGER_CURRENT_RESPONSE_MA)
            {
                if(dock_charger_status.current_stall_count < UVX_DOCK_CHARGER_CURRENT_STALL_STEPS)
                {
                    dock_charger_status.current_stall_count++;
                }
            }
            else
            {
                dock_charger_status.current_stall_count = 0U;
            }
            dock_charger_status.current_step_pending = 0U;
        }
        /* Reduce voltage first if feedback exceeds the allowed supply voltage. */
        if(dock_charger_status.voltage_mv > voltage_limit_mv &&
           dock_charger_status.voltage_code > dock_charger_status.config.voltage_code_at_min)
        {
            dock_charger_status.voltage_code--;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
            return UVX_DOCK_CHARGER_BUSY;
        }
        if(dock_charger_status.temperature_limited)
        {
            /* Thermal reduction takes priority over chasing the current target. */
            dock_charger_status.current_stall_count = 0U;
            dock_charger_status.current_step_pending = 0U;
            if(dock_charger_status.current_code > 0U)
            {
                dock_charger_status.current_code--;
                uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
                return UVX_DOCK_CHARGER_BUSY;
            }
            /* At code zero, keep polling the dock without increasing either wiper. */
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
            return UVX_DOCK_CHARGER_BUSY;
        }
        if(dock_charger_status.cv_active && batt_data.voltage_max_cell >= UVX_DOCK_CHARGER_CV_CELL_MV - UVX_DOCK_CHARGER_CV_HYSTERESIS_MV)
        {
            /* Do not chase the full current target while holding 4.2 V/cell.
             * Above target, reduce one wiper step per settled 2-second interval.
             * At minimum PC, lower PV if current limiting cannot reduce voltage. */
            dock_charger_status.current_stall_count = 0U;
            dock_charger_status.current_step_pending = 0U;
            if(batt_data.voltage_max_cell > UVX_DOCK_CHARGER_CV_CELL_MV ||
               error_ma < -UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA)
            {
                if(dock_charger_status.current_code > 0U)
                {
                    dock_charger_status.current_code--;
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
                    return UVX_DOCK_CHARGER_BUSY;
                }
                if(dock_charger_status.voltage_code > dock_charger_status.config.voltage_code_at_min)
                {
                    dock_charger_status.voltage_code--;
                    dock_charger_status.target_voltage_mv = dock_charger_status.voltage_mv;
                    uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
                    return UVX_DOCK_CHARGER_BUSY;
                }
            }
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
            return UVX_DOCK_CHARGER_BUSY;
        }
        if(error_ma > UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA &&
           (dock_charger_status.voltage_mv + UVX_DOCK_CHARGER_VOLTAGE_TOLERANCE_MV >=
                dock_charger_status.target_voltage_mv ||
            dock_charger_status.current_stall_count >= UVX_DOCK_CHARGER_CURRENT_STALL_STEPS ||
            dock_charger_status.current_code >= dock_charger_status.config.current_code_at_max))
        {
            /* At the voltage target, increasing the current limit cannot help.
             * Reserve a full calibrated step below the voltage ceiling. */
            if(dock_charger_status.voltage_code < dock_charger_status.config.voltage_code_at_max &&
               dock_charger_status.voltage_mv + voltage_step_mv <= voltage_limit_mv)
            {
                /* Anchor to feedback: nominal wiper steps must not accumulate target error. */
                dock_charger_status.target_voltage_mv = dock_charger_status.voltage_mv + voltage_step_mv;
                dock_charger_status.current_stall_count = 0U;
                dock_charger_status.voltage_code++;
                uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_VOLTAGE);
                return UVX_DOCK_CHARGER_BUSY;
            }
            /* At the voltage ceiling, accept reduced current instead of winding up PC. */
        }
        else
        if(error_ma > UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA &&
           dock_charger_status.current_code < dock_charger_status.config.current_code_at_max)
        {
            dock_charger_status.current_before_step_ma = dock_charger_status.current_ma;
            dock_charger_status.current_step_pending = 1U;
            dock_charger_status.current_code++;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
            return UVX_DOCK_CHARGER_BUSY;
        }
        if(error_ma < -UVX_DOCK_CHARGER_CURRENT_TOLERANCE_MA &&
           dock_charger_status.current_code > 0U)
        {
            /* Code 4 is a starting point; allow reduction below it on overshoot. */
            dock_charger_status.current_stall_count = 0U;
            dock_charger_status.current_step_pending = 0U;
            dock_charger_status.current_code--;
            uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_WRITE_CURRENT);
            return UVX_DOCK_CHARGER_BUSY;
        }
    }

    /* Keep checking dock presence between current/voltage adjustments. */
    if((uint32_t)(now - dock_charger_status.voltage_ramp_tick) >= UVX_DOCK_CHARGER_RAMP_INTERVAL_MS)
    {
        dock_charger_status.voltage_ramp_tick = now;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_READ_VOLTAGE);
    }
    return dock_charger_status.current_ready ? UVX_DOCK_CHARGER_OK : UVX_DOCK_CHARGER_BUSY;
}

/* Track battery + 1000 mV, subject to the existing supply-voltage limits. */
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
        dock_charger_status.current_stall_count = 0U;
        dock_charger_status.current_step_pending = 0U;
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
    UVX_DOCK_CHARGER_RESULT result;

    if(dock_charger_status.i2c == NULL)
    {
        return UVX_DOCK_CHARGER_NOT_INITIALIZED;
    }

    /* Check before LED/setup dispatch so no state can hide a stalled owner. */
    if(dock_charger_status.owns_i2c_lock &&
       (uint32_t)(HAL_GetTick() - dock_charger_status.i2c_lock_tick) >= UVX_DOCK_I2C_TIMEOUT_MS &&
       uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c) == UVX_I2C_BUSY)
    {
        dock_charger_status.i2c_timeout_count++;
        if(uvx_dock_charger_recover_i2c() != UVX_I2C_OK)
        {
            dock_charger_status.i2c_lock_tick = HAL_GetTick();
            return UVX_DOCK_CHARGER_ERROR_I2C;
        }
        dock_led_pending = 0U;
        uvx_dock_charger_change_state(UVX_DOCK_CHARGER_STATE_ERROR);
        if(dock_charger_status.fc_shutdown_state != UVX_DOCK_FC_IDLE &&
           dock_charger_status.fc_shutdown_state != UVX_DOCK_FC_DONE)
            return uvx_dock_fc_fail();
        return UVX_DOCK_CHARGER_ERROR_I2C;
    }

    if(dock_charger_status.fc_shutdown_state != UVX_DOCK_FC_IDLE)
    {
        dock_fc_access = 1U;
        result = uvx_dock_fc_process();
        dock_fc_access = 0U;
        return result;
    }

    /* Drain the persistent LED buffer before any other dock transaction. */
    if(dock_led_pending)
    {
        i2c_state = uvx_dock_charger_transfer_state(&dock_charger_status.i2c->hal_i2c);
        if(i2c_state == UVX_I2C_BUSY) return UVX_DOCK_CHARGER_BUSY;
        uvx_dock_charger_unlock_i2c();
        dock_led_pending = 0U;
        if(i2c_state == UVX_I2C_OK) dock_gpio_output = dock_led_data;
        else dock_charger_status.led_ready = 0U;
    }

    /* Initialize red even before battery data or voltage-pot detection is ready.
     * Never interleave the setup RMW with another dock transaction. */
    if(!dock_charger_status.owns_i2c_lock &&
       !dock_supply_io.pending && dock_supply_io.step == 0U)
    {
        if(uvx_dock_led_setup_process() == UVX_I2C_BUSY)
            return UVX_DOCK_CHARGER_BUSY;
    }
    else if(dock_led_io.pending || dock_led_io.step != 0U)
    {
        if(uvx_dock_led_setup_process() == UVX_I2C_BUSY)
            return UVX_DOCK_CHARGER_BUSY;
    }

    if(batt_data.batt_voltage == 0U)
    {
        dock_voltage_adjusting = 0U;
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

        dock_led_io.step = 0U;
        dock_led_io.pending = 0U;
        dock_supply_io.step = 0U;
        dock_supply_io.pending = 0U;
        dock_charger_status.supply_enabled = 0U;
        dock_charger_status.supply_ready = 0U;
        dock_charger_state.state_current = UVX_DOCK_CHARGER_STATE_READ_VOLTAGE;
        uvx_dock_led_color_process();
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
                    if(dock_supply_requested && dock_charger_status.voltage_detected && dock_charger_status.voltage_ready &&
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
                dock_charger_status.last_voltage_read_result = i2c_state;

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
                /* Complete an LED sequence before starting a supply register update. */
                if(dock_led_io.pending || dock_led_io.step != 0U)
                {
                    if(uvx_dock_led_setup_process() == UVX_I2C_BUSY)
                    {
                        return UVX_DOCK_CHARGER_BUSY;
                    }
                }
                i2c_state = uvx_dock_supply_process();
                if(i2c_state == UVX_I2C_BUSY)
                {
                    return UVX_DOCK_CHARGER_BUSY;
                }
                if(uvx_dock_led_setup_process() == UVX_I2C_BUSY)
                {
                    return UVX_DOCK_CHARGER_BUSY;
                }
                if(!dock_supply_requested || !dock_charger_status.supply_enabled || !dock_charger_status.supply_ready ||
                   (uint32_t)(HAL_GetTick() - dock_charger_status.supply_enable_tick) < UVX_DOCK_SUPPLY_STARTUP_MS)
                {
                    UVX_APP_PWR_FET(0U);
                    dock_charger_status.voltage_ready = 0U;
                    dock_charger_status.current_ready = 0U;
                    dock_charger_status.regulation_state = UVX_DOCK_CHARGER_REGULATION_MATCH_VOLTAGE;
                    dock_voltage_adjusting = 0U;
                    uvx_dock_led_color_process();
                    return UVX_DOCK_CHARGER_BUSY;
                }
                if(dock_charger_status.regulation_state == UVX_DOCK_CHARGER_REGULATION_CONTROL_CURRENT)
                {
                    result = uvx_dock_charger_regulate_current();
                }
                else
                {
                    dock_voltage_adjusting = 1U;
                    result = uvx_dock_charger_regulate_voltage();
                }
                /* Regulation only schedules the next transaction. Update the LED
                 * now, including the voltage-ready transition to WRITE_CURRENT;
                 * the pending LED write is drained before that transaction. */
                uvx_dock_led_color_process();
                return result;

            case UVX_DOCK_CHARGER_STATE_ERROR:
                dock_charger_status.voltage_detected = 0U;
                dock_charger_status.voltage_ready = 0U;
                uvx_dock_led_color_process();
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
