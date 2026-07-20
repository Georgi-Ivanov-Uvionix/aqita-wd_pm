/**
  ******************************************************************************
  * @file    uvx_tpl0401x_10.c
  * @brief   TPL0401A/B-10 digital potentiometer I2C driver.
  *
  * This source file is intentionally not included by the application yet.  A
  * public header and build-system integration can be added after review.
  ******************************************************************************
  */

#include "uvx_i2c.h"

#define UVX_TPL0401A_10_I2C_ADDRESS       0x2EU
#define UVX_TPL0401B_10_I2C_ADDRESS       0x3EU
#define UVX_TPL0401X_10_WIPER_REGISTER    0x00U
#define UVX_TPL0401X_10_WIPER_MAX         0x7FU

/**
 * @brief Check whether an address belongs to a supported TPL0401x-10 variant.
 */
static uint8_t uvx_tpl0401x_10_address_is_valid(uint8_t device_address)
{
    return (uint8_t)((device_address == UVX_TPL0401A_10_I2C_ADDRESS) ||
                     (device_address == UVX_TPL0401B_10_I2C_ADDRESS));
}

/**
 * @brief Initialize the I2C bus used by the TPL0401x-10.
 * @param i2c Pointer to the project I2C driver instance.
 * @retval UVX_I2C_STATE Result returned by uvx_i2c_init().
 *
 * The TPL0401x-10 has no configuration register to initialize.  Its wiper
 * position is volatile and powers up at 0x40, so this function only prepares
 * the I2C peripheral.
 */
UVX_I2C_STATE uvx_tpl0401x_10_init(UVX_I2C *i2c)
{
    if(i2c == NULL)
    {
        return UVX_I2C_INIT_ERROR;
    }

    return uvx_i2c_init(i2c);
}

/**
 * @brief Start an interrupt-driven write of the wiper position.
 * @param i2c Pointer to the initialized project I2C driver instance.
 * @param device_address Seven-bit device address: 0x2E (A) or 0x3E (B).
 * @param wiper_position Wiper code in the range 0x00 to 0x7F.
 * @retval UVX_I2C_OK if the transfer was started, otherwise an I2C error state.
 */
UVX_I2C_STATE uvx_tpl0401x_10_write(UVX_I2C *i2c,
                                    uint8_t device_address,
                                    uint8_t wiper_position)
{
    uint8_t tx_data[2];

    if((i2c == NULL) ||
       (i2c->is_Initilized == 0U) ||
       (uvx_tpl0401x_10_address_is_valid(device_address) == 0U) ||
       (wiper_position > UVX_TPL0401X_10_WIPER_MAX))
    {
        return UVX_I2C_ERROR;
    }

    tx_data[0] = UVX_TPL0401X_10_WIPER_REGISTER;
    tx_data[1] = wiper_position;

    /* uvx_i2c_send() copies this local array to its persistent IT buffer. */
    return uvx_i2c_send(&i2c->hal_i2c, device_address, tx_data, sizeof(tx_data));
}

/**
 * @brief Start an interrupt-driven read of the wiper position.
 * @param i2c Pointer to the initialized project I2C driver instance.
 * @param device_address Seven-bit device address: 0x2E (A) or 0x3E (B).
 * @param wiper_position Destination that remains valid until I2C completion.
 * @retval UVX_I2C_OK if the transfer was started, otherwise an I2C error state.
 *
 * Completion is asynchronous.  The caller must wait for the existing I2C
 * receive-complete path before using the value stored in wiper_position.
 */
UVX_I2C_STATE uvx_tpl0401x_10_read(UVX_I2C *i2c,
                                   uint8_t device_address,
                                   uint8_t *wiper_position)
{
    if((i2c == NULL) ||
       (wiper_position == NULL) ||
       (i2c->is_Initilized == 0U) ||
       (uvx_tpl0401x_10_address_is_valid(device_address) == 0U))
    {
        return UVX_I2C_ERROR;
    }

    return uvx_i2c_read_mem(&i2c->hal_i2c,
                            device_address,
                            UVX_TPL0401X_10_WIPER_REGISTER,
                            I2C_MEMADD_SIZE_8BIT,
                            wiper_position,
                            1U);
}
