/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_WS2812_H
#define __UVX_WS2812_H

#include "uvx_spi.h"
#include <stdint.h>

/* WS2812C timing definitions */
#define WS2812_ZERO_PATTERN     0xC0  // 0b11000000 - represents logical '0'  (25% duty cycle)
#define WS2812_ONE_PATTERN      0xF0  // 0b11110000 - represents logical '1'  (50% duty cycle)
#define WS2812_RESET_BYTES      500   // Reset time > 50us (10 bytes at 5Mbps)

/* LED strip configuration */
#define WS2812_LED_COUNT        23                                              // Number of LEDs in the strip
#define WS2812_LED_HALFCOUNT    WS2812_LED_COUNT/2                              // Half Number of LEDs in the strip (where is mirroring center)
#define WS2812_COLOR_BYTES      3                                               // RGB color bytes per LED
#define WS2812_BITS_PER_BYTE    8                                               // Bits per color byte
#define WS2812_BYTES_PER_LED    (WS2812_COLOR_BYTES * WS2812_BITS_PER_BYTE)     // 24 bytes per LED
/**
 * WS2812 Buffer Structure:
 * [500 reset bytes (0x00)] + [LED data: 23 LEDs * 24 bytes each] + [500 reset bytes (0x00)]
 * Total: 1150 bytes
 * 
 * Each LED uses 24 bytes (8 bytes per color component):
 * - Byte 0-7: Green (MSB first)
 * - Byte 8-15: Red (MSB first)  
 * - Byte 16-23: Blue (MSB first)
 * 
 * Each bit in the color data is encoded using 1 byte in SPI:
 * - Logical '1': 0xF0 (1111 0000) - ~50% duty cycle
 * - Logical '0': 0xC0 (1100 0000) - ~25% duty cycle
 */
#define WS2812_BUFFER_SIZE      (WS2812_LED_COUNT * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES)

/**
 * @brief RGB color structure
 */
typedef struct {
    uint8_t     red   ;   
    uint8_t     green ;
    uint8_t     blue  ;
} WS2812_RGB_Color;

typedef struct {
    uint16_t    hue        ;
    uint8_t     saturation ;
    uint8_t     value      ;
} WS2812_HSV_Color;

/**
 * @brief WS2812 driver structure
 */
typedef struct {
    UVX_SPI* spi;                          // Pointer to SPI instance
    uint8_t tx_buffer[WS2812_BUFFER_SIZE]; // SPI transmission buffer
    uint16_t led_count;                    // Number of LEDs
    uint8_t update_pending;                // Flag indicating update in progress
} WS2812_Driver;

/**
 * @brief Predefined colors
 */
#define WS2812_COLOR_OFF        ((WS2812_RGB_Color){0, 0, 0})
#define WS2812_COLOR_RED        ((WS2812_RGB_Color){255, 0, 0})
#define WS2812_COLOR_GREEN      ((WS2812_RGB_Color){0, 255, 0})
#define WS2812_COLOR_BLUE       ((WS2812_RGB_Color){0, 0, 255})
#define WS2812_COLOR_WHITE      ((WS2812_RGB_Color){255, 255, 255})
#define WS2812_COLOR_YELLOW     ((WS2812_RGB_Color){255, 255, 0})
#define WS2812_COLOR_CYAN       ((WS2812_RGB_Color){0, 255, 255})
#define WS2812_COLOR_MAGENTA    ((WS2812_RGB_Color){255, 0, 255})
#define WS2812_COLOR_ORANGE     ((WS2812_RGB_Color){255, 165, 0})
#define WS2812_COLOR_PURPLE     ((WS2812_RGB_Color){128, 0, 128})

/**
  * @brief Enable animations blocks 
  */
//#define WS2812_ENABLE_DEFAULT_EFFECT
#define WS2812_ENABLE_BREATHING_EFFECT
#define WS2812_ENABLE_DROPLET_LAUNCH_EFFECT

/* ================================================================================= 
 * ==== Init function ==============================================================
 * =================================================================================
 */

/**
 * @brief Initialize WS2812 driver
 * @param driver Pointer to WS2812_Driver structure
 * @param spi Pointer to initialized SPI instance
 * @param led_count Number of LEDs in the strip
 */

/* ================================================================================= 
 * ==== Control led buffer functions ================================================
 * =================================================================================
 */
 
/**
 * @brief Set color of a single LED
 * @param driver Pointer to WS2812_Driver structure
 * @param led_index Index of LED (0-based)
 * @param color RGB color to set
 */
void ws2812_set_led(WS2812_Driver* driver, uint16_t led_index, WS2812_RGB_Color color);

/**
 * @brief Set all LEDs to the same color
 * @param driver Pointer to WS2812_Driver structure
 * @param color RGB color to set
 */
void ws2812_set_all(WS2812_Driver* driver, WS2812_RGB_Color color);

/**
 * @brief Clear all LEDs (turn off)
 * @param driver Pointer to WS2812_Driver structure
 */
void ws2812_clear(WS2812_Driver* driver);

/**
 * @brief Mirror the first half of the LED strip to the second half
 * @param driver Pointer to WS2812_Driver structure
 */
void ws2812_mirror(WS2812_Driver* driver);
/* ================================================================================= 
 * ==== Update and status functions ================================================
 * =================================================================================
 */

/**
 * @brief Update LED strip with current buffer data
 * @param driver Pointer to WS2812_Driver structure
 * @return UVX_SPI_STATE Status of the operation
 */
UVX_SPI_STATE ws2812_update(WS2812_Driver* driver);

/**
 * @brief Check if update is complete
 * @param driver Pointer to WS2812_Driver structure
 * @return 1 if complete, 0 if still in progress
 */
uint8_t ws2812_is_ready(WS2812_Driver* driver);



/* ================================================================================= 
 * ==== Color utility functions ==================================================== 
 * =================================================================================
 */

/**
 * @brief Create RGB color from individual components
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return WS2812_RGB_Color structure
 */
static inline WS2812_RGB_Color ws2812_color_RGB(uint8_t red, uint8_t green, uint8_t blue) {
    WS2812_RGB_Color color = {red, green, blue};
    return color;
}

static inline WS2812_HSV_Color ws2812_color_HSV(uint8_t hue, uint8_t saturation, uint8_t value) {
    WS2812_HSV_Color color = {hue, saturation, value};
    return color;
}

/**
 * @brief   Convert HSV color to RGB color
 * @param   hue         Hue component (0-65535)   
 * @param   sat         Saturation component (0-255)
 * @param   val         Value component (0-255)
 * @return  Corresponding RGB color - WS2812_RGB_Color 
 */
WS2812_RGB_Color WS2812_HSV(uint8_t hue, uint8_t sat, uint8_t val);

/**
 * @brief   An gamma-correction function for basic pixel brightness
 *          adjustment. Makes color transitions appear more perceptially
 *          correct.
 * @param   WS2812_RGB_Color color  
 *          Input brightness, 0 (minimum or off/black) to 255 (maximum)
 *          for each color component.
 * @return  Gamma-adjusted brightness color. 
 *          This uses a fixed gamma correction exponent of 2.6
 */
WS2812_RGB_Color gamma32(WS2812_RGB_Color color) ;
uint8_t          gamma8(uint8_t x);

/**
 * @brief Dim color by a factor
 * @param color Original color
 * @param brightness Brightness factor (0-255, where 255 is full brightness)
 * @return Dimmed color
 */
WS2812_RGB_Color ws2812_dim_color(WS2812_RGB_Color color, uint8_t brightness);

void ws2812_init(WS2812_Driver* driver, UVX_SPI* spi);
void uvx_led_strip_effect_heartbeat(WS2812_Driver* driver);
void uvx_led_strip_effect_idle(WS2812_Driver* driver);
void uvx_led_strip_effect_insane_storm(WS2812_Driver* driver);
void uvx_led_strip_effect_electric_storm(WS2812_Driver* driver);
void uvx_led_strip_effect_breath(WS2812_Driver* driver);
void uvx_led_strip_effect_battery_level( WS2812_Driver* driver, uint8_t battery_percent, uint8_t marker_percent);
void uvx_led_strip_effect_charge_button(WS2812_Driver* driver, uint8_t percent);
void uvx_led_strip_effect_active_comm(WS2812_Driver* driver);
void uvx_led_strip_effect_off(WS2812_Driver* driver);
void uvx_led_strip_effect_waiting(WS2812_Driver* driver);
void uvx_led_strip_effect_solid_color(WS2812_Driver* driver, uint8_t hue, uint8_t saturation, uint8_t value);

#endif /* __UVX_WS2812_H */
