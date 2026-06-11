/* Includes ------------------------------------------------------------------*/
#include "uvx_ws2812.h"
#include <string.h>

static const uint8_t WS2812_GAMMA_TABLE[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
    3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
    17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
    25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
    36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
    49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
    64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
    82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
    103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
    127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
    154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
    184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
    218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
    255};

#ifdef WS2812_ENABLE_DROPLET_LAUNCH_EFFECT
    typedef struct {
        int  position ;
        int  length   ;
        int  speed    ;
        bool active   ;
    } Drop;
    #define MAX_DROPS 5
    Drop drops[MAX_DROPS];
#endif 


/* ================================================================================= 
 * ==== Private function ===========================================================
 * =================================================================================
 */

/**
 * @brief Convert a single byte to WS2812 SPI pattern
 * @param byte Input byte to convert
 * @param output Pointer to 8-byte output buffer
 */
static void ws2812_byte_to_spi_pattern(uint8_t byte, uint8_t* output)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        // Check each bit from MSB to LSB
        if(byte & (0x80 >> i))
        {
            output[i] = WS2812_ONE_PATTERN;  // Logical '1'
        }
        else
        {
            output[i] = WS2812_ZERO_PATTERN; // Logical '0'
        }
    }
}

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
void ws2812_init(WS2812_Driver* driver, UVX_SPI* spi)
{
    if(driver && spi)
    {
        driver->spi = spi;
        driver->led_count = WS2812_LED_COUNT;
        driver->update_pending = 0;
        driver->spi->TX_Ready  = 1;  // Must be 1 to allow first update
        
        // Initialize buffer: 500 reset bytes + LED data + 500 reset bytes
        // Reset pattern is 0x00 (all zeros)
        memset(driver->tx_buffer, 0, WS2812_BUFFER_SIZE);
    }
    
    // Initialize SPI peripheral
    UVX_SPI_STATE spi_status = uvx_spi_init(driver->spi, driver->tx_buffer, NULL);
	if(spi_status != UVX_SPI_OK) 
	{
		// Error initializing SPI - DO NOT use while(1) here
		// Return or handle error at application level
		return;
	}

    ws2812_clear(driver); // clear pixels in buffer 

    // Animations Pre processing:
    #ifdef WS2812_ENABLE_DROPLET_LAUNCH_EFFECT
        for(uint8_t i = 0; i < MAX_DROPS; i++){
            drops[i].active = false ;
        }
    #endif
}

/* ================================================================================= 
 * ==== General Porpulse function ==================================================
 * =================================================================================
 */
int uxv_random(int min, int max)
{
    return (rand() % (max - min)) + min ;
}


void uvx_led_strip_effect_heartbeat(WS2812_Driver* driver)
{
    if (!driver) return;

    static uint16_t pulsePos = 0;
    static uint8_t  phase = 0;      // 0 = first beat, 1 = second beat, 2 = pause
    static uint16_t pauseCounter = 0;

    uint16_t half = WS2812_LED_HALFCOUNT;

    ws2812_clear(driver);

    /* ---- Purple background ---- */
    for (uint16_t i = 0; i < half; i++)
    {
        ws2812_set_led(driver, i,
                       WS2812_HSV(120, 10, 10));  // calm purple
    }

    /* ---- Heartbeat pulse ---- */
    uint8_t redHue = 0;          // red
    uint8_t redSat = 255;
    uint8_t redVal = 255;        // strong but not max

    if (phase < 2)
    {
        if (pulsePos < half)
        {
            ws2812_set_led(driver, pulsePos - 1,
                WS2812_HSV(redHue, redSat, redVal));
            ws2812_set_led(driver, pulsePos,
                           WS2812_HSV(redHue, redSat, redVal));
            ws2812_set_led(driver, pulsePos + 1,
                           WS2812_HSV(redHue, redSat, redVal));    
            ws2812_set_led(driver, pulsePos + 2,
                           WS2812_HSV(redHue, redSat, redVal));                                                  

            pulsePos++;
        }
        else
        {
            pulsePos = 0;
            phase++;
        }
    }
    else
    {
        pauseCounter++;
        if (pauseCounter > 40)   // heartbeat pause timing
        {
            pauseCounter = 0;
            phase = 0;
        }
    }

    ws2812_mirror(driver);
    ws2812_update(driver);
}

void uvx_led_strip_effect_idle(WS2812_Driver* driver)
{
    if (!driver) return;

    static uint16_t phase = 0;

    uint16_t half = WS2812_LED_HALFCOUNT;
    uint8_t  baseBlue = 20;     // background blue brightness
    uint8_t  pulseWidth = 8;

    ws2812_clear(driver);

    /* Move pulse toward center */
    phase++;
    if (phase > half)
        phase = 0;

    for (uint16_t i = 0; i < half; i++)
    {
        /* ---- Default background: calm blue ---- */
        uint16_t hue = 220;          // blue
        uint8_t  sat = 255;
        uint8_t  brightness = baseBlue;

        /* Distance from pulses */
        int16_t distLeft  = i - phase;
        if (distLeft < 0) distLeft = -distLeft;

        int16_t distRight = (half - 1 - i) - phase;
        if (distRight < 0) distRight = -distRight;

        /* ---- Red pulse override ---- */
        if (distLeft < pulseWidth || distRight < pulseWidth)
        {
            int16_t dist = (distLeft < distRight) ? distLeft : distRight;

            uint8_t pulseBrightness = (pulseWidth - dist) * 30;
            if (pulseBrightness > 255) pulseBrightness = 255;

            hue = 0;                 // red
            brightness = pulseBrightness;
        }

        ws2812_set_led(driver, i, WS2812_HSV(hue, sat, brightness));
    }

    ws2812_mirror(driver);
    ws2812_update(driver);
}

void uvx_led_strip_effect_insane_storm(WS2812_Driver* driver)
{
    if (!driver) return;

    /* --- Static state variables --- */
    static uint8_t breath      = 0;   // background pulse brightness
    static uint8_t breathDir   = 1;   // direction of breathing
    static uint16_t drops_pos[MAX_DROPS] = {0};   // positions of storm streaks
    static uint8_t drops_active[MAX_DROPS] = {0}; // active flag
    static uint8_t sparkCooldown = 0;

    ws2812_clear(driver);

    /* --- Soft breathing background --- */
    if (breathDir)
    {
        breath++;
        if (breath >= 180) breathDir = 0;
    }
    else
    {
        breath--;
        if (breath <= 50) breathDir = 1;
    }

    uint8_t bgBrightness = gamma8(breath);

    for(uint16_t i=0;i<WS2812_LED_HALFCOUNT;i++)
    {
        ws2812_set_led(driver, i, WS2812_HSV(280, 180, bgBrightness)); // purple background
    }

    /* --- Random lightning sparks --- */
    if(sparkCooldown == 0)
    {
        for(uint16_t i=0;i<WS2812_LED_HALFCOUNT;i++)
        {
            if(uxv_random(0,100) < 10) // 10% chance per LED
            {
                uint8_t sparkBright = uxv_random(200,255);
                ws2812_set_led(driver, i, WS2812_HSV(60, 0, sparkBright)); // white flash
            }
        }
        sparkCooldown = 3; // frames until next spark
    }
    else
    {
        sparkCooldown--;
    }

    /* --- Droplet-like streaks --- */
    for(uint16_t i=0;i<MAX_DROPS;i++)
    {
        if(!drops_active[i] && uxv_random(0,100) < 5) // 5% chance to launch
        {
            drops_active[i] = 1;
            drops_pos[i] = 0;
        }

        if(drops_active[i])
        {
            for(int j=0;j<4;j++) // streak length
            {
                int pos = drops_pos[i]-j;
                if(pos>=0 && pos<WS2812_LED_HALFCOUNT)
                {
                    uint8_t val = 255 - j*60;
                    ws2812_set_led(driver, pos, WS2812_HSV(320 + uxv_random(-10,10), 200, val)); // pink/purple streak
                }
            }

            drops_pos[i] += uxv_random(1,3); // speed
            if(drops_pos[i] > WS2812_LED_HALFCOUNT)
                drops_active[i] = 0;
        }
    }

    /* --- Mirror & update --- */
    ws2812_mirror(driver);
    ws2812_update(driver);
}


void uvx_led_strip_effect_electric_storm(WS2812_Driver* driver)
{
    if (!driver) return;

    /* --- Static state variables --- */
    static uint16_t baseHue        = 0;   // rainbow wave base hue
    static uint8_t  breath         = 0;   // brightness pulse
    static uint8_t  breathDir      = 1;   // breathing direction
    static uint8_t  sparkCooldown  = 0;   // cooldown timer for random sparks

    ws2812_clear(driver);  // clear previous frame

    /* --- Breathing Brightness --- */
    if (breathDir)
    {
        breath++;
        if (breath >= 200) breathDir = 0;
    }
    else
    {
        breath--;
        if (breath <= 80) breathDir = 1;
    }

    uint8_t brightness = gamma8(breath);

    /* --- Rainbow Wave --- */
    for (uint16_t i = 0; i < WS2812_LED_HALFCOUNT; i++)
    {
        uint16_t hue = baseHue + (i * 12);  // rainbow spread
        if (hue >= 360) hue %= 360;

        ws2812_set_led(driver, i, WS2812_HSV(hue, 255, brightness));
    }

    baseHue += 3;  // wave speed
    if (baseHue >= 360) baseHue = 0;

    /* --- Electric Sparks --- */
    if (sparkCooldown == 0)
    {
        // Random spark chance
        for (uint16_t i = 0; i < WS2812_LED_HALFCOUNT; i++)
        {
            if (uxv_random(0, 100) < 5)  // 5% chance per LED
            {
                uint8_t sparkBright = uxv_random(180, 255);
                ws2812_set_led(driver, i, WS2812_HSV(60, 0, sparkBright)); // white/bright spark
            }
        }
        sparkCooldown = 2;  // frame cooldown for next sparks
    }
    else
    {
        sparkCooldown--;
    }

    /* --- Mirror & update --- */
    ws2812_mirror(driver);
    ws2812_update(driver);
}


void uvx_led_strip_effect_breath(WS2812_Driver* driver)
{
    if(driver)
    {
        // constant parameters for animations 
        static uint8_t breathBrightness_var_min   =    100 ; // variable min brightness for breathing effect
        static uint8_t breathBrightness_var_max   =    250 ; // variable max brightness for breathing effect
        static uint8_t brightness_s_min           =     50 ; // minimum brightness after gamma correction
        static uint8_t brightness_s_max           =    255 ; // maximum brightness after gamma correction
        static float   spectar_variation_max      =  30.0f ;
        static float   spectar_variation_min      = -10.0f ;
        static float   spectar_variation_step     =   0.8f ;
        static uint16_t basebreathHue             =    200 ; // base hue for breathing effect (nuans na lilavoto) 

        // working variables for animations 
        static uint8_t  breathBrightness  =   0  ; // 0.0f to 1.0f 
        static bool     breathDirection   =   1  ; // 1 - increasing, 0 - decreasing
        static float    spectar_variation = 0.0f ;
        static bool     spectar_direction =   1  ; // 1 - increasing, 0 - decreasing
        static uint16_t breathHue         = 200  ; // hue for breathing effect (nuans na lilavoto)
        static uint8_t  brightness_s      =   0  ; // brightness after gamma correction

        /* LED ANIMATIONS EXECUTE */
        //WS2812_RGB_Color color = WS2812_HSV(breathHue, 255, brightness_s);
        //ws2812_set_all(driver, color);

        /* ANIMATION BLOCKS */
        #ifdef WS2812_ENABLE_BREATHING_EFFECT
            if(breathBrightness <= breathBrightness_var_min  )
            {
                breathDirection = 1 ;   // change direction
            } 

            if(breathBrightness >= breathBrightness_var_max )
            {
                breathDirection = 0 ;
            }  

            if(breathDirection)
            {
                breathBrightness += 1;            // inc/dec brightness
            } 
            else
            {
                breathBrightness -= 1;
            }                

            brightness_s = gamma8(breathBrightness);              // exponential gamma correction
            if( brightness_s < brightness_s_min )
            {
                brightness_s = brightness_s_min; 
            }

            if( brightness_s > brightness_s_max ) 
            {
                brightness_s = brightness_s_max;
            }
            
            ws2812_set_all(driver, WS2812_HSV(breathHue, 255, brightness_s));

            // Modulate hue slightly for dynamic effect
            if( spectar_variation < spectar_variation_min )
            {
                spectar_direction = 1 ;
            } 

            if( spectar_variation >  spectar_variation_max )
            {
                spectar_direction = 0 ;
            } 

            if(spectar_direction)
            {
                spectar_variation += spectar_variation_step ;
            } 
            else
            {
                spectar_variation -= spectar_variation_step ;    
            }
            breathHue = basebreathHue + (uint16_t)spectar_variation ;
        #endif 

        #ifdef WS2812_ENABLE_DROPLET_LAUNCH_EFFECT
            for(uint16_t i = 0; i < MAX_DROPS; i++)
            {
                if(!drops[i].active && uxv_random(0,100) < 4) 
                { // 4% chance to launch new drop
                    drops[i].active   = true ;
                    drops[i].position = 0    ;
                    drops[i].length   = uxv_random(3,8) ; // random length
                    drops[i].speed    = uxv_random(1,3) ; // random speed
                }

                if( drops[i].active )
                {
                    for(int j=0; j < drops[i].length; j++ )
                    {
                        int inx = drops[i].position - j ;
                        if( inx >= 0 && inx < WS2812_LED_HALFCOUNT )
                        {
                            // the color of the drop may be a little lighter purple
                            ws2812_set_led(driver, inx, WS2812_HSV(breathHue+20, 255, 255-j*40));
                        }
                    }

                    drops[i].position += drops[i].speed ;
                    if(drops[i].position - drops[i].length > WS2812_LED_HALFCOUNT) 
                    {
                        drops[i].active = false ;
                    }
                }
            }
        #endif

        /*  Example of simple chase animation */
        #ifdef WS2812_ENABLE_DEFAULT_EFFECT
            static uint16_t i = 0;
            ws2812_set_led(driver, i, WS2812_COLOR_PURPLE);
            ws2812_set_led(driver, i+1, WS2812_COLOR_BLUE);
            if(i < WS2812_LED_COUNT)
            {
                i++;
            } 
            else
            {
                ws2812_clear(driver);
                i = 0;
            }
        #endif

        /* END OF ANIMATIONS BLOCKS */
        ws2812_mirror(driver);  // mirror first half to second half    
        ws2812_update(driver);  // update frame[k]
        //HAL_Delay(20);          // delay between frames
    }
}

// void uvx_led_strip_effect_battery_level(WS2812_Driver* driver, uint8_t battery_percent)
// {
//     if (!driver) return;

//     if (battery_percent > 100)
//         battery_percent = 100;

//     /* Clear previous frame */
//     ws2812_clear(driver);

//     uint16_t leds_to_light = (WS2812_LED_HALFCOUNT * battery_percent) / 100;

//     /* Determine color based on battery percentage */
//     uint16_t hue;

//     if (battery_percent <= 20)
//     {
//         // Red zone
//         hue = 0;   // Red
//     }
//     else if (battery_percent <= 50)
//     {
//         // Red -> Yellow (0 → 60)
//         hue = (battery_percent - 20) * 60 / 30;
//     }
//     else if (battery_percent <= 80)
//     {
//         // Yellow -> Green (60 → 120)
//         hue = 60 + ((battery_percent - 50) * 60 / 30);
//     }
//     else
//     {
//         // Full green
//         hue = 120;
//     }

//     /* Draw battery bar */
//     for (uint16_t i = 0; i < leds_to_light; i++)
//     {
//         ws2812_set_led(driver, i, WS2812_HSV(hue, 255, 255));
//     }

//     // /* Optional: blinking warning when <10% */
//     // if (battery_percent < 10)
//     // {
//     //     static uint8_t blink = 0;
//     //     blink++;

//     //     if (blink & 0x10)  // simple blink timing
//     //     {
//     //         ws2812_clear(driver);
//     //     }
//     // }

//     ws2812_mirror(driver);
//     ws2812_update(driver);
// }

void uvx_led_strip_effect_battery_level( WS2812_Driver* driver, uint8_t battery_percent, uint8_t marker_percent)
{
    if (!driver) return;

    if (battery_percent > 100) battery_percent = 100;
    if (marker_percent > 100)  marker_percent  = 100;

    ws2812_clear(driver);

    uint16_t half = WS2812_LED_HALFCOUNT;

    /* ---- Determine background color from battery ---- */
    uint16_t hue;

    if (battery_percent <= 20)
    {
        hue = 0;   // Red
    }
    else if (battery_percent <= 50)
    {
        hue = (battery_percent - 20) * 60 / 30;   // Red → Yellow
    }
    else if (battery_percent <= 80)
    {
        hue = 60 + ((battery_percent - 50) * 60 / 30);  // Yellow → Green
    }
    else
    {
        hue = 120;  // Green
    }

    /* ---- Background fill ---- */
    for (uint16_t i = 0; i < half; i++)
    {
        ws2812_set_led(driver, i,
                       WS2812_HSV(hue, 255, 35));  // dim background
    }

    /* ---- Marker position from marker_percent ---- */
    uint16_t marker_pos = (half * marker_percent) / 100;

    if (marker_pos >= half)
        marker_pos = half - 1;

    /* ---- Blue marker ---- */
    ws2812_set_led(driver,
                   marker_pos,
                   WS2812_HSV(170, 255, 255));  // bright blue

    /* Small glow around marker */
    if (marker_pos > 0)
        ws2812_set_led(driver,
                       marker_pos - 1,
                       WS2812_HSV(170, 255, 120));

    ws2812_mirror(driver);
    ws2812_update(driver);
}


void uvx_led_strip_effect_charge_button(WS2812_Driver* driver, uint8_t percent)
{
    if (!driver) return;

    if (percent > 100)
        percent = 100;

    static uint8_t shimmer = 0;
    shimmer++;

    uint16_t half = WS2812_LED_HALFCOUNT;
    uint16_t filled = (half * percent) / 100;

    ws2812_clear(driver);

    for (uint16_t i = 0; i < half; i++)
    {
        if (i < filled)
        {
            /* Filled area - GREEN */

            uint8_t brightness = 120 + (shimmer % 40);  // soft shimmer
            if (brightness > 255) brightness = 255;

            ws2812_set_led(driver, i,
                           WS2812_HSV(120, 255, brightness)); // GREEN
        }
        else
        {
            /* Background area - dim dark green/blue */

            ws2812_set_led(driver, i,
                           WS2812_HSV(120, 255, 10)); // very dim green background
        }
    }

    /* White glowing head */
    if (filled > 0 && filled < half)
    {
        ws2812_set_led(driver, filled - 1,
                       WS2812_HSV(0, 0, 255)); // WHITE tip
    }

    ws2812_mirror(driver);
    ws2812_update(driver);
}

void uvx_led_strip_effect_active_comm(WS2812_Driver* driver)
{
    if (!driver) return;

    static uint16_t offset = 0;
    uint16_t half = WS2812_LED_HALFCOUNT;

    ws2812_clear(driver);

    /* ---- Dark blue background ---- */
    for (uint16_t i = 0; i < half; i++)
    {
        ws2812_set_led(driver, i,
                       WS2812_HSV(170, 255, 15));  // deep blue
    }

    /* ---- Data packets moving outward ---- */
    for (uint16_t i = 0; i < half; i++)
    {
        if ((i + offset) % 8 == 0)
        {
            // bright packet
            ws2812_set_led(driver, i,
                           WS2812_HSV(140, 255, 255));  // cyan
        }
        else if ((i + offset) % 8 == 1)
        {
            // fading tail
            ws2812_set_led(driver, i,
                           WS2812_HSV(140, 255, 120));
        }
    }

    offset++;
    if (offset >= 8)
        offset = 0;

    ws2812_mirror(driver);   // makes it go from center outward
    ws2812_update(driver);
}


void uvx_led_strip_effect_off(WS2812_Driver* driver)
{
    if (!driver) return;

    ws2812_clear(driver);     // set all LEDs to 0
    ws2812_update(driver);    // transmit data
}

void uvx_led_strip_effect_waiting(WS2812_Driver* driver)
{
    if (!driver) return;

    static uint16_t position = 0;
    static uint8_t  direction = 1;

    uint16_t total = WS2812_LED_COUNT;  // full strip

    ws2812_clear(driver);

    /* ---- Purple background ---- */
    for (uint16_t i = 0; i < total; i++)
    {
        ws2812_set_led(driver, i,
                       WS2812_HSV(120, 255, 25));  // correct purple
    }

    /* ---- White dot ---- */
    ws2812_set_led(driver, position,
                   WS2812_HSV(0, 0, 255));  // white dot

    /* ---- Move dot ---- */
    if (direction)
    {
        position++;
        if (position >= total - 1)
            direction = 0;
    }
    else
    {
        if (position > 0)
            position--;
        else
            direction = 1;
    }

    ws2812_update(driver);
}

void uvx_led_strip_effect_solid_color(WS2812_Driver* driver,
                                      uint8_t hue,
                                      uint8_t saturation,
                                      uint8_t value)
{
    if (!driver) return;

    uint16_t total = WS2812_LED_COUNT;

    for (uint16_t i = 0; i < total; i++)
    {
        ws2812_set_led(driver, i,
                       WS2812_HSV(hue, saturation, value));
    }

    ws2812_update(driver);
}



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
void ws2812_set_led(WS2812_Driver* driver, uint16_t led_index, WS2812_RGB_Color color)
{
    if(driver && led_index < driver->led_count)
    {
        // Calculate buffer offset for this LED
        uint16_t offset = led_index * WS2812_BYTES_PER_LED;
        
        // WS2812C uses GRB order (not RGB!)
        ws2812_byte_to_spi_pattern(color.green, &driver->tx_buffer[offset + 0]);
        ws2812_byte_to_spi_pattern(color.red,   &driver->tx_buffer[offset + 8]);
        ws2812_byte_to_spi_pattern(color.blue,  &driver->tx_buffer[offset + 16]);
    }
}

/**
 * @brief Set all LEDs to the same color
 * @param driver Pointer to WS2812_Driver structure
 * @param color RGB color to set
 */
void ws2812_set_all(WS2812_Driver* driver, WS2812_RGB_Color color)
{
    if(driver)
    {
        for(uint16_t i = 0; i < driver->led_count; i++)
        {
            ws2812_set_led(driver, i, color);
        }
    }
}

/**
 * @brief Clear all LEDs (turn off)
 * @param driver Pointer to WS2812_Driver structure
 */
void ws2812_clear(WS2812_Driver* driver)
{
    ws2812_set_all(driver, WS2812_COLOR_OFF);
}

/**
 * @brief Mirror the first half of the LED strip to the second half
 * @param driver Pointer to WS2812_Driver structure
 */
void ws2812_mirror(WS2812_Driver* driver){
    if(driver)
    {
        for(uint16_t i = 0; i < WS2812_LED_HALFCOUNT; i++)
        {
            // Read color from first half
            uint16_t offset_src = i * WS2812_BYTES_PER_LED;
            WS2812_RGB_Color color;
            // Extract GRB from SPI pattern
            // Green
            color.green = 0;
            for(uint8_t bit = 0; bit < 8; bit++)
            {
                if(driver->tx_buffer[offset_src + bit] == WS2812_ONE_PATTERN)
                {
                    color.green |= (0x80 >> bit);
                }
            }
            // Red
            color.red = 0;
            for(uint8_t bit = 0; bit < 8; bit++)
            {
                if(driver->tx_buffer[offset_src + 8 + bit] == WS2812_ONE_PATTERN)
                {
                    color.red |= (0x80 >> bit);
                }
            }
            // Blue
            color.blue = 0;
            for(uint8_t bit = 0; bit < 8; bit++)
            {
                if(driver->tx_buffer[offset_src + 16 + bit] == WS2812_ONE_PATTERN)
                {
                    color.blue |= (0x80 >> bit);
                }
            }

            // Set color to mirrored LED in second half
            ws2812_set_led(driver, driver->led_count - 1 - i, color);
        }
    }
}
/* ================================================================================= 
 * ==== Update and status functions ================================================
 * =================================================================================
 */

/**
 * @brief Update LED strip with current buffer data
 * @param driver Pointer to WS2812_Driver structure
 * @return UVX_SPI_STATE Status of the operation
 */
UVX_SPI_STATE ws2812_update(WS2812_Driver* driver)
{
    if(driver && driver->spi)
    {       
        // Mark as pending
        driver->update_pending = 1;
        
        // Start transmission
        UVX_SPI_STATE status = uvx_spi_transmit(driver->spi, driver->tx_buffer, WS2812_BUFFER_SIZE);
        
        if(status != UVX_SPI_OK)
        {
            driver->update_pending = 0;
        }
        
        return status;
    }
    
    return UVX_SPI_ERROR;
}

/**
 * @brief Check if update is complete
 * @param driver Pointer to WS2812_Driver structure
 * @return 1 if complete, 0 if still in progress
 */
uint8_t ws2812_is_ready(WS2812_Driver* driver)
{
    if(driver && driver->spi)
    {   
        if( driver->spi->hal_spi.hspi.hdmatx->State == HAL_DMA_STATE_READY && driver->spi->hal_spi.hspi.State != HAL_SPI_STATE_BUSY_TX)
        {   
            driver->spi->TX_Ready  = 0;
            driver->update_pending = 0;
            return 1;  // SPI is ready
        }

        // Check if SPI transmission is complete
        if(driver->spi->TX_Ready == 0 && driver->update_pending == 1){
                return 0; 									// await spi to set led strip 
        }
        
		if(driver->spi->TX_Ready == 1 && driver->update_pending == 1)
        {
            driver->update_pending = 0;
            driver->spi->TX_Ready  = 0;                     // Clear the flag
            return 1;										// led strip is ready
        }
        if(driver->spi->TX_Ready == 1 && driver->update_pending == 0){
            driver->spi->TX_Ready  = 0;                     // Clear the flag
            return 1;										// led strip is ready 
	    }
				
		if(driver->spi->TX_Ready == 0 && driver->update_pending == 0){
            return 1;										// led strip is ready
		}		
    }
    
    return 0;
}


/* ================================================================================= 
 * ==== Color utility functions ==================================================== 
 * =================================================================================
 */

/**
 * @brief   An 8-bit gamma-correction function for basic pixel brightness
 *          adjustment. Makes color transitions appear more perceptially 
 *          correct.
 * @param   brightness Input brightness, 0 (minimum or off/black) to 255 (maximum)
 * @return  Gamma-adjusted brightness.
 *          This uses a fixed gamma correction exponent of 2.6
 */
uint8_t gamma8(uint8_t x) {
    return WS2812_GAMMA_TABLE[x];
}

/**
 * @brief   An 32-bit gamma-correction function for basic pixel brightness
 *          adjustment. Makes color transitions appear more perceptially
 *          correct.
 * @param   WS2812_RGB_Color color  
 *          Input brightness, 0 (minimum or off/black) to 255 (maximum)
 *          for each color component.
 * @return  Gamma-adjusted brightness color. 
 *          This uses a fixed gamma correction exponent of 2.6
 */
WS2812_RGB_Color gamma32(WS2812_RGB_Color color) {
    return (WS2812_RGB_Color) { 
        gamma8(color.red), 
        gamma8(color.green), 
        gamma8(color.blue) 
    };
}

/**
 * @brief   Convert HSV color to RGB color
 * @param   hue         Hue component (0-65535)   
 * @param   sat         Saturation component (0-255)
 * @param   val         Value component (0-255)
 * @return  Corresponding RGB color - WS2812_RGB_Color 
 */
WS2812_RGB_Color WS2812_HSV(uint8_t hue8, uint8_t sat, uint8_t val)
{   
    uint16_t hue = ((uint16_t)hue8 << 8) | hue8;  // Convert 8-bit hue to 16-bit 
    uint8_t r, g, b;

    // --- Hue → RGB (копираш дословно от Adafruit) ---
    hue = (hue * 1530L + 32768) / 65536;

    if (hue < 510) {
        b = 0;
        if (hue < 255) { r = 255; g = hue; }
        else           { r = 510 - hue; g = 255; }
    }
    else if (hue < 1020) {
        r = 0;
        if (hue < 765) { g = 255; b = hue - 510; }
        else           { g = 1020 - hue; b = 255; }
    }
    else if (hue < 1530) {
        g = 0;
        if (hue < 1275) { r = hue - 1020; b = 255; }
        else            { r = 255; b = 1530 - hue; }
    }
    else {
        r = 255; g = 0; b = 0;
    }

    // --- Saturation + Value ---
    uint16_t v1 = 1 + val;
    uint16_t s1 = 1 + sat;
    uint8_t  s2 = 255 - sat;

    uint16_t t;

    t = (((r * s1) >> 8) + s2) * v1;
    r = (t & 0xff00) >> 8;

    t = (((g * s1) >> 8) + s2) * v1;
    g = (t & 0xff00) >> 8;

    t = (((b * s1) >> 8) + s2) * v1;
    b = (t & 0xff00) >> 8;

    return ws2812_color_RGB(r, g, b);
}

/**
 * @brief Dim color by a factor
 * @param color Original color
 * @param brightness Brightness factor (0-255, where 255 is full brightness)
 * @return Dimmed color
 */
WS2812_RGB_Color ws2812_dim_color(WS2812_RGB_Color color, uint8_t brightness)
{
    WS2812_RGB_Color dimmed;
    
    dimmed.red   = (color.red   * brightness) / 255;
    dimmed.green = (color.green * brightness) / 255;
    dimmed.blue  = (color.blue  * brightness) / 255;
    
    return dimmed;
}
