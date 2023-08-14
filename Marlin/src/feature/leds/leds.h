/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * leds.h - Marlin general RGB LED support
 */

#include "../../inc/MarlinConfigPre.h"

#include <string.h>

// A white component can be passed
#if EITHER(RGBW_LED, PCA9632_RGBW)
#    define HAS_WHITE_LED 1
#endif

#if ENABLED(NEOPIXEL_LED)
#    define _NEOPIXEL_INCLUDE_
#    include "neopixel.h"
#    undef _NEOPIXEL_INCLUDE_
#endif

#if ENABLED(BLINKM)
#    include "blinkm.h"
#endif

#if ENABLED(PCA9533)
#    include "pca9533.h"
#endif

#if ENABLED(PCA9632)
#    include "pca9632.h"
#endif

/**
 * LEDcolor type for use with leds.set_color
 */
struct LEDColor
{
    uint8_t r, g, b OPTARG(HAS_WHITE_LED, w) OPTARG(NEOPIXEL_LED, i);

    constexpr LEDColor()
        : r(255)
        , g(255)
        , b(255) OPTARG(HAS_WHITE_LED, w(255)) OPTARG(NEOPIXEL_LED, i(NEOPIXEL_BRIGHTNESS))
    {}

    LEDColor(const LEDColor&) = default;

    constexpr LEDColor(uint8_t r,
                       uint8_t g,
                       uint8_t b OPTARG(HAS_WHITE_LED, uint8_t w = 0)
                           OPTARG(NEOPIXEL_LED, uint8_t i = NEOPIXEL_BRIGHTNESS))
        : r(r)
        , g(g)
        , b(b) OPTARG(HAS_WHITE_LED, w(w)) OPTARG(NEOPIXEL_LED, i(i))
    {}

    constexpr LEDColor(const uint8_t (&rgbw)[4])
        : r(rgbw[0])
        , g(rgbw[1])
        , b(rgbw[2]) OPTARG(HAS_WHITE_LED, w(rgbw[3])) OPTARG(NEOPIXEL_LED, i(NEOPIXEL_BRIGHTNESS))
    {}

    constexpr LEDColor& operator=(const uint8_t (&rgbw)[4])
    {
        r = rgbw[0];
        g = rgbw[1];
        b = rgbw[2];
        TERN_(HAS_WHITE_LED, w = rgbw[3]);
        return *this;
    }

    constexpr bool operator==(const LEDColor& right) const
    {
        return this == &right
               || (this->r == right.r && this->b == right.b && this->g == right.g
                   && TERN1(HAS_WHITE_LED, this->w == right.w)
                   && TERN1(NEOPIXEL_LED, this->i == right.i));
    }

    constexpr bool operator!=(const LEDColor& right) const { return !operator==(right); }

    constexpr bool is_off() const { return 3 > r + g + b + TERN0(HAS_WHITE_LED, w); }

    uint32_t to_neopixel_color() const;
};

/**
 * Color presets
 */

constexpr LEDColor LEDColorOff(0, 0, 0);
constexpr LEDColor LEDColorRed(255, 0, 0);
#if ENABLED(LED_COLORS_REDUCE_GREEN)
constexpr LEDColor LEDColorOrange(255, 25, 0);
constexpr LEDColor LEDColorYellow(255, 75, 0);
#else
constexpr LEDColor LEDColorOrange(255, 80, 0);
constexpr LEDColor LEDColorYellow(255, 255, 0);
#endif
constexpr LEDColor LEDColorGreen(0, 255, 0);
constexpr LEDColor LEDColorBlue(0, 0, 255);
constexpr LEDColor LEDColorIndigo(0, 255, 255);
constexpr LEDColor LEDColorViolet(255, 0, 255);
#if HAS_WHITE_LED && DISABLED(RGB_LED)
constexpr LEDColor LEDColorWhite(0, 0, 0, 255);
#else
constexpr LEDColor LEDColorWhite(255, 255, 255);
#endif

class LEDLights
{
public:
    static LEDColor color; // last non-off color
    static bool lights_on; // the last set color was "on"

    LEDLights() {}         // ctor

    static void setup();   // init()

    static void set_color(const LEDColor& color OPTARG(NEOPIXEL_IS_SEQUENTIAL,
                                                       bool isSequence = false));

    static void set_color(uint8_t r,
                          uint8_t g,
                          uint8_t b OPTARG(HAS_WHITE_LED, uint8_t w = 0)
                              OPTARG(NEOPIXEL_LED, uint8_t i = NEOPIXEL_BRIGHTNESS)
                                  OPTARG(NEOPIXEL_IS_SEQUENTIAL, bool isSequence = false))
    {
        set_color(LEDColor(r, g, b OPTARG(HAS_WHITE_LED, w) OPTARG(NEOPIXEL_LED, i))
                      OPTARG(NEOPIXEL_IS_SEQUENTIAL, isSequence));
    }

    static void set_off() { set_color(LEDColorOff); }
    static void set_green() { set_color(LEDColorGreen); }
    static void set_white() { set_color(LEDColorWhite); }

#if ENABLED(LED_COLOR_PRESETS)
    static const LEDColor defaultLEDColor;
    static void set_default() { set_color(defaultLEDColor); }
    static void set_red() { set_color(LEDColorRed); }
    static void set_orange() { set_color(LEDColorOrange); }
    static void set_yellow() { set_color(LEDColorYellow); }
    static void set_blue() { set_color(LEDColorBlue); }
    static void set_indigo() { set_color(LEDColorIndigo); }
    static void set_violet() { set_color(LEDColorViolet); }
#endif

    static LEDColor get_color() { return lights_on ? color : LEDColorOff; }

#if ENABLED(LED_CONTROL_MENU)
    static void toggle(); // swap "off" with color
#endif
#if EITHER(LED_CONTROL_MENU, CASE_LIGHT_USE_RGB_LED) || LED_POWEROFF_TIMEOUT > 0
    static void update() { set_color(color); }
#endif

#if LED_POWEROFF_TIMEOUT > 0
private:
    static millis_t led_off_time;

public:
    static void reset_timeout(const millis_t& ms)
    {
        led_off_time = ms + LED_POWEROFF_TIMEOUT;
        if (!lights_on)
            update();
    }
    static void update_timeout(const bool power_on);
#endif
};

extern LEDLights leds;

#if ENABLED(NEOPIXEL2_SEPARATE)

class LEDLights2
{
public:
    LEDLights2() {}

    static void setup(); // init()

    static void set_color(const LEDColor& color);

    static void set_color(uint8_t r,
                          uint8_t g,
                          uint8_t b OPTARG(HAS_WHITE_LED, uint8_t w = 0)
                              OPTARG(NEOPIXEL_LED, uint8_t i = NEOPIXEL_BRIGHTNESS))
    {
        set_color(LEDColor(r, g, b OPTARG(HAS_WHITE_LED, w) OPTARG(NEOPIXEL_LED, i)));
    }

    static void set_off() { set_color(LEDColorOff); }
    static void set_green() { set_color(LEDColorGreen); }
    static void set_white() { set_color(LEDColorWhite); }

#    if ENABLED(NEO2_COLOR_PRESETS)
    static const LEDColor defaultLEDColor;
    static void set_default() { set_color(defaultLEDColor); }
    static void set_red() { set_color(LEDColorRed); }
    static void set_orange() { set_color(LEDColorOrange); }
    static void set_yellow() { set_color(LEDColorYellow); }
    static void set_blue() { set_color(LEDColorBlue); }
    static void set_indigo() { set_color(LEDColorIndigo); }
    static void set_violet() { set_color(LEDColorViolet); }
#    endif

#    if ENABLED(NEOPIXEL2_SEPARATE)
    static LEDColor color; // last non-off color
    static bool lights_on; // the last set color was "on"
    static void toggle();  // swap "off" with color
    static void update() { set_color(color); }
    static LEDColor get_color() { return lights_on ? color : LEDColorOff; }

#    endif
};

extern LEDLights2 leds2;

#    if ENABLED(RGB_LED_FADE_COMMAND)
enum class LEDStrip {
    None,
    StripOne,
#        if ENABLED(NEOPIXEL2_SEPARATE)
    StripTwo,
#        endif
    All,
};

struct LedFade
{
    millis_t start_time;
    millis_t duration;
    LEDColor start_color;
    LEDColor end_color;
    LEDStrip strip;
    int8_t pixel;
    constexpr LedFade()
        : start_time{0}
        , duration{0}
        , start_color{}
        , end_color{}
        , strip{LEDStrip::None}
        , pixel{-1}
    {}
    constexpr explicit LedFade(LEDStrip target_strip,
                               millis_t start_time_,
                               millis_t duration_,
                               LEDColor start,
                               LEDColor end,
                               int8_t pixel_)
        : start_time{start_time_}
        , duration{duration_}
        , start_color{start}
        , end_color{end}
        , strip{target_strip}
        , pixel{pixel_}
    {}
    constexpr millis_t expiration() const { return start_time + duration; }
    constexpr LEDColor next_state(millis_t current_time) const
    {
        if (current_time >= expiration()) {
            return end_color;
        }
        auto interpolate = [](float proportion, uint8_t start, uint8_t end) {
            return static_cast<uint8_t>(
                static_cast<float>(start)
                + (proportion * (static_cast<float>(end) - static_cast<float>(start))));
        };
        float animation_state = static_cast<float>(current_time - start_time)
                                / static_cast<float>(duration);
        return LEDColor(interpolate(animation_state, start_color.r, end_color.r),
                        interpolate(animation_state, start_color.g, end_color.g),
                        interpolate(animation_state, start_color.b, end_color.b),
                        interpolate(animation_state, start_color.i, end_color.i));
    }
};

struct AnimationManager
{
    LedFade active_fade;
    millis_t next_tick;
    size_t remaining_cycles;
    static constexpr millis_t MIN_FADE_TICK = 25;
    bool running() const;
    void update();
};
extern AnimationManager animation_manager;
#    endif

#endif // NEOPIXEL2_SEPARATE
