#include "../../../inc/MarlinConfig.h"

#if HAS_COLOR_LEDS && ENABLED(RGB_LED_FADE_COMMAND)

#    include "../../../MarlinCore.h"
#    include "../../../feature/leds/leds.h"
#    include "../../gcode.h"

bool AnimationManager::running() const
{
    return remaining_cycles > 0 && millis() <= active_fade.expiration();
}

void AnimationManager::update()
{
    millis_t current_time = millis();
    if (current_time < next_tick)
        return;

    LEDColor next_color = active_fade.next_state(current_time);
    switch (active_fade.strip) {
    case LEDStrip::None:
        return;
    case LEDStrip::All:
        TERN_(NEOPIXEL2_SEPARATE, leds2.set_color(next_color));
        [[fallthrough]];
    case LEDStrip::StripOne:
        TERN_(NEOPIXEL_LED, neo.neoindex = active_fade.pixel);
        leds.set_color(next_color);
        break;
#    if ENABLED(NEOPIXEL2_SEPARATE)
    case LEDStrip::StripTwo:
        neo2.neoindex = active_fade.pixel;
        leds2.set_color(next_color);
        break;
#    endif
    }
    if (current_time >= active_fade.expiration()) {
        if (remaining_cycles > 0) {
            LedFade next_fade(active_fade.strip,
                              millis(),
                              active_fade.duration,
                              active_fade.end_color,
                              active_fade.start_color,
                              active_fade.pixel);
            active_fade = next_fade;
            --remaining_cycles;
        } else {
            active_fade.strip = LEDStrip::None;
        }
    }
    next_tick = millis() + MIN_FADE_TICK;
}

AnimationManager animation_manager;

constexpr LEDStrip int_to_strip(int16_t index)
{
    switch (index) {
    case 0:
        return LEDStrip::StripOne;
    case 1:
        return LEDStrip::StripTwo;

    default:
        return LEDStrip::All;
    }
}

void GcodeSuite::M151()
{
    if (!parser.seenval('D'))
        return;
    millis_t duration = parser.value_millis_from_seconds();
    int8_t pixel = parser.intval('I', -1);
    LEDStrip strip = int_to_strip(parser.intval('S', -1));

    LEDColor old_color;
    if (strip == LEDStrip::StripOne || strip == LEDStrip::All) {
        old_color = leds.get_color();
    }
#    if ENABLED(NEOPIXEL2_SEPARATE)
    else if (strip == LEDStrip::StripTwo) {
        old_color = leds2.get_color();
    }
#    endif

    const LEDColor
        new_color((parser.seen('R') ? (parser.has_value() ? parser.value_byte() : 255) : old_color.r),
                  (parser.seen('U') ? (parser.has_value() ? parser.value_byte() : 255) : old_color.g),
                  (parser.seen('B') ? (parser.has_value() ? parser.value_byte() : 255) : old_color.b)
                      OPTARG(HAS_WHITE_LED,
                             parser.seen('W') ? (parser.has_value() ? parser.value_byte() : 255)
                                              : old_color.w)
                          OPTARG(NEOPIXEL_LED,
                                 parser.seen('P') ? parser.has_value() ? (parser.value_byte()) : 255
                                                  : neo.brightness()));

    animation_manager.active_fade = LedFade(strip, millis(), duration, old_color, new_color, pixel);
    animation_manager.remaining_cycles = parser.ulongval('C');

    if (parser.boolval('N')) {
        while (animation_manager.running())
            idle();
    }
}

#endif