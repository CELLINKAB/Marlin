// copyright cellink 2022 GPLv3
#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../../MarlinCore.h"
#    include "../gcode.h"

void GcodeSuite::M806()
{
    static auto init_pins [[maybe_unused]] = []() {
        OUT_WRITE(UVC_RELAY_PIN, LOW);
        OUT_WRITE(UVC_PWM_PIN, LOW);
        SET_INPUT_PULLDOWN(UVC_TFAULT_PIN);
        return true;
    }();

    const uint8_t intensity = parser.byteval('I', 255);
    const auto exposure_seconds = min(parser.ulongval('S', 600), 1200UL);
    const auto frequency = min(parser.ulongval('F', 1000), 2000UL);
    const bool safety_override = parser.boolval('O');
    
    const millis_t end_time = millis() + SEC_TO_MS(exposure_seconds);

    const uint32_t us_per_period = 1'000'000 / frequency;
    const uint32_t us_on_time = us_per_period / 255 * intensity;
    const uint32_t us_off_time = us_per_period - us_on_time;

    const auto time_fixed_idle = [](uint32_t micro_seconds) {
        uint32_t turn_off_time = micros() + micro_seconds;
        idle();
        delayMicroseconds(turn_off_time - micros());
    };

    WRITE(UVC_RELAY_PIN, HIGH);
    while ((safety_override || (READ(UVC_TFAULT_PIN) == LOW)) && millis() < end_time) {
        WRITE(UVC_PWM_PIN, HIGH);
        if (intensity >= 128)
            time_fixed_idle(us_on_time);
        else
            delayMicroseconds(us_on_time);
        WRITE(UVC_PWM_PIN, LOW);
        if (intensity < 128)
            time_fixed_idle(us_off_time);
        else
            delayMicroseconds(us_off_time);
    }
    WRITE(UVC_RELAY_PIN, LOW);
}

#endif // UVC_STERILIZATION