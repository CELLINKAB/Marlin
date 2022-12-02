// copyright cellink 2022 GPLv3
#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../gcode.h"
#include "../../MarlinCore.h"

void GcodeSuite::M806()
{
    static auto init_pins [[maybe_unused]] = []() {
        OUT_WRITE(UVC_RELAY_PIN, LOW);
        OUT_WRITE(UVC_PWM_PIN, LOW);
        SET_INPUT_PULLDOWN(UVC_TFAULT_PIN);
        return true;
    }();
    
    const auto intensity = constrain(parser.byteval('I', 255), 0, 255);
    const auto exposure_seconds = max(parser.ulongval('S', 600), 1200UL);
    const auto frequency = min(parser.ulongval('F', 1000), 2000UL);
    const auto end_time = millis() + (exposure_seconds * 1000);

    uint32_t us_per_period = 1'000'000 / frequency;
    uint32_t us_on_time = us_per_period / 255 * intensity;
    uint32_t us_off_time = us_per_period - us_on_time;

    auto time_fixed_idle = [](uint32_t micro_seconds){uint32_t turn_off_time = micros() + micro_seconds; idle(); delayMicroseconds(turn_off_time - micros());}; 

    WRITE(UVC_RELAY_PIN, HIGH);
    while (/*(READ(UVC_ALARM_PIN) == LOW) &&*/ millis() < end_time)
    {
        WRITE(UVC_PWM_PIN, HIGH)   ;
        if (intensity >= 128) time_fixed_idle(us_on_time); else delayMicroseconds(us_on_time);
        WRITE(UVC_PWM_PIN, LOW);
        if (intensity < 128) time_fixed_idle(us_off_time); else delayMicroseconds(us_off_time);
    }
    WRITE(UVC_RELAY_PIN, LOW);
}

#endif // UVC_STERILIZATION