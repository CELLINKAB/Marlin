// copyright cellink 2022 GPLv3
#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../gcode.h"
#include "../../MarlinCore.h"

void GcodeSuite::M806()
{
    static const auto init_pins [[maybe_unused]] = []() {
        OUT_WRITE(UVC_ENABLE_PIN, LOW);
        OUT_WRITE(UVC_PWM_PIN, LOW);
        SET_INPUT(UVC_ALARM_PIN);
        return true;
    }();
    
    const auto intensity = parser.byteval('I', 255);
    const auto exposure_seconds = parser.ushortval('S', 600);
    const auto frequency = parser.ulongval('F', 1000);
    const auto end_time = millis() + (exposure_seconds * 1000);

    WRITE(UVC_ENABLE_PIN, HIGH);
    pwm_start(digitalPinToPinName(UVC_PWM_PIN), frequency, intensity, TimerCompareFormat_t::RESOLUTION_8B_COMPARE_FORMAT);
    while ((READ(UVC_ALARM_PIN) == LOW) && PENDING(millis(), end_time))
    {
        delay(100);
        idle();
    }
    pwm_stop(digitalPinToPinName(UVC_PWM_PIN));
    WRITE(UVC_ENABLE_PIN, LOW);
}

#endif //UVC_STERILIZATION