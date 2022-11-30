// copyright cellink 2022 GPLv3
#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../gcode.h"
#include "../../MarlinCore.h"

void GcodeSuite::M806()
{
    static auto init_pins [[maybe_unused]] = []() {
        OUT_WRITE(UVC_ENABLE_PIN, LOW);
        OUT_WRITE(UVC_PWM_PIN, LOW);
        SET_INPUT_PULLDOWN(UVC_ALARM_PIN);
        return true;
    }();
    
    const auto intensity = parser.byteval('I', 255);
    const auto exposure_seconds = parser.ushortval('S', 600);
    const auto frequency = parser.ulongval('F', 1000);
    const auto end_time = millis() + (exposure_seconds * 1000);

    WRITE(UVC_ENABLE_PIN, HIGH);
    analogWrite(UVC_PWM_PIN, intensity);
    while (/*(READ(UVC_ALARM_PIN) == LOW) &&*/ millis() < end_time)
    {
        delay(100);
        idle();
    }
    analogWrite(UVC_PWM_PIN, 0);
    WRITE(UVC_ENABLE_PIN, LOW);
}

#endif //UVC_STERILIZATION