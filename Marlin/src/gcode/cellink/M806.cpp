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

#    if PINS_EXIST(UVC_SWITCH_1, UVC_SWITCH_2, UVC_SWITCH_3, UVC_SWITCH_4, UVC_SWITCH_5)
        OUT_WRITE(UVC_SWITCH_1_PIN, LOW);
        OUT_WRITE(UVC_SWITCH_2_PIN, LOW);
        OUT_WRITE(UVC_SWITCH_3_PIN, LOW);
        OUT_WRITE(UVC_SWITCH_4_PIN, LOW);
        OUT_WRITE(UVC_SWITCH_5_PIN, LOW);
#    endif

#    if PINS_EXIST(UVC_SH_CS_1, UVC_SH_CS_2, UVC_SH_CS_3, UVC_SH_CS_4, UVC_SH_CS_5)
        pinMode(UVC_SH_CS_1_PIN, MODE_ANALOG);
        pinMode(UVC_SH_CS_2_PIN, MODE_ANALOG);
        pinMode(UVC_SH_CS_3_PIN, MODE_ANALOG);
        pinMode(UVC_SH_CS_4_PIN, MODE_ANALOG);
        pinMode(UVC_SH_CS_5_PIN, MODE_ANALOG);
#    endif

        SET_INPUT_PULLDOWN(UVC_TFAULT_PIN);
        return true;
    }();

    static auto write_uvc_switches = [](bool state) {
        WRITE(UVC_RELAY_PIN, state);
#    if PINS_EXIST(UVC_SH_CS_1, UVC_SH_CS_2, UVC_SH_CS_3, UVC_SH_CS_4, UVC_SH_CS_5)
        WRITE(UVC_SWITCH_1_PIN, state);
        WRITE(UVC_SWITCH_2_PIN, state);
        WRITE(UVC_SWITCH_3_PIN, state);
        WRITE(UVC_SWITCH_4_PIN, state);
        WRITE(UVC_SWITCH_5_PIN, state);
#    endif
    };

    static auto report_current_sense = []() {
#    if PINS_EXIST(UVC_SH_CS_1, UVC_SH_CS_2, UVC_SH_CS_3, UVC_SH_CS_4, UVC_SH_CS_5)
        SERIAL_ECHOLNPGM("CS_1:",
                         analogRead(UVC_SH_CS_1_PIN),
                         ", CS_2:",
                         analogRead(UVC_SH_CS_2_PIN),
                         ", CS_3:",
                         analogRead(UVC_SH_CS_3_PIN),
                         ", CS_4:",
                         analogRead(UVC_SH_CS_4_PIN),
                         ", CS_5:",
                         analogRead(UVC_SH_CS_5_PIN), );
#    endif
    };

    const uint8_t intensity = parser.byteval('I', 255);
    const auto exposure_seconds = min(parser.ulongval('S', 300), 1200UL);
    const auto frequency = min(parser.ulongval('F', 1000), 5000UL);
    const bool safety_override = parser.boolval('O');

    const millis_t end_time = millis() + SEC_TO_MS(exposure_seconds);

    const uint32_t us_per_period = 1'000'000 / frequency;
    const uint32_t us_on_time = (us_per_period / 255) * intensity;
    const uint32_t us_off_time = us_per_period - us_on_time;

    static auto time_fixed_idle = [](uint32_t micro_seconds) {
        static millis_t next_idle_time = millis();
        uint32_t turn_off_time = micros() + micro_seconds;
        if (millis() > next_idle_time) {
            idle();
            next_idle_time += 1000;
        }
        delayMicroseconds(turn_off_time - micros());
    };

    write_uvc_switches(HIGH);
    //TODO: this logic seems messy
    while ((safety_override || (READ(UVC_TFAULT_PIN) == UVC_TFAULT_ACTIVE_STATE)) && millis() < end_time) {
        if (intensity > 0)
            WRITE(UVC_PWM_PIN, HIGH);
        if (intensity >= 128)
            time_fixed_idle(us_on_time);
        else
            delayMicroseconds(us_on_time);
        if (intensity < 255)
            WRITE(UVC_PWM_PIN, LOW);
        if (intensity < 128)
            time_fixed_idle(us_off_time);
        else
            delayMicroseconds(us_off_time);

        if DEBUGGING (INFO)
            report_current_sense();
    }
    WRITE(UVC_PWM_PIN, LOW);
    write_uvc_switches(LOW);
}

#endif // UVC_STERILIZATION