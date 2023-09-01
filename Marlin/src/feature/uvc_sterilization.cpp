#include "../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "uvc_sterilization.h"

void UVCController::init()
{
    static bool is_init = false;

    if (is_init)
        return;

#    if PIN_EXISTS(UVC_RELAY)
    OUT_WRITE(UVC_RELAY_PIN, LOW);
#    endif

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

#    if PIN_EXISTS(UVC_TFAULT)
    SET_INPUT_PULLDOWN(UVC_TFAULT_PIN);
#    endif

    is_init = true;
}

void UVCController::write_uvc_switches(bool state)
{
#    if PIN_EXISTS(UVC_RELAY)
    WRITE(UVC_RELAY_PIN, state);
#    endif
#    if PINS_EXIST(UVC_SH_CS_1, UVC_SH_CS_2, UVC_SH_CS_3, UVC_SH_CS_4, UVC_SH_CS_5)
    WRITE(UVC_SWITCH_1_PIN, state);
    WRITE(UVC_SWITCH_2_PIN, state);
    WRITE(UVC_SWITCH_3_PIN, state);
    WRITE(UVC_SWITCH_4_PIN, state);
    WRITE(UVC_SWITCH_5_PIN, state);
#    endif
}

void UVCController::report_current_sense()
{
#    if PINS_EXIST(UVC_SH_CS_1, UVC_SH_CS_2, UVC_SH_CS_3, UVC_SH_CS_4, UVC_SH_CS_5)
    cellink::serial_echoln_kv("CS_1",
                              analogRead(UVC_SH_CS_1_PIN),
                              "CS_2",
                              analogRead(UVC_SH_CS_2_PIN),
                              "CS_3",
                              analogRead(UVC_SH_CS_3_PIN),
                              "CS_4",
                              analogRead(UVC_SH_CS_4_PIN),
                              "CS_5",
                              analogRead(UVC_SH_CS_5_PIN));
#    endif
}

bool UVCController::ot_prewarn()
{
#    if PIN_EXISTS(UVC_TFAULT)
    if (safety_override)
        return false;
    else
        return READ(UVC_TFAULT_PIN) == UVC_TFAULT_ACTIVE_STATE;
#    else
    return false;
#    endif
}

void UVCController::start(uint8_t intensity)
{
    write_uvc_switches(true);
    TERN_(UVC_PWM_INVERTING, intensity = 255 - intensity);
    analogWrite(UVC_PWM_PIN, intensity);
}

void UVCController::update(millis_t now)
{
    if ((auto_off_time > 0 && now >= auto_off_time) || ot_prewarn()) {
        stop();
    }

    static millis_t next_report = now;
    if (send_reports && now >= next_report) {
        report_current_sense();
        next_report = now + 1000;
    }
}

void UVCController::stop()
{
    analogWrite(UVC_PWM_PIN, TERN(UVC_PWM_INVERTING, 255, 0));
    write_uvc_switches(false);
    auto_off_time = 0;
    send_reports = false;
}

UVCController uvc_controller;

#endif