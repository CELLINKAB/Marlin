// copyright cellink 2022 GPLv3
#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../../MarlinCore.h"
#    include "../../feature/uvc_sterilization.h"
#    include "../gcode.h"

/**
 * @brief UVC sterilization all-in-one command.
 * @brief params: I(intensity) S(seconds)
 * @brief options:  O(override flag) A(async flag) V(verbosity flag)
 * 
 */
void GcodeSuite::M806()
{
    const uint8_t intensity = parser.byteval('I', 255);

    if (intensity == 0) {
        uvc_controller.stop();
        return;
    }

    uvc_controller.safety_override = parser.boolval('O');
    const uint32_t exposure_seconds = min(parser.ulongval('S',
                                                          UVCController::DEFAULT_EXPOSURE_SECONDS),
                                          UVCController::MAX_EXPOSURE_SECONDS);
    uvc_controller.auto_off_time = millis() + SEC_TO_MS(exposure_seconds);
    uvc_controller.send_reports = parser.boolval('V');

    const bool async = parser.boolval('A');

    uvc_controller.start(intensity);

    if (async)
        return;

    while (millis() <= uvc_controller.auto_off_time)
        idle();
}

#endif // UVC_STERILIZATION