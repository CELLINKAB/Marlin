/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
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

#include "../../inc/MarlinConfig.h"

#if ENABLED(UVC_STERILIZATION)

#    include "../../MarlinCore.h"
#    include "../../feature/uvc_sterilization.h"
#    include "../gcode.h"

/**
 * @brief UVC sterilization all-in-one command.
 * @brief params: I(intensity) S(seconds)
 * @brief options:  O(override flag) A(async flag) V(verbosity flag) W(wait flag)
 * 
 */
void GcodeSuite::M806()
{
    const uint8_t intensity = parser.byteval('I', 255);

    if (intensity == 0) {
        uvc_controller.stop();
        return;
    }

    if (parser.boolval('W'))
    {
        while (uvc_controller.running())
            idle();
    }

    uvc_controller.safety_override = parser.boolval('O');
    const uint32_t exposure_seconds = min(parser.ulongval('S',
                                                          UVCController::DEFAULT_EXPOSURE_SECONDS),
                                          UVCController::MAX_EXPOSURE_SECONDS);
    uvc_controller.auto_off_time = millis() + SEC_TO_MS(exposure_seconds);
    uvc_controller.send_reports = parser.boolval('V');

    // if no seconds or A are given, user probably expects async
    const bool async = !parser.seen("SA") || parser.boolval('A');

    uvc_controller.start(intensity);

    if (async)
        return;

    while (uvc_controller.running())
        idle();
}

#endif // UVC_STERILIZATION