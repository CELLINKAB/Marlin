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

#if ALL(CELLINK_REPORTING, OPTICAL_AUTOCAL)

#    include "../../feature/cellink_reporter.h"
#    include "../../feature/optical_autocal.h"
#    include "../../module/tool_change.h"
#    include "../gcode.h"

void cellink::Reporter::M798::report()
{
    cellink::serial_echoln_kv("AT",
                              active_extruder,
                              "AUTOCAL",
                              optical_autocal.is_calibrated(active_extruder));
}

void cellink::Reporter::M799::report()
{
    const auto& offset = optical_autocal.offset(active_extruder);
    cellink::serial_echoln_kv("AT",
                              active_extruder,
                              "AUTOCAL_XOFF",
                              offset.x,
                              "AUTOCAL_YOFF",
                              offset.y,
                              "AUTOCAL_ZOFF",
                              offset.z);
}

/**
   * @brief Reset nozzle calibration status, aliases G92.1
   * 
   */
void GcodeSuite::M797()
{
    if (!parser.seenval('T'))
        optical_autocal.reset_all();
    const auto current_tool = active_extruder;
    const auto target_tool = get_target_extruder_from_command();
    planner.synchronize(); // moving while changing calibration is unsafe
    if (current_tool != target_tool) {
        tool_change(target_tool, true);
    }
    process_subcommands_now(F("G510 R"));
    if (current_tool != target_tool) {
        tool_change(current_tool, true);
    }
}

/**
   * @brief Get current tool calibration status
   * 
   */
void GcodeSuite::M798()
{
    cellink::reporter.m798.set_interval(parser.boolval('S'));
    cellink::reporter.m798.report();
}

/**
   * @brief Get current tool calibration offsets
   * 
   */
void GcodeSuite::M799()
{
    cellink::reporter.m799.set_interval(parser.boolval('S'));
    cellink::reporter.m799.report();
}

#elif ENABLED(CELLINK_REPORTING)

#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

void cellink::Reporter::M798::report() {}
void cellink::Reporter::M799::report() {}

void GcodeSuite::M797() {}
void GcodeSuite::M798() {}
void GcodeSuite::M799() {}
#endif // CELLINK_REPORTING && OPTICAL_AUTOCAL