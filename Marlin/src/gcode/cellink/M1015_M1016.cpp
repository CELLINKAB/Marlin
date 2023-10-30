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

#if ENABLED(CELLINK_REPORTING)

#    include "../../feature/cellink_reporter.h"
#    include "../../module/planner.h"
#    include "../gcode.h"

void cellink::Reporter::M1015::report()
{
    const auto pos = current_position.asLogical();
    serial_echoln_kv("XPOS", pos.x, "YPOS", pos.y, "ZPOS", pos.z);
}

void cellink::Reporter::M1016::report()
{
    const auto pos = planner.get_axis_positions_mm();
    serial_echoln_kv("XMPOS", pos.x, "YMPOS", pos.y, "ZMPOS", pos.z);
}

/**
   * @brief Get current position in cellink protocol format
   * 
   */
void GcodeSuite::M1015()
{
    cellink::reporter.m1015.set_interval(parser.byteval('S'));
    cellink::reporter.m1015.report();
}

/**
   * @brief Get current machine position in cellink protocol format
   * 
   */
void GcodeSuite::M1016()
{
    cellink::reporter.m1016.set_interval(parser.byteval('S'));
    cellink::reporter.m1016.report();
}

#endif // CELLINK_REPORTING