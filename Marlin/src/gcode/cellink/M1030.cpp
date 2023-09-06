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

#if ENABLED(DYNAMIC_3POINT_LEVELING)

#    include "../../module/probe.h"
#    include "../gcode.h"

void GcodeSuite::M1030()
{
    if (!parser.seenval('I')) {
        SERIAL_ECHOLNPGM("PT_1_X:",
                         Probe::dynamic_three_point_points[0].x,
                         ", PT_1_Y:",
                         Probe::dynamic_three_point_points[0].y,
                         ", PT_2_X:",
                         Probe::dynamic_three_point_points[1].x,
                         ", PT_2_Y:",
                         Probe::dynamic_three_point_points[1].y,
                         ", PT_3_X:",
                         Probe::dynamic_three_point_points[2].x,
                         ", PT_3_Y:",
                         Probe::dynamic_three_point_points[2].y);
        return;
    }
    
    size_t index = parser.value_ulong();
    if (!WITHIN(index, 1, 3))
    {
        SERIAL_ERROR_MSG("INDEX_OUT_OF_BOUNDS");
        return;
    }
    index -= 1;

    xy_pos_t point{parser.axisunitsval('X',
                                       AxisEnum::X_AXIS,
                                       Probe::dynamic_three_point_points[index].x),
                   parser.axisunitsval('Y',
                                       AxisEnum::Y_AXIS,
                                       Probe::dynamic_three_point_points[index].y)};

    if (!Probe::build_time::can_reach(point)) {
        SERIAL_ERROR_MSG("POINT_UNREACHABLE");
        return;
    }

    Probe::dynamic_three_point_points[index] = point;
}

#endif // DYNAMIC_3POINT_LEVELING