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

#if ALL(FESTO_PNEUMATICS, CELLINK_REPORTING)

#    include "../../feature/air_system/pneumatics.h"
#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

void cellink::Reporter::M816::report()
{
    pneumatics::report_sensors();
}

// get pressure sensors
void GcodeSuite::M816()
{
    cellink::reporter.m816.report();
    cellink::reporter.m816.set_interval(parser.byteval('S'));
}

#endif