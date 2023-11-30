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

#include "../inc/MarlinConfig.h"
#if ENABLED(CELLINK_REPORTING)

#    include "../module/endstops.h"

#    include "cellink_reporter.h"

using namespace cellink;

void Reporter::M119::report()
{
    endstops.report_states();
}
void Reporter::M814::report() {}
void Reporter::M825::report() {}

void Reporter::tick_all()
{
    m119.tick();
    m772.tick();
    m798.tick();
    m799.tick();
    m802.tick();
    m816.tick();
    m821.tick();
    m825.tick();
    m1015.tick();
    m1016.tick();
}

Reporter cellink::reporter;
#endif
