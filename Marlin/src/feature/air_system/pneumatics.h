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


#pragma once

#include "../../inc/MarlinConfigPre.h"

#define AUTO_REPORT_PNEUMATIC_SENSORS 1
#if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
#    include "../../libs/autoreport.h"
#endif

#include "analog_sensor.h"
#include "pump.h"
#include "regulator.h"

namespace pneumatics {

void init();

void update();

enum class GripperState {
    Close,
    Release,
    Grip,
};

void set_gripper_valves(GripperState state);

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);

void report_sensors();

#if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
struct Reporter : AutoReporter<Reporter>
{
    static void report();
};

extern Reporter reporter;
#endif

} // namespace pneumatics