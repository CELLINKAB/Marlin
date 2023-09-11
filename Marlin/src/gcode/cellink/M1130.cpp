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


#include "../../feature/cellink_reporter.h"
#include "../gcode.h"

#if PINS_EXIST(CS_24V, CS_BED_24V_CS)

void GcodeSuite::M1130()
{
    pinMode(CS_24V_PIN, INPUT_ANALOG);
    pinMode(CS_BED_24V_CS_PIN, INPUT_ANALOG);

    auto current_1 = analogRead(CS_24V_PIN);
    auto current_2 = analogRead(CS_BED_24V_CS_PIN);

    // TODO: scale these values properly

    cellink::serial_echoln_kv("LOAD_SWITCH_1_CURRENT", current_1, "LOAD_SWITCH_2_CURRENT", current_2);
}

#endif // PINS_EXIST(CS_24V, CS_BED_24V_CS)
