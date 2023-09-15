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

#if ENABLED(CHANTARELLE_SUPPORT)

#    include "../../feature/cellink_reporter.h"
#    include "../../feature/guppi_printhead/chantarelle.h"
#    include "../gcode.h"

//DebugGetEncoders
void GcodeSuite::M2200()
{
    const auto res = ph_controller.debug_get_encoders();
    cellink::serial_echoln_kv("SLIDER_0_ENCODER",
                              res.packet.payload[0],
                              "EXTRUDER_0_ENCODER",
                              res.packet.payload[1],
                              "SLIDER_1_ENCODER",
                              res.packet.payload[2],
                              "EXTRUDER_1_ENCODER",
                              res.packet.payload[3],
                              "SLIDER_2_ENCODER",
                              res.packet.payload[4],
                              "EXTRUDER_2_ENCODER",
                              res.packet.payload[5]);

}

#endif // CHANTARELLE_SUPPORT