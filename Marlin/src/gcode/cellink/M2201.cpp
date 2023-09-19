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

size_t printhead::printhead_rx_err_counter = 0;
size_t printhead::printhead_tx_err_counter = 0;

// get printhead communication error counters
void GcodeSuite::M2201()
{
    cellink::serial_echoln_kv("PRINTHEAD_TX_ERRORS",
                              printhead::printhead_tx_err_counter,
                              "PRINTHEAD_RX_ERRORS",
                              printhead::printhead_rx_err_counter);
}

#endif